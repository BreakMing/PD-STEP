/*
 * as5600_hw — AS5600 I2C 读层（驱动核心层）
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   ZPOS / MPOS / CONF / MANG are one-time-programmable (OTP); a wrong
 *   write permanently destroys the chip.  This file therefore contains
 *   NO register write: the single bus operation is
 *   as5600_hw_read_raw(), which uses i2c_master_transmit_receive()
 *   (register address out + N data bytes in, one transaction — a read).
 *   Do not add i2c_master_transmit() or any write helper here.
 *
 * 层内约束：参数全部由 as5600_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务。方向镜像（invert_dir，磁铁
 * 安装标定）发生在本读层，业务壳与纯逻辑 tracker 只见 0..4095
 * 的已定向读数。
 */

#include "as5600_hw.h"

#include "as5600.h"     /* AS5600_COUNTS_PER_REV */

static as5600_hw_config_t s_cfg;
static i2c_master_dev_handle_t s_dev = NULL;

/* ---- Register map (READ ONLY — see the red line above) ---- */
#define AS5600_REG_STATUS       0x0B   /* bit5 MD / bit4 ML / bit3 MH (magnet) */
#define AS5600_REG_RAW_ANGLE    0x0C   /* 0x0C = [11:8] (low nibble), 0x0D = [7:0] */
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE    0x1C

/* ---- Timing / limits ---- */
#define AS5600_I2C_TIMEOUT_MS   50

/* ================================================================
 * Lifecycle
 * ================================================================ */

esp_err_t as5600_hw_init(const as5600_hw_config_t *cfg)
{
    if (cfg == NULL || cfg->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->dev_addr,
        .scl_speed_hz    = cfg->scl_speed_hz,
    };
    return i2c_master_bus_add_device(cfg->bus, &dev_cfg, &s_dev);
}

/* ================================================================
 * Read path (the only bus operation in this layer)
 * ================================================================ */

/* Read the 12-bit raw angle as a single 2-byte burst.
 * Returns false if the transaction failed (value left untouched). */
bool as5600_hw_read_raw(uint16_t *raw_out)
{
    const uint8_t reg = AS5600_REG_RAW_ANGLE;
    uint8_t       buf[2] = { 0, 0 };

    /* READ: write the register address, read 2 bytes back, one transaction.
     * No data is ever written to the device. */
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1,
                                                buf, sizeof(buf),
                                                AS5600_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return false;
    }

    uint16_t raw = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);   /* 0..4095 */

    if (s_cfg.invert_dir) {
        /* Mounting calibration — invert_dir 由业务壳按 Kconfig
         * AS5600_INVERT_DIR 注入。Mirroring the angle reverses the
         * counting sense and keeps the value inside 0..4095, so a +X
         * motor move increases the reported position.  Applied here
         * (read layer) so the pure-logic tracker and its self-test
         * cases stay hardware-independent. */
        raw = (uint16_t)((AS5600_COUNTS_PER_REV - 1) - raw);
    }

    *raw_out = raw;
    return true;
}
