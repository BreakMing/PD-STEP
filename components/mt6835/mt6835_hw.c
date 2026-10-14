/*
 * mt6835_hw — SPI 角度帧层（驱动核心层）
 *
 * SPI 角度读取事务（抄录见 docs/reviews/T4-brief.md，来源
 * SimpleFOC Arduino-FOC-drivers MT6835 驱动，与手册 §SSI 一致）：
 *   MODE3、MSB 先行；48 bit（6 字节）单次事务，CS 拉低期间完成：
 *     TX: [0]=OP_ANGLE(0b1010)<<4, [1]=ANGLE1 地址 0x003, [2..5]=0
 *     RX: [2]=ANGLE[20:13]  [3]=ANGLE[12:5]
 *         [4]=ANGLE[4:0]<<3 | STATUS[2:0]  [5]=CRC8
 *   raw21 = (rx[2]<<13) | (rx[3]<<5) | (rx[4]>>3)
 *   STATUS：bit0 过速、bit1 磁场弱、bit2 欠压（仅记录，不上报协议）。
 *   CRC8（poly 0x07、init 0x00）与器件回传寄存器 0x006 比较。
 *   帧校验：拔线时 MISO 悬空常读全零——CRC 与全零帧一并拒绝，
 *   失败帧原文留存（mt6835_hw_last_failure）供设备组件分诊。
 *
 * 层内约束：参数全部由 mt6835_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务。
 */

#include "mt6835_hw.h"

#include "mt6835_urad.h"   /* mt6835_crc8 */

/* ---- 帧常量（T4-brief.md 抄录） ---- */
#define MT6835_OP_ANGLE     0x0Au       /* 0b1010 */
#define MT6835_REG_ANGLE1   0x03u

static spi_device_handle_t s_dev = NULL;

/* 失败分诊：SPI 错误 / 全零帧 / CRC 不符；进入失败态时由设备组件 dump 一次 */
static mt6835_hw_fail_t s_last_fail = MT6835_HW_FAIL_SPI_ERR;
static uint8_t s_last_frame[MT6835_HW_FRAME_BYTES];

/* ================================================================
 * Lifecycle
 * ================================================================ */

esp_err_t mt6835_hw_init(const mt6835_hw_config_t *cfg)
{
    if (cfg == NULL || cfg->host < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->clock_speed_hz,
        .mode = 3,                          /* CPOL=1 CPHA=2（MODE3） */
        .spics_io_num = cfg->spics_io_num,
        .queue_size = 1,
    };
    return spi_bus_add_device(cfg->host, &dev_cfg, &s_dev);
}

/* ================================================================
 * SPI 读取
 * ================================================================ */

/* 读一次 21-bit 原始角。返回 false = 事务/帧校验失败（raw 不变）。 */
bool mt6835_hw_read_raw(uint32_t *raw_out, uint8_t *status_out)
{
    uint8_t tx[MT6835_HW_FRAME_BYTES] = {
        (uint8_t)(MT6835_OP_ANGLE << 4),
        MT6835_REG_ANGLE1,
        0, 0, 0, 0,
    };
    uint8_t rx[MT6835_HW_FRAME_BYTES] = { 0 };

    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = MT6835_HW_FRAME_BYTES * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    if (err != ESP_OK) {
        s_last_fail = MT6835_HW_FAIL_SPI_ERR;
        goto fail;
    }

    uint32_t raw = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5)
                 | ((uint32_t)rx[4] >> 3);
    uint8_t status = rx[4] & 0x07;

    /* 帧校验：拔线时 MISO 悬空常读全零——CRC 与全零帧一并拒绝，
     * 失败计入 fail_streak（10 ms 内 is_ok 变 false → FAULT/MT6835） */
    if (raw == 0 && status == 0 && rx[5] == 0) {
        s_last_fail = MT6835_HW_FAIL_ALL_ZERO;
        goto fail;
    }
    if (mt6835_crc8(raw, status) != rx[5]) {
        s_last_fail = MT6835_HW_FAIL_CRC;
        goto fail;
    }

    *raw_out = raw;
    *status_out = status;
    return true;

fail:
    /* 帧原文留存，进入失败态时一次性 dump（接线/算法分诊用） */
    for (int i = 0; i < MT6835_HW_FRAME_BYTES; i++) {
        s_last_frame[i] = rx[i];
    }
    return false;
}

/* ================================================================
 * 失败分诊快照
 * ================================================================ */

void mt6835_hw_last_failure(mt6835_hw_fail_t *kind,
                            uint8_t frame[MT6835_HW_FRAME_BYTES])
{
    if (kind != NULL) {
        *kind = s_last_fail;
    }
    if (frame != NULL) {
        for (int i = 0; i < MT6835_HW_FRAME_BYTES; i++) {
            frame[i] = s_last_frame[i];
        }
    }
}
