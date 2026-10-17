/*
 * ch224q — CH224Q I2C 调档实现（契约见 ch224q.h）
 */

#include "ch224q.h"

#include <stdlib.h>

/* ---- 寄存器（CH224DS1 V2.0） ---- */
#define CH224Q_REG_STATUS   0x09   /* 只读：握手状态位 */
#define CH224Q_REG_VSEL     0x0A   /* 只写：电压控制 0–5 */
#define CH224Q_REG_CURRENT  0x50   /* 只读：当前档最大电流，50 mA 单位 */

#define CH224Q_I2C_TIMEOUT_MS  50
#define CH224Q_CURRENT_UNIT_MA 50

struct ch224q_dev_t {
    i2c_master_dev_handle_t dev;
};

/* 状态位解析在 ch224q_status.c（纯逻辑独立 TU，宿主可测） */

/* ---- I2C 单字节读/写 ---- */

static esp_err_t reg_read(ch224q_handle_t h, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, val, 1,
                                       CH224Q_I2C_TIMEOUT_MS);
}

static esp_err_t reg_write(ch224q_handle_t h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(h->dev, buf, sizeof(buf),
                               CH224Q_I2C_TIMEOUT_MS);
}

/* ---- 生命周期与契约面 ---- */

esp_err_t ch224q_init(const ch224q_config_t *cfg, ch224q_handle_t *handle_out)
{
    if (cfg == NULL || handle_out == NULL || cfg->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ch224q_handle_t h = calloc(1, sizeof(*h));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->dev_addr,
        .scl_speed_hz    = 400000,   /* 手册：I2C 400 kHz */
    };
    esp_err_t err = i2c_master_bus_add_device(cfg->bus, &dev_cfg, &h->dev);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    *handle_out = h;
    return ESP_OK;
}

esp_err_t ch224q_set_voltage(ch224q_handle_t h, ch224q_vsel_t vsel)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((int)vsel < (int)CH224Q_VSEL_5V || (int)vsel > (int)CH224Q_VSEL_28V) {
        return ESP_ERR_INVALID_ARG;
    }
    return reg_write(h, CH224Q_REG_VSEL, (uint8_t)vsel);
}

esp_err_t ch224q_get_status(ch224q_handle_t h, ch224q_status_t *out)
{
    if (h == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t raw = 0;
    esp_err_t err = reg_read(h, CH224Q_REG_STATUS, &raw);
    if (err != ESP_OK) {
        return err;
    }
    ch224q_decode_status(raw, out);
    return ESP_OK;
}

esp_err_t ch224q_get_current_ma(ch224q_handle_t h, uint16_t *ma_out)
{
    if (h == NULL || ma_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t raw = 0;
    esp_err_t err = reg_read(h, CH224Q_REG_CURRENT, &raw);
    if (err != ESP_OK) {
        return err;
    }
    *ma_out = (uint16_t)(raw * CH224Q_CURRENT_UNIT_MA);
    return ESP_OK;
}
