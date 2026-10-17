/*
 * ch224q — CH224Q PD 诱骗芯片 I2C 调档层（驱动核心层）
 *
 * 共存语义（CH224DS1 V2.0 §5.2.3）：单电阻配置（本板 CF1 外接
 * 120 kΩ 定档 20V）下 I2C 功能自动启用——上电默认请求档由电阻
 * 决定，本层提供运行时改档与状态查询，上电路径不依赖本层。
 *
 * 寄存器：0x0A 电压控制（只写，0–5 = 5/9/12/15/20/28V）；
 * 0x09 状态（只读，位解析见 ch224q_status.h）；
 * 0x50 电流数据（只读，当前档最大电流，50 mA 单位）。
 * I2C 400 kHz、单字节读写；7-bit 地址 0x22/0x23 两种变体
 * （上板扫描确认，Kconfig 注入）。
 *
 * 层内约束：参数全部由 init 的 config 注入；不 include settings.h；
 * 不调用 bsp；不创建任务。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "ch224q_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 电压档（枚举值 = 0x0A 寄存器写入值） */
typedef enum {
    CH224Q_VSEL_5V  = 0,
    CH224Q_VSEL_9V  = 1,
    CH224Q_VSEL_12V = 2,
    CH224Q_VSEL_15V = 3,
    CH224Q_VSEL_20V = 4,
    CH224Q_VSEL_28V = 5,
} ch224q_vsel_t;

typedef struct {
    i2c_master_bus_handle_t bus;   /* 已初始化的 I2C 总线（组装点从 bsp 获取） */
    uint8_t                 dev_addr;   /* 7-bit 地址（0x22/0x23，上板扫描确认） */
} ch224q_config_t;

typedef struct ch224q_dev_t *ch224q_handle_t;

/**
 * @brief 在 config 给出的总线上添加 CH224Q 设备，创建实例。
 *
 * @return ESP_OK；cfg/handle_out 为 NULL 或 bus 为 NULL 返回
 *         ESP_ERR_INVALID_ARG；其余错误码与
 *         i2c_master_bus_add_device 一致。
 */
esp_err_t ch224q_init(const ch224q_config_t *cfg, ch224q_handle_t *handle_out);

/**
 * @brief 设定请求电压档（写 0x0A，覆盖 CF1 电阻默认档）。
 *
 * 档位能否成立取决于适配器能力与协商结果——以 get_status 的
 * PD 位和 vbus ADC 回读共同确认。
 */
esp_err_t ch224q_set_voltage(ch224q_handle_t h, ch224q_vsel_t vsel);

/**
 * @brief 读 0x09 并解析握手状态（位语义见 ch224q_status.h）。
 */
esp_err_t ch224q_get_status(ch224q_handle_t h, ch224q_status_t *out);

/**
 * @brief 读 0x50 当前档最大电流（适配器供电能力，供实验前评估）。
 *
 * @param ma_out 出参：档最大电流（mA，寄存器值 × 50）。
 */
esp_err_t ch224q_get_current_ma(ch224q_handle_t h, uint16_t *ma_out);

#ifdef __cplusplus
}
#endif
