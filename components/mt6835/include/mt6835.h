/*
 * mt6835 — 摆角编码器组件（SPI 读取 + 1 kHz 采样任务）
 *
 * 纯传感器职责：读原始 21-bit 角、±π 环绕换算（标定前的原始角）、
 * 16 样本窗口角速度估计、连续失败 ≥10 次的 is_ok。零位/符号标定
 * （POLE_ZERO_URAD / POLE_INV）由 telemetry 组帧时应用，
 * 本组件不读参数。
 *
 * 采样任务：CPU1、优先级 3、1 kHz vTaskDelayUntil（结构照抄 as5600：
 * 失败计数、is_ok、原子发布）。SPI 事务阻塞在 CPU1，不干扰 CPU0
 * 的 STEP ISR。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "mt6835_urad.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 接线参数（组装点读取 Kconfig 宏与 bsp 主机号后注入） */
typedef struct {
    spi_host_device_t host;
    int               clock_speed_hz;
    int               spics_io_num;
} mt6835_config_t;

/** 幂等。接线经 config 注入并启动 1 kHz 采样任务。 */
esp_err_t mt6835_init(const mt6835_config_t *cfg);

/**
 * @brief 最新摆角（±π 环绕 µrad，标定前原始角）。
 */
int32_t mt6835_theta_urad(void);

/**
 * @brief 摆角速度（mrad/s，16 ms 窗口）。
 */
int32_t mt6835_vth_mrad_s(void);

/**
 * @brief false = 连续 ≥10 次 SPI 读失败。
 */
bool mt6835_is_ok(void);

/**
 * @brief 最新原始 21-bit 读数（调试/自测）。
 */
uint32_t mt6835_last_raw(void);

/**
 * @brief 多圈连续位置（counts，自上电累计；调试/自测）。
 */
int32_t mt6835_position_counts(void);

#ifdef __cplusplus
}
#endif
