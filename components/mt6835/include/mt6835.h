/*
 * mt6835 — 摆角编码器组件（SPI 读取 + 1 kHz 采样任务）
 *
 * 纯传感器职责：读原始 21-bit 角、±π 环绕换算（标定前的原始角）、
 * 16 样本窗口角速度估计、连续失败 ≥10 次的 is_ok。零位/符号标定
 * （POLE_ZERO_URAD / POLE_INV）由 telemetry 组帧时应用（任务 7），
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
#include "mt6835_urad.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 在 bsp 的 SPI 总线上添加设备并启动 1 kHz 采样任务。幂等。
 *
 * @return ESP_OK；ESP_ERR_INVALID_STATE 总线未初始化；否则 SPI/任务错误。
 */
esp_err_t mt6835_init(void);

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

/* ---- 上板自测（CONFIG_MT6835_SELF_TEST=y） ---- */
#ifdef CONFIG_MT6835_SELF_TEST
/**
 * 运行自测，结束行 "SELF-TEST DONE: <n>/<total> PASS"。
 */
int mt6835_self_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
