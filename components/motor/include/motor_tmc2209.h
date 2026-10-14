/*
 * motor_tmc2209 — motor 类别的 TMC2209 + GPTimer 实现
 * （init 与接线 config）
 *
 * 组合 tmc2209（UART 配置/电流/细分/EN）与 step_engine（STEP/DIR
 * 脉冲引擎）；类别层无 mm 换算（归 cartaxis 解释组件）。
 */
#pragma once

#include "esp_err.h"
#include "tmc2209.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    tmc2209_config_t tmc;       /* 驱动器配置（含 en_gpio 与上电初值） */
    int              step_gpio; /* STEP 输出引脚 */
    int              dir_gpio;  /* DIR 输出引脚 */
} motor_tmc2209_config_t;

/**
 * @brief 内部完成 tmc2209_init 与 step_engine_init（次序即此）。幂等。
 *
 * @return ESP_OK；cfg 或 handle_out 为 NULL 返回 ESP_ERR_INVALID_ARG；
 *         tmc2209_init / step_engine_init 失败时返回其错误码。
 */
esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg,
                             motor_handle_t *handle_out);

#ifdef __cplusplus
}
#endif
