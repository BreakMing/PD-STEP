/*
 * cartaxis — 小车直线轴解释组件（mm 域命令面 ↔ motor steps 域）
 *
 * 换算即读即算 settings 派生 steps_per_mm（$0×$1÷$2）；X_INV 经
 * 指令与读数统一取反（cartfsm 的调用面签名与旧 motor.h 一致）；
 * $0 细分档由本组件单一写者持有（决策表见规格「cartaxis」节）。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 绑定电机实例并订阅 $0/$1/$2（$0 变更按静止/运动两路处理）。 */
void cartaxis_init(motor_handle_t m);

/** 以下签名与语义同旧 motor.h 业务面（消费方 cartfsm / flowtest 只改名）。 */
esp_err_t cartaxis_enable(bool en);
void cartaxis_set_vel_mm_s(long v_mm_s);
void cartaxis_halt(void);
void cartaxis_stop(void);
bool cartaxis_at_rest(void);
int64_t cartaxis_cmd_position_um(void);
void cartaxis_set_position_um(long um);   /* 前置：电机静止 */
void cartaxis_set_accel_mm_s2(long a_mm_s2);
void cartaxis_set_decel_mm_s2(long d_mm_s2);
void cartaxis_set_wdt_ms(long t_ms);

#ifdef __cplusplus
}
#endif
