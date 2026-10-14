/*
 * motor — 步进电机类别契约（steps 域，句柄式）
 *
 * steps 指当前细分档的一个微步——微步档位变更即单位含义变更
 * （motor_set_microstep）。契约无方向旋钮：应用方向约定归解释层
 * （cartaxis 经指令与读数取反）。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motor_s *motor_handle_t;

/** 驱动器使能引脚（EN，低有效）使能/关断；en=true 等待 ≥1 ms 唤醒。 */
esp_err_t motor_enable(motor_handle_t m, bool en);

/** 设定带符号目标速度（steps/s）并喂命令看门狗；引擎内部定点递推
 *  平滑逼近，换向经零速。 */
void motor_set_vel(motor_handle_t m, long steps_per_s);

/** 目标清 0 斜坡停；不喂看门狗。 */
void motor_halt(motor_handle_t m);

/** 急停：停脉冲、清递推状态与目标、冻结指令位置。 */
void motor_stop(motor_handle_t m);

/** true ⇔ 当前速度精确为零（引擎内部定点域判零，不经整数舍入）。 */
bool motor_at_rest(motor_handle_t m);

/** 指令位置（微步，自上电累计）。 */
int32_t motor_position_steps(motor_handle_t m);

/** 指令位置重定基；前置条件：电机静止（无脉冲发出），违反为未定义。 */
void motor_set_position(motor_handle_t m, int32_t steps);

/** 追踪加速度 / 减速度档（steps/s²）。 */
void motor_set_accel(motor_handle_t m, uint32_t steps_per_s2);
void motor_set_decel(motor_handle_t m, uint32_t steps_per_s2);

/** 命令新鲜度看门狗：超过 ms 未收到有效速度命令，按减速档斜坡停。 */
void motor_set_wdt_ms(motor_handle_t m, uint32_t ms);

/** 细分档位（2^n，1–256）；前置条件：电机静止；变更后 steps 单位按
 *  新档解释，指令位置计数不重置——调用方负责重定基。 */
esp_err_t motor_set_microstep(motor_handle_t m, int microstep);

/** 运行 / 保持电流（mA）。 */
esp_err_t motor_set_current(motor_handle_t m, int run_ma, int hold_ma);

#ifdef __cplusplus
}
#endif
