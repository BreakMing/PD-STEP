/*
 * motor — 电机运动域契约（指令位置）
 *
 * 链接期绑定：实现源文件在同组件内（当前实现 motor_tmc2209.c，
 * 经 Kconfig choice 选择）。速度/位置的 Q16.16 为 step_engine 现有
 * API 的定点记法（16 位整数位 + 16 位小数位）。
 * 语义：docs/superpowers/specs/2026-10-14-device-decoupling-design.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 驱动器使能引脚（EN，低有效）使能/关断；en=true 等待 ≥1 ms 唤醒。 */
esp_err_t motor_enable(bool en);

/** 设定带符号目标速度（mm/s）并喂命令看门狗。 */
void motor_set_vel_mm_s(long v_mm_s);

/** 目标清 0 斜坡停；不喂看门狗。 */
void motor_halt(void);

/** 急停：停脉冲、清递推状态与目标、冻结指令位置。 */
void motor_stop(void);

/** true ⇔ 当前速度精确为零（不经 mm/s 整数舍入）。 */
bool motor_at_rest(void);

/** 指令位置（steps → µm 四舍五入）。 */
int64_t motor_cmd_position_um(void);

/** 指令位置重定基；前置条件：电机停止（无脉冲发出）。 */
void motor_set_position_um(long um);

/** 参数作用点：追踪加速度 / 看门狗减速 / 看门狗超时 / 方向反转。 */
void motor_set_accel_mm_s2(long a_mm_s2);
void motor_set_decel_mm_s2(long d_mm_s2);
void motor_set_wdt_ms(long t_ms);
void motor_set_dir_invert(bool invert);

#ifdef __cplusplus
}
#endif
