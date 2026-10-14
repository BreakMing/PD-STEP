/*
 * motor_conv — mm 域与 steps 域换算纯函数（宿主可测）
 *
 * 公式与迁移前 cartfsm 的四个换算函数逐式一致；steps_per_mm 语义
 * 由 settings 派生访问器定义（$0 细分 × $1 每转整步数 ÷ $2 每转
 * 行程 mm）。
 * 前置条件：steps_per_mm > 0（settings 校验域保证）；不满足为未定义。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** mm/s → Q16.16 steps/s（带符号；64 位承载 VEL_MAX 上限）。 */
int64_t motor_conv_mmps_to_q16(long v_mm_s, double steps_per_mm);

/** mm/s² → steps/s²。 */
uint32_t motor_conv_accel_to_steps(long a_mm_s2, double steps_per_mm);

/** 指令位置 steps → µm（四舍五入，半数远离零）。 */
long motor_conv_steps_to_um(int32_t steps, double steps_per_mm);

/** µm → 指令位置 steps（上一函数的逆换算，四舍五入）。 */
int32_t motor_conv_um_to_steps(long um, double steps_per_mm);

#ifdef __cplusplus
}
#endif
