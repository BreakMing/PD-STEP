/*
 * cartaxis_conv — mm 域与 steps 域换算纯函数（宿主可测）
 *
 * steps_per_mm 语义由 settings 派生访问器定义（$0 细分 × $1 每转
 * 整步数 ÷ $2 每转行程 mm）。舍入统一 llround（半数远离零）。
 * 前置条件：steps_per_mm > 0（settings 校验域保证）；不满足为未定义。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** mm/s → steps/s（带符号）。 */
long cartaxis_mmps_to_steps(long v_mm_s, double steps_per_mm);

/** mm/s² → steps/s²。 */
uint32_t cartaxis_accel_to_steps(long a_mm_s2, double steps_per_mm);

/** 指令位置 steps → µm。 */
long cartaxis_steps_to_um(int32_t steps, double steps_per_mm);

/** µm → 指令位置 steps（上一函数的逆换算）。 */
int32_t cartaxis_um_to_steps(long um, double steps_per_mm);

#ifdef __cplusplus
}
#endif
