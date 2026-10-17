/*
 * AS5600 magnetic encoder — 12-bit angle（驱动核心层）
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   The AS5600 ZPOS / MPOS / CONF / MANG registers are one-time
 *   programmable (OTP): a wrong write destroys the chip permanently.
 *   Register-level red-line details live in as5600_hw.c.
 *   Do not add write helpers.
 *
 * 硬件事实：7-bit 地址 0x36、RAW ANGLE 0x0C/0D、12 bit → 4096
 * counts/rev；2GT 20T × 2 mm = 40 mm/rev（settings 每转行程默认值）。
 * 方向镜像（安装标定）经 magnetic_encoder_as5600_config_t 注入，
 * 在读层应用于原始角；采样任务与契约面在 magnetic_encoder 组件，
 * 多圈累计（原 as5600_tracker）为类别组件参数化纯逻辑
 * （magnetic_encoder_tracker.c）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Counts per mechanical revolution (12-bit raw angle). */
#define AS5600_COUNTS_PER_REV   4096

#ifdef __cplusplus
}
#endif
