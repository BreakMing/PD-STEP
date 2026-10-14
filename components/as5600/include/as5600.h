/*
 * AS5600 magnetic encoder — 12-bit angle + multi-turn tracker（驱动核心层）
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   The AS5600 ZPOS / MPOS / CONF / MANG registers are one-time
 *   programmable (OTP): a wrong write destroys the chip permanently.
 *   Register-level red-line details live in as5600_hw.c.
 *   Do not add write helpers.
 *
 * 硬件事实：7-bit 地址 0x36、RAW ANGLE 0x0C/0D、12 bit → 4096
 * counts/rev；2GT 20T × 2 mm = 40 mm/rev（settings $2）。
 * 方向镜像（安装标定）经 magnetic_encoder_as5600_config_t 注入，
 * 在读层应用于原始角；采样任务与契约面在 magnetic_encoder 组件。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Counts per mechanical revolution (12-bit raw angle). */
#define AS5600_COUNTS_PER_REV   4096

/* ================================================================
 * Multi-turn tracker — pure logic, hardware independent (testable)
 * ================================================================ */

typedef struct {
    int32_t  pos_counts;   /* continuous position (counts, multi-turn, signed) */
    uint16_t last_raw;     /* previous raw angle 0..4095 */
    bool     seeded;       /* first sample received? */
} as5600_tracker_t;

/**
 * @brief Drop history — the next sample becomes position 0.
 */
void as5600_tracker_reset(as5600_tracker_t *t);

/**
 * @brief Feed one raw angle, return the updated continuous position.
 *
 * First sample only seeds the tracker (position 0, no displacement).
 * Wraparound correction:
 *   diff = raw - last_raw
 *   diff >  +2048 → diff -= 4096   (backwards across the 0/4095 edge)
 *   diff <  -2047 → diff += 4096   (forwards across the 0/4095 edge)
 * The asymmetric pair of thresholds covers all 4096 diff values with
 * no gap and no overlap.
 *
 * @param t     Tracker state.
 * @param raw   Raw angle 0..4095 (higher bits are masked off).
 * @return Continuous position in counts.
 */
int32_t as5600_tracker_update(as5600_tracker_t *t, uint16_t raw);

#ifdef __cplusplus
}
#endif
