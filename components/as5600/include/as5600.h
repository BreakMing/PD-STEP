/*
 * AS5600 magnetic encoder — 12-bit angle + multi-turn position
 *
 * Scope (PLAN T8): sensor layer only — I2C angle read → multi-turn
 * counting → continuous position (counts / mm) + velocity estimate +
 * G92 origin shift.  NO loss-of-step detection and NO closed-loop
 * compensation (D6 explicitly defers both to phase 2), no status-frame
 * packing (T9).
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   The AS5600 ZPOS / MPOS / CONF / MANG registers are one-time
 *   programmable (OTP): a wrong write destroys the chip permanently.
 *   This component therefore issues NO register write of any kind —
 *   its only bus operation is the read helper
 *   i2c_master_transmit_receive() (register address out, data in,
 *   one transaction).  Do not add write helpers here.
 *
 * Hardware facts (T0/T2):
 *   7-bit address 0x36 (CONFIG_BSP_AS5600_I2C_ADDR), I2C_NUM_0,
 *   SDA=GPIO8 / SCL=GPIO9 @ 400 kHz, external pullups present.
 *   RAW ANGLE = 0x0C/0x0D, 12 bit → 4096 counts/rev.
 *   Drive: 2GT 20T × 2 mm = 40 mm/rev (D7, settings $2) →
 *   1 count = 40 / 4096 = 9.7656 µm.
 *
 * Sampling: dedicated task at 1 kHz (PLAN §3.1/§3.2), pinned to CPU1 so
 * the blocking I2C transaction cannot delay the STEP-pulse ISR on CPU0.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

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
 * Wraparound correction (PLAN T8):
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

/* ================================================================
 * Driver + sampling task
 * ================================================================ */

/**
 * @brief Add the I2C device on the BSP bus and start the 1 kHz task.
 *
 * Idempotent.  Uses bsp_get_i2c_bus() (must already be initialized by
 * bsp_init()).  mm/count comes from settings $2 and tracks runtime
 * changes via one settings subscription slot.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if the bus is not ready,
 *         or an I2C / task-creation error.
 */
esp_err_t as5600_init(void);

/* ================================================================
 * Telemetry (consumed by T9 monitor)
 * ================================================================ */

/** Continuous position in mm, including the G92 origin shift. */
float    as5600_position_mm(void);

/** Continuous position in counts (multi-turn, signed) — the truth value. */
int32_t  as5600_position_counts(void);

/** Smoothed velocity in mm/s (16 ms window); ≈0 when at standstill. */
float    as5600_velocity_mm_s(void);

/** Most recent raw angle 0..4095 (debug / self-test). */
uint16_t as5600_last_raw(void);

/** false = I2C reads have been failing (≥10 consecutive failures). */
bool     as5600_is_ok(void);

/* ================================================================
 * G92 origin shift (D10: moves together with the commanded position)
 * ================================================================ */

/**
 * @brief Declare the current encoder position to be new_pos_mm (no motion).
 *
 * Implementation: offset = raw_position_mm − new_pos_mm;
 * reported position = raw_position_mm − offset.
 *
 * @param new_pos_mm  New reported coordinate in mm.
 */
void     as5600_set_origin(float new_pos_mm);

/* ================================================================
 * Self-test (18 cases: 12 pure logic + 6 hardware, plus a guided
 * manual segment that is logged but not counted)
 * ================================================================ */
#ifdef CONFIG_AS5600_SELF_TEST
int  as5600_self_test_run(void);   /* final line: "SELF-TEST DONE: <n>/<total> PASS" */
#endif

#ifdef __cplusplus
}
#endif
