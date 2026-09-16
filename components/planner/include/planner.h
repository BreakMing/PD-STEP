/*
 * Motion Planner — mm-domain → step-domain segment solver
 *
 * Consumes absolute target coordinates (mm) from the G-code parser and
 * produces trapezoidal / triangular step_seg_t descriptors for the step
 * engine (T6).  Responsibilities:
 *
 *   1. Soft-limit validation        (settings $12/$13)
 *   2. mm-domain clamping           (settings $7 max feed, $8 max accel)
 *   3. mm → steps conversion        (k = steps_per_mm, settings $0/$1/$2)
 *   4. Trapezoid / triangle solve   (with engine-domain clamping)
 *   5. Blocking enqueue with '!' race handling
 *   6. Commanded-position commit
 *
 * All work happens in task context.  No ISR callbacks, no float in ISR.
 *
 * Dependencies: step_engine (segment consumer), settings (parameters).
 * Deliberately does NOT depend on tmc2209 / protocol / bsp.
 *
 * Position model: pos_steps starts at 0 on power-up (D10) and tracks the
 * *commanded* position (target of the last accepted command), not the
 * encoder.  Single writer = comm task; 32-bit aligned reads are atomic
 * on Xtensa.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "step_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Status codes ---- */
typedef enum {
    PLANNER_OK             = 0,
    PLANNER_ERR_SOFT_LIMIT = 2,   /* Target outside soft limits, or
                                   * absurd travel → error:2 */
} planner_status_t;

/* ================================================================
 * Lifecycle
 * ================================================================ */

/**
 * @brief Initialize the planner.
 *
 * Sets commanded position to 0 (D10: power-up origin), caches
 * steps_per_mm from settings, and subscribes to $0/$1/$2 changes
 * (one settings subscriber slot).
 *
 * Must be called after step_engine_init() succeeded and before the
 * parser is created.  Idempotent — repeated calls are no-ops.
 *
 * @return ESP_OK on success.
 */
esp_err_t planner_init(void);

/* ================================================================
 * Core planning
 * ================================================================ */

/**
 * @brief Plan and enqueue a move to an absolute target.
 *
 * Pipeline: soft-limit check → $7/$8 mm-domain clamp → mm→steps →
 * trapezoid/triangle solve (with engine-domain clamp and floor) →
 * blocking enqueue → position commit.
 *
 * Blocking back-pressure: if the 16-deep queue is full, this function
 * waits (vTaskDelay 1 ms) until space frees up.  If the queue is
 * observed empty right after a failed enqueue, the pending move was
 * cancelled by step_engine_stop() ('!' immediate stop, D11 — no resume
 * semantics); the segment is dropped, position is NOT committed, and
 * PLANNER_OK is returned so the G-code line still acknowledges "ok".
 *
 * @param target_mm    Absolute target position (mm).
 * @param feed_mm_min  Feed rate (mm/min), clamped to $7 inside.
 * @param accel_mm_s2  Acceleration (mm/s²), clamped to $8 inside.
 *
 * @return PLANNER_OK, or PLANNER_ERR_SOFT_LIMIT (→ error:2).
 */
planner_status_t planner_enqueue(float target_mm, float feed_mm_min,
                                 float accel_mm_s2);

/**
 * @brief Set the commanded position (G92 — no motion produced).
 *
 * Does NOT validate soft limits: these constrain move *targets* only.
 * The parser already guarantees |X| <= 1e6.
 *
 * @param new_pos_mm  New commanded coordinate (mm).
 */
void planner_set_origin(float new_pos_mm);

/* ================================================================
 * Telemetry (T9 monitor consumes these)
 * ================================================================ */

/**
 * @brief Commanded position in steps (signed).
 */
int32_t planner_position_steps(void);

/**
 * @brief Commanded position in mm (= pos_steps / k).
 */
float planner_position_mm(void);

/* ================================================================
 * Pure solver
 * ================================================================ */

/**
 * @brief Solve a velocity profile for |delta| = delta_steps.
 *
 * Fills *out_seg with accel/cruise/decel step counts and the resulting
 * accel / v_cruise values.  The caller sets out_seg->dir.
 *
 * Inputs v_max_steps and accel_steps (> 0) are clamped internally to
 * the engine domain:
 *   upper bound 65535.0f  — float_to_q16(v) = (uint32)(v*65536+0.5f);
 *                           exactly 65536 would wrap to 0 (T6 fact #2)
 *   lower bound 1e-3f     — prevents float_to_q16 rounding to 0
 *                           (v_q16=0 → ISR divide-by-zero;
 *                            a_q16=0 → never accelerates)
 *
 * Profile selection:
 *   d = v²/(2a) steps needed to reach v
 *   2d <= N → trapezoid: accel = decel = floor(d), cruise = N - 2*accel
 *   2d >  N → triangle : accel = N/2, decel = N - accel, cruise = 0,
 *                        v_cruise = sqrt(2*a*accel) (actual peak, < v)
 *
 * Invariant (asserted by self-test): accel+cruise+decel == N exactly.
 *
 * @param delta_steps   |step distance|, must be >= 1.
 * @param v_max_steps   Requested cruise velocity (steps/s), > 0.
 * @param accel_steps   Requested acceleration (steps/s²), > 0.
 * @param out_seg       Output segment (dir field left untouched).
 * @return true on success, false if arguments are invalid.
 */
bool planner_solve(uint32_t delta_steps, float v_max_steps,
                   float accel_steps, step_seg_t *out_seg);

/* ================================================================
 * On-target self-test (only when CONFIG_PLANNER_SELF_TEST=y)
 * ================================================================ */

#ifdef CONFIG_PLANNER_SELF_TEST
/** Copy of the most recently enqueued (accepted) segment. */
void planner_selftest_last_seg(step_seg_t *out);

/** Run all 21 planner self-tests.  Returns pass count.
 *  Final log line: "SELF-TEST DONE: <n>/21 PASS". */
int  planner_self_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
