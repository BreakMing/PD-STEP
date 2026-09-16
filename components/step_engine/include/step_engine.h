/*
 * Step Engine — GPTimer ISR stepper pulse generator
 *
 * Consumes pre-computed trapezoidal segments from a SPSC ring queue
 * and generates STEP/DIR pulses via GPTimer one-shot alarms in ISR
 * context.  All ISR math is integer / fixed-point; float→fixed
 * conversion happens at enqueue time (task context).
 *
 * Queue: SPSC ring, depth 16 (power of two).  Task side writes,
 * ISR side reads.  Volatile head/tail indices, publish-after-write.
 *
 * Immediate stop (step_engine_stop): clears queue, stops pulses,
 * freezes position counter.  Race-safe vs ISR.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Segment (input: task side, float fields) ---- */

typedef struct {
    uint32_t steps;         /* total steps in segment, >0 */
    bool     dir;           /* logical direction (true = forward) */
    uint32_t accel_steps;   /* steps in acceleration phase */
    uint32_t cruise_steps;  /* steps in cruise phase */
    uint32_t decel_steps;   /* steps in deceleration phase (= steps - accel - cruise) */
    float    accel;         /* acceleration in steps/s^2, >0 */
    float    v_cruise;      /* cruise velocity in steps/s, >0 */
} step_seg_t;

/* ---- Public API ---- */

/**
 * @brief Initialize the step engine.
 *
 * Sets up GPTimer @ 1 MHz resolution, one-shot alarm, ISR pinned
 * to CPU0.  STEP and DIR GPIOs are NOT re-initialized (BSP already
 * configured them as OUT LOW); we only record the pin numbers.
 *
 * @param step_gpio  GPIO number for STEP output.
 * @param dir_gpio   GPIO number for DIR output.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t step_engine_init(int step_gpio, int dir_gpio);

/**
 * @brief Enqueue a trapezoidal segment for execution.
 *
 * Converts float fields to fixed-point and pushes the segment into
 * the SPSC ring queue.  If the ISR is idle, starts the timer.
 *
 * @param seg  Segment descriptor (must have accel+cruise+decel == steps).
 * @return true if enqueued, false if queue is full (back-pressure).
 */
bool step_engine_enqueue(const step_seg_t *seg);

/**
 * @brief Immediate stop — clear queue, stop pulses, freeze position.
 *
 * Called from task context (comm_task '!' real-time callback).
 * Guarantees no further STEP pulses are issued after this call
 * returns.  Position counter is frozen at the last executed step.
 *
 * Safe to call from task context; uses a critical section to
 * synchronize with the ISR.
 */
void step_engine_stop(void);

/* ---- Telemetry snapshot (T9 monitor consumes these) ---- */

/**
 * @brief Current commanded position in steps (signed, cumulative).
 *
 * Includes $10 direction inversion.  32-bit aligned read is atomic
 * on Xtensa, no lock needed.
 */
int32_t step_engine_position_steps(void);

/**
 * @brief Current instantaneous velocity in steps/s.
 *
 * Returns 0 when idle.  Read is approximate (snapshot of ISR state).
 */
uint32_t step_engine_velocity_steps(void);

/**
 * @brief True if queue is non-empty or a pulse is in progress.
 */
bool step_engine_busy(void);

/**
 * @brief Set runtime direction inversion ($10 setting).
 *
 * Applied at the output layer via XOR.  Takes effect on the next
 * segment; currently-executing segment direction is NOT changed.
 */
void step_engine_set_dir_invert(bool invert);

/* ---- Self-test hooks (only available when SELF_TEST=y) ---- */

#ifdef CONFIG_STEP_ENGINE_SELF_TEST

/** Maximum number of interval samples captured by self-test hooks. */
#define STEP_SELFTEST_MAX_SAMPLES  256

/**
 * @brief Enable ISR interval sampling (stores alarm-to-alarm deltas).
 *
 * @param buf   Caller-provided buffer for µs deltas.
 * @param max   Buffer capacity.
 * @param count Pointer to written count (updated by ISR).
 */
void step_selftest_start_sampling(uint32_t *buf, int max, volatile int *count);

/**
 * @brief Stop interval sampling and return count.
 */
void step_selftest_stop_sampling(void);

/**
 * @brief Get max ISR execution time in CPU cycles (since init/reset).
 */
uint32_t step_selftest_max_isr_cycles(void);

/**
 * @brief Reset the max ISR cycle counter.
 */
void step_selftest_reset_isr_cycles(void);

/**
 * @brief Run all step-engine self-tests (≥20 cases).
 *
 * Prints PASS/FAIL per case via ESP_LOG and final summary line:
 * "SELF-TEST DONE: <n>/<total> PASS"
 */
void step_engine_self_test_run(void);

#endif /* CONFIG_STEP_ENGINE_SELF_TEST */

#ifdef __cplusplus
}
#endif
