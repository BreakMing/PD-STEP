/*
 * System Control (T10) — state machine + immediate stop
 *
 * State machine (no task — PLAN §3.2 has no sysctl_task, so the state is
 * derived on demand from the step engine and cached):
 *
 *   BOOT ──sysctl_init()──▶ IDLE ⇄ RUN      (RUN ⇔ step_engine_busy())
 *   任意状态 ──sysctl_stop()──▶ IDLE
 *
 * '!' end-to-end path (PLAN §4.4 / T10):
 *   protocol (real-time char, mid-line allowed)
 *     → stop_cb  (registered by app_main)
 *       → sysctl_stop(): ① step_engine_stop()             pulses stop, queue
 *                                                        cleared, counter frozen
 *                        ② planner_rebase_to_executed()   commanded position
 *                                                        drops back to the
 *                                                        executed position
 *                        ③ state = IDLE
 *
 * !! NO DECISIONS (D6) !!  State transitions and stopping only — no
 * loss-of-step detection, no fault handling, no limit interception.
 * !! D11 !!  Immediate stop only — no pause / resume / continue.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYSCTL_BOOT = 0,   /* before sysctl_init() */
    SYSCTL_IDLE,       /* queue empty, no pulse in flight */
    SYSCTL_RUN,        /* motion in progress */
} sysctl_state_t;

/**
 * @brief Initialize the state machine: BOOT → IDLE.
 *
 * Idempotent.  Must be called after protocol_start() / step_engine_init()
 * / planner_init() — it samples the engine's busy state.
 *
 * @return ESP_OK.
 */
esp_err_t sysctl_init(void);

/**
 * @brief Immediate stop — the end-to-end handling of '!'.
 *
 * Order is fixed (the order IS the correctness):
 *   1. step_engine_stop()             stop pulses, clear queue, freeze counter
 *   2. planner_rebase_to_executed()   drop the commanded position back to the
 *                                     actually executed position
 *   3. state = IDLE
 *
 * Called from comm_task context (protocol's stop_cb).
 */
void sysctl_stop(void);

/**
 * @brief Current state.
 *
 * BOOT stays BOOT until sysctl_init(); afterwards the state is re-derived
 * from step_engine_busy() and cached.
 */
sysctl_state_t sysctl_get_state(void);

/**
 * @brief State as a string — always "BOOT" / "IDLE" / "RUN" (for the frame).
 */
const char *sysctl_state_str(void);

/* ================================================================
 * Self-test (only when CONFIG_SYSCTL_SELF_TEST=y)
 * ================================================================ */
#ifdef CONFIG_SYSCTL_SELF_TEST
/**
 * @brief Run all 14 sysctl self-tests.
 *
 * Final log line: "SELF-TEST DONE: <n>/<total> PASS".
 */
int sysctl_self_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
