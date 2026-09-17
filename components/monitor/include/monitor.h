/*
 * Monitor / Telemetry (T9) — status-frame reporter
 *
 * Scope: format the data that already exists (commanded position from the
 * planner, measured position/velocity from the AS5600, driver flags from
 * the TMC2209) into the PLAN §4.5 status frame and put it on the wire —
 * periodically while the axis is moving ($9 report_hz) and on demand for
 * the '?' real-time query.
 *
 * !! REPORT ONLY — ZERO DECISIONS (D6) !!
 *   No loss-of-step detection, no fault shutdown, no limit interception.
 *   Whatever the sources report is forwarded as-is.
 *
 * Frame (PLAN §4.5, TX ends with \r\n):
 *   <STATE|C:<cmd mm>|E:<enc mm>|V:<mm/min>|T:<flag>>
 *   STATE ∈ IDLE / RUN (upper case), C/E/V with 3 decimals.
 *
 * C is the EXECUTED commanded position — the pulse ISR's step counter
 * mapped into the G92 coordinate frame — so that C−E equals the live
 * loss-of-step amount at every instant during a move, not just at the
 * end (PLAN T9: "commanded position, atomic snapshot of the ISR-updated
 * value").  The frame source is step_engine_position_steps().
 *
 * Task: monitor_task — priority 2 (below comm 5 / planner 4 / as5600 3),
 * CPU1, 10 ms tick driven by vTaskDelayUntil (never busy-waits).
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "tmc2209.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Pure functions (hardware independent — self-test targets)
 * ================================================================ */

/**
 * @brief Map a decoded DRV_STATUS to a single flag string.
 *
 * Highest-priority (single) flag wins, in this order:
 *   ERR (st == NULL, UART read failed) > OT > S2GA > S2GB > OLA > OLB
 *   > OTPW > OK
 *
 * stst / cs_actual are not faults and never appear in the frame.
 *
 * @param st   Decoded DRV_STATUS; NULL means the read failed.
 * @param buf  Output buffer.
 * @param len  Buffer length (>= 8 bytes).
 */
void monitor_tmc_flag(const tmc2209_status_t *st, char *buf, size_t len);

/**
 * @brief Format one status frame (no trailing \r\n).
 *
 * Format: <STATE|C:x.xxx|E:x.xxx|V:x.xxx|T:FLAG>
 *
 * @param buf        Output buffer.
 * @param len        Buffer length (>= 64 bytes).
 * @param state      "IDLE" or "RUN".
 * @param c_mm       Commanded position (mm).
 * @param e_mm       Encoder position (mm).
 * @param v_mm_min   Velocity (mm/min, signed).
 * @param flag       Flag string (output of monitor_tmc_flag()).
 * @return Bytes written (excluding the terminating NUL).
 */
int monitor_format_frame(char *buf, size_t len, const char *state,
                         float c_mm, float e_mm, float v_mm_min,
                         const char *flag);

/* ================================================================
 * Runtime
 * ================================================================ */

/**
 * @brief Initialize: register the '?' status callback + create monitor_task.
 *
 * Must be called after protocol_start(), step_engine_init(), planner_init(),
 * as5600_init() and tmc2209_init() — the component reads their state.
 * Idempotent.
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the task cannot be created.
 */
esp_err_t monitor_init(void);

/**
 * @brief Align the executed-position frame with a new coordinate origin.
 *
 * MUST be called from the G92 path together with planner_set_origin() and
 * as5600_set_origin() — that is what keeps the commanded position (C) and
 * the encoder position (E) shifting together (PLAN D10).  The power-up
 * alignment (0 mm) is done inside monitor_init().
 *
 * @param new_pos_mm  Coordinate the current position is being declared as.
 */
void monitor_note_origin(float new_pos_mm);

/**
 * @brief Build one frame from the live sources and send it.
 *
 * Used both by the '?' query path (comm_task context) and by the periodic
 * report in monitor_task.  Non-blocking apart from one ~1 ms TMC UART
 * transaction; the frame buffer is a stack local, so the two callers
 * share no state.
 */
void monitor_send_status(void);

/**
 * @brief Number of periodic frames sent so far.
 *
 * Incremented by monitor_task only (on-demand '?' frames do not count).
 */
uint32_t monitor_frames_sent(void);

/* ================================================================
 * Self-test (only when CONFIG_MONITOR_SELF_TEST=y)
 * ================================================================ */
#ifdef CONFIG_MONITOR_SELF_TEST

/**
 * @brief Test hook — build a frame through the exact same path that
 *        monitor_send_status() uses, without writing to UART1.
 *
 * @param buf  Output buffer (>= 64 bytes).
 * @param len  Buffer length.
 * @return Bytes written (excluding the terminating NUL).
 */
int monitor_selftest_build_frame(char *buf, size_t len);

/**
 * @brief Run all 16 monitor self-tests.
 *
 * Final log line: "SELF-TEST DONE: <n>/<total> PASS".
 */
int monitor_self_test_run(void);

#endif /* CONFIG_MONITOR_SELF_TEST */

#ifdef __cplusplus
}
#endif
