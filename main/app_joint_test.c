/*
 * T5 + T6 Joint Motion Test — real motor, runs on boot
 *
 * WARNING: motor physically moves; 12V supply required.
 * Runs after all subsystems are up (called from app_main).
 *
 * Three phases:
 *   A — Forward trapezoid, 2 revolutions (dir=true)
 *   B — Reverse trapezoid, back to origin (dir=false)
 *   C — Slow long move, user presses '!' via UART1 for immediate stop
 *
 * Teardown: DRV_STATUS readback, motor disabled (always).
 * Final summary: "JOINT TEST: PASS" / "FAIL (<reason>)"
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "step_engine.h"
#include "tmc2209.h"
#include "settings.h"

static const char *TAG = "joint_test";

/* ---- Safety limit (50 kHz ISR performance ceiling) ---- */
#define MAX_SAFE_V_CRUISE_STEPS_S   40000

/* ---- Timeout per phase (ms) ---- */
#define TIMEOUT_PHASE_AB_MS   5000
#define TIMEOUT_PHASE_C_MS   25000
#define POLL_INTERVAL_MS        10

/* ---- Phase result tracking ---- */
typedef enum {
    PHASE_PASS = 0,
    PHASE_FAIL_TIMEOUT,
    PHASE_FAIL_ENQUEUE,
    PHASE_FAIL_POSITION,
    PHASE_FAIL_VELOCITY,
    PHASE_FAIL_SAFETY,
    PHASE_FAIL_DRV,
} phase_result_t;

static const char *phase_result_str(phase_result_t r)
{
    switch (r) {
    case PHASE_PASS:          return "PASS";
    case PHASE_FAIL_TIMEOUT:  return "FAIL (timeout)";
    case PHASE_FAIL_ENQUEUE:  return "FAIL (enqueue)";
    case PHASE_FAIL_POSITION: return "FAIL (position)";
    case PHASE_FAIL_VELOCITY: return "FAIL (velocity)";
    case PHASE_FAIL_SAFETY:   return "FAIL (safety)";
    case PHASE_FAIL_DRV:      return "FAIL (drv_status)";
    default:                  return "FAIL (unknown)";
    }
}

/* ---- Helpers ---- */

/* Poll step_engine_busy() with vTaskDelay until idle or timeout.
 * Returns true if completed (not busy), false on timeout. */
static bool wait_until_idle(uint32_t timeout_ms, uint32_t *elapsed_ms_out)
{
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;

    while (step_engine_busy() && elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start;
    }

    if (elapsed_ms_out != NULL) {
        *elapsed_ms_out = elapsed;
    }
    return !step_engine_busy();
}

/* Verify accel + cruise + decel == steps (self-check, T6 P0#3 lesson) */
static bool seg_check_sum(const step_seg_t *seg)
{
    uint64_t sum = (uint64_t)seg->accel_steps +
                   (uint64_t)seg->cruise_steps +
                   (uint64_t)seg->decel_steps;
    return sum == seg->steps;
}

/* ================================================================
 * Phase A — Forward trapezoid (2 revolutions)
 * ================================================================ */
static phase_result_t run_phase_a(uint32_t spr, uint32_t *duration_ms_out)
{
    ESP_LOGI(TAG, "=== PHASE A: Forward 2-rev trapezoid ===");

    /* Parameters:
     *   v_cruise = spr (1 rev/s)
     *   accel    = spr (1 rev/s²)
     *   steps    = 2 * spr
     *   accel_steps = v²/(2a) = spr²/(2*spr) = spr/2
     *   decel_steps = spr/2 (symmetric)
     *   cruise_steps = steps - accel - decel = 2*spr - spr = spr */
    uint32_t v_cruise = spr;
    float    accel = (float)spr;
    uint32_t total_steps = 2 * spr;
    uint32_t accel_steps = spr / 2;
    uint32_t decel_steps = spr / 2;
    uint32_t cruise_steps = total_steps - accel_steps - decel_steps;

    step_seg_t seg = {
        .steps        = total_steps,
        .dir          = true,
        .accel_steps  = accel_steps,
        .cruise_steps = cruise_steps,
        .decel_steps  = decel_steps,
        .accel        = accel,
        .v_cruise     = (float)v_cruise,
    };

    if (!seg_check_sum(&seg)) {
        ESP_LOGE(TAG, "  seg sum mismatch: %u + %u + %u != %u",
                 accel_steps, cruise_steps, decel_steps, total_steps);
        return PHASE_FAIL_ENQUEUE;
    }

    ESP_LOGI(TAG, "  steps=%u  accel=%u  cruise=%u  decel=%u  v=%u  a=%.0f",
             total_steps, accel_steps, cruise_steps, decel_steps,
             v_cruise, (double)accel);

    if (!step_engine_enqueue(&seg)) {
        ESP_LOGE(TAG, "  enqueue FAILED (queue full)");
        return PHASE_FAIL_ENQUEUE;
    }
    ESP_LOGI(TAG, "  enqueued, running...");

    /* Profile is 3 s total (accel 1 s + cruise 1 s + decel 1 s, since
     * v = a = spr and cruise_steps = spr).  Mid-cruise ≈ 1.5 s —
     * sample instantaneous velocity there and check ±5%. */
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;
    bool     vel_sampled = false;
    uint32_t vel_sample = 0;
    bool     vel_ok = false;

    while (step_engine_busy() && elapsed < TIMEOUT_PHASE_AB_MS) {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start;

        if (!vel_sampled && elapsed >= 1500) {
            vel_sample = step_engine_velocity_steps();
            vel_ok = (vel_sample >= v_cruise * 95 / 100) &&
                     (vel_sample <= v_cruise * 105 / 100);
            vel_sampled = true;
            ESP_LOGI(TAG, "  mid-cruise velocity: %u (expected %u, ±5%%: %s)",
                     vel_sample, v_cruise, vel_ok ? "OK" : "OUT OF RANGE");
        }
    }

    if (duration_ms_out != NULL) *duration_ms_out = elapsed;

    if (step_engine_busy()) {
        ESP_LOGE(TAG, "  TIMEOUT after %u ms (busy still set)", elapsed);
        return PHASE_FAIL_TIMEOUT;
    }

    int32_t pos = step_engine_position_steps();
    ESP_LOGI(TAG, "  done: position=%ld  elapsed=%u ms", (long)pos, elapsed);

    if (pos != (int32_t)total_steps) {
        ESP_LOGE(TAG, "  POSITION MISMATCH: expected %u, got %ld",
                 total_steps, (long)pos);
        return PHASE_FAIL_POSITION;
    }

    if (!vel_sampled || !vel_ok) {
        return PHASE_FAIL_VELOCITY;
    }

    ESP_LOGI(TAG, "  Phase A PASS");
    return PHASE_PASS;
}

/* ================================================================
 * Phase B — Reverse back to origin (same profile, dir=false)
 * ================================================================ */
static phase_result_t run_phase_b(uint32_t spr, uint32_t *duration_ms_out)
{
    ESP_LOGI(TAG, "=== PHASE B: Reverse back to origin ===");

    /* Same parameters as Phase A, just direction reversed */
    uint32_t v_cruise = spr;
    float    accel = (float)spr;
    uint32_t total_steps = 2 * spr;
    uint32_t accel_steps = spr / 2;
    uint32_t decel_steps = spr / 2;
    uint32_t cruise_steps = total_steps - accel_steps - decel_steps;

    /* Settling time between A and B: >= 100 ms */
    vTaskDelay(pdMS_TO_TICKS(100));

    step_seg_t seg = {
        .steps        = total_steps,
        .dir          = false,
        .accel_steps  = accel_steps,
        .cruise_steps = cruise_steps,
        .decel_steps  = decel_steps,
        .accel        = accel,
        .v_cruise     = (float)v_cruise,
    };

    if (!seg_check_sum(&seg)) {
        ESP_LOGE(TAG, "  seg sum mismatch");
        return PHASE_FAIL_ENQUEUE;
    }

    ESP_LOGI(TAG, "  steps=%u  dir=false  v=%u", total_steps, v_cruise);

    if (!step_engine_enqueue(&seg)) {
        ESP_LOGE(TAG, "  enqueue FAILED");
        return PHASE_FAIL_ENQUEUE;
    }

    uint32_t elapsed = 0;
    bool completed = wait_until_idle(TIMEOUT_PHASE_AB_MS, &elapsed);
    if (duration_ms_out != NULL) *duration_ms_out = elapsed;

    if (!completed) {
        ESP_LOGE(TAG, "  TIMEOUT after %u ms", elapsed);
        return PHASE_FAIL_TIMEOUT;
    }

    int32_t pos = step_engine_position_steps();
    ESP_LOGI(TAG, "  done: position=%ld  elapsed=%u ms", (long)pos, elapsed);

    if (pos != 0) {
        ESP_LOGE(TAG, "  POSITION MISMATCH: expected 0, got %ld", (long)pos);
        return PHASE_FAIL_POSITION;
    }

    ESP_LOGI(TAG, "  Phase B PASS");
    return PHASE_PASS;
}

/* ================================================================
 * Phase C — Slow long move with '!' immediate stop
 * ================================================================ */
static phase_result_t run_phase_c(uint32_t spr, uint32_t *duration_ms_out,
                                  int32_t *stop_pos_out, bool *stopped_early_out)
{
    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "  PHASE C: send '!' NOW via UART1 to test immediate stop");
    ESP_LOGI(TAG, "  (raw character, no Enter needed — ~19s window)");
    ESP_LOGI(TAG, "========================================================");

    /* Parameters:
     *   v_cruise = spr/4 (0.25 rev/s)
     *   accel    = spr/8 (0.125 rev/s²)
     *   accel_steps = v²/(2a) = (spr/4)² / (2*spr/8) = spr²/16 / (spr/4) = spr/4 = 800
     *   decel_steps = 800 (symmetric)
     *   cruise_time = 15 s → cruise_steps = 15 * v_cruise = 15 * spr/4
     *   total = accel + cruise + decel */
    uint32_t v_cruise = spr / 4;
    float    accel = (float)spr / 8.0f;
    uint32_t accel_steps  = spr / 4;
    uint32_t decel_steps  = spr / 4;
    uint32_t cruise_steps = 15 * (spr / 4);   /* 15 seconds at v_cruise */
    uint32_t total_steps  = accel_steps + cruise_steps + decel_steps;

    step_seg_t seg = {
        .steps        = total_steps,
        .dir          = true,
        .accel_steps  = accel_steps,
        .cruise_steps = cruise_steps,
        .decel_steps  = decel_steps,
        .accel        = accel,
        .v_cruise     = (float)v_cruise,
    };

    if (!seg_check_sum(&seg)) {
        ESP_LOGE(TAG, "  seg sum mismatch: %u + %u + %u != %u",
                 accel_steps, cruise_steps, decel_steps, total_steps);
        return PHASE_FAIL_ENQUEUE;
    }

    ESP_LOGI(TAG, "  steps=%u  accel=%u  cruise=%u  decel=%u  v=%u  a=%.0f",
             total_steps, accel_steps, cruise_steps, decel_steps,
             v_cruise, (double)accel);

    if (!step_engine_enqueue(&seg)) {
        ESP_LOGE(TAG, "  enqueue FAILED");
        return PHASE_FAIL_ENQUEUE;
    }

    /* Poll every 1s, log position + velocity + busy.
     * Stop when engine is idle OR timeout reached. */
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;
    uint32_t last_log_sec = 0;

    while (step_engine_busy() && elapsed < TIMEOUT_PHASE_C_MS) {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start;

        uint32_t sec = elapsed / 1000;
        if (sec > last_log_sec) {
            last_log_sec = sec;
            int32_t  pos = step_engine_position_steps();
            uint32_t vel = step_engine_velocity_steps();
            bool     busy = step_engine_busy();
            ESP_LOGI(TAG, "  t=%us  pos=%ld  vel=%u  busy=%d",
                     sec, (long)pos, vel, busy ? 1 : 0);
        }
    }

    if (duration_ms_out != NULL) *duration_ms_out = elapsed;

    /* Loop exits when idle OR timeout.  Distinguish the two: if the
     * engine is STILL busy we hit the timeout (stuck engine — the
     * failure mode this phase must catch, never report it as a pass). */
    if (step_engine_busy()) {
        ESP_LOGE(TAG, "  TIMEOUT after %u ms (still busy — engine stuck?)",
                 elapsed);
        return PHASE_FAIL_TIMEOUT;
    }

    int32_t final_pos = step_engine_position_steps();
    bool stopped_early = (final_pos < (int32_t)total_steps);

    if (stop_pos_out != NULL)     *stop_pos_out = final_pos;
    if (stopped_early_out != NULL) *stopped_early_out = stopped_early;

    ESP_LOGI(TAG, "  done: position=%ld  elapsed=%u ms  early_stop=%s",
             (long)final_pos, elapsed, stopped_early ? "yes" : "no");

    if (!step_engine_busy() && elapsed >= TIMEOUT_PHASE_C_MS) {
        /* Engine was still busy at timeout — that's a real timeout */
        ESP_LOGE(TAG, "  TIMEOUT after %u ms (still busy)", elapsed);
        return PHASE_FAIL_TIMEOUT;
    }

    if (stopped_early) {
        ESP_LOGI(TAG, "  Phase C PASS (stopped early by '!')");
    } else {
        ESP_LOGW(TAG, "  Phase C: no '!' received (segment completed) — not a failure");
    }
    return PHASE_PASS;
}

/* ================================================================
 * Teardown — DRV_STATUS + disable
 * ================================================================ */
static phase_result_t run_teardown(void)
{
    ESP_LOGI(TAG, "=== TEARDOWN: DRV_STATUS + motor disable ===");

    /* TMC2209 standstill detection needs ~2^20 internal clock cycles
     * (≈84 ms @ 12.5 MHz) without step pulses before stst asserts.
     * Reading immediately after motion ends returns stst=0 (observed
     * on-board: read at +9 ms → stst=0, fault flags all clear).
     * Wait with margin. */
    vTaskDelay(pdMS_TO_TICKS(300));

    tmc2209_status_t st;
    if (!tmc2209_read_status(&st)) {
        ESP_LOGE(TAG, "  DRV_STATUS read FAILED (UART comm error)");
        /* Still disable the motor */
        tmc2209_enable(false);
        return PHASE_FAIL_DRV;
    }

    ESP_LOGI(TAG, "  DRV_STATUS: stst=%d ola=%d olb=%d s2ga=%d s2gb=%d "
                  "otpw=%d ot=%d cs_actual=%d",
             st.stst ? 1 : 0, st.ola ? 1 : 0, st.olb ? 1 : 0,
             st.s2ga ? 1 : 0, st.s2gb ? 1 : 0,
             st.otpw ? 1 : 0, st.ot ? 1 : 0, st.cs_actual);

    /* Expected at standstill with hold current: stst=1, no OT, no
     * shorts (s2ga/s2gb), no open load (ola/olb = 0 — coils are
     * energized by hold current; T5 bring-up confirmed 0/0). */
    bool drv_ok = !st.ot && !st.s2ga && !st.s2gb && st.stst &&
                  !st.ola && !st.olb;

    /* Always disable motor — final state = disabled */
    tmc2209_enable(false);
    ESP_LOGI(TAG, "  motor disabled");

    if (!drv_ok) {
        ESP_LOGE(TAG, "  DRV_STATUS shows fault (ot=%d s2ga=%d s2gb=%d stst=%d)",
                 st.ot ? 1 : 0, st.s2ga ? 1 : 0, st.s2gb ? 1 : 0, st.stst ? 1 : 0);
        return PHASE_FAIL_DRV;
    }

    ESP_LOGI(TAG, "  Teardown PASS");
    return PHASE_PASS;
}

/* ================================================================
 * Entry point
 * ================================================================ */
void app_joint_test_run(void)
{
    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "  T5 + T6 JOINT MOTION TEST");
    ESP_LOGI(TAG, "  Real motor — ensure 12V supply is connected");
    ESP_LOGI(TAG, "========================================================");

    /* ---- Derive parameters from settings ---- */
    int microstep = settings_microstep();
    int full_steps = settings_full_steps();
    uint32_t spr = (uint32_t)full_steps * (uint32_t)microstep;

    ESP_LOGI(TAG, "  microstep=%d  full_steps/rev=%d  spr=%lu",
             microstep, full_steps, (unsigned long)spr);

    /* ---- Safety check: v_cruise must not exceed 40000 steps/s ---- */
    uint32_t max_v = spr;  /* Phase A uses spr as v_cruise (highest) */
    if (max_v > MAX_SAFE_V_CRUISE_STEPS_S) {
        ESP_LOGE(TAG, "  SAFETY: v_cruise=%lu exceeds limit %d — ABORT",
                 (unsigned long)max_v, MAX_SAFE_V_CRUISE_STEPS_S);
        ESP_LOGI(TAG, "JOINT TEST: FAIL (safety: v_cruise > 40000 steps/s)");
        return;
    }

    /* ---- Motor enable ---- */
    ESP_LOGI(TAG, "  Enabling TMC2209...");
    esp_err_t err = tmc2209_enable(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  TMC2209 enable FAILED: %s", esp_err_to_name(err));
        ESP_LOGI(TAG, "JOINT TEST: FAIL (tmc2209_enable)");
        return;
    }
    /* Driver wake-up time: tmc2209_enable already waits ≥1ms internally.
     * Add extra 10ms margin before first step pulse. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ---- Run phases ---- */
    phase_result_t res_a = PHASE_FAIL_TIMEOUT;
    phase_result_t res_b = PHASE_FAIL_TIMEOUT;
    phase_result_t res_c = PHASE_FAIL_TIMEOUT;
    phase_result_t res_drv = PHASE_FAIL_DRV;

    uint32_t dur_a = 0, dur_b = 0, dur_c = 0;
    int32_t  stop_pos_c = 0;
    bool     stopped_early_c = false;

    res_a = run_phase_a(spr, &dur_a);

    if (res_a == PHASE_PASS) {
        res_b = run_phase_b(spr, &dur_b);
    } else {
        ESP_LOGW(TAG, "  Skipping Phase B (Phase A failed)");
    }

    /* Phase C runs regardless of A/B — still tests the stop chain */
    res_c = run_phase_c(spr, &dur_c, &stop_pos_c, &stopped_early_c);

    /* ---- Teardown (always — even on failure paths) ---- */
    res_drv = run_teardown();

    /* ---- Summary ---- */
    bool overall_pass = (res_a == PHASE_PASS) &&
                        (res_b == PHASE_PASS) &&
                        (res_c == PHASE_PASS) &&
                        (res_drv == PHASE_PASS);

    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "  RESULTS");
    ESP_LOGI(TAG, "    Phase A: %s  (dur=%u ms)", phase_result_str(res_a), dur_a);
    ESP_LOGI(TAG, "    Phase B: %s  (dur=%u ms)", phase_result_str(res_b), dur_b);
    ESP_LOGI(TAG, "    Phase C: %s  (dur=%u ms, stop_pos=%ld, early=%d)",
             phase_result_str(res_c), dur_c, (long)stop_pos_c, stopped_early_c ? 1 : 0);
    ESP_LOGI(TAG, "    DRV:     %s", phase_result_str(res_drv));
    ESP_LOGI(TAG, "--------------------------------------------------------");

    if (overall_pass) {
        ESP_LOGI(TAG, "  JOINT TEST: PASS");
    } else {
        ESP_LOGE(TAG, "  JOINT TEST: FAIL");
    }
    ESP_LOGI(TAG, "========================================================");
}
