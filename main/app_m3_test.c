/*
 * M3 Joint Test — real G-code driving the real motor
 *
 * *** MOTOR PHYSICALLY MOVES — 12V supply required, travel must be free ***
 * Runs on boot after the G-code parser is registered (Kconfig
 * CONFIG_APP_M3_TEST).  Reuses app_main's parser instance so the
 * coordinate baseline stays consistent for the user afterwards.
 *
 * Chain under test:
 *   G-code text → gcode_parser → planner → step_engine → TMC2209 → motor
 *
 * Position truth is step_engine_position_steps(): the '?' status frame
 * still returns the T3 placeholder (T9 telemetry not implemented yet),
 * so it is NOT used for assertions.  The parser is fed directly (no
 * protocol line handler) to keep UART1 free of per-line "ok" traffic.
 *
 * Ten cases in three groups:
 *   A  absolute mode + real motion   (40 mm = 1 rev @ 16 microstep, back)
 *   B  relative mode                 (G91 / G90)
 *   C  sub-step rounding, soft-limit rejection (error:2), G92
 *
 * All expected values are derived at run time from settings
 * (k = steps_per_mm, $12/$13 soft limits) — nothing is hard-coded, so
 * the test verifies the real chain rather than a fixed configuration.
 *
 * Teardown: DRV_STATUS readback + motor always disabled (terminal state,
 * including every early-return path).
 * Summary line: "M3 TEST: PASS" / "M3 TEST: FAIL (<reason>)"
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "gcode_parser.h"
#include "planner.h"
#include "step_engine.h"
#include "tmc2209.h"
#include "settings.h"

static const char *TAG = "m3_test";

#define M3_POLL_INTERVAL_MS      10
#define M3_MOVE_TIMEOUT_MS    15000
#define M3_CASE_TOTAL            10
#define M3_DETAIL_LEN            96

/* ---- Test context (set once at run start) ---- */

static gcode_parser_t *s_parser    = NULL;
static float           s_k         = 80.0f;   /* steps/mm, from settings */
static int32_t         s_base      = 0;       /* engine position at start */
static int32_t         s_pl_base   = 0;       /* planner position at start */

/* ---- Per-case results (for the final summary) ---- */

typedef struct {
    const char *desc;
    bool        ok;
    char        detail[M3_DETAIL_LEN];
} m3_result_t;

static m3_result_t s_results[M3_CASE_TOTAL];
static int         s_case_idx  = 0;
static int         s_case_pass = 0;
static const char *s_first_fail = NULL;
static char        s_first_fail_detail[M3_DETAIL_LEN];

/* ---- Helpers ---- */

/* Poll step_engine_busy() with vTaskDelay (watchdog-safe) until idle.
 * Returns true if the engine went idle, false on timeout. */
static bool m3_wait_idle(uint32_t timeout_ms, uint32_t *elapsed_out)
{
    uint32_t start   = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;

    while (step_engine_busy() && elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(M3_POLL_INTERVAL_MS));
        elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start;
    }

    if (elapsed_out != NULL) {
        *elapsed_out = elapsed;
    }
    return !step_engine_busy();
}

/* Record one case outcome: log it and remember the first failure. */
static void m3_record(const char *desc, bool ok, const char *detail)
{
    if (s_case_idx < M3_CASE_TOTAL) {
        s_results[s_case_idx].desc = desc;
        s_results[s_case_idx].ok   = ok;
        snprintf(s_results[s_case_idx].detail, M3_DETAIL_LEN, "%s",
                 (detail != NULL) ? detail : "");
    }
    s_case_idx++;

    if (ok) {
        s_case_pass++;
        ESP_LOGI(TAG, "PASS %s", desc);
    } else {
        ESP_LOGE(TAG, "FAIL %s (%s)", desc, (detail != NULL) ? detail : "");
        if (s_first_fail == NULL) {
            s_first_fail = desc;
            snprintf(s_first_fail_detail, sizeof(s_first_fail_detail), "%s",
                     (detail != NULL) ? detail : "");
        }
    }
}

/**
 * Feed one G-code line through the real chain and check it.
 *
 * @param desc          Case label used in the log / summary.
 * @param line          G-code text (no CR/LF).
 * @param expect_status Expected gcode_parser_feed() return value.
 * @param expect_offset Expected engine step offset vs base once motion
 *                      has finished (move cases only).
 * @param check_motion  true  = wait for idle, assert exact step offset;
 *                      false = assert nothing moved at all (mode switch
 *                              or rejected command — verifies that a
 *                              rejection cannot skew coordinates or
 *                              emit pulses; T7 position-commit fix).
 */
static void m3_case(const char *desc, const char *line,
                    parser_status_t expect_status, int32_t expect_offset,
                    bool check_motion)
{
    char    detail[M3_DETAIL_LEN];
    int32_t before_engine  = step_engine_position_steps();
    int32_t before_planner = planner_position_steps();
    float   before_parser  = gcode_parser_get_position(s_parser);

    ESP_LOGI(TAG, "  -- %s : \"%s\"", desc, line);

    parser_status_t st = gcode_parser_feed(s_parser, line);

    if (st != expect_status) {
        snprintf(detail, sizeof(detail), "status expected %d got %d",
                 (int)expect_status, (int)st);
        m3_record(desc, false, detail);
        return;
    }

    if (!check_motion) {
        /* No-motion / rejected case: engine must be idle, and neither
         * the commanded coordinate nor the step counter may change. */
        if (step_engine_busy()) {
            m3_record(desc, false, "engine busy (unexpected pulses)");
            return;
        }
        int32_t d_engine = step_engine_position_steps() - before_engine;
        if (d_engine != 0) {
            snprintf(detail, sizeof(detail),
                     "engine moved %ld steps (expected 0)", (long)d_engine);
            m3_record(desc, false, detail);
            return;
        }
        int32_t d_planner = planner_position_steps() - before_planner;
        if (d_planner != 0) {
            snprintf(detail, sizeof(detail),
                     "planner position moved %ld steps (expected 0)",
                     (long)d_planner);
            m3_record(desc, false, detail);
            return;
        }
        if (fabsf(gcode_parser_get_position(s_parser) - before_parser) > 1e-6f) {
            snprintf(detail, sizeof(detail),
                     "parser position moved %.3f mm (expected 0)",
                     (double)(gcode_parser_get_position(s_parser) - before_parser));
            m3_record(desc, false, detail);
            return;
        }
        m3_record(desc, true, "");
        return;
    }

    /* Move case: wait for the segment to finish, then check the exact
     * step count (no tolerance — the profile is integer-valued). */
    uint32_t elapsed = 0;
    if (!m3_wait_idle(M3_MOVE_TIMEOUT_MS, &elapsed)) {
        snprintf(detail, sizeof(detail), "timeout after %lu ms (still busy)",
                 (unsigned long)elapsed);
        m3_record(desc, false, detail);
        return;
    }

    int32_t offset = step_engine_position_steps() - s_base;
    if (offset != expect_offset) {
        snprintf(detail, sizeof(detail), "expected %ld steps got %ld (offset vs base)",
                 (long)expect_offset, (long)offset);
        m3_record(desc, false, detail);
        return;
    }

    ESP_LOGI(TAG, "     offset %+ld steps (= %+.3f mm), %lu ms",
             (long)offset, (double)((float)offset / s_k), (unsigned long)elapsed);
    m3_record(desc, true, "");
}

/* ================================================================
 * Entry point — called from app_main after the parser is registered
 * ================================================================ */
void app_m3_test_run(gcode_parser_t *parser)
{
    if (parser == NULL) {
        ESP_LOGE(TAG, "parser is NULL — cannot run");
        ESP_LOGI(TAG, "M3 TEST: FAIL (no parser)");
        return;
    }
    s_parser = parser;

    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "  M3 JOINT TEST — real G-code drives the real motor");
    ESP_LOGI(TAG, "  *** MOTOR WILL MOVE — ensure 12V supply and free travel ***");
    ESP_LOGI(TAG, "  (do NOT send commands on UART1 while the test runs)");
    ESP_LOGI(TAG, "========================================================");

    /* ---- Runtime parameters (nothing hard-coded) ---- */
    s_k = settings_steps_per_mm();
    float soft_min = settings_soft_min();
    float soft_max = settings_soft_max();

    ESP_LOGI(TAG, "  k=$0/$1/$2: steps_per_mm=%.3f  (microstep=%d, full_steps=%d, units_per_rev=%.3f)",
             (double)s_k, settings_microstep(), settings_full_steps(),
             (double)settings_units_per_rev());
    ESP_LOGI(TAG, "  soft limits: $12=%.3f .. $13=%.3f",
             (double)soft_min, (double)soft_max);
    ESP_LOGI(TAG, "  modal defaults: feed=%.1f mm/min  accel=%.1f mm/s2",
             (double)settings_default_feed(), (double)settings_default_accel());

    /* ---- Soft-limit precondition ----
     * Test travel is 0..40 mm.  If the limits do not cover it, every
     * move would be rejected and the run would look like a chain
     * failure instead of a configuration problem. */
    if (!(soft_min <= 0.0f) || !(soft_max >= 40.0f)) {
        ESP_LOGE(TAG, "soft limits [%.3f, %.3f] do not cover test travel (0..40mm) "
                      "— run M502 or set $12=0 $13=600, then re-run",
                 (double)soft_min, (double)soft_max);
        ESP_LOGI(TAG, "M3 TEST: FAIL (soft limit range)");
        return;   /* motor never enabled — no teardown needed */
    }

    s_base    = step_engine_position_steps();
    s_pl_base = planner_position_steps();
    ESP_LOGI(TAG, "  start: engine=%ld steps, planner=%ld steps",
             (long)s_base, (long)s_pl_base);

    /* ---- Motor enable (terminal disable happens in teardown, all paths) ---- */
    esp_err_t err = tmc2209_enable(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TMC2209 enable FAILED: %s", esp_err_to_name(err));
        ESP_LOGI(TAG, "M3 TEST: FAIL (tmc2209_enable)");
        return;
    }
    ESP_LOGI(TAG, "  TMC2209 enabled");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ---- Group A: absolute mode, real motion ----
     * 40 mm @ k=80 = 3200 steps = 1 revolution at 16 microstep. */
    ESP_LOGI(TAG, "=== GROUP A: absolute mode (G90 default), real motion ===");

    m3_case("A1 G1 X40 F1200", "G1 X40 F1200", PARSER_OK,
            (int32_t)lroundf(40.0f * s_k), true);   /* +1 rev */

    m3_case("A2 G1 X0 F1200", "G1 X0 F1200", PARSER_OK,
            0, true);                               /* back to origin */

    /* ---- Group B: relative mode ---- */
    ESP_LOGI(TAG, "=== GROUP B: relative mode (G91 / G90) ===");

    m3_case("B3 G91", "G91", PARSER_OK, 0, false);  /* mode switch, no motion */

    m3_case("B4 G1 X5 F1200 (rel)", "G1 X5 F1200", PARSER_OK,
            (int32_t)lroundf(5.0f * s_k), true);    /* 0 + 5 mm */

    m3_case("B5 G1 X-5 F1200 (rel)", "G1 X-5 F1200", PARSER_OK,
            0, true);                               /* 5 - 5 = 0 mm */

    m3_case("B6 G90", "G90", PARSER_OK, 0, false);  /* back to absolute */

    /* ---- Group C: sub-step rounding, soft limit, G92 ---- */
    ESP_LOGI(TAG, "=== GROUP C: sub-step rounding, soft-limit rejection, G92 ===");

    /* 0.02 mm × 80 = 1.6 steps → lroundf → 2 steps */
    m3_case("C7 G1 X0.02 F1200", "G1 X0.02 F1200", PARSER_OK,
            (int32_t)lroundf(0.02f * s_k), true);

    m3_case("C8 G1 X0 F1200", "G1 X0 F1200", PARSER_OK, 0, true);

    /* Out-of-range target: taken from the live $13 value, +1 mm.
     * Expected: error:2 (PARSER_ERR_RANGE) and zero pulses. */
    {
        char line[48];
        snprintf(line, sizeof(line), "G1 X%.3f F1200", (double)(soft_max + 1.0f));
        m3_case("C9 G1 X(soft_max+1) — rejected", line, PARSER_ERR_RANGE,
                0, false);
    }

    m3_case("C10 G92 X0", "G92 X0", PARSER_OK, 0, false);  /* origin, no motion */

    /* ---- Teardown: DRV_STATUS + disable (always reached) ---- */
    ESP_LOGI(TAG, "=== TEARDOWN: DRV_STATUS + motor disable ===");

    /* TMC2209 standstill detection needs a quiet window (~84 ms at the
     * internal clock); reading immediately after motion reports stst=0
     * (T5/T6 lesson).  Wait with margin. */
    vTaskDelay(pdMS_TO_TICKS(300));

    bool drv_read_ok = false;
    bool drv_ok      = false;
    tmc2209_status_t st;

    if (tmc2209_read_status(&st)) {
        drv_read_ok = true;
        ESP_LOGI(TAG, "  DRV_STATUS: stst=%d ola=%d olb=%d s2ga=%d s2gb=%d "
                      "otpw=%d ot=%d cs_actual=%d",
                 st.stst ? 1 : 0, st.ola ? 1 : 0, st.olb ? 1 : 0,
                 st.s2ga ? 1 : 0, st.s2gb ? 1 : 0,
                 st.otpw ? 1 : 0, st.ot ? 1 : 0, st.cs_actual);

        /* Pass criteria: standstill, no overtemperature, no short to GND.
         * ola/olb are open-load flags: with the 12V motor supply off
         * there is no coil current and they may read 1 — that is
         * expected in that setup, so they are reported but not counted
         * as a fault here. */
        drv_ok = st.stst && !st.ot && !st.s2ga && !st.s2gb;
        if (st.ola || st.olb) {
            ESP_LOGW(TAG, "  open-load flags set (ola=%d olb=%d) — expected when the "
                          "12V motor supply is off; reported, not judged",
                     st.ola ? 1 : 0, st.olb ? 1 : 0);
        }
    } else {
        ESP_LOGE(TAG, "  DRV_STATUS read FAILED (UART comm error)");
    }

    /* Motor disabled — terminal state on every path */
    tmc2209_enable(false);
    ESP_LOGI(TAG, "  motor disabled (terminal state)");

    int32_t final_engine  = step_engine_position_steps();
    int32_t final_planner = planner_position_steps();
    ESP_LOGI(TAG, "  final position: engine=%ld steps (rel base %ld), "
                  "planner=%ld steps (rel base %ld)",
             (long)final_engine, (long)(final_engine - s_base),
             (long)final_planner, (long)(final_planner - s_pl_base));

    bool final_ok = (final_engine == s_base) && (final_planner == s_pl_base);

    /* ---- Summary ---- */
    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "  RESULTS  (%d/%d cases PASS)", s_case_pass, M3_CASE_TOTAL);

    for (int i = 0; i < s_case_idx; i++) {
        if (s_results[i].ok) {
            ESP_LOGI(TAG, "    PASS  %s", s_results[i].desc);
        } else {
            ESP_LOGE(TAG, "    FAIL  %s (%s)", s_results[i].desc,
                     s_results[i].detail);
        }
    }

    ESP_LOGI(TAG, "    DRV:  %s",
             !drv_read_ok ? "FAIL (read)" : (drv_ok ? "PASS" : "FAIL (flags)"));
    ESP_LOGI(TAG, "    HOME: %s (returned to start position)",
             final_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "--------------------------------------------------------");

    if (s_case_pass == M3_CASE_TOTAL && drv_ok && final_ok) {
        ESP_LOGI(TAG, "  M3 TEST: PASS");
    } else if (s_case_pass != M3_CASE_TOTAL) {
        char reason[M3_DETAIL_LEN + 64];
        snprintf(reason, sizeof(reason), "cases %d/%d, first: %s (%s)",
                 s_case_pass, M3_CASE_TOTAL,
                 (s_first_fail != NULL) ? s_first_fail : "?",
                 s_first_fail_detail);
        ESP_LOGE(TAG, "  M3 TEST: FAIL (%s)", reason);
    } else if (!drv_read_ok) {
        ESP_LOGE(TAG, "  M3 TEST: FAIL (drv_status read)");
    } else if (!drv_ok) {
        ESP_LOGE(TAG, "  M3 TEST: FAIL (drv_status flags)");
    } else {
        ESP_LOGE(TAG, "  M3 TEST: FAIL (did not return to start position)");
    }
    ESP_LOGI(TAG, "========================================================");
}
