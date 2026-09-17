/*
 * Monitor / Telemetry — on-target self-test (16 cases)
 *
 * Built into the firmware when CONFIG_MONITOR_SELF_TEST=y.
 * Runs from app_main AFTER monitor_init() (i.e. after tmc2209_init /
 * planner_init / step_engine_init / protocol_start), because the live
 * cases read those components' state.
 *
 * The TMC2209 EN pin is still HIGH here (nothing enabled it), so the
 * motor does not turn — case 16 only makes the pulse queue busy.
 *
 * Final log line: "SELF-TEST DONE: <n>/16 PASS" (tag: monitor_selftest)
 */

#include "monitor.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "planner.h"
#include "as5600.h"
#include "step_engine.h"
#include "tmc2209.h"
#include "settings.h"

static const char *TAG = "monitor_selftest";

/* ---- Counters ---- */
static int s_pass  = 0;
static int s_fail  = 0;
static int s_total = 0;

/* ---- Assertion macros (mirror the other self-tests) ---- */

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_EQ(expected, actual) do { \
    if ((int64_t)(expected) != (int64_t)(actual)) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %ld, got %ld (line %d)", \
                 __func__, (long)(expected), (long)(actual), __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_STR_EQ(expected, actual) do { \
    if (strcmp((expected), (actual)) != 0) { \
        ESP_LOGE(TAG, "  FAIL %s: expected \"%s\", got \"%s\" (line %d)", \
                 __func__, (expected), (actual), __LINE__); \
        return false; \
    } \
} while (0)

/* ================================================================
 * Helpers
 * ================================================================ */

/* Build a DRV_STATUS decode result with the named faults set. */
static tmc2209_status_t mk_status(bool ot, bool s2ga, bool s2gb,
                                  bool ola, bool olb, bool otpw)
{
    tmc2209_status_t st;
    memset(&st, 0, sizeof(st));
    st.ot   = ot;
    st.s2ga = s2ga;
    st.s2gb = s2gb;
    st.ola  = ola;
    st.olb  = olb;
    st.otpw = otpw;
    return st;
}

/* Split a "<STATE|C:x|E:x|V:x|T:flag>" frame into its fields. */
static bool parse_frame(const char *frame, char *state, size_t state_len,
                        float *c, float *e, float *v,
                        char *flag, size_t flag_len)
{
    char st[8] = { 0 };
    char tg[8] = { 0 };
    float fc = 0.0f, fe = 0.0f, fv = 0.0f;
    size_t flen = strlen(frame);

    int n = sscanf(frame, "<%7[^|]|C:%f|E:%f|V:%f|T:%7[^>]>",
                   st, &fc, &fe, &fv, tg);
    if (n != 5 || flen < 2 || frame[0] != '<' || frame[flen - 1] != '>') {
        ESP_LOGE(TAG, "  frame parse failed (%d fields): \"%s\"", n, frame);
        return false;
    }

    snprintf(state, state_len, "%s", st);
    snprintf(flag, flag_len, "%s", tg);
    *c = fc;
    *e = fe;
    *v = fv;
    return true;
}

/* Value inside [min(a,b)-tol, max(a,b)+tol] — tolerates a one-count
 * encoder step between the frame build and the comparison read. */
static bool within_readings(float val, float a, float b, float tol)
{
    float lo = fminf(a, b) - tol;
    float hi = fmaxf(a, b) + tol;
    return (val >= lo) && (val <= hi);
}

/* ================================================================
 * A. Pure functions — flag mapping (1-9) + frame format (10)
 * ================================================================ */

/* 1. Read failure → ERR */
static bool test_flag_null(void)
{
    char buf[8];
    monitor_tmc_flag(NULL, buf, sizeof(buf));
    ASSERT_STR_EQ("ERR", buf);
    return true;
}

/* 2. No fault → OK */
static bool test_flag_ok(void)
{
    tmc2209_status_t st = mk_status(false, false, false, false, false, false);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("OK", buf);
    return true;
}

/* 3. Overtemperature shutdown */
static bool test_flag_ot(void)
{
    tmc2209_status_t st = mk_status(true, false, false, false, false, false);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("OT", buf);
    return true;
}

/* 4. Short to GND, phase A */
static bool test_flag_s2ga(void)
{
    tmc2209_status_t st = mk_status(false, true, false, false, false, false);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("S2GA", buf);
    return true;
}

/* 5. Short to GND, phase B */
static bool test_flag_s2gb(void)
{
    tmc2209_status_t st = mk_status(false, false, true, false, false, false);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("S2GB", buf);
    return true;
}

/* 6. Open load, phase A */
static bool test_flag_ola(void)
{
    tmc2209_status_t st = mk_status(false, false, false, true, false, false);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("OLA", buf);
    return true;
}

/* 7. Open load, phase B */
static bool test_flag_olb(void)
{
    tmc2209_status_t st = mk_status(false, false, false, false, true, false);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("OLB", buf);
    return true;
}

/* 8. Overtemperature pre-warning */
static bool test_flag_otpw(void)
{
    tmc2209_status_t st = mk_status(false, false, false, false, false, true);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("OTPW", buf);
    return true;
}

/* 9. Several faults at once → most severe only */
static bool test_flag_priority(void)
{
    tmc2209_status_t st = mk_status(true, true, false, false, false, true);
    char buf[8];
    monitor_tmc_flag(&st, buf, sizeof(buf));
    ASSERT_STR_EQ("OT", buf);
    return true;
}

/* 10. Frame layout, byte for byte */
static bool test_frame_format(void)
{
    char buf[64];
    int n = monitor_format_frame(buf, sizeof(buf), "IDLE",
                                 0.0f, 0.0f, 0.0f, "OK");
    ASSERT_STR_EQ("<IDLE|C:0.000|E:0.000|V:0.000|T:OK>", buf);
    ASSERT_EQ((int)strlen(buf), n);

    /* RUN frame with signed values — same layout rules */
    n = monitor_format_frame(buf, sizeof(buf), "RUN",
                             120.0f, -1.5f, 3000.0f, "OTPW");
    ASSERT_STR_EQ("<RUN|C:120.000|E:-1.500|V:3000.000|T:OTPW>", buf);
    ASSERT_EQ((int)strlen(buf), n);
    return true;
}

/* ================================================================
 * B. Integration — live sources + periodic reporting (11-16)
 * ================================================================ */

/* 11. monitor_init() is idempotent and returns OK */
static bool test_init_idempotent(void)
{
    ASSERT_EQ(ESP_OK, monitor_init());
    ASSERT_EQ(ESP_OK, monitor_init());
    return true;
}

/* 12. Static status: '?' path runs, frame is an IDLE frame */
static bool test_idle_frame(void)
{
    monitor_send_status();      /* the '?' path — must not crash/block */

    char frame[128];
    int n = monitor_selftest_build_frame(frame, sizeof(frame));
    ASSERT_TRUE(n > 0, "empty frame");

    char  state[8];
    char  flag[8];
    float c = 0.0f, e = 0.0f, v = 0.0f;
    ASSERT_TRUE(parse_frame(frame, state, sizeof(state), &c, &e, &v,
                            flag, sizeof(flag)), "frame did not parse");
    ASSERT_TRUE(strncmp(frame, "<IDLE|", 6) == 0, "frame does not start with <IDLE|");
    ASSERT_STR_EQ("IDLE", state);
    ESP_LOGI(TAG, "  static frame: %s", frame);
    return true;
}

/* 13. C in the frame == the EXECUTED position (engine counter / k).
 *     At rest with no G92 issued, the monitor's origin reference is 0, so
 *     this reduces to engine steps ÷ steps-per-mm. */
static bool test_frame_c(void)
{
    float k = settings_steps_per_mm();
    ASSERT_TRUE(k > 0.0f, "steps_per_mm invalid");

    int32_t s_before = step_engine_position_steps();
    char    frame[128];
    int     n = monitor_selftest_build_frame(frame, sizeof(frame));
    int32_t s_after = step_engine_position_steps();
    ASSERT_TRUE(n > 0, "empty frame");

    char  state[8];
    char  flag[8];
    float c = 0.0f, e = 0.0f, v = 0.0f;
    ASSERT_TRUE(parse_frame(frame, state, sizeof(state), &c, &e, &v,
                            flag, sizeof(flag)), "frame did not parse");

    float lo = (float)s_before / k;
    float hi = (float)s_after / k;

    ESP_LOGI(TAG, "  C frame=%.3f, engine steps=[%ld,%ld] → [%.3f,%.3f] mm",
             (double)c, (long)s_before, (long)s_after, (double)lo, (double)hi);
    ASSERT_TRUE(within_readings(c, lo, hi, 0.001f),
                "frame C does not match the executed (engine) position");
    return true;
}

/* 14. E in the frame == as5600_position_mm() */
static bool test_frame_e(void)
{
    float e_before = as5600_position_mm();
    char  frame[128];
    int   n = monitor_selftest_build_frame(frame, sizeof(frame));
    float e_after = as5600_position_mm();
    ASSERT_TRUE(n > 0, "empty frame");

    char  state[8];
    char  flag[8];
    float c = 0.0f, e = 0.0f, v = 0.0f;
    ASSERT_TRUE(parse_frame(frame, state, sizeof(state), &c, &e, &v,
                            flag, sizeof(flag)), "frame did not parse");

    ESP_LOGI(TAG, "  E frame=%.3f as5600=[%.3f,%.3f]", (double)e,
             (double)e_before, (double)e_after);
    ASSERT_TRUE(within_readings(e, e_before, e_after, 0.001f),
                "frame E does not match as5600_position_mm()");
    return true;
}

/* 15. T in the frame == mapping of the current DRV_STATUS
 *     (retried: a single UART glitch must not fail the case) */
static bool test_frame_t(void)
{
    char frame[128];
    char state[8];
    char flag[8];
    float c = 0.0f, e = 0.0f, v = 0.0f;

    for (int attempt = 0; attempt < 3; attempt++) {
        int n = monitor_selftest_build_frame(frame, sizeof(frame));
        ASSERT_TRUE(n > 0, "empty frame");
        ASSERT_TRUE(parse_frame(frame, state, sizeof(state), &c, &e, &v,
                                flag, sizeof(flag)), "frame did not parse");

        tmc2209_status_t st;
        char expected[8];
        bool ok = tmc2209_read_status(&st);
        monitor_tmc_flag(ok ? &st : NULL, expected, sizeof(expected));

        if (strcmp(expected, flag) == 0) {
            ESP_LOGI(TAG, "  T=%s (matches live DRV_STATUS)", flag);
            return true;
        }
        ESP_LOGW(TAG, "  attempt %d: frame T=%s, live T=%s — retrying",
                 attempt + 1, flag, expected);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "  FAIL %s: frame T never matched live DRV_STATUS", __func__);
    return false;
}

/* 16. Periodic reporting: busy queue → frames at report_hz */
static bool test_periodic_reporting(void)
{
    int hz = settings_report_hz();
    if (hz < 1) {
        hz = 1;
    } else if (hz > 50) {
        hz = 50;
    }

    ASSERT_TRUE(!step_engine_busy(), "engine busy before the periodic test");

    /* 5000 steps, v=2000 steps/s, a=2000 steps/s² → trapezoid
     * 1000 accel + 3000 cruise + 1000 decel ≈ 3.5 s of busytime.
     * TMC stays disabled, so no motion — only the queue is busy. */
    step_seg_t seg = {
        .steps       = 5000u,
        .dir         = true,
        .accel_steps = 1000u,
        .cruise_steps = 3000u,
        .decel_steps = 1000u,
        .accel       = 2000.0f,
        .v_cruise    = 2000.0f,
    };
    ASSERT_TRUE(step_engine_enqueue(&seg), "step_engine_enqueue failed");

    /* STATE derivation must say RUN while the queue is busy */
    char frame[128];
    char state[8];
    char flag[8];
    float c = 0.0f, e = 0.0f, v = 0.0f;
    monitor_selftest_build_frame(frame, sizeof(frame));
    ASSERT_TRUE(parse_frame(frame, state, sizeof(state), &c, &e, &v,
                            flag, sizeof(flag)), "frame did not parse");
    ASSERT_STR_EQ("RUN", state);

    float c_first = c;          /* C right after enqueue ≈ start position */

    uint32_t f0 = monitor_frames_sent();

    /* C must follow the EXECUTED position: sample it again mid-move and
     * require it to have advanced.  A target-position implementation would
     * hold C at the destination for the whole move instead — which is
     * exactly the defect this case guards against. */
    vTaskDelay(pdMS_TO_TICKS(500));
    monitor_selftest_build_frame(frame, sizeof(frame));
    ASSERT_TRUE(parse_frame(frame, state, sizeof(state), &c, &e, &v,
                            flag, sizeof(flag)), "second frame did not parse");

    ESP_LOGI(TAG, "  executed C: %.3f → %.3f mm over 500 ms",
             (double)c_first, (double)c);
    ASSERT_TRUE(c > c_first + 0.1f,
                "C did not advance — is it still reporting the target position?");

    /* Top up to a full 1 s window for the reporting-rate check below. */
    vTaskDelay(pdMS_TO_TICKS(500));
    uint32_t f1 = monitor_frames_sent();
    uint32_t delta = f1 - f0;

    step_engine_stop();         /* teardown — no pulses left running */

    ESP_LOGI(TAG, "  periodic: %lu frames in 1 s, $9=%d Hz",
             (unsigned long)delta, hz);

    int lo = hz - 2;
    int hi = hz + 2;
    if (delta < (uint32_t)(lo < 0 ? 0 : lo) || delta > (uint32_t)hi) {
        ESP_LOGE(TAG, "  FAIL %s: %lu frames outside [%d,%d]", __func__,
                 (unsigned long)delta, lo, hi);
        return false;
    }
    return true;
}

/* ================================================================
 * Runner
 * ================================================================ */

#define RUN_TEST(name) do { \
    s_total++; \
    if (name()) { \
        s_pass++; \
        ESP_LOGI(TAG, "  PASS %s", #name); \
    } else { \
        s_fail++; \
    } \
} while (0)

int monitor_self_test_run(void)
{
    s_pass  = 0;
    s_fail  = 0;
    s_total = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Monitor / Telemetry Self-Test (16 cases)");
    ESP_LOGI(TAG, " TMC2209 EN = HIGH (disabled — no motion)");
    ESP_LOGI(TAG, "========================================");

    esp_err_t err = monitor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "monitor_init failed: %s — aborting self-test",
                 esp_err_to_name(err));
        return 0;
    }

    ESP_LOGI(TAG, "Pure functions (TMC flag map + frame format):");
    RUN_TEST(test_flag_null);
    RUN_TEST(test_flag_ok);
    RUN_TEST(test_flag_ot);
    RUN_TEST(test_flag_s2ga);
    RUN_TEST(test_flag_s2gb);
    RUN_TEST(test_flag_ola);
    RUN_TEST(test_flag_olb);
    RUN_TEST(test_flag_otpw);
    RUN_TEST(test_flag_priority);
    RUN_TEST(test_frame_format);

    ESP_LOGI(TAG, "Integration (live sources + periodic report):");
    RUN_TEST(test_init_idempotent);
    RUN_TEST(test_idle_frame);
    RUN_TEST(test_frame_c);
    RUN_TEST(test_frame_e);
    RUN_TEST(test_frame_t);
    RUN_TEST(test_periodic_reporting);

    /* Teardown: make sure no pulses are left running. */
    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, s_total);
    if (s_fail > 0) {
        ESP_LOGE(TAG, " %d test(s) FAILED", s_fail);
    }
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
