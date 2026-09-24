/*
 * AS5600 self-test — 18 counted cases + 1 guided manual segment
 *
 *   Group A (12)  pure logic: multi-turn counting rules on synthetic
 *                 raw sequences — no hardware dependency.
 *   Group B (6)   hardware: live I2C reads on the real sensor.
 *   Group C       guided manual check (3 turns by hand) — logged for
 *                 human judgement, NOT counted in the PASS total.
 *
 * Built into the firmware only when CONFIG_AS5600_SELF_TEST=y.
 * Runs early in startup, right after the sampling task is up and before
 * tmc2209_init() — the motor is not enabled, and this component only
 * ever reads the encoder.  Keep hands off the shaft during groups A/B:
 * groups B15/B16 judge standstill noise and standstill velocity.
 *
 * Final line: "SELF-TEST DONE: <n>/18 PASS"  (tag: as5600_selftest)
 */

#include "as5600.h"
#include "bsp.h"
#include "settings.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "as5600_selftest";

/* ---- Counters ---- */
static int s_pass  = 0;
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

#define ASSERT_NEAR(expected, actual, tol) do { \
    if (fabsf((float)(expected) - (float)(actual)) > (tol)) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %.4f±%.4f, got %.4f (line %d)", \
                 __func__, (double)(expected), (double)(tol), \
                 (double)(actual), __LINE__); \
        return false; \
    } \
} while (0)

/* ---- Helpers ---- */

/* Advance the simulated shaft by `step` counts, `steps` times, wrapping
 * the raw angle at 4096 exactly like the real sensor would. */
static int32_t sim_run(as5600_tracker_t *t, uint16_t *raw, int steps, int32_t step)
{
    for (int i = 0; i < steps; i++) {
        int32_t next = ((int32_t)*raw + step) % AS5600_COUNTS_PER_REV;
        if (next < 0) {
            next += AS5600_COUNTS_PER_REV;
        }
        *raw = (uint16_t)next;
        as5600_tracker_update(t, *raw);
    }
    return t->pos_counts;
}

static float test_upr(void)
{
    float upr = settings_units_per_rev();
    return (upr > 0.0f) ? upr : 40.0f;
}

/* ================================================================
 * Group A — pure logic (12 cases)
 * ================================================================ */

/* A1: the first sample only seeds — it must not create displacement. */
static bool case_a1_seed(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);

    ASSERT_EQ(0, as5600_tracker_update(&t, 100));
    ASSERT_TRUE(t.seeded, "tracker not seeded after first sample");
    ASSERT_EQ(0, t.pos_counts);
    ASSERT_EQ(100, t.last_raw);
    return true;
}

/* A2: same raw twice → zero displacement (standstill). */
static bool case_a2_static(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 100);

    ASSERT_EQ(0, as5600_tracker_update(&t, 100));
    return true;
}

/* A3: plain forward motion, no edge crossing. */
static bool case_a3_monotonic(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 100);

    as5600_tracker_update(&t, 200);
    ASSERT_EQ(200, as5600_tracker_update(&t, 300));   /* +100 +100 */
    return true;
}

/* A4: forward across the 0/4095 edge: 4090 → 10 is +16 counts.
 * Naive subtraction would give −4080. */
static bool case_a4_wrap_forward(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 4090);

    ASSERT_EQ(16, as5600_tracker_update(&t, 10));
    return true;
}

/* A5: backwards across the edge: 10 → 4090 is −16 counts. */
static bool case_a5_wrap_backward(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 10);

    ASSERT_EQ(-16, as5600_tracker_update(&t, 4090));
    return true;
}

/* A6: dithering right on the edge 0 ↔ 4095 must degrade to ±1 count,
 * never to ±4095 — this is why no extra dead-band is needed. */
static bool case_a6_edge_dither(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 0);

    for (int i = 0; i < 8; i++) {
        uint16_t raw = ((i % 2) == 0) ? 4095 : 0;
        int32_t  pos = as5600_tracker_update(&t, raw);
        if (pos > 1 || pos < -1) {
            ESP_LOGE(TAG, "  FAIL %s: pos=%ld after %d edge steps (expected |pos|<=1)",
                     __func__, (long)pos, i + 1);
            return false;
        }
    }

    ASSERT_TRUE(t.pos_counts <= 1 && t.pos_counts >= -1,
                "edge dither accumulated drift");
    return true;
}

/* A7: three forward turns = +12288 counts = 120.0 mm — the pure-logic
 * equivalent of the PLAN acceptance criterion. */
static bool case_a7_three_turns_fwd(void)
{
    as5600_tracker_t t;
    uint16_t         raw = 1000;
    float            upr = test_upr();

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    int32_t pos = sim_run(&t, &raw, 192, 64);   /* 192 × 64 = 12288 */

    ASSERT_EQ(3 * AS5600_COUNTS_PER_REV, pos);

    float mm = (float)pos * upr / (float)AS5600_COUNTS_PER_REV;
    ESP_LOGI(TAG, "  (3 turns = %ld counts = %.3f mm @ %.1f mm/rev)",
             (long)pos, (double)mm, (double)upr);
    ASSERT_NEAR(3.0f * upr, mm, 0.001f);
    return true;
}

/* A8: three reverse turns = −12288 counts (sign symmetry). */
static bool case_a8_three_turns_rev(void)
{
    as5600_tracker_t t;
    uint16_t         raw = 1000;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    int32_t pos = sim_run(&t, &raw, 192, -64);

    ASSERT_EQ(-3 * AS5600_COUNTS_PER_REV, pos);
    return true;
}

/* A9: diff exactly +2048 (half a turn) must pass through unchanged —
 * the wrap correction is strictly "> 2048". */
static bool case_a9_half_turn_pos(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 0);

    ASSERT_EQ(2048, as5600_tracker_update(&t, 2048));
    return true;
}

/* A10: diff exactly −2047 must pass through unchanged — the correction
 * is strictly "< −2047", which is what makes the two thresholds cover
 * all 4096 diffs with no gap and no overlap. */
static bool case_a10_half_turn_neg(void)
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 2048);

    ASSERT_EQ(-2047, as5600_tracker_update(&t, 1));
    return true;
}

/* A11: fast forward — 20 × 500 counts across the edge, no lost turns. */
static bool case_a11_fast_forward(void)
{
    as5600_tracker_t t;
    uint16_t         raw = 0;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    int32_t pos = sim_run(&t, &raw, 20, 500);

    ASSERT_EQ(10000, pos);
    return true;
}

/* A12: two turns forward then two back → net zero (direction symmetric). */
static bool case_a12_round_trip(void)
{
    as5600_tracker_t t;
    uint16_t         raw = 3000;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    sim_run(&t, &raw, 128, 64);      /* +8192 = 2 turns */
    ASSERT_EQ(2 * AS5600_COUNTS_PER_REV, t.pos_counts);

    int32_t pos = sim_run(&t, &raw, 128, -64);
    ASSERT_EQ(0, pos);
    return true;
}

/* ================================================================
 * Group B — hardware (6 cases)
 * ================================================================ */

/* B13: the BSP bus exists and the device can be mounted on it. */
static bool case_b13_bus_and_init(void)
{
    ASSERT_TRUE(bsp_get_i2c_bus() != NULL, "bsp_get_i2c_bus() returned NULL");
    ASSERT_EQ(ESP_OK, as5600_init());
    return true;
}

/* B14: 20 live samples, all valid 12-bit values and the link healthy.
 * (Only reads — see the red line in as5600.c.) */
static bool case_b14_live_reads(void)
{
    for (int i = 0; i < 20; i++) {
        uint16_t raw = as5600_last_raw();
        if (raw > 4095) {
            ESP_LOGE(TAG, "  FAIL %s: raw %u out of 12-bit range (sample %d)",
                     __func__, raw, i);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ASSERT_TRUE(as5600_is_ok(), "as5600_is_ok() false — I2C reads failing");
    ESP_LOGI(TAG, "  (last raw=%u, counts=%ld)",
             as5600_last_raw(), (long)as5600_position_counts());
    return true;
}

/* B15: standstill noise — peak-to-peak of 20 samples ≤2 counts
 * (PLAN T8 acceptance criterion).  Keep the shaft still.
 *
 * The peak-to-peak is measured on the *unwrapped* difference from the
 * first sample, using the same ±2048 rule as the tracker: comparing raw
 * values directly would report 4095 counts of "noise" for a shaft parked
 * exactly on the 0/4095 wrap point, where the sensor legitimately
 * alternates between 4095 and 0.  The raw extremes are logged too, so
 * the edge case stays visible instead of being silently swallowed. */
static bool case_b15_noise(void)
{
    int32_t  ref     = (int32_t)as5600_last_raw();
    uint16_t raw_min = 0xFFFF;
    uint16_t raw_max = 0;
    int32_t  d_min   = 0;
    int32_t  d_max   = 0;

    for (int i = 0; i < 20; i++) {
        uint16_t raw = as5600_last_raw();
        if (raw < raw_min) raw_min = raw;
        if (raw > raw_max) raw_max = raw;

        int32_t d = (int32_t)raw - ref;
        if (d > 2048) {
            d -= 4096;
        } else if (d < -2047) {
            d += 4096;
        }
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    int32_t pp = d_max - d_min;

    ESP_LOGI(TAG, "  (noise: raw %u..%u, unwrapped p-p=%ld counts)",
             raw_min, raw_max, (long)pp);

    if ((uint16_t)(raw_max - raw_min) > 2 && pp <= 2) {
        ESP_LOGW(TAG, "  note: raw extremes straddle the 0/4095 edge "
                      "(%u raw counts) — shaft parked on the wrap point, "
                      "unwrapped noise is still ≤2",
                 (unsigned)(raw_max - raw_min));
    }

    ASSERT_TRUE(pp <= 2, "standstill noise exceeds 2 counts (was the shaft touched?)");
    return true;
}

/* B16: standstill velocity ≈ 0 (|v| ≤ 5 mm/s on the 16 ms window). */
static bool case_b16_static_velocity(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));   /* let the 16 ms window fill */

    float v = as5600_velocity_mm_s();
    ESP_LOGI(TAG, "  (standstill velocity = %.3f mm/s)", (double)v);

    ASSERT_TRUE(fabsf(v) <= 5.0f, "standstill velocity exceeds 5 mm/s");
    return true;
}

/* B17: position/scale consistency — no origin shift issued yet, the
 * offset is 0 (D10), so E = counts × units_per_rev / 4096. */
static bool case_b17_scale_consistency(void)
{
    int32_t counts = as5600_position_counts();
    float   expect = (float)counts * test_upr() / (float)AS5600_COUNTS_PER_REV;
    float   actual = as5600_position_mm();

    ESP_LOGI(TAG, "  (counts=%ld → expect %.3f mm, reported %.3f mm)",
             (long)counts, (double)expect, (double)actual);

    ASSERT_NEAR(expect, actual, 0.01f);
    return true;
}

/* B18: origin shift — the reported position follows set_origin()
 * (this is the interface D10 uses to keep C and E comparable). */
static bool case_b18_origin_shift(void)
{
    as5600_set_origin(100.0f);
    float at100 = as5600_position_mm();
    ESP_LOGI(TAG, "  (after set_origin(100): E=%.3f mm)", (double)at100);
    ASSERT_NEAR(100.0f, at100, 0.01f);

    as5600_set_origin(0.0f);
    float at0 = as5600_position_mm();
    ESP_LOGI(TAG, "  (after set_origin(0):   E=%.3f mm)", (double)at0);
    ASSERT_NEAR(0.0f, at0, 0.01f);

    return true;
}

/* ================================================================
 * Case table
 * ================================================================ */

typedef bool (*as5600_case_fn_t)(void);

typedef struct {
    const char          *name;
    as5600_case_fn_t     fn;
} as5600_case_t;

static const as5600_case_t s_cases[] = {
    { "A1  first sample seeds, position 0 (no motion)",       case_a1_seed },
    { "A2  identical raw → 0 counts (standstill)",            case_a2_static },
    { "A3  100→200→300 → +200 counts",                        case_a3_monotonic },
    { "A4  4090→10 → +16 counts (forward edge wrap)",         case_a4_wrap_forward },
    { "A5  10→4090 → −16 counts (backward edge wrap)",        case_a5_wrap_backward },
    { "A6  0↔4095 dither ×8 → drift ≤1 count",                case_a6_edge_dither },
    { "A7  forward 3 turns → +12288 counts = 120 mm",         case_a7_three_turns_fwd },
    { "A8  reverse 3 turns → −12288 counts",                  case_a8_three_turns_rev },
    { "A9  diff +2048 → +2048 (no wrap correction)",          case_a9_half_turn_pos },
    { "A10 diff −2047 → −2047 (no wrap correction)",          case_a10_half_turn_neg },
    { "A11 20 × +500 counts → +10000 (no lost turn)",         case_a11_fast_forward },
    { "A12 +2 turns then −2 turns → 0 (symmetric)",           case_a12_round_trip },
    { "B13 I2C bus present + as5600_init() OK",               case_b13_bus_and_init },
    { "B14 20 live reads, all in 0..4095, link OK",           case_b14_live_reads },
    { "B15 standstill noise p-p ≤2 counts",                   case_b15_noise },
    { "B16 standstill velocity |v| ≤5 mm/s",                  case_b16_static_velocity },
    { "B17 E == counts × units_per_rev / 4096",               case_b17_scale_consistency },
    { "B18 origin shift (set 100 / back to 0)",               case_b18_origin_shift },
};

#define AS5600_CASE_COUNT  (int)(sizeof(s_cases) / sizeof(s_cases[0]))

/* ================================================================
 * Group C — guided manual check (not counted)
 * ================================================================ */
static void run_manual_check(void)
{
    const int   secs = 15;
    const float upr  = test_upr();

    ESP_LOGI(TAG, "============ MANUAL CHECK（= 上板清单 A9）===========");
    ESP_LOGI(TAG, "Turn the motor shaft 3 FULL TURNS by hand (either direction), now.");
    ESP_LOGI(TAG, "(电机轴经同步带连着小车——转轴即移动小车)");
    ESP_LOGI(TAG, "Sampling for %d s — position will be logged every second.", secs);

    float start = as5600_position_mm();

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "  t=%2ds  E=%.3f mm  (%.1f counts)",
                 sec, (double)as5600_position_mm(),
                 (double)as5600_position_counts());
    }

    float delta = as5600_position_mm() - start;

    ESP_LOGI(TAG, "MANUAL RESULT: delta = %+.3f mm  (expect ~±%.1f for 3 turns; 1 turn = %.1f mm)",
             (double)delta, (double)(3.0f * upr), (double)upr);
    ESP_LOGI(TAG, "A9 记录：x_um 随转轴变化 %s——记下你转动的方向与此符号的关系（+/-），"
                  "留待 C4 符号约定标定时交叉核对", delta >= 0 ? "为 +" : "为 −");

    if (fabsf(delta) < 1.0f) {
        ESP_LOGI(TAG, "(no rotation detected — normal if you did not turn the shaft)");
    }
    ESP_LOGI(TAG, "======================================");
}

/* ================================================================
 * Entry point
 * ================================================================ */
int as5600_self_test_run(void)
{
    ESP_LOGI(TAG, "=== AS5600 SELF-TEST — 12 pure-logic + 6 hardware "
                  "(+ guided manual segment) ===");
    ESP_LOGI(TAG, "  keep hands off the shaft until the MANUAL CHECK banner");

    /* Idempotent: starts the 1 kHz sampling task if not running yet. */
    esp_err_t err = as5600_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "as5600_init() failed: %s", esp_err_to_name(err));
        ESP_LOGI(TAG, "SELF-TEST DONE: 0/%d PASS", AS5600_CASE_COUNT);
        return 0;
    }

    ESP_LOGI(TAG, "  $2 units_per_rev = %.3f mm/rev  →  1 count = %.6f mm",
             (double)settings_units_per_rev(),
             (double)(settings_units_per_rev() / (float)AS5600_COUNTS_PER_REV));

    for (int i = 0; i < AS5600_CASE_COUNT; i++) {
        s_total++;
        if (s_cases[i].fn()) {
            s_pass++;
            ESP_LOGI(TAG, "PASS %s", s_cases[i].name);
        } else {
            ESP_LOGE(TAG, "FAIL %s", s_cases[i].name);
        }
    }

    ESP_LOGI(TAG, "SELF-TEST DONE: %d/%d PASS", s_pass, s_total);

    /* Guided manual segment — informational, not part of the PASS total. */
    run_manual_check();

    return s_pass;
}
