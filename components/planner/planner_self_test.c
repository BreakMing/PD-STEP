/*
 * Motion Planner — on-target self-test (21 cases)
 *
 * Built into firmware when CONFIG_PLANNER_SELF_TEST=y.
 * Runs after the step-engine self-test and BEFORE tmc2209_init(), so
 * the TMC2209 EN pin is still high — the motor does not turn.  Only
 * STEP/DIR logic levels are exercised here; real motion verification
 * happens in the on-board acceptance run.
 *
 * Final log line: "SELF-TEST DONE: <n>/21 PASS" (tag: planner_selftest)
 */

#include "planner.h"
#include "settings.h"
#include "step_engine.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "planner_selftest";

/* ---- Counters ---- */
static int s_pass = 0;
static int s_fail = 0;
static int s_total = 0;

/* ---- Engine position at self-test start (never assume 0) ---- */
static int32_t s_base_pos = 0;

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

#define ASSERT_FLOAT_EQ(expected, actual) do { \
    if (fabsf((expected) - (actual)) > 0.001f) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %.4f, got %.4f (line %d)", \
                 __func__, (double)(expected), (double)(actual), __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_NEAR(expected, actual, tol) do { \
    if (fabsf((expected) - (actual)) > (tol)) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %.4f±%.4f, got %.4f (line %d)", \
                 __func__, (double)(expected), (double)(tol), \
                 (double)(actual), __LINE__); \
        return false; \
    } \
} while (0)

/* ================================================================
 * Helpers
 * ================================================================ */

static float test_k(void)
{
    return settings_steps_per_mm();
}

static float test_feed(void)
{
    return settings_max_feed();     /* fastest legal feed — keeps runtime short */
}

static float test_accel(void)
{
    return settings_max_accel();
}

/* Poll the engine until idle.  Returns false on timeout. */
static bool wait_idle(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (step_engine_busy()) {
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) > timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

/* Command a relative move of `steps` steps and wait for completion. */
static planner_status_t move_rel_steps(int32_t steps, float feed, float accel)
{
    float   k = test_k();
    int32_t target = planner_position_steps() + steps;
    float   target_mm = (float)target / k;

    planner_status_t st = planner_enqueue(target_mm, feed, accel);
    if (st == PLANNER_OK) {
        if (!wait_idle(10000)) {
            ESP_LOGE(TAG, "  move_rel_steps(%ld) timed out", (long)steps);
        }
    }
    return st;
}

/* Enqueue a relative move WITHOUT waiting (for queue back-pressure tests). */
static planner_status_t enqueue_rel_steps(int32_t steps)
{
    float   k = test_k();
    int32_t target = planner_position_steps() + steps;
    return planner_enqueue((float)target / k, test_feed(), test_accel());
}

/* Force a parameter to its documented default so the hardcoded
 * expectations in this file hold.  Logs when a change is needed. */
static void ensure_setting(int idx, float expected)
{
    float cur = settings_get(idx);
    if (fabsf(cur - expected) > 0.001f) {
        ESP_LOGW(TAG, "precondition: $%d was %.3f, restoring documented "
                      "default %.3f", idx, (double)cur, (double)expected);
        settings_set(idx, expected);
    }
}

/* ================================================================
 * Pure solver group (cases 1-9) — no engine interaction
 * ================================================================ */

/* 1. Trapezoid, comfortably long move */
static bool test_solve_trapezoid(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(10000, 1000.0f, 1000.0f, &s), "solve failed");
    ASSERT_EQ(500, s.accel_steps);
    ASSERT_EQ(9000, s.cruise_steps);
    ASSERT_EQ(500, s.decel_steps);
    ASSERT_FLOAT_EQ(1000.0f, s.v_cruise);
    ASSERT_EQ(10000, s.steps);
    return true;
}

/* 2. Triangle — distance too short to reach v */
static bool test_solve_triangle(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(100, 1000.0f, 1000.0f, &s), "solve failed");
    ASSERT_EQ(50, s.accel_steps);
    ASSERT_EQ(0,  s.cruise_steps);
    ASSERT_EQ(50, s.decel_steps);
    /* v_peak = sqrt(2*a*accel_steps) = sqrt(2*1000*50) = 316.2278 */
    ASSERT_NEAR(316.2278f, s.v_cruise, 0.01f);
    ASSERT_EQ(100, s.steps);
    return true;
}

/* 3. Triangle with odd N — extra step goes to decel */
static bool test_solve_triangle_odd(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(101, 1000.0f, 1000.0f, &s), "solve failed");
    ASSERT_EQ(50, s.accel_steps);
    ASSERT_EQ(0,  s.cruise_steps);
    ASSERT_EQ(51, s.decel_steps);
    ASSERT_EQ(101, s.steps);
    return true;
}

/* 4. Critical case 2d == N — trapezoid with zero cruise */
static bool test_solve_critical(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(1000, 1000.0f, 1000.0f, &s), "solve failed");
    ASSERT_EQ(500, s.accel_steps);
    ASSERT_EQ(0,   s.cruise_steps);
    ASSERT_EQ(500, s.decel_steps);
    ASSERT_EQ(1000, s.steps);
    ASSERT_FLOAT_EQ(1000.0f, s.v_cruise);
    return true;
}

/* 5. Engine-domain clamp: v = 200000 must clamp to 65535 (Q16.16 guard) */
static bool test_solve_v_clamp(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(10000000u, 200000.0f, 1000.0f, &s), "solve failed");
    ASSERT_FLOAT_EQ(65535.0f, s.v_cruise);
    ASSERT_EQ(10000000, (int64_t)s.accel_steps + s.cruise_steps + s.decel_steps);
    return true;
}

/* 6. P3 case: $8 default 2000 mm/s² × 80 steps/mm = 160000 must clamp */
static bool test_solve_a_clamp(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(10000, 1000.0f, 200000.0f, &s), "solve failed");
    ASSERT_FLOAT_EQ(65535.0f, s.accel);
    ASSERT_EQ(10000, (int64_t)s.accel_steps + s.cruise_steps + s.decel_steps);
    return true;
}

/* 7. Lower-bound clamp — float_to_q16 must never round to 0 */
static bool test_solve_lower_clamp(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(10, 1e-9f, 1e-9f, &s), "solve failed");
    ASSERT_TRUE(s.v_cruise >= 1e-3f, "v_cruise below floor");
    ASSERT_TRUE(s.accel    >= 1e-3f, "accel below floor");
    ASSERT_EQ(10, (int64_t)s.accel_steps + s.cruise_steps + s.decel_steps);
    return true;
}

/* 8. N=1 degenerate — floor must keep v_cruise > 0 */
static bool test_solve_single_step(void)
{
    step_seg_t s;
    memset(&s, 0, sizeof(s));
    ASSERT_TRUE(planner_solve(1, 100.0f, 100.0f, &s), "solve failed");
    ASSERT_EQ(0, s.accel_steps);
    ASSERT_EQ(0, s.cruise_steps);
    ASSERT_EQ(1, s.decel_steps);
    ASSERT_TRUE(s.v_cruise > 0.0f, "v_cruise must be > 0 (engine rejects 0)");
    return true;
}

/* 9. Property sweep — invariants must hold for any input */
static bool test_solve_properties(void)
{
    static const struct { uint32_t n; float v; float a; } cases[] = {
        {          1u,       100.0f,     100.0f },
        {          7u,       500.0f,     250.0f },
        {        999u,      1234.5f,     987.6f },
        {     100000u,      3000.0f,    1500.0f },
        {    1000000u,     65535.0f,   65535.0f },
        {     500000u,   1000000.0f, 1000000.0f },
    };
    const int ncases = (int)(sizeof(cases) / sizeof(cases[0]));

    for (int i = 0; i < ncases; i++) {
        step_seg_t s;
        memset(&s, 0, sizeof(s));
        ASSERT_TRUE(planner_solve(cases[i].n, cases[i].v, cases[i].a, &s),
                    "solve failed in property sweep");

        int64_t sum = (int64_t)s.accel_steps + s.cruise_steps + s.decel_steps;
        if (sum != (int64_t)cases[i].n) {
            ESP_LOGE(TAG, "  prop[%d]: sum %ld != N %lu", i, (long)sum,
                     (unsigned long)cases[i].n);
            return false;
        }
        if (s.accel_steps > cases[i].n) {
            ESP_LOGE(TAG, "  prop[%d]: accel_steps %lu > N %lu", i,
                     (unsigned long)s.accel_steps, (unsigned long)cases[i].n);
            return false;
        }
        float v_cap = fminf(cases[i].v, 65535.0f);
        if (s.v_cruise > v_cap + 0.001f) {
            ESP_LOGE(TAG, "  prop[%d]: v_cruise %.4f > cap %.4f", i,
                     (double)s.v_cruise, (double)v_cap);
            return false;
        }
    }
    return true;
}

/* ================================================================
 * Integration group (cases 10-21) — planner → step_engine
 * ================================================================ */

/* 10. Single forward move, exact step count */
static bool test_move_forward_exact(void)
{
    int32_t p0 = step_engine_position_steps();
    planner_status_t st = move_rel_steps(1234, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    int32_t p1 = step_engine_position_steps();
    ASSERT_EQ(1234, p1 - p0);
    ASSERT_TRUE(!step_engine_busy(), "engine still busy after completion");
    return true;
}

/* 11. Single reverse move */
static bool test_move_reverse_exact(void)
{
    int32_t p0 = step_engine_position_steps();
    planner_status_t st = move_rel_steps(-567, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    int32_t p1 = step_engine_position_steps();
    ASSERT_EQ(-567, p1 - p0);
    return true;
}

/* 12. Three consecutive moves — no lost steps across segments */
static bool test_consecutive_moves(void)
{
    int32_t p0 = step_engine_position_steps();

    ASSERT_EQ(PLANNER_OK, move_rel_steps( 1234, test_feed(), test_accel()));
    ASSERT_EQ(PLANNER_OK, move_rel_steps( -567, test_feed(), test_accel()));
    ASSERT_EQ(PLANNER_OK, move_rel_steps(  891, test_feed(), test_accel()));

    int32_t p1 = step_engine_position_steps();
    ASSERT_EQ(1234 - 567 + 891, p1 - p0);   /* == 1558 */
    return true;
}

/* 13. Target == current position → no segment produced */
static bool test_zero_move_no_segment(void)
{
    step_seg_t before, after;
    planner_selftest_last_seg(&before);

    int32_t p0 = step_engine_position_steps();
    float   cur_mm = planner_position_mm();

    planner_status_t st = planner_enqueue(cur_mm, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);

    planner_selftest_last_seg(&after);
    ASSERT_TRUE(memcmp(&before, &after, sizeof(before)) == 0,
                "last_seg was updated by a zero-distance move");
    ASSERT_TRUE(!step_engine_busy(), "engine busy after no-op move");
    ASSERT_EQ(0, step_engine_position_steps() - p0);
    return true;
}

/* 14. Sub-step rounding: 0.02 mm → 2 steps (lroundf), 0.001 mm → 0 steps */
static bool test_substep_rounding(void)
{
    float k = test_k();
    ASSERT_FLOAT_EQ(80.0f, k);   /* precondition: documented default */

    /* 0.02 mm × 80 = 1.6 steps → nearest = 2 */
    int32_t p0 = step_engine_position_steps();
    float   base_mm = planner_position_mm();
    planner_status_t st = planner_enqueue(base_mm + 0.02f,
                                          test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    ASSERT_TRUE(wait_idle(10000), "timeout on 0.02mm move");
    ASSERT_EQ(2, step_engine_position_steps() - p0);

    /* 0.001 mm × 80 = 0.08 steps → nearest = 0 → no motion */
    step_seg_t before, after;
    planner_selftest_last_seg(&before);
    base_mm = planner_position_mm();
    st = planner_enqueue(base_mm + 0.001f, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    planner_selftest_last_seg(&after);
    ASSERT_TRUE(memcmp(&before, &after, sizeof(before)) == 0,
                "0.001mm produced a segment");
    ASSERT_EQ(2, step_engine_position_steps() - p0);
    return true;
}

/* 15. Feed clamped to $7: 9000 mm/min → 3000 mm/min → 4000 steps/s */
static bool test_feed_clamped_to_max(void)
{
    ASSERT_FLOAT_EQ(3000.0f, settings_max_feed());   /* precondition */

    planner_status_t st = move_rel_steps(1000, 9000.0f, 2000.0f);
    ASSERT_EQ(PLANNER_OK, st);

    step_seg_t s;
    planner_selftest_last_seg(&s);
    /* 3000 mm/min / 60 × 80 steps/mm = 4000 steps/s */
    ASSERT_FLOAT_EQ(4000.0f, s.v_cruise);
    return true;
}

/* 16. Accel clamped to $8 then engine-clamped (P3 end-to-end) */
static bool test_accel_clamped_to_engine(void)
{
    ASSERT_FLOAT_EQ(2000.0f, settings_max_accel());   /* precondition */

    planner_status_t st = move_rel_steps(1000, 3000.0f, 9000.0f);
    ASSERT_EQ(PLANNER_OK, st);

    step_seg_t s;
    planner_selftest_last_seg(&s);
    /* 2000 mm/s² × 80 = 160000 → engine domain clamp 65535 */
    ASSERT_FLOAT_EQ(65535.0f, s.accel);
    return true;
}

/* 17. Soft-limit rejection — outside range, nothing moves */
static bool test_soft_limit_reject(void)
{
    ASSERT_FLOAT_EQ(600.0f, settings_soft_max());   /* precondition */

    int32_t p0 = step_engine_position_steps();
    int32_t cmd0 = planner_position_steps();

    planner_status_t st = planner_enqueue(601.0f, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_ERR_SOFT_LIMIT, st);
    ASSERT_EQ(0, step_engine_position_steps() - p0);
    ASSERT_EQ(0, planner_position_steps() - cmd0);
    ASSERT_TRUE(!step_engine_busy(), "engine ran on a rejected target");
    return true;
}

/* 18. Boundary targets are inclusive ($12=0 and $13=600 accepted) */
static bool test_soft_limit_boundary(void)
{
    ASSERT_FLOAT_EQ(0.0f,   settings_soft_min());   /* precondition */
    ASSERT_FLOAT_EQ(600.0f, settings_soft_max());   /* precondition */

    /* Upper boundary — park just below it so the travel is short
     * (a full 0→600 mm traverse would blow the 30 s budget). */
    planner_set_origin(599.5f);
    ASSERT_FLOAT_EQ(599.5f, planner_position_mm());
    int32_t p0 = step_engine_position_steps();
    planner_status_t st = planner_enqueue(600.0f, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    ASSERT_TRUE(wait_idle(10000), "timeout on boundary move");
    ASSERT_EQ(40, step_engine_position_steps() - p0);   /* 0.5 mm × 80 */

    /* Lower boundary */
    planner_set_origin(0.5f);
    int32_t p1 = step_engine_position_steps();
    st = planner_enqueue(0.0f, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    ASSERT_TRUE(wait_idle(10000), "timeout on boundary move");
    ASSERT_EQ(-40, step_engine_position_steps() - p1);
    return true;
}

/* 19. G92 origin set then a relative move */
static bool test_set_origin_then_move(void)
{
    planner_set_origin(100.0f);
    ASSERT_FLOAT_EQ(100.0f, planner_position_mm());
    ASSERT_EQ(8000, planner_position_steps());   /* 100 mm × 80 */

    int32_t p0 = step_engine_position_steps();
    planner_status_t st = planner_enqueue(100.5f, test_feed(), test_accel());
    ASSERT_EQ(PLANNER_OK, st);
    ASSERT_TRUE(wait_idle(10000), "timeout on +40 step move");
    ASSERT_EQ(40, step_engine_position_steps() - p0);
    return true;
}

/* 20. $0 change re-derives k ($0=8 → k=40, then restore $0=16 → k=80) */
static bool test_microstep_recomputes_k(void)
{
    float saved_p0 = settings_get(0);
    ASSERT_FLOAT_EQ(16.0f, saved_p0);   /* precondition */

    /* k = 200 × 8 / 40 = 40 */
    ASSERT_EQ(SETTINGS_OK, settings_set(0, 8.0f));
    ASSERT_FLOAT_EQ(40.0f, test_k());

    int32_t a0 = step_engine_position_steps();
    ASSERT_EQ(PLANNER_OK, move_rel_steps(40, test_feed(), test_accel()));
    ASSERT_EQ(40, step_engine_position_steps() - a0);

    /* Restore — k = 200 × 16 / 40 = 80 */
    ASSERT_EQ(SETTINGS_OK, settings_set(0, 16.0f));
    ASSERT_FLOAT_EQ(80.0f, test_k());

    int32_t b0 = step_engine_position_steps();
    ASSERT_EQ(PLANNER_OK, move_rel_steps(80, test_feed(), test_accel()));
    ASSERT_EQ(80, step_engine_position_steps() - b0);
    return true;
}

/* 21. 17 rapid enqueues of 64 steps — exercises queue back-pressure */
static bool test_queue_backpressure(void)
{
    int32_t p0 = step_engine_position_steps();

    /* Enqueue without waiting: the 16-deep queue fills and the 17th
     * call blocks inside planner_enqueue until space frees up. */
    for (int i = 0; i < 17; i++) {
        planner_status_t st = enqueue_rel_steps(64);
        if (st != PLANNER_OK) {
            ESP_LOGE(TAG, "  enqueue %d returned %d", i, (int)st);
            return false;
        }
    }

    ASSERT_TRUE(wait_idle(15000), "timeout draining 17 segments");
    ASSERT_EQ(17 * 64, step_engine_position_steps() - p0);   /* 1088 */
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

int planner_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;
    s_total = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Motion Planner Self-Test (21 cases)");
    ESP_LOGI(TAG, " TMC2209 EN = HIGH (disabled — no motion)");
    ESP_LOGI(TAG, "========================================");

    /* --- Own initialization (both are idempotent; app_main's later
     *     calls become no-ops) --- */
    esp_err_t err = step_engine_init(CONFIG_BSP_STEP_GPIO, CONFIG_BSP_DIR_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "step_engine_init failed: %s — aborting self-test",
                 esp_err_to_name(err));
        return 0;
    }
    err = planner_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "planner_init failed: %s — aborting self-test",
                 esp_err_to_name(err));
        return 0;
    }

    /* --- Preconditions: restore the documented parameter defaults so
     *     the hardcoded expectations below are meaningful. --- */
    ensure_setting(7,  3000.0f);    /* max feed  */
    ensure_setting(8,  2000.0f);    /* max accel */
    ensure_setting(12,    0.0f);    /* soft min  */
    ensure_setting(13,  600.0f);    /* soft max  */
    ensure_setting(0,    16.0f);    /* microstep */
    ensure_setting(1,   200.0f);    /* full steps */
    ESP_LOGI(TAG, "preconditions ok: $0=%.0f $1=%.0f $7=%.0f $8=%.0f "
                  "$12=%.0f $13=%.0f  k=%.2f",
             (double)settings_get(0), (double)settings_get(1),
             (double)settings_get(7), (double)settings_get(8),
             (double)settings_get(12), (double)settings_get(13),
             (double)test_k());

    s_base_pos = step_engine_position_steps();
    ESP_LOGI(TAG, "engine position baseline: %ld steps", (long)s_base_pos);

    ESP_LOGI(TAG, "Pure solver:");
    RUN_TEST(test_solve_trapezoid);
    RUN_TEST(test_solve_triangle);
    RUN_TEST(test_solve_triangle_odd);
    RUN_TEST(test_solve_critical);
    RUN_TEST(test_solve_v_clamp);
    RUN_TEST(test_solve_a_clamp);
    RUN_TEST(test_solve_lower_clamp);
    RUN_TEST(test_solve_single_step);
    RUN_TEST(test_solve_properties);

    ESP_LOGI(TAG, "Integration (planner -> step_engine):");
    RUN_TEST(test_move_forward_exact);
    RUN_TEST(test_move_reverse_exact);
    RUN_TEST(test_consecutive_moves);
    RUN_TEST(test_zero_move_no_segment);
    RUN_TEST(test_substep_rounding);
    RUN_TEST(test_feed_clamped_to_max);
    RUN_TEST(test_accel_clamped_to_engine);
    RUN_TEST(test_soft_limit_reject);
    RUN_TEST(test_soft_limit_boundary);
    RUN_TEST(test_set_origin_then_move);
    RUN_TEST(test_microstep_recomputes_k);
    RUN_TEST(test_queue_backpressure);

    /* --- Teardown: stop pulses and settle --- */
    step_engine_stop();
    wait_idle(2000);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, s_total);
    if (s_fail > 0) {
        ESP_LOGE(TAG, " %d test(s) FAILED", s_fail);
    }
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
