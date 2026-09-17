/*
 * System Control — on-target self-test (14 cases)
 *
 * Built into the firmware when CONFIG_SYSCTL_SELF_TEST=y.
 * Runs from app_main after sysctl_init() (i.e. after step_engine_init /
 * planner_init / protocol_start), because the live cases drive the planner
 * and read the engine.
 *
 * The TMC2209 EN pin is still HIGH here (nothing enabled it), so the motor
 * does not turn — the pulse queue is exercised, the shaft is not.
 *
 * Final log line: "SELF-TEST DONE: <n>/14 PASS" (tag: sysctl_selftest)
 */

#include "sysctl.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "planner.h"
#include "step_engine.h"
#include "settings.h"

static const char *TAG = "sysctl_selftest";

/* ---- Counters ---- */
static int s_pass  = 0;
static int s_fail  = 0;
static int s_total = 0;

/* ---- Movement parameters (mm domain, inside the default $7/$8) ---- */
#define TEST_ACCEL      25.0f    /* mm/s²  (<= $8 default 2000) */
#define FEED_MOVE     1200.0f    /* mm/min for the short moves */
#define FEED_LONG     1500.0f    /* mm/min for the long move  */
#define LONG_TARGET    250.0f    /* mm — ≈ 11 s at FEED_LONG   */

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

#define ASSERT_NEAR_MM(expected, actual) do { \
    if (fabsf((expected) - (actual)) > 0.001f) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %.4f mm, got %.4f mm (line %d)", \
                 __func__, (double)(expected), (double)(actual), __LINE__); \
        return false; \
    } \
} while (0)

/* ================================================================
 * Helpers
 * ================================================================ */

static float test_k(void)
{
    float k = settings_steps_per_mm();
    return (k > 0.0f) ? k : 80.0f;      /* documented default */
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

/* ---- Executed↔commanded frame reference (see align_frame()) ---- */
static int32_t s_ref_eng  = 0;      /* engine steps at the declared origin */
static float   s_ref_mm   = 0.0f;   /* commanded mm declared at that point  */

/**
 * @brief Declare the current position (G92-equivalent) so the commanded
 *        frame is known relative to the engine counter.
 *
 * Case 8 asserts "commanded position == executed steps", which is only
 * literal when the commanded frame has no G92 offset.  Declaring the
 * current position as engine/k is a no-op in that case; if the engine
 * happens to sit outside the soft-limit window, a safe coordinate is
 * declared instead and the offset is carried by s_ref_mm / s_ref_eng.
 */
static void align_frame(void)
{
    float   k   = test_k();
    int32_t eng = step_engine_position_steps();
    float   cur_mm = (float)eng / k;

    float decl = (cur_mm >= 0.0f && cur_mm <= 300.0f) ? cur_mm : 100.0f;

    planner_set_origin(decl);       /* G92 path — re-anchors the planner too */
    s_ref_eng = eng;
    s_ref_mm  = decl;

    ESP_LOGI(TAG, "frame aligned: engine %ld steps (%.3f mm) declared as %.3f mm",
             (long)eng, (double)cur_mm, (double)decl);
}

/* Commanded position the planner must report for an executed engine count. */
static float expected_cmd_mm(int32_t engine_steps)
{
    int32_t d = engine_steps - s_ref_eng;
    if (settings_invert_dir()) {
        d = -d;                     /* engine counts physical steps ($10) */
    }
    return s_ref_mm + (float)d / test_k();
}

/* Case 7 hands these to cases 8/9. */
static bool  s_freeze_valid   = false;
static int32_t s_frozen_eng   = 0;
static float s_frozen_mm      = 0.0f;
static float s_cancelled_target = 0.0f;

/* ================================================================
 * A. State machine (1-6)
 * ================================================================ */

/* 1. sysctl_init() is idempotent and returns OK */
static bool test_init_idempotent(void)
{
    ASSERT_EQ(ESP_OK, sysctl_init());
    ASSERT_EQ(ESP_OK, sysctl_init());
    ASSERT_TRUE(sysctl_get_state() != SYSCTL_BOOT, "state still BOOT after init");
    return true;
}

/* 2. After init the state is IDLE, not BOOT */
static bool test_state_after_init(void)
{
    ASSERT_EQ(SYSCTL_IDLE, sysctl_get_state());
    return true;
}

/* 3. State string is one of the three upper-case values; IDLE at rest */
static bool test_state_str(void)
{
    const char *s = sysctl_state_str();

    if (!(strcmp(s, "BOOT") == 0 || strcmp(s, "IDLE") == 0 ||
          strcmp(s, "RUN") == 0)) {
        ESP_LOGE(TAG, "  FAIL %s: unexpected state string \"%s\"", __func__, s);
        return false;
    }
    ASSERT_STR_EQ("IDLE", s);
    return true;
}

/* 4. A queued segment puts the state in RUN */
static bool test_run_while_moving(void)
{
    float base = planner_position_mm();
    ASSERT_TRUE(!step_engine_busy(), "engine busy before the RUN test");
    ASSERT_TRUE(planner_enqueue(base + 25.0f, FEED_MOVE, TEST_ACCEL)
                == PLANNER_OK, "enqueue failed");

    ASSERT_TRUE(step_engine_busy(), "engine not busy right after enqueue");
    ASSERT_EQ(SYSCTL_RUN, sysctl_get_state());
    ASSERT_STR_EQ("RUN", sysctl_state_str());
    return true;
}

/* 5. When the segment finishes the state returns to IDLE */
static bool test_idle_after_completion(void)
{
    ASSERT_TRUE(wait_idle(15000), "segment did not finish");
    ASSERT_EQ(SYSCTL_IDLE, sysctl_get_state());
    ASSERT_STR_EQ("IDLE", sysctl_state_str());
    return true;
}

/* 6. stop() while standing still: state IDLE, nothing moves */
static bool test_stop_when_idle(void)
{
    ASSERT_TRUE(!step_engine_busy(), "engine busy before the idle-stop test");
    float p0 = planner_position_mm();
    int32_t e0 = step_engine_position_steps();

    sysctl_stop();

    ASSERT_EQ(SYSCTL_IDLE, sysctl_get_state());
    ASSERT_TRUE(!step_engine_busy(), "engine busy after sysctl_stop");
    ASSERT_EQ(e0, step_engine_position_steps());
    ASSERT_NEAR_MM(p0, planner_position_mm());
    return true;
}

/* ================================================================
 * B. Immediate stop and the position reference (7-14)
 * ================================================================ */

/* 7. '!' during a long move: pulses stop and the counter freezes */
static bool test_stop_freezes_position(void)
{
    float base = planner_position_mm();

    ASSERT_TRUE(planner_enqueue(base + LONG_TARGET, FEED_LONG, TEST_ACCEL)
                == PLANNER_OK, "long enqueue failed");
    vTaskDelay(pdMS_TO_TICKS(800));

    ASSERT_TRUE(step_engine_busy(), "long move already over — check $7/$8");

    sysctl_stop();

    ASSERT_TRUE(!step_engine_busy(), "engine still busy after sysctl_stop");

    s_frozen_eng = step_engine_position_steps();
    s_frozen_mm  = planner_position_mm();
    s_cancelled_target = base + LONG_TARGET;

    vTaskDelay(pdMS_TO_TICKS(200));
    ASSERT_EQ(s_frozen_eng, step_engine_position_steps());   /* still frozen */
    ASSERT_NEAR_MM(s_frozen_mm, planner_position_mm());

    s_freeze_valid = true;
    ESP_LOGI(TAG, "  stopped at engine %ld steps (%.3f mm), target was %.3f mm",
             (long)s_frozen_eng, (double)s_frozen_mm, (double)s_cancelled_target);
    return true;
}

/* 8. Commanded position == executed position (PLAN §4.4) */
static bool test_position_matches_executed(void)
{
    ASSERT_TRUE(!step_engine_busy(), "engine busy — nothing to compare");

    int32_t eng  = step_engine_position_steps();
    float   want = expected_cmd_mm(eng);
    float   got  = planner_position_mm();

    ESP_LOGI(TAG, "  commanded %.3f mm vs executed %.3f mm (engine %ld steps)",
             (double)got, (double)want, (long)eng);
    ASSERT_NEAR_MM(want, got);
    return true;
}

/* 9. The frozen position is well below the cancelled target (rebase ran) */
static bool test_position_below_target(void)
{
    ASSERT_TRUE(s_freeze_valid, "case 7 did not produce a frozen position");
    ASSERT_TRUE(s_frozen_mm < s_cancelled_target - 1.0f,
                "commanded position equals the cancelled target — no rebase");

    ESP_LOGI(TAG, "  cancelled target %.3f mm, frozen at %.3f mm",
             (double)s_cancelled_target, (double)s_frozen_mm);
    return true;
}

/* 10. The next move after '!' runs from the stopped position (T10 hard case) */
static bool test_new_move_after_stop(void)
{
    float   base  = planner_position_mm();
    int32_t e0    = step_engine_position_steps();

    ASSERT_TRUE(planner_enqueue(base + 20.0f, FEED_MOVE, TEST_ACCEL)
                == PLANNER_OK, "enqueue after '!' failed");
    ASSERT_TRUE(wait_idle(15000), "move after '!' did not finish");

    int32_t want = (int32_t)lroundf(20.0f * test_k());
    int32_t got  = step_engine_position_steps() - e0;
    if (settings_invert_dir()) {
        want = -want;
    }

    ESP_LOGI(TAG, "  after '!': %.3f mm -> %.3f mm (%ld steps, expected %ld)",
             (double)base, (double)planner_position_mm(), (long)got, (long)want);
    ASSERT_EQ(want, got);
    ASSERT_NEAR_MM(base + 20.0f, planner_position_mm());
    return true;
}

/* 11. The relative-move base (G91) is the rebased position */
static bool test_relative_base_after_stop(void)
{
    /* G91 resolution is "current commanded position + X" (T4 parser), so
     * this is exactly the base a relative move would use. */
    ASSERT_TRUE(!step_engine_busy(), "engine busy");
    float base = planner_position_mm();
    ASSERT_NEAR_MM(expected_cmd_mm(step_engine_position_steps()), base);

    int32_t e0 = step_engine_position_steps();
    ASSERT_TRUE(planner_enqueue(base + 5.0f, FEED_MOVE, TEST_ACCEL)
                == PLANNER_OK, "relative move enqueue failed");
    ASSERT_TRUE(wait_idle(15000), "relative move did not finish");

    int32_t want = (int32_t)lroundf(5.0f * test_k());
    if (settings_invert_dir()) {
        want = -want;
    }
    ASSERT_EQ(want, step_engine_position_steps() - e0);
    ASSERT_NEAR_MM(base + 5.0f, planner_position_mm());
    return true;
}

/* 12. Two consecutive stops: idempotent, no drift */
static bool test_stop_idempotent(void)
{
    float base = planner_position_mm();
    ASSERT_TRUE(planner_enqueue(base + LONG_TARGET, FEED_LONG, TEST_ACCEL)
                == PLANNER_OK, "long enqueue failed");
    vTaskDelay(pdMS_TO_TICKS(500));

    sysctl_stop();
    int32_t e1 = step_engine_position_steps();
    float   p1 = planner_position_mm();

    sysctl_stop();      /* second stop — must be a no-op */

    ASSERT_EQ(SYSCTL_IDLE, sysctl_get_state());
    ASSERT_TRUE(!step_engine_busy(), "engine busy after the second stop");
    ASSERT_EQ(e1, step_engine_position_steps());
    ASSERT_NEAR_MM(p1, planner_position_mm());
    return true;
}

/* 13. Multi-segment queue reuse after a stop */
static bool test_multi_segments_after_stop(void)
{
    sysctl_stop();                      /* clean slate */
    float   base = planner_position_mm();
    int32_t e0   = step_engine_position_steps();

    for (int i = 1; i <= 3; i++) {
        float target = base + 25.0f * (float)i;
        ASSERT_TRUE(planner_enqueue(target, FEED_LONG, TEST_ACCEL) == PLANNER_OK,
                    "segment enqueue failed");
        ASSERT_TRUE(wait_idle(15000), "segment did not finish");
    }

    int32_t want = (int32_t)lroundf(75.0f * test_k());
    if (settings_invert_dir()) {
        want = -want;
    }

    ESP_LOGI(TAG, "  3 segments after '!': engine %ld steps (expected %ld)",
             (long)(step_engine_position_steps() - e0), (long)want);
    ASSERT_EQ(want, step_engine_position_steps() - e0);
    ASSERT_NEAR_MM(base + 75.0f, planner_position_mm());
    return true;
}

/* 14. State and frame state agree right after a stop during motion */
static bool test_state_consistent_after_stop(void)
{
    float base = planner_position_mm();
    ASSERT_TRUE(planner_enqueue(base + LONG_TARGET, FEED_LONG, TEST_ACCEL)
                == PLANNER_OK, "long enqueue failed");
    vTaskDelay(pdMS_TO_TICKS(500));

    ASSERT_EQ(SYSCTL_RUN, sysctl_get_state());      /* still moving */

    sysctl_stop();

    ASSERT_EQ(SYSCTL_IDLE, sysctl_get_state());
    ASSERT_STR_EQ("IDLE", sysctl_state_str());
    ASSERT_TRUE(!step_engine_busy(), "engine busy while state says IDLE");
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

int sysctl_self_test_run(void)
{
    s_pass  = 0;
    s_fail  = 0;
    s_total = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " System Control Self-Test (14 cases)");
    ESP_LOGI(TAG, " TMC2209 EN = HIGH (disabled — no motion)");
    ESP_LOGI(TAG, "========================================");

    esp_err_t err = sysctl_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sysctl_init failed: %s — aborting self-test",
                 esp_err_to_name(err));
        return 0;
    }

    /* Preconditions: the documented defaults keep the targets legal. */
    ESP_LOGI(TAG, "preconditions: $7=%.0f $8=%.0f $12=%.0f $13=%.0f $10=%.0f k=%.3f",
             (double)settings_max_feed(), (double)settings_max_accel(),
             (double)settings_soft_min(), (double)settings_soft_max(),
             (double)(settings_invert_dir() ? 1.0f : 0.0f),
             (double)test_k());

    if (step_engine_busy()) {
        ESP_LOGE(TAG, "engine busy at self-test start — aborting");
        return 0;
    }

    align_frame();

    ESP_LOGI(TAG, "State machine:");
    RUN_TEST(test_init_idempotent);
    RUN_TEST(test_state_after_init);
    RUN_TEST(test_state_str);
    RUN_TEST(test_run_while_moving);
    RUN_TEST(test_idle_after_completion);
    RUN_TEST(test_stop_when_idle);

    ESP_LOGI(TAG, "Immediate stop ('!') + position reference:");
    RUN_TEST(test_stop_freezes_position);
    RUN_TEST(test_position_matches_executed);
    RUN_TEST(test_position_below_target);
    RUN_TEST(test_new_move_after_stop);
    RUN_TEST(test_relative_base_after_stop);
    RUN_TEST(test_stop_idempotent);
    RUN_TEST(test_multi_segments_after_stop);
    RUN_TEST(test_state_consistent_after_stop);

    /* Teardown: no pulses left running, state back to IDLE. */
    step_engine_stop();
    sysctl_stop();
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, s_total);
    if (s_fail > 0) {
        ESP_LOGE(TAG, " %d test(s) FAILED", s_fail);
    }
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
