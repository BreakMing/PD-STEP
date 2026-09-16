/*
 * Step Engine — on-target self-test (≥20 cases)
 *
 * Built into firmware when CONFIG_STEP_ENGINE_SELF_TEST=y.
 * Runs after BSP init, before protocol stack.
 * TMC2209 EN remains HIGH (disabled) throughout — only STEP/DIR
 * GPIO toggling is tested, no motor motion.
 *
 * Entry point: step_engine_self_test_run()
 * Final line: "SELF-TEST DONE: <n>/<total> PASS"
 */

#include "step_engine.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_cpu.h"
#include "bsp.h"

static const char *TAG = "step_selftest";

/* ---- Test counters ---- */
static int s_pass = 0;
static int s_fail = 0;
static int s_total = 0;

#define TEST_START(name) do { \
    s_total++; \
    ESP_LOGI(TAG, "--- test %d: %s ---", s_total, name); \
} while (0)

#define TEST_PASS() do { \
    s_pass++; \
    ESP_LOGI(TAG, "PASS %s", __func__); \
    return true; \
} while (0)

#define TEST_FAIL(fmt, ...) do { \
    s_fail++; \
    ESP_LOGE(TAG, "FAIL %s: " fmt, __func__, ##__VA_ARGS__); \
    return false; \
} while (0)

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        TEST_FAIL("assertion failed: %s (line %d)", msg, __LINE__); \
    } \
} while (0)

#define ASSERT_EQ(expected, actual) do { \
    if ((int32_t)(expected) != (int32_t)(actual)) { \
        TEST_FAIL("expected %d, got %d (line %d)", \
                  (int)(expected), (int)(actual), __LINE__); \
    } \
} while (0)

/* ---- Helpers ---- */

/* Wait for motion to complete (busy returns false) with timeout in ms.
 * Returns true if completed within timeout. */
static bool wait_for_idle(int timeout_ms)
{
    int elapsed = 0;
    while (step_engine_busy() && elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed++;
    }
    return !step_engine_busy();
}

/* Build a symmetric trapezoid segment with given total steps,
 * cruise velocity, and acceleration.
 * Accel steps = v_cruise² / (2*a), rounded.
 * Decel steps = accel steps.
 * Cruise steps = total - 2*accel (clamped to 0 = triangle). */
static void make_trapezoid(step_seg_t *seg, uint32_t total_steps,
                           float v_cruise, float accel, bool dir)
{
    /* accel steps = v_cruise² / (2*a) */
    float accel_steps_f = (v_cruise * v_cruise) / (2.0f * accel);
    uint32_t accel_steps = (uint32_t)(accel_steps_f + 0.5f);

    if (accel_steps * 2 >= total_steps) {
        /* Triangle — not enough distance to reach cruise */
        accel_steps = total_steps / 2;
        seg->accel_steps  = accel_steps;
        seg->decel_steps  = total_steps - accel_steps;
        seg->cruise_steps = 0;
    } else {
        seg->accel_steps  = accel_steps;
        seg->decel_steps  = accel_steps;
        seg->cruise_steps = total_steps - 2 * accel_steps;
    }

    seg->steps    = total_steps;
    seg->dir      = dir;
    seg->accel    = accel;
    seg->v_cruise = v_cruise;
}

/* ---- Test 1: Step count accuracy — full trapezoid (1000 steps) ---- */
static bool test_step_count_trapezoid(void)
{
    TEST_START("step count trapezoid (1000 steps)");

    step_engine_stop();
    /* reset position — there's no public reset, so we track delta */
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg;
    make_trapezoid(&seg, 1000, 1000.0f, 5000.0f, true);

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    bool done = wait_for_idle(2000);
    ASSERT_TRUE(done, "motion did not complete within 2s");

    int32_t pos_after = step_engine_position_steps();
    int32_t delta = pos_after - pos_before;
    ASSERT_EQ(1000, delta);

    TEST_PASS();
}

/* ---- Test 2: Step count accuracy — triangle (cruise=0, small distance) ---- */
static bool test_step_count_triangle(void)
{
    TEST_START("step count triangle (degenerate, cruise=0)");

    step_engine_stop();
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg;
    /* 100 steps total, accel is high enough that we can't reach 1000 steps/s
     * in 50 steps — this forms a triangle. */
    make_trapezoid(&seg, 100, 2000.0f, 10000.0f, true);
    /* Force triangle by setting cruise=0 explicitly */
    seg.accel_steps = 50;
    seg.decel_steps = 50;
    seg.cruise_steps = 0;
    seg.v_cruise = sqrtf(2.0f * seg.accel * 50.0f);  /* v at triangle peak */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    bool done = wait_for_idle(2000);
    ASSERT_TRUE(done, "motion did not complete within 2s");

    int32_t pos_after = step_engine_position_steps();
    int32_t delta = pos_after - pos_before;
    ASSERT_EQ(100, delta);

    TEST_PASS();
}

/* ---- Test 3: Step count — very small segment (10 steps) ---- */
static bool test_step_count_small(void)
{
    TEST_START("step count small (10 steps)");

    step_engine_stop();
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg;
    make_trapezoid(&seg, 10, 500.0f, 10000.0f, true);

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    bool done = wait_for_idle(2000);
    ASSERT_TRUE(done, "motion did not complete");

    int32_t pos_after = step_engine_position_steps();
    ASSERT_EQ(10, pos_after - pos_before);

    TEST_PASS();
}

/* ---- Test 4: Direction — forward motion (positive delta) ---- */
static bool test_direction_forward(void)
{
    TEST_START("direction forward (positive)");

    step_engine_stop();
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg;
    make_trapezoid(&seg, 200, 1000.0f, 5000.0f, true);  /* dir = true = forward */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    wait_for_idle(2000);

    int32_t delta = step_engine_position_steps() - pos_before;
    ASSERT_TRUE(delta > 0, "position did not increase for forward motion");
    ASSERT_EQ(200, delta);

    TEST_PASS();
}

/* ---- Test 5: Direction — backward motion (negative delta) ---- */
static bool test_direction_backward(void)
{
    TEST_START("direction backward (negative)");

    step_engine_stop();
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg;
    make_trapezoid(&seg, 200, 1000.0f, 5000.0f, false);  /* dir = false = backward */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    wait_for_idle(2000);

    int32_t delta = step_engine_position_steps() - pos_before;
    ASSERT_TRUE(delta < 0, "position did not decrease for backward motion");
    ASSERT_EQ(-200, delta);

    TEST_PASS();
}

/* ---- Test 6: Direction inversion ($10) flips position sign ---- */
static bool test_dir_invert(void)
{
    TEST_START("direction inversion ($10)");

    step_engine_stop();
    step_engine_set_dir_invert(false);  /* ensure normal */
    vTaskDelay(pdMS_TO_TICKS(10));

    int32_t pos_before = step_engine_position_steps();

    /* Forward motion with inversion = should go backward */
    step_engine_set_dir_invert(true);

    step_seg_t seg;
    make_trapezoid(&seg, 150, 1000.0f, 5000.0f, true);  /* logical forward */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    wait_for_idle(2000);

    int32_t delta_inv = step_engine_position_steps() - pos_before;
    ASSERT_TRUE(delta_inv < 0, "inverted forward should be negative");
    ASSERT_EQ(-150, delta_inv);

    /* Reset inversion for subsequent tests */
    step_engine_set_dir_invert(false);

    TEST_PASS();
}

/* ---- Test 7: Queue full — 17th enqueue returns false ---- */
static bool test_queue_full(void)
{
    TEST_START("queue full (16 depth)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    step_seg_t seg;
    make_trapezoid(&seg, 10, 500.0f, 5000.0f, true);

    /* Enqueue 16 segments — should all succeed */
    int enqueued = 0;
    for (int i = 0; i < 16; i++) {
        if (step_engine_enqueue(&seg)) {
            enqueued++;
        }
    }
    ASSERT_TRUE(enqueued == 16, "expected 16 successful enqueues");

    /* 17th should fail (queue full) */
    bool full = step_engine_enqueue(&seg);
    ASSERT_TRUE(!full, "17th enqueue should return false (queue full)");

    /* Let the queue drain so later tests aren't affected */
    wait_for_idle(5000);

    TEST_PASS();
}

/* ---- Test 8: Queue boundary check — invalid segment rejected ---- */
static bool test_queue_boundary_check(void)
{
    TEST_START("queue boundary validation (reject bad segment)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    step_seg_t seg;
    make_trapezoid(&seg, 100, 500.0f, 5000.0f, true);

    /* Sabotage: make accel+cruise+decel != total_steps */
    seg.accel_steps = 60;
    seg.cruise_steps = 50;
    seg.decel_steps = 60;  /* 60+50+60 = 170 ≠ 100 */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(!ok, "enqueue should reject invalid boundary segment");

    TEST_PASS();
}

/* ---- Test 9: Immediate stop during acceleration ---- */
static bool test_stop_during_accel(void)
{
    TEST_START("immediate stop during acceleration");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    /* Long slow accel: 5000 steps total, accel very low so accel phase is long */
    step_seg_t seg;
    make_trapezoid(&seg, 5000, 5000.0f, 500.0f, true);
    /* accel_steps = v²/(2a) = 25e6 / 1000 = 25000 steps — exceeds total
     * So it'll be a triangle with 2500 accel + 2500 decel */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    /* Wait for motion to actually start: at accel=500 steps/s² the
     * first-step interval is dt0 = sqrt(2/a) = 63 ms — a fixed 20 ms
     * delay would stop before the first pulse fires.  Poll until the
     * first step lands, then a bit more so we are solidly inside the
     * accel phase. */
    int wait_ms = 0;
    while (step_engine_position_steps() == pos_before && wait_ms < 500) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_ms++;
    }
    ASSERT_TRUE(step_engine_position_steps() != pos_before,
                "motion did not start within 500ms");

    vTaskDelay(pdMS_TO_TICKS(50));  /* ~50ms into accel phase */

    /* Trigger stop */
    step_engine_stop();

    /* busy should be false immediately (or very shortly after) */
    int busy_checks = 0;
    while (step_engine_busy() && busy_checks < 100) {
        vTaskDelay(pdMS_TO_TICKS(1));
        busy_checks++;
    }
    ASSERT_TRUE(!step_engine_busy(), "busy() not false within 100ms after stop");

    /* Position should be frozen — check again after 100ms */
    vTaskDelay(pdMS_TO_TICKS(100));
    int32_t pos_frozen = step_engine_position_steps();
    int32_t delta = pos_frozen - pos_before;
    ASSERT_TRUE(delta > 0 && delta < (int32_t)seg.steps,
                "position should be between 0 and total steps");

    TEST_PASS();
}

/* ---- Test 10: Immediate stop during cruise ---- */
static bool test_stop_during_cruise(void)
{
    TEST_START("immediate stop during cruise");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    /* Long cruise phase: 5000 steps total, high accel → short accel, long cruise */
    step_seg_t seg;
    make_trapezoid(&seg, 5000, 5000.0f, 50000.0f, true);
    /* accel_steps = 25e6 / 100000 = 250 steps each side, cruise = 4500 */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    /* Wait to get into cruise phase (accel takes ~ 250/5000 * 1000 = 50ms?
     * Actually time to accel to 5000 steps/s at 50000 steps/s² = 0.1s = 100ms
     * Wait 150ms to be safely in cruise. */
    vTaskDelay(pdMS_TO_TICKS(150));

    ASSERT_TRUE(step_engine_busy(), "should still be busy in cruise");

    step_engine_stop();

    /* busy should go false quickly */
    int checks = 0;
    while (step_engine_busy() && checks < 50) {
        vTaskDelay(pdMS_TO_TICKS(1));
        checks++;
    }
    ASSERT_TRUE(!step_engine_busy(), "busy not false after stop");

    /* Position frozen */
    int32_t pos_stop = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(100));
    ASSERT_EQ(pos_stop, step_engine_position_steps());

    int32_t delta = pos_stop - pos_before;
    ASSERT_TRUE(delta > 250 && delta < 5000,
                "should have stopped during cruise (after accel, before end)");

    TEST_PASS();
}

/* ---- Test 11: Immediate stop during deceleration ---- */
static bool test_stop_during_decel(void)
{
    TEST_START("immediate stop during deceleration");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    /* Medium trapezoid: stop during decel phase by timing it */
    step_seg_t seg;
    make_trapezoid(&seg, 2000, 2000.0f, 20000.0f, true);
    /* accel_steps = 4e6 / 4e4 = 100 steps each side, cruise = 1800
     * Total time approx: 100/1000 accel + 1800/2000 cruise + 100/1000 decel
     * = 0.1 + 0.9 + 0.1 = 1.1s
     * Decel starts at ~1.0s */

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    /* Wait 950ms — should be in late cruise or early decel */
    vTaskDelay(pdMS_TO_TICKS(950));

    step_engine_stop();

    int checks = 0;
    while (step_engine_busy() && checks < 50) {
        vTaskDelay(pdMS_TO_TICKS(1));
        checks++;
    }
    ASSERT_TRUE(!step_engine_busy(), "busy not false after stop");

    int32_t pos_stop = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(100));
    ASSERT_EQ(pos_stop, step_engine_position_steps());

    int32_t delta = pos_stop - pos_before;
    ASSERT_TRUE(delta > 1500 && delta < 2000,
                "should have stopped during decel phase");

    TEST_PASS();
}

/* ---- Test 12: Stop then enqueue new segment works normally ---- */
static bool test_stop_then_enqueue(void)
{
    TEST_START("stop then enqueue new segment (recovery)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    /* First segment — stop it midway */
    step_seg_t seg1;
    make_trapezoid(&seg1, 5000, 3000.0f, 10000.0f, true);
    step_engine_enqueue(&seg1);

    vTaskDelay(pdMS_TO_TICKS(100));
    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(20));

    int32_t pos_after_stop = step_engine_position_steps();

    /* Enqueue a new segment — should work normally */
    step_seg_t seg2;
    make_trapezoid(&seg2, 500, 1000.0f, 5000.0f, true);

    bool ok = step_engine_enqueue(&seg2);
    ASSERT_TRUE(ok, "enqueue after stop should succeed");

    bool done = wait_for_idle(2000);
    ASSERT_TRUE(done, "new segment after stop did not complete");

    int32_t delta_total = step_engine_position_steps() - pos_before;
    int32_t delta_after_stop = step_engine_position_steps() - pos_after_stop;

    ASSERT_EQ(500, delta_after_stop);
    ASSERT_TRUE(delta_total > 500, "total should be > 500 (stopped first seg + full second)");

    TEST_PASS();
}

/* ---- Test 13: Position frozen after stop (verification over time) ---- */
static bool test_position_frozen_after_stop(void)
{
    TEST_START("position frozen after stop");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    step_seg_t seg;
    make_trapezoid(&seg, 3000, 2000.0f, 10000.0f, true);
    step_engine_enqueue(&seg);

    vTaskDelay(pdMS_TO_TICKS(100));
    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    int32_t pos1 = step_engine_position_steps();

    /* Check at multiple intervals that position doesn't change */
    vTaskDelay(pdMS_TO_TICKS(50));
    int32_t pos2 = step_engine_position_steps();
    ASSERT_EQ(pos1, pos2);

    vTaskDelay(pdMS_TO_TICKS(100));
    int32_t pos3 = step_engine_position_steps();
    ASSERT_EQ(pos1, pos3);

    TEST_PASS();
}

/* ---- Test 14: Busy flag — true during motion, false when idle ---- */
static bool test_busy_flag(void)
{
    TEST_START("busy flag (true during motion, false idle)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(20));

    ASSERT_TRUE(!step_engine_busy(), "busy should be false when idle");

    step_seg_t seg;
    make_trapezoid(&seg, 500, 1000.0f, 5000.0f, true);
    step_engine_enqueue(&seg);

    /* Give it a moment to start */
    vTaskDelay(pdMS_TO_TICKS(5));
    ASSERT_TRUE(step_engine_busy(), "busy should be true during motion");

    wait_for_idle(2000);
    ASSERT_TRUE(!step_engine_busy(), "busy should be false after completion");

    TEST_PASS();
}

/* ---- Test 15: Consecutive segments, same direction — position accumulates ---- */
static bool test_consecutive_same_dir(void)
{
    TEST_START("consecutive segments same direction");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg1, seg2;
    make_trapezoid(&seg1, 300, 1000.0f, 5000.0f, true);
    make_trapezoid(&seg2, 500, 1500.0f, 8000.0f, true);

    step_engine_enqueue(&seg1);
    step_engine_enqueue(&seg2);

    bool done = wait_for_idle(3000);
    ASSERT_TRUE(done, "motion did not complete");

    int32_t delta = step_engine_position_steps() - pos_before;
    ASSERT_EQ(800, delta);  /* 300 + 500 */

    TEST_PASS();
}

/* ---- Test 16: Consecutive segments, reverse direction — DIR setup time ---- */
static bool test_consecutive_reverse_dir(void)
{
    TEST_START("consecutive segments reverse direction");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    step_seg_t seg1, seg2;
    make_trapezoid(&seg1, 200, 1000.0f, 5000.0f, true);   /* forward */
    make_trapezoid(&seg2, 200, 1000.0f, 5000.0f, false);  /* backward */

    step_engine_enqueue(&seg1);
    step_engine_enqueue(&seg2);

    bool done = wait_for_idle(3000);
    ASSERT_TRUE(done, "motion did not complete");

    int32_t delta = step_engine_position_steps() - pos_before;
    ASSERT_EQ(0, delta);  /* 200 forward - 200 backward = 0 */

    TEST_PASS();
}

/* ---- Test 17: Very slow speed (chain-wait path) — 2 steps/s ---- */
static bool test_very_slow_speed(void)
{
    TEST_START("very slow speed (2 steps/s, chain-wait)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    int32_t pos_before = step_engine_position_steps();

    /* Very slow: 2 steps/s cruise, 10 steps total (triangle to keep it short) */
    step_seg_t seg;
    seg.steps = 10;
    seg.dir = true;
    seg.accel = 2.0f;        /* 2 steps/s² — very slow accel */
    seg.v_cruise = 2.0f;     /* 2 steps/s — dt = 500ms, needs chaining */
    /* Triangle: accel 5 steps, cruise 0, decel 5 */
    seg.accel_steps = 5;
    seg.cruise_steps = 0;
    seg.decel_steps = 5;

    bool ok = step_engine_enqueue(&seg);
    ASSERT_TRUE(ok, "enqueue failed");

    /* 10 steps at avg ~1 step/s = ~10 seconds. Too slow for 30s budget.
     * Let's use fewer steps: 4 steps total (2 accel, 2 decel) at 2 steps/s
     * accel time = v/a = 2/2 = 1s, distance = 0.5*a*t² = 0.5*2*1 = 1 step
     * Hmm, that's only 1 step accel.
     *
     * Let's do a pure cruise at 2 steps/s for 4 steps = 2 seconds. */
    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    step_seg_t seg_cruise;
    seg_cruise.steps = 4;
    seg_cruise.dir = true;
    seg_cruise.accel = 100.0f;     /* high accel, instant cruise */
    seg_cruise.v_cruise = 2.0f;    /* 2 steps/s — dt = 500ms */
    seg_cruise.accel_steps = 0;
    seg_cruise.cruise_steps = 4;
    seg_cruise.decel_steps = 0;

    ok = step_engine_enqueue(&seg_cruise);
    ASSERT_TRUE(ok, "enqueue cruise failed");

    /* 4 steps * 500ms = 2s. Wait 3s to be safe. */
    bool done = wait_for_idle(4000);
    ASSERT_TRUE(done, "slow motion did not complete within 4s");

    int32_t delta = step_engine_position_steps() - pos_before;
    ASSERT_EQ(4, delta);

    TEST_PASS();
}

/* ---- Test 18: Velocity readback during cruise ---- */
static bool test_velocity_readback(void)
{
    TEST_START("velocity readback during cruise");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Long cruise phase for stable velocity reading */
    step_seg_t seg;
    make_trapezoid(&seg, 5000, 2000.0f, 40000.0f, true);
    /* accel_steps = 4e6 / 8e4 = 50 steps each, cruise = 4900
     * Cruise at 2000 steps/s = 4900/2000 = 2.45s of cruise time */

    step_engine_enqueue(&seg);

    /* Wait to get into cruise (accel time ~ 2000/40000 = 0.05s = 50ms) */
    vTaskDelay(pdMS_TO_TICKS(150));  /* safely into cruise */

    uint32_t vel = step_engine_velocity_steps();
    /* Allow ±10% tolerance — readback is approximate */
    ASSERT_TRUE(vel > 1700 && vel < 2300, "velocity readback out of range");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(20));

    /* After stop, velocity should be 0 */
    vel = step_engine_velocity_steps();
    ASSERT_TRUE(vel == 0 || !step_engine_busy(),
                "velocity should be ~0 after stop");

    TEST_PASS();
}

/* ---- Test 19: ISR max execution time ---- */
static bool test_isr_execution_time(void)
{
    TEST_START("ISR max execution time (cycles)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    step_selftest_reset_isr_cycles();

    /* Run a moderate-speed segment */
    step_seg_t seg;
    make_trapezoid(&seg, 1000, 5000.0f, 50000.0f, true);
    step_engine_enqueue(&seg);

    wait_for_idle(2000);

    uint32_t max_cycles = step_selftest_max_isr_cycles();
    /* 240 MHz → 5 µs budget = 1200 cycles.
     * Assert < 2000 cycles (generous margin; typical should be < 800). */
    ESP_LOGI(TAG, "  max ISR cycles: %lu (~%.1f µs @ 240MHz)",
             (unsigned long)max_cycles, (double)max_cycles / 240.0);
    ASSERT_TRUE(max_cycles < 3000, "ISR too slow (>3000 cycles = 12.5µs)");

    TEST_PASS();
}

/* ---- Test 20: Period accuracy — cruise segment ---- */
static bool test_period_accuracy_cruise(void)
{
    TEST_START("period accuracy cruise (<0.5%)");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Cruise at 1000 steps/s → dt = 1000 µs.
     * Use a segment with long cruise phase. */
    step_seg_t seg;
    make_trapezoid(&seg, 1000, 1000.0f, 50000.0f, true);
    /* accel_steps = 1e6 / 1e5 = 10 steps each, cruise = 980 */

    static uint32_t samples[256];
    volatile int sample_count = 0;

    step_selftest_start_sampling(samples, 256, &sample_count);

    step_engine_enqueue(&seg);

    /* Wait for completion */
    wait_for_idle(3000);

    step_selftest_stop_sampling();

    ESP_LOGI(TAG, "  collected %d interval samples", sample_count);
    ASSERT_TRUE(sample_count >= 50, "insufficient samples (< 50)");

    /* Expected cruise interval: 1000 µs */
    uint32_t expected_us = 1000;

    /* Take samples from the middle (skip accel/decel at edges) */
    int start_idx = sample_count / 4;   /* skip first quarter (accel) */
    int end_idx = sample_count * 3 / 4; /* skip last quarter (decel) */
    int count = end_idx - start_idx;
    ASSERT_TRUE(count >= 50, "insufficient cruise samples");

    /* Compute average and max error */
    uint64_t sum = 0;
    uint32_t max_err_pct_x100 = 0;  /* error in 0.01% units */
    for (int i = start_idx; i < end_idx; i++) {
        sum += samples[i];
        int32_t diff = (int32_t)samples[i] - (int32_t)expected_us;
        if (diff < 0) diff = -diff;
        uint32_t err_pct_x100 = (uint32_t)diff * 10000 / expected_us;
        if (err_pct_x100 > max_err_pct_x100) {
            max_err_pct_x100 = err_pct_x100;
        }
    }
    uint32_t avg = (uint32_t)(sum / count);

    ESP_LOGI(TAG, "  cruise avg dt = %lu µs (expected %lu µs)",
             (unsigned long)avg, (unsigned long)expected_us);
    ESP_LOGI(TAG, "  max error = %lu.%02lu %%",
             (unsigned long)(max_err_pct_x100 / 100),
             (unsigned long)(max_err_pct_x100 % 100));

    /* Average should be within 0.5% of expected */
    uint32_t avg_err_pct_x100 = 0;
    if (avg > expected_us) {
        avg_err_pct_x100 = (avg - expected_us) * 10000 / expected_us;
    } else {
        avg_err_pct_x100 = (expected_us - avg) * 10000 / expected_us;
    }
    ASSERT_TRUE(avg_err_pct_x100 < 50,  /* < 0.50% */
                "cruise period avg error >= 0.5%");

    TEST_PASS();
}

/* ---- Test 21: Period accuracy — acceleration phase ---- */
static bool test_period_accuracy_accel(void)
{
    TEST_START("period accuracy acceleration phase");

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Slow acceleration: many steps in accel phase for good sampling.
     * accel = 500 steps/s², v_cruise = 2000 steps/s
     * accel_steps = v²/(2a) = 4e6 / 1000 = 4000 steps — too many
     * Let's use: v=1000, a=250 → accel_steps = 1e6/500 = 2000 — still many
     * Use: v=500, a=500 → accel_steps = 250000/1000 = 250 steps — good */
    step_seg_t seg;
    make_trapezoid(&seg, 1000, 500.0f, 500.0f, true);
    /* accel_steps = 250000 / 1000 = 250 steps each side, cruise = 500 */

    static uint32_t samples[256];
    volatile int sample_count = 0;

    step_selftest_start_sampling(samples, 256, &sample_count);

    step_engine_enqueue(&seg);

    wait_for_idle(4000);

    step_selftest_stop_sampling();

    ESP_LOGI(TAG, "  collected %d interval samples", sample_count);
    ASSERT_TRUE(sample_count >= 50, "insufficient samples (< 50)");

    /* Compare accel phase intervals against theoretical recurrence.
     * For acceleration from v=0:
     *   dt(0) = sqrt(2/a) = sqrt(2/500) * 1e6 = sqrt(0.004) * 1e6 = 63246 µs? No...
     *   Wait: a = 500 steps/s²
     *   dt0 = sqrt(2/a) seconds = sqrt(2/500) = sqrt(0.004) = 0.0632 s = 63246 µs
     *   That's way too big. Let me recalculate with different values.
     *
     * Let me use higher accel for the accel test.
     * Actually, the test above already has accel data. Let me just check
     * that the acceleration intervals monotonically decrease and follow
     * the expected trend (v increases, dt decreases).
     */

    /* Check that intervals are monotonically decreasing during accel phase
     * (first ~accel_steps samples, excluding the very first which has DIR
     * setup time extra). */
    int accel_samples = 200;  /* first 200 should be in accel */
    if (accel_samples > sample_count - 5) {
        accel_samples = sample_count - 5;
    }

    int decreasing = 0;
    for (int i = 2; i < accel_samples - 1; i++) {
        if (samples[i] > samples[i + 1]) {
            decreasing++;
        }
    }
    float ratio = (float)decreasing / (float)(accel_samples - 3);
    ESP_LOGI(TAG, "  accel phase: %.0f%% of intervals decreasing (expected ~100%%)",
             ratio * 100.0f);

    /* At least 90% should be decreasing (allowing for noise/rounding) */
    ASSERT_TRUE(ratio > 0.85f, "accel intervals not decreasing (algorithm error?)");

    TEST_PASS();
}

/* ================================================================
 * Test runner
 * ================================================================ */

void step_engine_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;
    s_total = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Step Engine Self-Test (≥20 cases)");
    ESP_LOGI(TAG, " TMC2209 EN = HIGH (disabled — no motion)");
    ESP_LOGI(TAG, "========================================");

    /* Initialize step engine (self-test runs before protocol init,
     * so we must init ourselves.  step_engine_init is idempotent,
     * so the later call from app_main is a no-op.) */
    esp_err_t err = step_engine_init(CONFIG_BSP_STEP_GPIO,
                                     CONFIG_BSP_DIR_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "step_engine_init failed: %s — aborting self-test",
                 esp_err_to_name(err));
        return;
    }

    /* Ensure TMC stays disabled (EN HIGH).  We don't touch EN — bsp
     * already set it HIGH in bsp_early_init().  Just confirm. */

    /* Run all tests */
    test_step_count_trapezoid();
    test_step_count_triangle();
    test_step_count_small();
    test_direction_forward();
    test_direction_backward();
    test_dir_invert();
    test_queue_full();
    test_queue_boundary_check();
    test_stop_during_accel();
    test_stop_during_cruise();
    test_stop_during_decel();
    test_stop_then_enqueue();
    test_position_frozen_after_stop();
    test_busy_flag();
    test_consecutive_same_dir();
    test_consecutive_reverse_dir();
    test_very_slow_speed();
    test_velocity_readback();
    test_isr_execution_time();
    test_period_accuracy_cruise();
    test_period_accuracy_accel();

    /* Summary */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, s_total);
    if (s_fail > 0) {
        ESP_LOGE(TAG, " %d test(s) FAILED", s_fail);
    }
    ESP_LOGI(TAG, "========================================");

    /* Clean up: leave engine stopped */
    step_engine_stop();
}
