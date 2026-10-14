/*
 * as5600 多圈计数纯逻辑测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * A 组（12 例）：合成 raw 序列，无硬件依赖。硬件 B 组与人工段
 * 已随设备面移至 magnetic_encoder 测试组件（test_magnetic_encoder.c）。
 */

#include "as5600.h"

#include "unity.h"
#include "esp_log.h"

static const char *TAG = "as5600_test";

/* 推进模拟轴 step counts × steps 次，raw 按 4096 取模回绕（同真实传感器） */
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

/* ================= A 组：纯逻辑 ================= */

TEST_CASE("A1 首采样只播种，位移为 0", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);

    TEST_ASSERT_EQUAL_INT(0, as5600_tracker_update(&t, 100));
    TEST_ASSERT_TRUE(t.seeded);
    TEST_ASSERT_EQUAL_INT(0, t.pos_counts);
    TEST_ASSERT_EQUAL_INT(100, t.last_raw);
}

TEST_CASE("A2 相同 raw → 0 counts（静止）", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 100);

    TEST_ASSERT_EQUAL_INT(0, as5600_tracker_update(&t, 100));
}

TEST_CASE("A3 100→200→300 → +200 counts", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 100);

    as5600_tracker_update(&t, 200);
    TEST_ASSERT_EQUAL_INT(200, as5600_tracker_update(&t, 300));
}

/* 4090 → 10 正向跨界是 +16 counts，朴素相减会得 −4080 */
TEST_CASE("A4 4090→10 → +16 counts（正向跨界）", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 4090);

    TEST_ASSERT_EQUAL_INT(16, as5600_tracker_update(&t, 10));
}

TEST_CASE("A5 10→4090 → −16 counts（反向跨界）", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 10);

    TEST_ASSERT_EQUAL_INT(-16, as5600_tracker_update(&t, 4090));
}

/* 0/4095 边界抖动必须退化为 ±1 count 而非 ±4095——
 * 这是无需额外死区的原因 */
TEST_CASE("A6 0↔4095 抖动 ×8 → 累计漂移 ≤1 count", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 0);

    for (int i = 0; i < 8; i++) {
        uint16_t raw = ((i % 2) == 0) ? 4095 : 0;
        int32_t  pos = as5600_tracker_update(&t, raw);
        TEST_ASSERT_INT_WITHIN(1, 0, pos);
    }

    TEST_ASSERT_TRUE_MESSAGE(t.pos_counts <= 1 && t.pos_counts >= -1,
                             "边界抖动累计漂移超限");
}

/* 3 整圈 = +12288 counts = 120.0 mm @40 mm/rev（默认 $2；mm 解释
 * 归 cartpos，此处仅作计数换算示意）——上板验收判据的纯逻辑等价 */
TEST_CASE("A7 正转 3 圈 → +12288 counts = 120 mm", "[as5600]")
{
    as5600_tracker_t t;
    uint16_t         raw = 1000;
    const float      upr = 40.0f;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    int32_t pos = sim_run(&t, &raw, 192, 64);   /* 192 × 64 = 12288 */

    TEST_ASSERT_EQUAL_INT(3 * AS5600_COUNTS_PER_REV, pos);

    float mm = (float)pos * upr / (float)AS5600_COUNTS_PER_REV;
    ESP_LOGI(TAG, "  3 圈 = %ld counts = %.3f mm @ %.1f mm/rev",
             (long)pos, (double)mm, (double)upr);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f * upr, mm);
}

TEST_CASE("A8 反转 3 圈 → −12288 counts（符号对称）", "[as5600]")
{
    as5600_tracker_t t;
    uint16_t         raw = 1000;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    int32_t pos = sim_run(&t, &raw, 192, -64);

    TEST_ASSERT_EQUAL_INT(-3 * AS5600_COUNTS_PER_REV, pos);
}

/* diff 恰为 +2048（半圈）应原样通过——回绕修正是严格「> 2048」 */
TEST_CASE("A9 diff +2048 → +2048（不修正）", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 0);

    TEST_ASSERT_EQUAL_INT(2048, as5600_tracker_update(&t, 2048));
}

/* diff 恰为 −2047 应原样通过——修正严格「< −2047」，两个阈值
 * 恰好无缺口无重叠覆盖全部 4096 种 diff */
TEST_CASE("A10 diff −2047 → −2047（不修正）", "[as5600]")
{
    as5600_tracker_t t;
    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, 2048);

    TEST_ASSERT_EQUAL_INT(-2047, as5600_tracker_update(&t, 1));
}

TEST_CASE("A11 20 × +500 counts 快转 → +10000（不丢圈）", "[as5600]")
{
    as5600_tracker_t t;
    uint16_t         raw = 0;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    int32_t pos = sim_run(&t, &raw, 20, 500);

    TEST_ASSERT_EQUAL_INT(10000, pos);
}

TEST_CASE("A12 +2 圈再 −2 圈 → 0（方向对称）", "[as5600]")
{
    as5600_tracker_t t;
    uint16_t         raw = 3000;

    as5600_tracker_reset(&t);
    as5600_tracker_update(&t, raw);

    sim_run(&t, &raw, 128, 64);      /* +8192 = 2 圈 */
    TEST_ASSERT_EQUAL_INT(2 * AS5600_COUNTS_PER_REV, t.pos_counts);

    int32_t pos = sim_run(&t, &raw, 128, -64);   /* −8192 */
    TEST_ASSERT_EQUAL_INT(0, pos);
}
