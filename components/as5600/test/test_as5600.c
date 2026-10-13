/*
 * as5600 单元测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * A 组（12 例）多圈计数纯逻辑：合成 raw 序列，无硬件依赖。
 * B 组（6 例）真实硬件：I2C 实时读取。
 * 人工引导段（1 例，[manual]）：手转 3 圈观察，仅显式选择时运行。
 *
 * B15/B16 判静止噪声与静止速度：跑 B 组时手勿碰轴。
 */

#include "as5600.h"
#include "bsp.h"
#include "settings.h"

#include <math.h>

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

static float test_upr(void)
{
    float upr = settings_units_per_rev();
    return (upr > 0.0f) ? upr : 40.0f;
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

/* 3 整圈 = +12288 counts = 120.0 mm——上板验收判据的纯逻辑等价 */
TEST_CASE("A7 正转 3 圈 → +12288 counts = 120 mm", "[as5600]")
{
    as5600_tracker_t t;
    uint16_t         raw = 1000;
    float            upr = test_upr();

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

    int32_t pos = sim_run(&t, &raw, 128, -64);
    TEST_ASSERT_EQUAL_INT(0, pos);
}

/* ================= B 组：真实硬件 ================= */

TEST_CASE("B13 I2C 总线在位 + as5600_init() OK", "[as5600]")
{
    TEST_ASSERT_NOT_NULL(bsp_get_i2c_bus());
    TEST_ASSERT_EQUAL(ESP_OK, as5600_init());
}

/* 只读（见 as5600.c 红线） */
TEST_CASE("B14 20 次实时读全部 12 bit 内且链路健康", "[as5600]")
{
    for (int i = 0; i < 20; i++) {
        uint16_t raw = as5600_last_raw();
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(4095, raw);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    TEST_ASSERT_TRUE_MESSAGE(as5600_is_ok(), "as5600_is_ok() 假——I2C 读失败");
    ESP_LOGI(TAG, "  last raw=%u, counts=%ld",
             as5600_last_raw(), (long)as5600_position_counts());
}

/* 静止噪声：20 样本峰峰值 ≤2 counts（验收判据见
 * docs/reviews/T8-acceptance.md），保持轴静止。
 *
 * 峰峰值按相对首样本的「展开差」度量（与 tracker 同一 ±2048 规则）：
 * 直接比 raw 会在轴恰好停在 0/4095 回绕点时报 4095 counts 的「噪声」
 * ——那里传感器合法地在 4095 与 0 之间交替。raw 极值也一并打印，
 * 让这种边界情形可见而非被吞掉。 */
TEST_CASE("B15 静止噪声峰峰值 ≤2 counts", "[as5600]")
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

    ESP_LOGI(TAG, "  噪声：raw %u..%u，展开 p-p=%ld counts",
             raw_min, raw_max, (long)pp);

    if ((uint16_t)(raw_max - raw_min) > 2 && pp <= 2) {
        ESP_LOGW(TAG, "  注：raw 极值跨 0/4095 边界（%u raw counts）——"
                      "轴停在回绕点上，展开噪声仍 ≤2",
                 (unsigned)(raw_max - raw_min));
    }

    TEST_ASSERT_TRUE_MESSAGE(pp <= 2, "静止噪声超 2 counts（是否碰了轴？）");
}

TEST_CASE("B16 静止速度 |v| ≤5 mm/s（16 ms 窗）", "[as5600]")
{
    vTaskDelay(pdMS_TO_TICKS(50));   /* 等速度窗填充 */

    float v = as5600_velocity_mm_s();
    ESP_LOGI(TAG, "  静止速度 = %.3f mm/s", (double)v);

    TEST_ASSERT_TRUE_MESSAGE(fabsf(v) <= 5.0f, "静止速度超 5 mm/s");
}

/* 位置/标度一致性：未发过原点偏移时 E = counts × units_per_rev / 4096 */
TEST_CASE("B17 E == counts × units_per_rev / 4096", "[as5600]")
{
    int32_t counts = as5600_position_counts();
    float   expect = (float)counts * test_upr() / (float)AS5600_COUNTS_PER_REV;
    float   actual = as5600_position_mm();

    ESP_LOGI(TAG, "  counts=%ld → 期望 %.3f mm，实测 %.3f mm",
             (long)counts, (double)expect, (double)actual);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, expect, actual);
}

/* 原点重置：上报位置跟随 set_origin()（保持 C 与 E 可比） */
TEST_CASE("B18 原点重置（设 100 / 回 0）", "[as5600]")
{
    as5600_set_origin(100.0f);
    float at100 = as5600_position_mm();
    ESP_LOGI(TAG, "  set_origin(100) 后 E=%.3f mm", (double)at100);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, at100);

    as5600_set_origin(0.0f);
    float at0 = as5600_position_mm();
    ESP_LOGI(TAG, "  set_origin(0) 后   E=%.3f mm", (double)at0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, at0);
}

/* ================= 人工引导段（= 上板清单 A9）================= */

TEST_CASE("人工：手转 3 圈观察 x_um 变化", "[as5600][manual]")
{
    const int   secs = 15;
    const float upr  = test_upr();

    ESP_LOGI(TAG, "请将电机轴整转 3 圈后回到出发点，现在开始（采样 %d s）", secs);
    ESP_LOGI(TAG, "(电机轴经同步带连着小车——转轴即移动小车)");

    float start = as5600_position_mm();

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "  t=%2ds  E=%.3f mm  (%.1f counts)",
                 sec, (double)as5600_position_mm(),
                 (double)as5600_position_counts());
    }

    float delta = as5600_position_mm() - start;

    ESP_LOGI(TAG, "结果：delta = %+.3f mm（3 圈应 ~±%.1f；1 圈 = %.1f mm）",
             (double)delta, (double)(3.0f * upr), (double)upr);
    ESP_LOGI(TAG, "A9 记录：x_um 随转轴变化 %s——记下转动方向与此符号的关系"
                  "（+/-），留待 C4 符号约定标定时交叉核对",
             delta >= 0 ? "为 +" : "为 −");

    if (fabsf(delta) < 1.0f) {
        ESP_LOGI(TAG, "(未检测到转动——没转属正常)");
    }
}
