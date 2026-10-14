/*
 * magnetic_encoder 硬件自测（官方 test/ 子目录约定，仅测试工程编入）
 *
 * [as5600] 段承接原 test_as5600.c B13–B18 与 manual A9：
 *  - B13/B14/B15 走类别实现面（init / last_raw / is_ok）；
 *  - B16 静止速度改为 counts/s 口径（5 mm/s ≡ 512 counts/s @40 mm/4096）；
 *  - B17/B18 转测 cartpos 解释层（比例一致性、原点重置）。
 * 实例由 test_app_main 创建（幂等 init 返回既有句柄）。
 */

#include <math.h>

#include "magnetic_encoder.h"
#include "magnetic_encoder_as5600.h"
#include "cartpos.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp.h"
#include "settings.h"

static const char *TAG = "me_test";

static magnetic_encoder_handle_t enc(void)
{
    magnetic_encoder_handle_t h = NULL;
    magnetic_encoder_as5600_config_t cfg = {
        .bus          = bsp_get_i2c_bus(),
        .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
        .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
        .invert_dir   = magnetic_encoder_as5600_invert_dir_default(),
    };
    TEST_ASSERT_EQUAL(ESP_OK, magnetic_encoder_as5600_init(&cfg, &h));
    return h;
}

TEST_CASE("B13 I2C 总线在位 + magnetic_encoder_as5600_init OK", "[as5600]")
{
    TEST_ASSERT_NOT_NULL(bsp_get_i2c_bus());
    TEST_ASSERT_NOT_NULL(enc());
}

/* 只读（红线见 as5600_hw.c） */
TEST_CASE("B14 20 次实时读全部 12 bit 内且链路健康", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    for (int i = 0; i < 20; i++) {
        uint16_t raw = magnetic_encoder_as5600_last_raw(h);
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(4095, raw);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    TEST_ASSERT_TRUE_MESSAGE(magnetic_encoder_is_ok(h),
                             "is_ok 假——I2C 读失败");
    ESP_LOGI(TAG, "  last raw=%u, counts=%lld",
             magnetic_encoder_as5600_last_raw(h),
             (long long)magnetic_encoder_position_counts(h));
}

/* 静止噪声：20 样本峰峰值 ≤2 counts（验收判据见
 * docs/reviews/T8-acceptance.md），保持轴静止。峰峰值按相对首样本
 * 的「展开差」度量（与 tracker 同一 ±2048 规则）。 */
TEST_CASE("B15 静止噪声峰峰值 ≤2 counts", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    int32_t  ref     = (int32_t)magnetic_encoder_as5600_last_raw(h);
    uint16_t raw_min = 0xFFFF;
    uint16_t raw_max = 0;
    int32_t  d_min   = 0;
    int32_t  d_max   = 0;

    for (int i = 0; i < 20; i++) {
        uint16_t raw = magnetic_encoder_as5600_last_raw(h);
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
        ESP_LOGW(TAG, "  注：raw 极值跨 0/4095 边界——轴停在回绕点上，"
                      "展开噪声仍 ≤2");
    }
    TEST_ASSERT_TRUE_MESSAGE(pp <= 2, "静止噪声超 2 counts（是否碰了轴？）");
}

/* 静止速度：协议口径 σ≤5 mm/s @40 mm/4096 ≡ 512 counts/s */
TEST_CASE("B16 静止速度 |v| ≤512 counts/s（16 ms 窗）", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    vTaskDelay(pdMS_TO_TICKS(50));   /* 等速度窗填充 */

    int32_t v = magnetic_encoder_velocity_counts_per_s(h);
    ESP_LOGI(TAG, "  静止速度 = %d counts/s", v);
    TEST_ASSERT_TRUE_MESSAGE(v <= 512 && v >= -512, "静止速度超 512 counts/s");
}

/* 位置/标度一致性（cartpos 解释层）：x_um = counts × $2 / 4096（未设原点时） */
TEST_CASE("B17 x_um == counts × $2 ÷ 4096（cartpos 层）", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    cartpos_set_origin_mm(0.0f);   /* 清偏移，令 x = 原始位置 */

    long counts = (long)magnetic_encoder_position_counts(h);
    float upr   = settings_units_per_rev();
    float expect = (float)counts * upr / 4096.0f;   /* AS5600 每转计数 */
    float actual = (float)(cartpos_position_um() / 1000.0);

    ESP_LOGI(TAG, "  counts=%ld → 期望 %.3f mm，实测 %.3f mm",
             counts, (double)expect, (double)actual);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expect, actual);
}

/* 原点重置（cartpos 层）：上报位置跟随 set_origin */
TEST_CASE("B18 原点重置（设 100 / 回 0，cartpos 层）", "[as5600]")
{
    cartpos_set_origin_mm(100.0f);
    float at100 = (float)(cartpos_position_um() / 1000.0);
    ESP_LOGI(TAG, "  set_origin(100) 后 x=%.3f mm", (double)at100);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, at100);

    cartpos_set_origin_mm(0.0f);
    float at0 = (float)(cartpos_position_um() / 1000.0);
    ESP_LOGI(TAG, "  set_origin(0) 后   x=%.3f mm", (double)at0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, at0);
}

/* ---- 人工引导段（= 上板清单 A9）---- */
TEST_CASE("人工：手转 3 圈观察 x_um 变化", "[as5600][manual]")
{
    const int   secs = 15;
    const float upr  = settings_units_per_rev();

    ESP_LOGI(TAG, "请将电机轴整转 3 圈后回到出发点，现在开始（采样 %d s）", secs);
    ESP_LOGI(TAG, "(电机轴经同步带连着小车——转轴即移动小车)");

    float start = (float)(cartpos_position_um() / 1000.0);

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "  t=%2ds  x=%.3f mm  (%lld counts)",
                 sec, (double)(cartpos_position_um() / 1000.0),
                 (long long)magnetic_encoder_position_counts(enc()));
    }

    float delta = (float)(cartpos_position_um() / 1000.0) - start;
    ESP_LOGI(TAG, "结果：delta = %+.3f mm（3 圈应 ~±%.1f；1 圈 = %.1f mm）",
             (double)delta, (double)(3.0f * upr), (double)upr);
    ESP_LOGI(TAG, "A9 记录：x_um 随转轴变化 %s——记下转动方向与符号关系，"
                  "留待符号约定标定交叉核对", delta >= 0 ? "为 +" : "为 −");

    if (fabsf(delta) < 1.0f) {
        ESP_LOGI(TAG, "(未检测到转动——没转属正常)");
    }
}
