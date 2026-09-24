/*
 * mt6835 — 上板自测
 *
 * CONFIG_MT6835_SELF_TEST=y 时编入，mt6835_init 完成后由 app_main
 * 调用。结束行 "SELF-TEST DONE: <n>/<total> PASS"。
 *
 * 覆盖：静止稳定性、读数连续有效与环绕合法域、is_ok、纯函数向量。
 * 「手转一圈的圈数守恒」与「拔线 10 ms 内 NOT OK」为人工上板项，
 * 记录在最终上板清单，不在本自测内自动化。
 */

#include "mt6835.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mt6835_selftest";

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

/* 1. 静止稳定性：1 s 内 raw21 抖动 ≤ 2 count */
static bool test_stationary_stability(void)
{
    uint32_t min_r = 0xFFFFFFFFu, max_r = 0;
    for (int i = 0; i < 1000; i++) {
        uint32_t r = mt6835_last_raw();
        if (r < min_r) min_r = r;
        if (r > max_r) max_r = r;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    CHECK(mt6835_is_ok(), "读取应持续成功");
    ESP_LOGI(TAG, "  静止 raw21 范围 [%u, %u]（抖动 %u）",
             min_r, max_r, max_r - min_r);
    CHECK(max_r - min_r <= 2, "静止抖动应 ≤ 2 count");
    return true;
}

/* 2. theta 合法域与速度有限性（2 s 连续读） */
static bool test_theta_domain(void)
{
    for (int i = 0; i < 2000; i++) {
        int32_t th = mt6835_theta_urad();
        int32_t vth = mt6835_vth_mrad_s();
        CHECK(th >= -3141593 && th <= 3141593, "theta 应在 ±π 内");
        CHECK(vth > -1000000 && vth < 1000000, "vth 应为有限值");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    CHECK(mt6835_is_ok(), "读取应持续成功");
    return true;
}

/* 3. 纯函数向量抽查（与宿主测试同源） */
static bool test_pure_vectors(void)
{
    CHECK(mt6835_raw_to_urad(0, 0, false) == 0, "raw 0 → 0");
    CHECK(mt6835_raw_to_urad(1048576u, 0, false) == 3141592, "半圈 → +π");
    CHECK(mt6835_raw_to_urad(2097151u, 0, false) == -3, "最大读数 → −3");
    CHECK(mt6835_raw_to_urad(524288u, 0, true) == -1570796, "invert 翻转");
    CHECK(mt6835_unwrap_diff(0, 2097151u) == 1, "跨零正转");
    CHECK(mt6835_counts_to_mrad_s(2097152, 16) == 392699, "速度换算");
    return true;
}

#define RUN_TEST(name) do { \
    if (name()) { \
        s_pass++; \
        ESP_LOGI(TAG, "  PASS  %s", #name); \
    } else { \
        s_fail++; \
    } \
} while (0)

int mt6835_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " MT6835 Self-Test");
    ESP_LOGI(TAG, "========================================");

    RUN_TEST(test_pure_vectors);
    RUN_TEST(test_stationary_stability);
    RUN_TEST(test_theta_domain);

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
