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

/* 1. 静止稳定性：先等机械稳定（|vth| 持续低于阈值），再测 1 s 内
 *    raw21 抖动 ≤ 2 count。摆杆晃动/刚手动转过轴都会表现为大范围 */
static bool test_stationary_stability(void)
{
    /* 等稳定：|vth| < 50 mrad/s 持续 300 ms，最多等 10 s */
    bool calm = false;
    for (int t = 0; t < 10000 && !calm; t += 10) {
        int32_t v = mt6835_vth_mrad_s();
        if (v > -50 && v < 50) {
            calm = true;
            for (int k = 0; k < 30; k++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                v = mt6835_vth_mrad_s();
                if (v <= -50 || v >= 50) {
                    calm = false;
                    break;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (!calm) {
        ESP_LOGW(TAG, "  10 s 内未稳定（vth=%d mrad/s）——扶稳摆杆后重试",
                 mt6835_vth_mrad_s());
    }

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

/* ---- A10：手转摆杆一整圈（提示 + 时间窗 + 自动判定，不计入 PASS 总数） ----
 * 圈数守恒：行程 ≥ 0.9 圈且净变化 ≤ 0.1 圈（转满一圈回到出发点）。 */
static void run_manual_circle_check(void)
{
    const int secs = 15;
    const int32_t REV = 2097152;   /* 2^21 counts/圈 */

    ESP_LOGI(TAG, "============ MANUAL CHECK（= 上板清单 A10）===========");
    ESP_LOGI(TAG, "请将摆杆缓慢转一整圈后回到出发点，现在开始（采样 %d s）", secs);

    int32_t start = mt6835_position_counts();
    int32_t lo = start, hi = start;

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int32_t p = mt6835_position_counts();
        if (p < lo) lo = p;
        if (p > hi) hi = p;
        ESP_LOGI(TAG, "  t=%2ds  pos=%+d counts（theta=%d µrad）",
                 sec, p, mt6835_theta_urad());
    }

    int32_t net = mt6835_position_counts() - start;
    int32_t excursion = hi - lo;

    ESP_LOGI(TAG, "MANUAL RESULT: 行程 %d counts（%.2f 圈），净变化 %d counts",
             excursion, (double)excursion / (double)REV, net);

    if (excursion >= REV - REV / 10 && (net <= REV / 10 && net >= -REV / 10)) {
        ESP_LOGI(TAG, "A10 判定：圈数守恒 OK");
    } else if (excursion < REV / 10) {
        ESP_LOGI(TAG, "A10 判定：未检测到整圈转动（没转属正常——重跑自测时转即可）");
    } else {
        ESP_LOGW(TAG, "A10 判定：行程够了但净变化 %d 超差——请转满一圈后停在同一位置再试",
                 net);
    }
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

    run_manual_circle_check();   /* A10：提示 + 自动判定，不计入总数 */

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
