/*
 * mt6835 单元测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 覆盖：静止稳定性、读数连续有效与环绕合法域、is_ok、纯函数向量，
 * 以及人工手转一圈圈数守恒段（[manual]，显式选择才运行，不计入
 * 常规结果）。「拔线 10 ms 内 NOT OK」为人工上板项，记录在最终
 * 上板清单，不在本测试内自动化。
 */

#include "mt6835.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mt6835_test";

/* 静止稳定性：先等机械稳定（|vth| 持续低于阈值），再测 1 s 内
 * raw21 抖动 ≤ 2 count。摆杆晃动/刚手动转过轴都会表现为大范围 */
TEST_CASE("静止稳定性：1 s 内 raw21 抖动 ≤2 count", "[mt6835]")
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
        ESP_LOGW(TAG, "10 s 内未稳定（vth=%d mrad/s）——扶稳摆杆后重试",
                 mt6835_vth_mrad_s());
    }

    uint32_t min_r = 0xFFFFFFFFu, max_r = 0;
    for (int i = 0; i < 1000; i++) {
        uint32_t r = mt6835_last_raw();
        if (r < min_r) min_r = r;
        if (r > max_r) max_r = r;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(mt6835_is_ok());
    ESP_LOGI(TAG, "静止 raw21 范围 [%u, %u]（抖动 %u）",
             min_r, max_r, max_r - min_r);
    TEST_ASSERT_TRUE_MESSAGE(max_r - min_r <= 2, "静止抖动应 ≤ 2 count");
}

/* theta 合法域与速度有限性（2 s 连续读） */
TEST_CASE("theta 合法域 ±π 与速度有限性", "[mt6835]")
{
    for (int i = 0; i < 2000; i++) {
        int32_t th = mt6835_theta_urad();
        int32_t vth = mt6835_vth_mrad_s();
        TEST_ASSERT_TRUE_MESSAGE(th >= -3141593 && th <= 3141593,
                                 "theta 应在 ±π 内");
        TEST_ASSERT_TRUE_MESSAGE(vth > -1000000 && vth < 1000000,
                                 "vth 应为有限值");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(mt6835_is_ok());
}

/* 纯函数向量抽查（与宿主测试同源） */
TEST_CASE("纯函数向量", "[mt6835]")
{
    TEST_ASSERT_EQUAL_INT(0, mt6835_raw_to_urad(0, 0, false));
    TEST_ASSERT_EQUAL_INT(3141592, mt6835_raw_to_urad(1048576u, 0, false));
    TEST_ASSERT_EQUAL_INT(-3, mt6835_raw_to_urad(2097151u, 0, false));
    TEST_ASSERT_EQUAL_INT(-1570796, mt6835_raw_to_urad(524288u, 0, true));
    TEST_ASSERT_EQUAL_INT(1, mt6835_unwrap_diff(0, 2097151u));
    TEST_ASSERT_EQUAL_INT(392699, mt6835_counts_to_mrad_s(2097152, 16));
}

/* ---- A10：手转摆杆一整圈（提示 + 时间窗 + 自动判定）----
 * 圈数守恒判据：行程 ≥ 0.9 圈（确实扫过整圈范围），且净变化与最近
 * 整数圈之差 ≤ 3°（转满一圈后停在出发点角度）。注意转恰整数圈时
 * 多圈计数的净变化本就是 ±k 圈，不能按「净变化 ≈ 0」判定；检测到
 * 的整圈数 k 随结果打印，供与实际转动圈数人工核对（丢圈/多圈会
 * 表现为 k 不符；行程不足会表现为行程 < 0.9 圈）。 */
TEST_CASE("人工 A10：手转摆杆一圈圈数守恒", "[mt6835][manual]")
{
    const int secs = 15;
    const int32_t REV = 2097152;   /* 2^21 counts/圈 */

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
    int32_t k = (net >= 0) ? (net + REV / 2) / REV : (net - REV / 2) / REV;
    int32_t resid = net - k * REV;              /* 距整数圈的残余 */
    double resid_deg = (double)resid * 360.0 / (double)REV;

    ESP_LOGI(TAG, "结果：行程 %d counts（%.2f 圈），净变化 %d counts"
             "（%d 圈 + 残余 %+d counts ≈ %+.2f°）",
             excursion, (double)excursion / (double)REV, net, k, resid, resid_deg);

    if (excursion < REV / 10) {
        ESP_LOGI(TAG, "A10 判定：未检测到整圈转动（没转属正常——重跑本用例时转即可）");
    } else if (excursion < REV - REV / 10) {
        ESP_LOGW(TAG, "A10 判定：行程 %.2f 圈不足（需 ≥ 0.9 圈）——请转满一圈再试",
                 (double)excursion / (double)REV);
    } else if (resid > REV / 120 || resid < -REV / 120) {
        ESP_LOGW(TAG, "A10 判定：未回到出发点角度（残余 %+.2f° 超过 ±3°）——"
                 "请转满整数圈后停在同一位置再试", resid_deg);
    } else {
        ESP_LOGI(TAG, "A10 判定：圈数守恒 OK（检测到 %d 圈，残余 %+.2f°）", k, resid_deg);
    }
}
