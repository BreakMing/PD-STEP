/*
 * magnetic_encoder 硬件自测（官方 test/ 子目录约定，仅测试工程编入）
 *
 * [cart] 段为小车位板载 MT6835（rev2 接替 AS5600 职责）：
 *  - B13 双实例在位、B14 读数合法域、B15 静止噪声（counts 口径）、
 *    B16 静止速度（5 mm/s ≡ 262144 counts/s @40 mm/2^21）；
 *  - B17/B18 转测 cartpos 解释层（比例一致性、原点重置）。
 * [mt6835] 段为摆角位外接 MT6835（静止稳定性、theta 合法域
 * 与 poleang 解释层对拍、A10 手动段）。
 * 实例由 test_app_main 组装（init 不幂等，每台位一次）。
 */

#include <math.h>

#include "magnetic_encoder.h"
#include "magnetic_encoder_mt6835.h"
#include "mt6835_urad.h"
#include "cartpos.h"
#include "poleang.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp.h"
#include "settings.h"
#include "params.h"

static const char *TAG = "me_test";

/* 实例由 test_app_main 组装（init 不幂等，每台位一次）——本文件
 * 经这对 accessor 共享句柄 */
extern magnetic_encoder_handle_t test_me_cart_enc(void);
extern magnetic_encoder_handle_t test_me_pole_enc(void);

static magnetic_encoder_handle_t enc(void)
{
    return test_me_cart_enc();
}

static magnetic_encoder_handle_t mt6835_enc(void)
{
    return test_me_pole_enc();
}

TEST_CASE("B13 小车位 SPI 在位 + 双实例 init OK", "[cart]")
{
    TEST_ASSERT_NOT_NULL(enc());
    TEST_ASSERT_NOT_NULL(mt6835_enc());
    /* 双实例同时在线（rev2 主线：同实现两实例各归其主） */
    TEST_ASSERT_TRUE(magnetic_encoder_is_ok(enc()));
    TEST_ASSERT_TRUE(magnetic_encoder_is_ok(mt6835_enc()));
}

/* 只读（帧校验含全零/CRC 拒收，见 mt6835_hw.c） */
TEST_CASE("B14 20 次实时读全部 21 bit 内且链路健康", "[cart]")
{
    magnetic_encoder_handle_t h = enc();
    for (int i = 0; i < 20; i++) {
        uint32_t raw = magnetic_encoder_mt6835_last_raw(h);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(MT6835_COUNTS_PER_REV - 1, raw);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    TEST_ASSERT_TRUE_MESSAGE(magnetic_encoder_is_ok(h),
                             "is_ok 假——SPI 读失败");
    ESP_LOGI(TAG, "  last raw=%u, counts=%lld",
             magnetic_encoder_mt6835_last_raw(h),
             (long long)magnetic_encoder_position_counts(h));
}

/* 静止噪声：20 样本峰峰值 ≤2 count。峰峰值按相对首样本的
 * 「展开差」度量（±cpr/2 规则，与 tracker 同式——磁铁停在
 * 0/2^21−1 边界时 raw 极值差虚高，展开差仍 ≤2）。 */
TEST_CASE("B15 静止噪声峰峰值 ≤2 count", "[cart]")
{
    magnetic_encoder_handle_t h = enc();
    int32_t  ref     = (int32_t)magnetic_encoder_mt6835_last_raw(h);
    uint32_t raw_min = 0xFFFFFFFFu;
    uint32_t raw_max = 0;
    int32_t  d_min   = 0;
    int32_t  d_max   = 0;

    for (int i = 0; i < 20; i++) {
        uint32_t raw = magnetic_encoder_mt6835_last_raw(h);
        if (raw < raw_min) raw_min = raw;
        if (raw > raw_max) raw_max = raw;

        int32_t d = (int32_t)raw - ref;
        if (d > (int32_t)(MT6835_COUNTS_PER_REV / 2)) {
            d -= (int32_t)MT6835_COUNTS_PER_REV;
        } else if (d < -(int32_t)(MT6835_COUNTS_PER_REV / 2 - 1)) {
            d += (int32_t)MT6835_COUNTS_PER_REV;
        }
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    int32_t pp = d_max - d_min;
    ESP_LOGI(TAG, "  噪声：raw %u..%u，展开 p-p=%ld counts",
             raw_min, raw_max, (long)pp);

    if ((raw_max - raw_min) > 2 && pp <= 2) {
        ESP_LOGW(TAG, "  注：raw 极值跨 0/2^21−1 边界——轴停在回绕点上，"
                      "展开噪声仍 ≤2");
    }
    TEST_ASSERT_TRUE_MESSAGE(pp <= 2, "静止噪声超 2 count（是否碰了轴？）");
}

/* 静止速度：协议口径 σ≤5 mm/s @40 mm/2^21 ≡ 262144 counts/s */
TEST_CASE("B16 静止速度 |v| ≤262144 counts/s（16 ms 窗）", "[cart]")
{
    magnetic_encoder_handle_t h = enc();
    vTaskDelay(pdMS_TO_TICKS(50));   /* 等速度窗填充 */

    int32_t v = magnetic_encoder_velocity_counts_per_s(h);
    ESP_LOGI(TAG, "  静止速度 = %d counts/s", v);
    TEST_ASSERT_TRUE_MESSAGE(v <= 262144 && v >= -262144,
                             "静止速度超 262144 counts/s");
}

/* 位置/标度一致性（cartpos 解释层）：x_um = counts × 每转行程 / 2^21（未设原点时） */
TEST_CASE("B17 x_um == counts × 每转行程 ÷ 2^21（cartpos 层）", "[cart]")
{
    magnetic_encoder_handle_t h = enc();
    cartpos_set_origin_mm(0.0f);   /* 清偏移，令 x = 原始位置 */

    long long counts = magnetic_encoder_position_counts(h);
    float upr   = settings_units_per_rev();
    float expect = (float)((double)counts * upr / 2097152.0);
    float actual = (float)(cartpos_position_um() / 1000.0);

    ESP_LOGI(TAG, "  counts=%lld → 期望 %.3f mm，实测 %.3f mm",
             counts, (double)expect, (double)actual);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expect, actual);
}

/* 原点重置（cartpos 层）：上报位置跟随 set_origin */
TEST_CASE("B18 原点重置（设 100 / 回 0，cartpos 层）", "[cart]")
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
TEST_CASE("人工：手转 3 圈观察 x_um 变化", "[cart][manual]")
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

/* ================= [mt6835] 段（承接原 test_mt6835.c 硬件用例） ================= */

/* 静止稳定性：等 |v| < 50 mrad/s（≡ 16700 counts/s）持续 300 ms，
 * 再测 1 s 内 raw21 抖动 ≤2 count。 */
TEST_CASE("静止稳定性：1 s 内 raw21 抖动 ≤2 count", "[mt6835]")
{
    magnetic_encoder_handle_t h = mt6835_enc();
    bool calm = false;
    for (int t = 0; t < 10000 && !calm; t += 10) {
        int32_t v = magnetic_encoder_velocity_counts_per_s(h);
        if (v > -16700 && v < 16700) {
            calm = true;
            for (int k = 0; k < 30; k++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                v = magnetic_encoder_velocity_counts_per_s(h);
                if (v <= -16700 || v >= 16700) {
                    calm = false;
                    break;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (!calm) {
        ESP_LOGW(TAG, "10 s 内未稳定（%d counts/s）——扶稳摆杆后重试",
                 magnetic_encoder_velocity_counts_per_s(h));
    }

    uint32_t min_r = 0xFFFFFFFFu, max_r = 0;
    for (int i = 0; i < 1000; i++) {
        uint32_t r = magnetic_encoder_mt6835_last_raw(h);
        if (r < min_r) min_r = r;
        if (r > max_r) max_r = r;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(magnetic_encoder_is_ok(h));
    ESP_LOGI(TAG, "静止 raw21 范围 [%u, %u]（抖动 %u）",
             min_r, max_r, max_r - min_r);
    TEST_ASSERT_TRUE_MESSAGE(max_r - min_r <= 2, "静止抖动应 ≤ 2 count");
}

/* theta 合法域与速度有限性（2 s 连续读，poleang 解释层）+
 * 算术对拍：zero=0/inv=0 下 poleang 与 mt6835_urad 纯函数同值
 * （Review Focus 2 的锚——常量算术逐位一致）。 */
TEST_CASE("theta 合法域 ±π 与解释层对拍", "[mt6835]")
{
    fw_params_set("POLE_ZERO_URAD", 0);
    fw_params_set("POLE_INV", 0);
    magnetic_encoder_handle_t h = mt6835_enc();

    for (int i = 0; i < 2000; i++) {
        int32_t th = poleang_theta_urad();
        int32_t vth = poleang_vth_mrad_s();
        TEST_ASSERT_TRUE_MESSAGE(th >= -3141593 && th <= 3141593,
                                 "theta 应在 ±π 内");
        TEST_ASSERT_TRUE_MESSAGE(vth > -1000000 && vth < 1000000,
                                 "vth 应为有限值");
        if (i % 500 == 0) {
            /* 两读竞态：theta 与 last_raw 各自取锁，两次读取间 1 kHz
             * 采样可插入新样本（运动中恰跨 ±π 环绕点更甚）——按
             * ±1 count（≈3 µrad）环绕感知容差断言 */
            int32_t expect = mt6835_raw_to_urad(
                magnetic_encoder_mt6835_last_raw(h), 0, false);
            int64_t d = (int64_t)expect - th;
            if (d > 3141593) {
                d -= 6283185;
            } else if (d < -3141593) {
                d += 6283185;
            }
            TEST_ASSERT_TRUE_MESSAGE(d >= -4 && d <= 4,
                                     "对拍差超 ±1 count 容差");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(magnetic_encoder_is_ok(h));
}

/* ---- A10：手转摆杆一整圈（圈数守恒判定逻辑与原用例一致） ---- */
TEST_CASE("人工 A10：手转摆杆一圈圈数守恒", "[mt6835][manual]")
{
    magnetic_encoder_handle_t h = mt6835_enc();
    const int secs = 15;
    const int32_t REV = 2097152;

    ESP_LOGI(TAG, "请将摆杆缓慢转一整圈后回到出发点，现在开始（采样 %d s）", secs);

    int32_t start = (int32_t)magnetic_encoder_position_counts(h);
    int32_t lo = start, hi = start;

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int32_t p = (int32_t)magnetic_encoder_position_counts(h);
        if (p < lo) lo = p;
        if (p > hi) hi = p;
        ESP_LOGI(TAG, "  t=%2ds  pos=%+d counts（theta=%d µrad）",
                 sec, p, poleang_theta_urad());
    }

    int32_t net = (int32_t)magnetic_encoder_position_counts(h) - start;
    int32_t excursion = hi - lo;
    int32_t k = (net >= 0) ? (net + REV / 2) / REV : (net - REV / 2) / REV;
    int32_t resid = net - k * REV;
    double resid_deg = (double)resid * 360.0 / (double)REV;

    ESP_LOGI(TAG, "结果：行程 %d counts（%.2f 圈），净变化 %d counts"
                  "（%d 圈 + 残余 %+d counts ≈ %+.2f°）",
             excursion, (double)excursion / (double)REV, net, k, resid, resid_deg);

    if (excursion < REV / 10) {
        ESP_LOGI(TAG, "A10 判定：未检测到整圈转动（没转属正常）");
    } else if (excursion < REV - REV / 10) {
        ESP_LOGW(TAG, "A10 判定：行程不足——请转满一圈再试");
    } else if (resid > REV / 120 || resid < -REV / 120) {
        ESP_LOGW(TAG, "A10 判定：未回到出发点角度（残余 %+.2f° 超过 ±3°）",
                 resid_deg);
    } else {
        ESP_LOGI(TAG, "A10 判定：圈数守恒 OK（检测到 %d 圈）", k);
    }
}
