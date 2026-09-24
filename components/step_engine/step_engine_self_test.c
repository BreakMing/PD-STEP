/*
 * Step Engine — 上板自测（速度模式）
 *
 * CONFIG_STEP_ENGINE_SELF_TEST=y 时编入固件，在 app_main 完成
 * step_engine_init 后运行。结束行 "SELF-TEST DONE: <n>/<total> PASS"。
 *
 * 纯函数向量（不依赖电机）+ ISR 级行为（间隔采样、看门狗、换向、
 * 急停、方向反转）。ISR 级用例只需脉冲发生器工作，电机可不使能、
 * 可不加 12V；位置计数与间隔采样均为逻辑量。
 */

#include "step_engine.h"
#include "vel_ramp.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "step_selftest";

#define STEPS(n) ((int64_t)(n) * VEL_RAMP_Q16_ONE)

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

/* 等待斜坡归零：vel_now == 0 且位置在确认窗内不再变化 */
static bool wait_stopped(int timeout_ms)
{
    for (int t = 0; t < timeout_ms; t += 10) {
        if (step_engine_vel_now_q16() == 0) {
            int32_t p1 = step_engine_position_steps();
            vTaskDelay(pdMS_TO_TICKS(50));
            int32_t p2 = step_engine_position_steps();
            if (p1 == p2 && step_engine_vel_now_q16() == 0) {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

static void settle(void)
{
    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ---- 1. 纯函数向量抽查（与宿主测试同源，不依赖硬件） ---- */
static bool test_pure_vectors(void)
{
    uint64_t v;
    uint32_t dt;
    bool     dchg;

    CHECK(vel_ramp_next(STEPS(100), true, STEPS(200), true, 2000, 5000,
                        false, &v, &dt, &dchg), "① 应继续运动");
    CHECK(v == 120ULL * VEL_RAMP_Q16_ONE, "① v' = 120 steps/s");
    CHECK(dt == 8333, "① dt = 8333");
    CHECK(!dchg, "① 无换向");

    CHECK(!vel_ramp_next(STEPS(5), true, 0, true, 2000, 5000,
                         false, &v, &dt, &dchg), "② 应停表");
    CHECK(v == 0, "② v' = 0");

    CHECK(vel_ramp_start_dt(2000) == 31622, "dt0 = 31622");
    return true;
}

/* ---- 2. 静止起步加速收敛：间隔单调不增，收敛 5000 µs ---- */
static bool test_accel_converge(void)
{
    static uint32_t buf[STEP_SELFTEST_MAX_SAMPLES];
    volatile int cnt = 0;

    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_set_decel(2000);
    step_engine_vel_set_wdt_us(0);

    step_selftest_start_sampling(buf, STEP_SELFTEST_MAX_SAMPLES, &cnt);
    step_engine_vel_target(STEPS(200));           /* 200 steps/s → 5000 µs */
    vTaskDelay(pdMS_TO_TICKS(800));
    step_selftest_stop_sampling();

    int n = cnt;
    CHECK(n >= 100 && n <= STEP_SELFTEST_MAX_SAMPLES, "样本数合理");
    /* 首个采样是第 1→2 拍间隔（首拍只置基准，dt0 不产生采样）。
     * 期望值用递推自身两步推导：v1 = a·dt0，v2 = v1 + a/v1，
     * 首间隔 = 1/v2。一阶递推在低速段比闭式 sqrt 剖面系统性偏快
     * （a=2000 时 ≈10.5 ms，闭式 1/v1 ≈ 15.8 ms，闭式两步间
     * 13.1 ms）——引擎以递推契约为准（vel_ramp 宿主测试锚定）。 */
    uint64_t v;
    uint32_t dt;
    bool     dchg;
    (void)vel_ramp_next(0, false, STEPS(200), true, 2000, 2000, false,
                        &v, &dt, &dchg);
    (void)vel_ramp_next(v, true, STEPS(200), true, 2000, 2000, false,
                        &v, &dt, &dchg);
    uint32_t expect = dt;
    ESP_LOGI(TAG, "  采样前 6 项：%u %u %u %u %u %u（n=%d，期望首拍 %u µs）",
             n > 0 ? buf[0] : 0, n > 1 ? buf[1] : 0, n > 2 ? buf[2] : 0,
             n > 3 ? buf[3] : 0, n > 4 ? buf[4] : 0, n > 5 ? buf[5] : 0,
             n, expect);
    CHECK(buf[0] >= expect - expect / 10 && buf[0] <= expect + expect / 10,
          "首采样应为 1/v2 量级");

    for (int i = 1; i < n; i++) {
        if (buf[i - 1] <= 5150) {
            /* 已进入收敛带（目标 5000 µs 平台）：此后名义降幅为 0，
             * 相邻 ISR 派发抖动即可翻单调，不变量从「单调」换成
             * 「稳定」，由下方末段断言覆盖 */
            break;
        }
        CHECK(buf[i] <= buf[i - 1] + 3, "加速期间间隔应单调不增（容 3 µs 抖动）");
    }
    for (int i = n - 20; i < n; i++) {
        CHECK(buf[i] > 4850 && buf[i] < 5150, "末段应稳定在 5000±150 µs");
    }

    step_engine_vel_halt();
    CHECK(wait_stopped(2000), "斜坡停应归零");
    return true;
}

/* ---- 3. 看门狗：断喂减速到 0，命令恢复继续跟踪（不需重启） ---- */
static bool test_watchdog_stop_and_recover(void)
{
    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_set_decel(2000);
    step_engine_vel_set_wdt_us(100000);           /* 100 ms */

    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(150));
    CHECK(step_engine_vel_now_q16() > 0, "喂狗期间应保持运动");

    /* 静默：不再发 vel_target，等待看门把速度减到 0
     * （100 ms 超时 + 2000/2000 减速 100 ms，2 s 裕量足够） */
    CHECK(wait_stopped(2000), "看门狗超时后应减速到 0");

    int32_t p1 = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    CHECK(step_engine_position_steps() == p1, "归零后位置应冻结");

    /* 命令恢复：以 50 Hz 持续喂狗（看门狗按命令流设计，单条命令
     * 只保 WDT_MS 窗口），恢复后应继续跟踪 */
    for (int i = 0; i < 15; i++) {
        step_engine_vel_target(STEPS(200));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    CHECK(step_engine_position_steps() > p1 + 30,
          "恢复命令后应继续运动（≈200 steps/s）");

    step_engine_vel_halt();
    CHECK(wait_stopped(2000), "收尾停");
    step_engine_vel_set_wdt_us(0);
    return true;
}

/* ---- 4. 换向经零速：恰一次 dt0 量级长拍，方向单调正确 ---- */
static bool test_reversal(void)
{
    static uint32_t buf[STEP_SELFTEST_MAX_SAMPLES];
    volatile int cnt = 0;

    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_set_wdt_us(0);

    /* 相位 A：+200，位置应单调不减 */
    step_engine_vel_target(STEPS(200));
    int32_t prev = step_engine_position_steps();
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(25));
        int32_t p = step_engine_position_steps();
        CHECK(p >= prev, "正向相位位置应单调不减");
        prev = p;
    }
    int32_t p_a = prev;

    /* 换向：−200 */
    step_selftest_start_sampling(buf, STEP_SELFTEST_MAX_SAMPLES, &cnt);
    step_engine_vel_target(-STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(1200));
    step_selftest_stop_sampling();

    CHECK(step_engine_vel_now_q16() < 0, "换向后速度应为负");

    int n = cnt;
    /* 过零邻域标志：换向必经近零速，出现 dt0 量级长拍（减速尾端
     * 也会产生长拍，不能按「恰一次」计数） */
    uint32_t max_dt = 0;
    for (int i = 0; i < n; i++) {
        if (buf[i] > max_dt) {
            max_dt = buf[i];
        }
    }
    CHECK(max_dt >= 30000, "过零处应出现 dt0 量级长拍（含 DIR setup）");

    for (int i = n - 20; i < n; i++) {
        CHECK(buf[i] > 4850 && buf[i] < 5150, "反向末段应稳定在 5000±150 µs");
    }

    /* 相位 B 稳态：位置单调不增 */
    prev = step_engine_position_steps();
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(25));
        int32_t p = step_engine_position_steps();
        CHECK(p <= prev, "反向相位位置应单调不增");
        prev = p;
    }
    CHECK(prev < p_a, "净位移应为负");

    step_engine_vel_halt();
    CHECK(wait_stopped(2000), "收尾停");
    return true;
}

/* ---- 5. halt 斜坡停 ---- */
static bool test_halt(void)
{
    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(300));
    CHECK(step_engine_vel_now_q16() > 0, "运动中");

    step_engine_vel_halt();
    CHECK(wait_stopped(2000), "halt 应斜坡归零");

    int32_t p = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    CHECK(step_engine_position_steps() == p, "halt 后位置应冻结");
    return true;
}

/* ---- 6. 急停：立即冻结，且之后引擎仍可用 ---- */
static bool test_emergency_stop(void)
{
    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(300));

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    int32_t p = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    CHECK(step_engine_position_steps() == p, "急停后不得再有脉冲");
    CHECK(step_engine_vel_now_q16() == 0, "急停后速度为 0");

    /* 急停后可重新启动 */
    step_engine_vel_target(STEPS(100));
    vTaskDelay(pdMS_TO_TICKS(300));
    CHECK(step_engine_position_steps() != p, "急停后引擎应可重启");

    step_engine_vel_halt();
    CHECK(wait_stopped(2000), "收尾停");
    return true;
}

/* ---- 7. 方向反转（X_INV 作用点）：开关立即翻转 DIR 电平（评审
 *      Critical 回归锚点：陈旧电平使首个同向运动转向与计数相反），
 *      且 +200 命令位置应减少 ---- */
static bool test_dir_invert(void)
{
    settle();
    int lvl0 = gpio_get_level(CONFIG_BSP_DIR_GPIO);
    step_engine_set_dir_invert(true);
    CHECK(gpio_get_level(CONFIG_BSP_DIR_GPIO) != lvl0,
          "翻转 X_INV 应立即翻转 DIR 电平");

    step_engine_vel_set_accel(2000);
    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(400));

    int32_t p0 = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    CHECK(step_engine_position_steps() < p0, "反转后 +命令 应使位置减少");

    step_engine_vel_halt();
    CHECK(wait_stopped(2000), "收尾停");
    step_engine_set_dir_invert(false);
    CHECK(gpio_get_level(CONFIG_BSP_DIR_GPIO) == lvl0, "恢复 X_INV 电平复原");
    return true;
}

/* ---- 运行器 ---- */

#define RUN_TEST(name) do { \
    step_selftest_reset_isr_cycles(); \
    if (name()) { \
        s_pass++; \
        ESP_LOGI(TAG, "  PASS  %s（ISR max %u：脉冲 %u / 递推 %u / 排拍 %u；" \
                 "avg：脉冲 %u / 递推 %u / 排拍 %u cycles）", \
                 #name, step_selftest_max_isr_cycles(), \
                 step_selftest_max_cycles_pulse(), \
                 step_selftest_max_cycles_ramp(), \
                 step_selftest_max_cycles_alarm(), \
                 step_selftest_avg_cycles_pulse(), \
                 step_selftest_avg_cycles_ramp(), \
                 step_selftest_avg_cycles_alarm()); \
    } else { \
        s_fail++; \
        ESP_LOGE(TAG, "  FAIL %s（ISR max %u：脉冲 %u / 递推 %u / 排拍 %u；" \
                 "avg：脉冲 %u / 递推 %u / 排拍 %u cycles）", \
                 #name, step_selftest_max_isr_cycles(), \
                 step_selftest_max_cycles_pulse(), \
                 step_selftest_max_cycles_ramp(), \
                 step_selftest_max_cycles_alarm(), \
                 step_selftest_avg_cycles_pulse(), \
                 step_selftest_avg_cycles_ramp(), \
                 step_selftest_avg_cycles_alarm()); \
    } \
} while (0)

void step_engine_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Step Engine Self-Test（速度模式）");
    ESP_LOGI(TAG, "========================================");

    RUN_TEST(test_pure_vectors);
    RUN_TEST(test_accel_converge);
    RUN_TEST(test_watchdog_stop_and_recover);
    RUN_TEST(test_reversal);
    RUN_TEST(test_halt);
    RUN_TEST(test_emergency_stop);
    RUN_TEST(test_dir_invert);

    step_engine_stop();

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, " ISR 最大执行周期数：%u", step_selftest_max_isr_cycles());
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
