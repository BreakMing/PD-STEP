/*
 * step_engine 单元测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 纯函数向量（不依赖电机）+ ISR 级行为（间隔采样、看门狗、换向、
 * 急停）。ISR 级用例只需脉冲发生器工作，电机可不使能、可不加驱动电压；
 * 位置计数与间隔采样均为逻辑量。
 *
 * 依赖 CONFIG_STEP_ENGINE_TEST_HOOKS=y（测试工程默认开启）：ISR
 * 间隔采样与周期计数插桩。
 */

#include "step_engine.h"
#include "vel_ramp.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "step_test";

#define STEPS(n) ((int64_t)(n) * VEL_RAMP_Q16_ONE)

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

/* ISR 周期统计（归因诊断：脉冲段 / 递推段 / 排拍段） */
static void log_isr_stats(void)
{
    ESP_LOGI(TAG, "  ISR max %u：脉冲 %u / 递推 %u / 排拍 %u；"
             "avg：脉冲 %u / 递推 %u / 排拍 %u cycles",
             step_selftest_max_isr_cycles(),
             step_selftest_max_cycles_pulse(),
             step_selftest_max_cycles_ramp(),
             step_selftest_max_cycles_alarm(),
             step_selftest_avg_cycles_pulse(),
             step_selftest_avg_cycles_ramp(),
             step_selftest_avg_cycles_alarm());
}

/* ---- 1. 纯函数向量抽查（与宿主测试同源，不依赖硬件） ---- */
TEST_CASE("纯函数向量", "[step]")
{
    uint64_t v;
    uint32_t dt;
    bool     dchg;

    TEST_ASSERT_TRUE(vel_ramp_next(STEPS(100), true, STEPS(200), true, 2000, 5000,
                                   false, &v, &dt, &dchg));
    TEST_ASSERT_EQUAL_UINT64(120ULL * VEL_RAMP_Q16_ONE, v);
    TEST_ASSERT_EQUAL_UINT32(8333, dt);
    TEST_ASSERT_FALSE(dchg);

    TEST_ASSERT_FALSE(vel_ramp_next(STEPS(5), true, 0, true, 2000, 5000,
                                    false, &v, &dt, &dchg));
    TEST_ASSERT_EQUAL_UINT64(0, v);

    TEST_ASSERT_EQUAL_UINT32(31622, vel_ramp_start_dt(2000));
}

/* ---- 2. 静止起步加速收敛：间隔单调不增，收敛 5000 µs ---- */
TEST_CASE("静止起步加速收敛到 5000 µs", "[step]")
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
    TEST_ASSERT_TRUE_MESSAGE(n >= 100 && n <= STEP_SELFTEST_MAX_SAMPLES, "样本数合理");
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
    TEST_ASSERT_TRUE_MESSAGE(buf[0] >= expect - expect / 10
                             && buf[0] <= expect + expect / 10,
                             "首采样应为 1/v2 量级");

    for (int i = 1; i < n; i++) {
        if (buf[i - 1] <= 5150) {
            /* 已进入收敛带（目标 5000 µs 平台）：此后名义降幅为 0，
             * 相邻 ISR 派发抖动即可翻单调，不变量从「单调」换成
             * 「稳定」，由下方末段断言覆盖 */
            break;
        }
        TEST_ASSERT_TRUE_MESSAGE(buf[i] <= buf[i - 1] + 3,
                                 "加速期间间隔应单调不增（容 3 µs 抖动）");
    }
    for (int i = n - 20; i < n; i++) {
        TEST_ASSERT_TRUE_MESSAGE(buf[i] > 4850 && buf[i] < 5150,
                                 "末段应稳定在 5000±150 µs");
    }

    step_engine_vel_halt();
    TEST_ASSERT_TRUE_MESSAGE(wait_stopped(2000), "斜坡停应归零");
    log_isr_stats();
}

/* ---- 3. 看门狗：断喂减速到 0，命令恢复继续跟踪（不需重启） ---- */
TEST_CASE("看门狗断喂减速与命令恢复", "[step]")
{
    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_set_decel(2000);
    step_engine_vel_set_wdt_us(100000);           /* 100 ms */

    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(150));
    TEST_ASSERT_TRUE_MESSAGE(step_engine_vel_now_q16() > 0, "喂狗期间应保持运动");

    /* 静默：不再发 vel_target，等待看门把速度减到 0
     * （100 ms 超时 + 2000/2000 减速 100 ms，2 s 裕量足够） */
    TEST_ASSERT_TRUE_MESSAGE(wait_stopped(2000), "看门狗超时后应减速到 0");

    int32_t p1 = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_TRUE_MESSAGE(step_engine_position_steps() == p1, "归零后位置应冻结");

    /* 命令恢复：以 50 Hz 持续喂狗（看门狗按命令流设计，单条命令
     * 只保 WDT_MS 窗口），恢复后应继续跟踪 */
    for (int i = 0; i < 15; i++) {
        step_engine_vel_target(STEPS(200));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    TEST_ASSERT_TRUE_MESSAGE(step_engine_position_steps() > p1 + 30,
                             "恢复命令后应继续运动（≈200 steps/s）");

    step_engine_vel_halt();
    TEST_ASSERT_TRUE_MESSAGE(wait_stopped(2000), "收尾停");
    step_engine_vel_set_wdt_us(0);
    log_isr_stats();
}

/* ---- 4. 换向经零速：恰一次 dt0 量级长拍，方向单调正确 ---- */
TEST_CASE("换向经零速（dt0 长拍 + 方向单调）", "[step]")
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
        TEST_ASSERT_TRUE_MESSAGE(p >= prev, "正向相位位置应单调不减");
        prev = p;
    }
    int32_t p_a = prev;

    /* 换向：−200 */
    step_selftest_start_sampling(buf, STEP_SELFTEST_MAX_SAMPLES, &cnt);
    step_engine_vel_target(-STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(1200));
    step_selftest_stop_sampling();

    TEST_ASSERT_TRUE_MESSAGE(step_engine_vel_now_q16() < 0, "换向后速度应为负");

    int n = cnt;
    /* 过零邻域标志：换向必经近零速，出现 dt0 量级长拍（减速尾端
     * 也会产生长拍，不能按「恰一次」计数） */
    uint32_t max_dt = 0;
    for (int i = 0; i < n; i++) {
        if (buf[i] > max_dt) {
            max_dt = buf[i];
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(max_dt >= 30000, "过零处应出现 dt0 量级长拍（含 DIR setup）");

    for (int i = n - 20; i < n; i++) {
        TEST_ASSERT_TRUE_MESSAGE(buf[i] > 4850 && buf[i] < 5150,
                                 "反向末段应稳定在 5000±150 µs");
    }

    /* 相位 B 稳态：位置单调不增 */
    prev = step_engine_position_steps();
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(25));
        int32_t p = step_engine_position_steps();
        TEST_ASSERT_TRUE_MESSAGE(p <= prev, "反向相位位置应单调不增");
        prev = p;
    }
    TEST_ASSERT_TRUE_MESSAGE(prev < p_a, "净位移应为负");

    step_engine_vel_halt();
    TEST_ASSERT_TRUE_MESSAGE(wait_stopped(2000), "收尾停");
    log_isr_stats();
}

/* ---- 5. halt 斜坡停 ---- */
TEST_CASE("halt 斜坡停与位置冻结", "[step]")
{
    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_TRUE_MESSAGE(step_engine_vel_now_q16() > 0, "运动中");

    step_engine_vel_halt();
    TEST_ASSERT_TRUE_MESSAGE(wait_stopped(2000), "halt 应斜坡归零");

    int32_t p = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_TRUE_MESSAGE(step_engine_position_steps() == p, "halt 后位置应冻结");

    /* 重定基 API（回零完成路径 E/C 同帧）：停止时可整定位置 */
    step_engine_set_position(p + 1234);
    TEST_ASSERT_EQUAL_INT(p + 1234, step_engine_position_steps());
    step_engine_set_position(p);
}

/* ---- 6. 急停：立即冻结，且之后引擎仍可用 ---- */
TEST_CASE("急停立即冻结且可重启", "[step]")
{
    settle();
    step_engine_vel_set_accel(2000);
    step_engine_vel_target(STEPS(200));
    vTaskDelay(pdMS_TO_TICKS(300));

    step_engine_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    int32_t p = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_TRUE_MESSAGE(step_engine_position_steps() == p, "急停后不得再有脉冲");
    TEST_ASSERT_EQUAL_INT(0, step_engine_vel_now_q16());

    /* 急停后可重新启动 */
    step_engine_vel_target(STEPS(100));
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_TRUE_MESSAGE(step_engine_position_steps() != p, "急停后引擎应可重启");

    step_engine_vel_halt();
    TEST_ASSERT_TRUE_MESSAGE(wait_stopped(2000), "收尾停");
    log_isr_stats();
}
