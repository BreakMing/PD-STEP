/*
 * settings 单元测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 5 参数表：默认值、set/get 往返、非法值矩阵、交叉检查、持久化、
 * 通知、派生值、坏 NVS 回落。改动状态的用例结尾恢复默认，保证
 * 任意顺序/单独执行都不留脏状态。
 */

#include "settings.h"

#include <string.h>
#include <math.h>

#include "unity.h"
#include "nvs.h"
#include "nvs_flash.h"

/* ---- 通知捕获 ---- */
static int  s_notify_idx = -1;
static float s_notify_val = 0.0f;
static int  s_notify_count = 0;

static void test_notify_cb(int idx, float new_value)
{
    s_notify_idx = idx;
    s_notify_val = new_value;
    s_notify_count++;
}

static void notify_reset(void)
{
    s_notify_idx = -1;
    s_notify_val = 0.0f;
    s_notify_count = 0;
}

/* ---- 用例 ---- */

/* 1. 默认值（含微步默认 8） */
TEST_CASE("默认值抽查（含微步 8）", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 8.0f, settings_get(SET_MICROSTEP));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 200.0f, settings_get(SET_FULL_STEP));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, settings_get(SET_MM_PER_REV));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 800.0f, settings_get(SET_RUN_MA));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 400.0f, settings_get(SET_HOLD_MA));
}

/* 2–6. 合法 set→get 往返 */
TEST_CASE("set/get 往返：微步", "[settings]")
{
    settings_status_t st = settings_set(SET_MICROSTEP, 16.0f);
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 16.0f, settings_get(SET_MICROSTEP));
    settings_reset_defaults();
}

TEST_CASE("set/get 往返：每转整步数", "[settings]")
{
    settings_status_t st = settings_set(SET_FULL_STEP, 400.0f);
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 400.0f, settings_get(SET_FULL_STEP));
    settings_reset_defaults();
}

TEST_CASE("set/get 往返：每转行程", "[settings]")
{
    settings_status_t st = settings_set(SET_MM_PER_REV, 36.0f);
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 36.0f, settings_get(SET_MM_PER_REV));
    settings_reset_defaults();
}

TEST_CASE("set/get 往返：运行电流", "[settings]")
{
    settings_status_t st = settings_set(SET_RUN_MA, 1500.0f);
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1500.0f, settings_get(SET_RUN_MA));
    settings_reset_defaults();
}

TEST_CASE("set/get 往返：保持电流", "[settings]")
{
    /* 运行电流处于默认 800，600 合法 */
    settings_status_t st = settings_set(SET_HOLD_MA, 600.0f);
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 600.0f, settings_get(SET_HOLD_MA));
    settings_reset_defaults();
}

/* 7. 非法：微步 */
TEST_CASE("非法值：微步", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 7.0f));   /* 非 2^n */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 0.0f));   /* < 1 */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 512.0f)); /* > 256 */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 8.5f));   /* 非整数 */
}

/* 8. 非法：每转整步数 */
TEST_CASE("非法值：每转整步数", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_FULL_STEP, 300.0f)); /* 非 200/400 */
}

/* 9. 非法：每转行程 */
TEST_CASE("非法值：每转行程", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_MM_PER_REV, 0.0f));   /* == 0 */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_MM_PER_REV, 501.0f)); /* > 500 */
}

/* 10. 非法：运行电流 */
TEST_CASE("非法值：运行电流", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_RUN_MA, 0.0f));      /* == 0 */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_RUN_MA, 2001.0f));   /* > 2000 */
}

/* 11. 交叉检查：SET_HOLD_MA > SET_RUN_MA 拒绝 */
TEST_CASE("交叉检查：保持电流 > 运行电流 拒绝", "[settings]")
{
    settings_reset_defaults();  /* 运行电流 = 800 */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_HOLD_MA, 1000.0f)); /* > 运行电流 800 */
}

/* 11b. 交叉检查：运行电流收缩到保持电流之下（反向） */
TEST_CASE("交叉检查：运行电流 < 保持电流 拒绝，收缩到保持电流为界", "[settings]")
{
    settings_reset_defaults();  /* 运行电流 = 800，保持电流 = 400 */
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SET_RUN_MA, 200.0f)); /* < 保持电流 400 */
    /* 收缩到保持电流即可 */
    TEST_ASSERT_EQUAL(SETTINGS_OK, settings_set(SET_RUN_MA, 400.0f));
    settings_reset_defaults();
}

/* 12. 索引超范围 */
TEST_CASE("索引超范围拒绝", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(SETTINGS_COUNT, 1.0f));
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(-1, 1.0f));
    TEST_ASSERT_EQUAL(SETTINGS_INVALID, settings_set(100, 1.0f));
}

/* 13. 持久化：set → reload → 仍相同 */
TEST_CASE("持久化：set 后 reload 仍相同", "[settings]")
{
    settings_reset_defaults();
    settings_set(SET_RUN_MA, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, settings_get(SET_RUN_MA));

    settings_status_t st = settings_reload();
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, settings_get(SET_RUN_MA));
    settings_reset_defaults();
}

/* 13b. 冷启动前向引用回归（沿袭 T11 板上发现）：
 * 载入必须对完整存储集校验，而非半载入状态。直接写入合法对
 * p3=1500/p4=600，内存当前为 p3=500/p4=400，reload 必须接受。 */
TEST_CASE("冷启动前向引用回归（T11）", "[settings]")
{
    settings_reset_defaults();
    settings_set(SET_RUN_MA, 500.0f);    /* NVS: p3=500, p4=400 */

    nvs_handle_t handle;
    TEST_ASSERT_TRUE(nvs_open("cart", NVS_READWRITE, &handle) == ESP_OK);
    uint32_t raw;
    float v;
    v = 1500.0f; memcpy(&raw, &v, sizeof(raw)); nvs_set_u32(handle, "p3", raw);
    v = 600.0f;  memcpy(&raw, &v, sizeof(raw)); nvs_set_u32(handle, "p4", raw);
    nvs_commit(handle);
    nvs_close(handle);

    TEST_ASSERT_EQUAL(SETTINGS_OK, settings_reload());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1500.0f, settings_get(SET_RUN_MA));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 600.0f, settings_get(SET_HOLD_MA));

    settings_reset_defaults();
}

/* 14. 恢复默认 */
TEST_CASE("恢复默认", "[settings]")
{
    settings_set(SET_MICROSTEP, 16.0f);
    settings_set(SET_RUN_MA, 1200.0f);

    settings_status_t st = settings_reset_defaults();
    TEST_ASSERT_EQUAL(SETTINGS_OK, st);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 8.0f, settings_get(SET_MICROSTEP));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 800.0f, settings_get(SET_RUN_MA));
}

/* 15. set 触发通知 */
TEST_CASE("set 触发通知", "[settings]")
{
    settings_reset_defaults();
    notify_reset();
    settings_subscribe(test_notify_cb);

    settings_set(SET_MICROSTEP, 16.0f);
    TEST_ASSERT_EQUAL_INT(1, s_notify_count);
    TEST_ASSERT_EQUAL_INT(SET_MICROSTEP, s_notify_idx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 16.0f, s_notify_val);

    notify_reset();
    settings_reset_defaults();
}

/* 16. 恢复默认触发通知 */
TEST_CASE("恢复默认触发通知", "[settings]")
{
    settings_reset_defaults();
    settings_set(SET_MICROSTEP, 16.0f);
    settings_set(SET_RUN_MA, 1000.0f);
    notify_reset();

    settings_reset_defaults();
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(2, s_notify_count,
                                             "reset 应至少触发 2 次通知");
}

/* 17. 派生：steps_per_mm（默认 200*8/40 = 40 steps/mm，即细分 8 契约） */
TEST_CASE("派生：steps_per_mm", "[settings]")
{
    settings_reset_defaults();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, settings_steps_per_mm());

    settings_set(SET_MICROSTEP, 32.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 160.0f, settings_steps_per_mm());
    settings_reset_defaults();
}

/* 18. 坏 NVS 值回落默认 */
TEST_CASE("坏 NVS 值回落默认", "[settings]")
{
    settings_reset_defaults();

    nvs_handle_t handle;
    TEST_ASSERT_TRUE(nvs_open("cart", NVS_READWRITE, &handle) == ESP_OK);
    uint32_t bad_raw;
    float bad_val = 7.0f;   /* 非 2^n */
    memcpy(&bad_raw, &bad_val, sizeof(bad_raw));
    nvs_set_u32(handle, "p0", bad_raw);
    nvs_commit(handle);
    nvs_close(handle);

    settings_reload();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 8.0f, settings_get(SET_MICROSTEP));

    settings_reset_defaults();
}

/* 19. count / name */
TEST_CASE("count 与 name", "[settings]")
{
    TEST_ASSERT_EQUAL_INT(5, settings_count());
    TEST_ASSERT_EQUAL_STRING("microstep", settings_name(SET_MICROSTEP));
    TEST_ASSERT_EQUAL_STRING("mm_per_rev", settings_name(SET_MM_PER_REV));
    TEST_ASSERT_EQUAL_STRING("hold_current_mA", settings_name(SET_HOLD_MA));
    TEST_ASSERT_EQUAL_STRING("?", settings_name(-1));
}

/* 兜底：订阅回调无法取消，留着无害；NVS 状态已由各用例收尾恢复。 */
