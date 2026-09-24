/*
 * Settings — 上板自测（5 参数表）
 *
 * CONFIG_SETTINGS_SELF_TEST=y 时编入固件，settings_init 后、
 * cartproto 启动前运行。结束行 "SELF-TEST DONE: <n>/<total> PASS"。
 *
 * 结束时调用 settings_reset_defaults() 留下干净状态。
 */

#include "settings.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>
#include <math.h>

static const char *TAG = "settings_selftest";

/* ---- 断言宏 ---- */
static int s_pass = 0;
static int s_fail = 0;

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_EQ(expected, actual) do { \
    if ((expected) != (actual)) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %d, got %d (line %d)", \
                 __func__, (int)(expected), (int)(actual), __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_FLOAT_EQ(expected, actual) do { \
    if (fabsf((expected) - (actual)) > 0.001f) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %.3f, got %.3f (line %d)", \
                 __func__, (double)(expected), (double)(actual), __LINE__); \
        return false; \
    } \
} while (0)

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

/* ================================================================
 * 用例
 * ================================================================ */

/* 1. 默认值（含微步默认 8） */
static bool test_default_values_spot_check(void)
{
    settings_reset_defaults();
    ASSERT_FLOAT_EQ(8.0f, settings_get(SET_MICROSTEP));   /* 微步默认 8 */
    ASSERT_FLOAT_EQ(200.0f, settings_get(SET_FULL_STEP));
    ASSERT_FLOAT_EQ(40.0f, settings_get(SET_MM_PER_REV));
    ASSERT_FLOAT_EQ(800.0f, settings_get(SET_RUN_MA));
    ASSERT_FLOAT_EQ(400.0f, settings_get(SET_HOLD_MA));
    return true;
}

/* 2–6. 合法 set→get 往返 */
static bool test_set_get_p0_microstep(void)
{
    settings_status_t st = settings_set(SET_MICROSTEP, 16.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(16.0f, settings_get(SET_MICROSTEP));
    return true;
}

static bool test_set_get_p1_full_steps(void)
{
    settings_status_t st = settings_set(SET_FULL_STEP, 400.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(400.0f, settings_get(SET_FULL_STEP));
    return true;
}

static bool test_set_get_p2_mm_per_rev(void)
{
    settings_status_t st = settings_set(SET_MM_PER_REV, 36.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(36.0f, settings_get(SET_MM_PER_REV));
    return true;
}

static bool test_set_get_p3_run_current(void)
{
    settings_status_t st = settings_set(SET_RUN_MA, 1500.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(1500.0f, settings_get(SET_RUN_MA));
    return true;
}

static bool test_set_get_p4_hold_current(void)
{
    /* $3 处于默认 800，600 合法 */
    settings_status_t st = settings_set(SET_HOLD_MA, 600.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(600.0f, settings_get(SET_HOLD_MA));
    return true;
}

/* 7. 非法：$0 */
static bool test_invalid_p0_values(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 7.0f));   /* 非 2^n */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 0.0f));   /* < 1 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 512.0f)); /* > 256 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_MICROSTEP, 8.5f));   /* 非整数 */
    return true;
}

/* 8. 非法：$1 */
static bool test_invalid_p1_full_steps(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_FULL_STEP, 300.0f)); /* 非 200/400 */
    return true;
}

/* 9. 非法：$2 */
static bool test_invalid_p2_mm_per_rev(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_MM_PER_REV, 0.0f));   /* == 0 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_MM_PER_REV, 501.0f)); /* > 500 */
    return true;
}

/* 10. 非法：$3 */
static bool test_invalid_p3_run_current(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_RUN_MA, 0.0f));      /* == 0 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_RUN_MA, 2001.0f));   /* > 2000 */
    return true;
}

/* 11. 交叉检查：SET_HOLD_MA > SET_RUN_MA 拒绝 */
static bool test_crosscheck_hold_gt_run(void)
{
    settings_reset_defaults();  /* $3 = 800 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_HOLD_MA, 1000.0f)); /* > $3=800 */
    return true;
}

/* 11b. 交叉检查：$3 收缩到 $4 之下（反向） */
static bool test_crosscheck_run_lt_hold(void)
{
    settings_reset_defaults();  /* $3 = 800, $4 = 400 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SET_RUN_MA, 200.0f)); /* < $4=400 */
    /* 收缩到 $4 即可 */
    ASSERT_EQ(SETTINGS_OK, settings_set(SET_RUN_MA, 400.0f));
    return true;
}

/* 12. 索引超范围 */
static bool test_unknown_idx(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(SETTINGS_COUNT, 1.0f));
    ASSERT_EQ(SETTINGS_INVALID, settings_set(-1, 1.0f));
    ASSERT_EQ(SETTINGS_INVALID, settings_set(100, 1.0f));
    return true;
}

/* 13. 持久化：set → reload → 仍相同 */
static bool test_persistence_set_reload(void)
{
    settings_reset_defaults();
    settings_set(SET_RUN_MA, 1000.0f);
    ASSERT_FLOAT_EQ(1000.0f, settings_get(SET_RUN_MA));

    settings_status_t st = settings_reload();
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(1000.0f, settings_get(SET_RUN_MA));
    return true;
}

/* 13b. 冷启动前向引用回归（沿袭 T11 板上发现）：
 * 载入必须对完整存储集校验，而非半载入状态。直接写入合法对
 * p3=1500/p4=600，内存当前为 p3=500/p4=400，reload 必须接受。 */
static bool test_load_forward_reference(void)
{
    settings_reset_defaults();
    settings_set(SET_RUN_MA, 500.0f);    /* NVS: p3=500, p4=400 */

    nvs_handle_t handle;
    ASSERT_TRUE(nvs_open("cart", NVS_READWRITE, &handle) == ESP_OK, "nvs_open failed");
    uint32_t raw;
    float v;
    v = 1500.0f; memcpy(&raw, &v, sizeof(raw)); nvs_set_u32(handle, "p3", raw);
    v = 600.0f;  memcpy(&raw, &v, sizeof(raw)); nvs_set_u32(handle, "p4", raw);
    nvs_commit(handle);
    nvs_close(handle);

    ASSERT_EQ(SETTINGS_OK, settings_reload());
    ASSERT_FLOAT_EQ(1500.0f, settings_get(SET_RUN_MA));
    ASSERT_FLOAT_EQ(600.0f, settings_get(SET_HOLD_MA));

    settings_reset_defaults();
    return true;
}

/* 14. 恢复默认 */
static bool test_reset_defaults(void)
{
    settings_set(SET_MICROSTEP, 16.0f);
    settings_set(SET_RUN_MA, 1200.0f);

    settings_status_t st = settings_reset_defaults();
    ASSERT_EQ(SETTINGS_OK, st);

    ASSERT_FLOAT_EQ(8.0f, settings_get(SET_MICROSTEP));
    ASSERT_FLOAT_EQ(800.0f, settings_get(SET_RUN_MA));
    return true;
}

/* 15. set 触发通知 */
static bool test_notification_on_set(void)
{
    settings_reset_defaults();
    notify_reset();
    settings_subscribe(test_notify_cb);

    settings_set(SET_MICROSTEP, 16.0f);
    ASSERT_EQ(1, s_notify_count);
    ASSERT_EQ(SET_MICROSTEP, s_notify_idx);
    ASSERT_FLOAT_EQ(16.0f, s_notify_val);

    notify_reset();
    return true;
}

/* 16. 恢复默认触发通知 */
static bool test_notification_on_reset(void)
{
    settings_set(SET_MICROSTEP, 16.0f);
    settings_set(SET_RUN_MA, 1000.0f);
    notify_reset();

    settings_reset_defaults();
    ASSERT_TRUE(s_notify_count >= 2, "reset 应至少触发 2 次通知");
    return true;
}

/* 17. 派生：steps_per_mm（默认 200*8/40 = 40 steps/mm，即细分 8 契约） */
static bool test_derived_steps_per_mm(void)
{
    settings_reset_defaults();
    ASSERT_FLOAT_EQ(40.0f, settings_steps_per_mm());

    settings_set(SET_MICROSTEP, 32.0f);
    ASSERT_FLOAT_EQ(160.0f, settings_steps_per_mm());
    return true;
}

/* 18. 坏 NVS 值回落默认 */
static bool test_bad_nvs_value_fallback(void)
{
    settings_reset_defaults();

    nvs_handle_t handle;
    esp_err_t err = nvs_open("cart", NVS_READWRITE, &handle);
    ASSERT_TRUE(err == ESP_OK, "nvs_open failed");
    uint32_t bad_raw;
    float bad_val = 7.0f;   /* 非 2^n */
    memcpy(&bad_raw, &bad_val, sizeof(bad_raw));
    nvs_set_u32(handle, "p0", bad_raw);
    nvs_commit(handle);
    nvs_close(handle);

    settings_reload();
    ASSERT_FLOAT_EQ(8.0f, settings_get(SET_MICROSTEP));   /* 回落默认 8 */

    settings_reset_defaults();
    return true;
}

/* 19. count / name */
static bool test_count_and_name(void)
{
    ASSERT_EQ(5, settings_count());
    ASSERT_TRUE(strcmp(settings_name(SET_MICROSTEP), "microstep") == 0, "p0 name");
    ASSERT_TRUE(strcmp(settings_name(SET_MM_PER_REV), "mm_per_rev") == 0, "p2 name");
    ASSERT_TRUE(strcmp(settings_name(SET_HOLD_MA), "hold_current_mA") == 0, "p4 name");
    ASSERT_TRUE(strcmp(settings_name(-1), "?") == 0, "out of range name");
    return true;
}

/* ================================================================
 * 运行器
 * ================================================================ */

#define RUN_TEST(name) do { \
    if (name()) { \
        s_pass++; \
        ESP_LOGI(TAG, "  PASS  %s", #name); \
    } else { \
        s_fail++; \
    } \
} while (0)

int settings_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Settings Self-Test");
    ESP_LOGI(TAG, "========================================");

    settings_reset_defaults();

    ESP_LOGI(TAG, "默认值：");
    RUN_TEST(test_default_values_spot_check);

    ESP_LOGI(TAG, "Set/get 往返（5 参数）：");
    RUN_TEST(test_set_get_p0_microstep);
    RUN_TEST(test_set_get_p1_full_steps);
    RUN_TEST(test_set_get_p2_mm_per_rev);
    RUN_TEST(test_set_get_p3_run_current);
    RUN_TEST(test_set_get_p4_hold_current);

    ESP_LOGI(TAG, "非法值矩阵：");
    RUN_TEST(test_invalid_p0_values);
    RUN_TEST(test_invalid_p1_full_steps);
    RUN_TEST(test_invalid_p2_mm_per_rev);
    RUN_TEST(test_invalid_p3_run_current);
    RUN_TEST(test_crosscheck_hold_gt_run);
    RUN_TEST(test_crosscheck_run_lt_hold);

    ESP_LOGI(TAG, "索引范围：");
    RUN_TEST(test_unknown_idx);

    ESP_LOGI(TAG, "持久化与恢复：");
    RUN_TEST(test_persistence_set_reload);
    RUN_TEST(test_load_forward_reference);
    RUN_TEST(test_reset_defaults);

    ESP_LOGI(TAG, "通知：");
    RUN_TEST(test_notification_on_set);
    RUN_TEST(test_notification_on_reset);

    ESP_LOGI(TAG, "派生值：");
    RUN_TEST(test_derived_steps_per_mm);

    ESP_LOGI(TAG, "边界：");
    RUN_TEST(test_bad_nvs_value_fallback);
    RUN_TEST(test_count_and_name);

    settings_reset_defaults();

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
