/*
 * Settings — on-target self-test (≥24 cases)
 *
 * Built into firmware when CONFIG_SETTINGS_SELF_TEST=y.
 * Runs after settings_init, before protocol stack start.
 * Final summary: "SELF-TEST DONE: <n>/<total> PASS" (tag: settings_selftest).
 *
 * IMPORTANT: Calls settings_reset_defaults() at the end to leave
 * the board in a clean state.
 */

#include "settings.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>
#include <math.h>

static const char *TAG = "settings_selftest";

/* ---- Test assertion macros ---- */
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

/* ---- Notification capture ---- */
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
 * Test cases (≥24)
 * ================================================================ */

/* 1. Default value spot checks after first boot / reset */
static bool test_default_values_spot_check(void)
{
    settings_reset_defaults();
    ASSERT_FLOAT_EQ(16.0f, settings_get(0));      /* $0 microstep = 16 */
    ASSERT_FLOAT_EQ(600.0f, settings_get(5));     /* $5 default_feed = 600 */
    ASSERT_FLOAT_EQ(2000.0f, settings_get(8));    /* $8 max_accel = 2000 */
    ASSERT_FLOAT_EQ(600.0f, settings_get(13));    /* $13 soft_max = 600 */
    return true;
}

/* 2–15: Legal set→get round-trip for each of 14 parameters */
static bool test_set_get_p0_microstep(void)
{
    settings_status_t st = settings_set(0, 32.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(32.0f, settings_get(0));
    return true;
}

static bool test_set_get_p1_full_steps(void)
{
    settings_status_t st = settings_set(1, 400.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(400.0f, settings_get(1));
    return true;
}

static bool test_set_get_p2_units_per_rev(void)
{
    settings_status_t st = settings_set(2, 50.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(50.0f, settings_get(2));
    return true;
}

static bool test_set_get_p3_run_current(void)
{
    settings_status_t st = settings_set(3, 1500.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(1500.0f, settings_get(3));
    return true;
}

static bool test_set_get_p4_hold_current(void)
{
    /* $3 is at default 800, so 400 is legal */
    settings_status_t st = settings_set(4, 600.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(600.0f, settings_get(4));
    return true;
}

static bool test_set_get_p5_default_feed(void)
{
    /* $7 = 3000, so 1000 is legal */
    settings_status_t st = settings_set(5, 1000.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(1000.0f, settings_get(5));
    return true;
}

static bool test_set_get_p6_default_accel(void)
{
    /* $8 = 2000, so 800 is legal */
    settings_status_t st = settings_set(6, 800.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(800.0f, settings_get(6));
    return true;
}

static bool test_set_get_p7_max_feed(void)
{
    settings_status_t st = settings_set(7, 5000.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(5000.0f, settings_get(7));
    return true;
}

static bool test_set_get_p8_max_accel(void)
{
    settings_status_t st = settings_set(8, 3000.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(3000.0f, settings_get(8));
    return true;
}

static bool test_set_get_p9_report_hz(void)
{
    settings_status_t st = settings_set(9, 25.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(25.0f, settings_get(9));
    return true;
}

static bool test_set_get_p10_invert_dir(void)
{
    settings_status_t st = settings_set(10, 1.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(1.0f, settings_get(10));
    return true;
}

static bool test_set_get_p11_uart_baud(void)
{
    settings_status_t st = settings_set(11, 230400.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(230400.0f, settings_get(11));
    return true;
}

static bool test_set_get_p12_soft_min(void)
{
    /* $13 = 600, so -100 is legal (< 600) */
    settings_status_t st = settings_set(12, -100.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(-100.0f, settings_get(12));
    return true;
}

static bool test_set_get_p13_soft_max(void)
{
    /* $12 = -100 now, so 500 is legal (> -100) */
    settings_status_t st = settings_set(13, 500.0f);
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(500.0f, settings_get(13));
    return true;
}

/* 16. Invalid matrix: $0 */
static bool test_invalid_p0_values(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(0, 7.0f));     /* not power of 2 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(0, 0.0f));     /* < 1 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(0, 512.0f));   /* > 256 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(0, 16.5f));    /* non-integer */
    return true;
}

/* 17. Invalid: $1 */
static bool test_invalid_p1_full_steps(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(1, 300.0f));  /* not 200/400 */
    return true;
}

/* 18. Invalid: $2 */
static bool test_invalid_p2_units_per_rev(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(2, 0.0f));     /* == 0 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(2, 501.0f));   /* > 500 */
    return true;
}

/* 19. Invalid: $3 */
static bool test_invalid_p3_run_current(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(3, 0.0f));      /* == 0 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(3, 2001.0f));   /* > 2000 */
    return true;
}

/* 20. Cross-check: $4 > $3 */
static bool test_crosscheck_p4_gt_p3(void)
{
    settings_reset_defaults();  /* $3 = 800 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(4, 1000.0f));  /* > $3=800 */
    return true;
}

/* 20b. Cross-check: $3 shrunk below $4 (reverse direction, PLAN §5) */
static bool test_crosscheck_p3_lt_p4(void)
{
    settings_reset_defaults();  /* $3 = 800, $4 = 400 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(3, 200.0f));  /* < $4=400 */
    /* Lowering $3 down to $4 is fine */
    ASSERT_EQ(SETTINGS_OK, settings_set(3, 500.0f));
    return true;
}

/* 21. Invalid: $5 (cross-check vs $7) + $5 zero */
static bool test_invalid_p5_default_feed(void)
{
    settings_reset_defaults();  /* $5 = 600, $7 = 3000 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(5, 0.0f));       /* == 0 */
    /* Shrinking $7 below current $5 is rejected (reverse direction) */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(7, 500.0f));    /* < $5=600 */
    /* Legal path: lower $5 first, then shrink $7 */
    ASSERT_EQ(SETTINGS_OK, settings_set(5, 400.0f));
    ASSERT_EQ(SETTINGS_OK, settings_set(7, 500.0f));
    ASSERT_EQ(SETTINGS_INVALID, settings_set(5, 600.0f));    /* > $7=500 */
    return true;
}

/* 22. Cross-check: $6 > $8 (reverse direction) */
static bool test_crosscheck_p6_gt_p8(void)
{
    settings_reset_defaults();  /* $8 = 2000, $6 = 500 */
    /* Shrink $8 below current $6 → should fail because $6 > new $8 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(8, 300.0f));
    return true;
}

/* 23. Invalid: $9 */
static bool test_invalid_p9_report_hz(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(9, 0.0f));    /* < 1 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(9, 51.0f));   /* > 50 */
    return true;
}

/* 24. Invalid: $10 */
static bool test_invalid_p10_invert_dir(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(10, 2.0f));  /* not 0/1 */
    return true;
}

/* 25. Invalid: $11 baud */
static bool test_invalid_p11_uart_baud(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(11, 123456.0f));
    return true;
}

/* 26. Cross-check: $12 >= $13 */
static bool test_crosscheck_p12_ge_p13(void)
{
    settings_reset_defaults();  /* $13 = 600, $12 = 0 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(12, 700.0f));   /* > $13 */
    ASSERT_EQ(SETTINGS_INVALID, settings_set(12, 600.0f));   /* == $13 */
    return true;
}

/* 27. Out of range idx */
static bool test_unknown_idx(void)
{
    settings_reset_defaults();
    ASSERT_EQ(SETTINGS_INVALID, settings_set(14, 1.0f));
    ASSERT_EQ(SETTINGS_INVALID, settings_set(-1, 1.0f));
    ASSERT_EQ(SETTINGS_INVALID, settings_set(100, 1.0f));
    return true;
}

/* 28. Persistence: set → reload → still same */
static bool test_persistence_set_reload(void)
{
    settings_reset_defaults();
    settings_set(5, 1000.0f);
    ASSERT_FLOAT_EQ(1000.0f, settings_get(5));

    settings_status_t st = settings_reload();
    ASSERT_EQ(SETTINGS_OK, st);
    ASSERT_FLOAT_EQ(1000.0f, settings_get(5));
    return true;
}

/* 28b. Cold-boot forward-reference regression (found on board, T11):
 * load must validate against the COMPLETE stored set, not partially-
 * loaded state.  Old single-pass code checked p5 against a not-yet-loaded
 * p7 (zero on cold boot, stale on reload) and discarded valid values.
 * Poison memory with $7=500 while NVS holds the valid set p5=1000/p7=3000,
 * then reload. */
static bool test_load_forward_reference(void)
{
    settings_reset_defaults();

    /* Shrink memory+NVS pair so $7=500 is settable */
    settings_set(5, 400.0f);
    settings_set(7, 500.0f);          /* NVS: p5=400, p7=500 */

    /* Direct-write a valid stored set: p5=1000, p7=3000 */
    nvs_handle_t handle;
    ASSERT_TRUE(nvs_open("settings", NVS_READWRITE, &handle) == ESP_OK, "nvs_open failed");
    uint32_t raw;
    float v;
    v = 1000.0f; memcpy(&raw, &v, sizeof(raw)); nvs_set_u32(handle, "p5", raw);
    v = 3000.0f; memcpy(&raw, &v, sizeof(raw)); nvs_set_u32(handle, "p7", raw);
    nvs_commit(handle);
    nvs_close(handle);

    /* Memory: $5=400, $7=500.  NVS: valid set p5=1000/p7=3000.
     * Reload must accept it. */
    ASSERT_EQ(SETTINGS_OK, settings_reload());
    ASSERT_FLOAT_EQ(3000.0f, settings_get(7));
    ASSERT_FLOAT_EQ(1000.0f, settings_get(5));

    settings_reset_defaults();
    return true;
}

/* 29. Reset defaults → all back to default */
static bool test_reset_defaults(void)
{
    settings_set(0, 32.0f);
    settings_set(5, 1000.0f);
    settings_set(10, 1.0f);

    settings_status_t st = settings_reset_defaults();
    ASSERT_EQ(SETTINGS_OK, st);

    ASSERT_FLOAT_EQ(16.0f, settings_get(0));
    ASSERT_FLOAT_EQ(600.0f, settings_get(5));
    ASSERT_FLOAT_EQ(0.0f, settings_get(10));
    ASSERT_FLOAT_EQ(800.0f, settings_get(3));
    return true;
}

/* 30. Change notification on set */
static bool test_notification_on_set(void)
{
    settings_reset_defaults();
    notify_reset();
    settings_subscribe(test_notify_cb);

    settings_set(0, 32.0f);
    ASSERT_EQ(1, s_notify_count);
    ASSERT_EQ(0, s_notify_idx);
    ASSERT_FLOAT_EQ(32.0f, s_notify_val);

    /* Unsubscribe hack: not in API, just reset count for cleanup */
    notify_reset();
    return true;
}

/* 31. Reset defaults triggers notifications */
static bool test_notification_on_reset(void)
{
    settings_set(0, 32.0f);
    settings_set(5, 1000.0f);
    notify_reset();

    settings_reset_defaults();
    /* At least 2 notifications (p0 and p5 changed).  Possibly more if
     * other params were left in non-default state from prior tests. */
    ASSERT_TRUE(s_notify_count >= 2, "expected >=2 notifications from reset");
    return true;
}

/* 32. Derived: steps_per_mm */
static bool test_derived_steps_per_mm(void)
{
    settings_reset_defaults();
    /* Default: full_steps=200, microstep=16, units_per_rev=40 */
    /* steps/mm = 200 * 16 / 40 = 80.0 */
    ASSERT_FLOAT_EQ(80.0f, settings_steps_per_mm());

    /* Change microstep to 32 → 200*32/40 = 160.0 */
    settings_set(0, 32.0f);
    ASSERT_FLOAT_EQ(160.0f, settings_steps_per_mm());
    return true;
}

/* 33. Bad NVS value fallback */
static bool test_bad_nvs_value_fallback(void)
{
    settings_reset_defaults();

    /* Directly write a bad value (p0=7, not power of 2) to NVS */
    nvs_handle_t handle;
    esp_err_t err = nvs_open("settings", NVS_READWRITE, &handle);
    ASSERT_TRUE(err == ESP_OK, "nvs_open failed");
    uint32_t bad_raw;
    float bad_val = 7.0f;
    memcpy(&bad_raw, &bad_val, sizeof(bad_raw));
    nvs_set_u32(handle, "p0", bad_raw);
    nvs_commit(handle);
    nvs_close(handle);

    /* Reload — should fall back to default 16 */
    settings_reload();
    ASSERT_FLOAT_EQ(16.0f, settings_get(0));

    /* Clean up: reset to defaults (erases the bad key) */
    settings_reset_defaults();
    return true;
}

/* 34. settings_count / settings_name */
static bool test_count_and_name(void)
{
    ASSERT_EQ(14, settings_count());
    ASSERT_TRUE(strcmp(settings_name(0), "microstep") == 0, "p0 name");
    ASSERT_TRUE(strcmp(settings_name(5), "default_feed") == 0, "p5 name");
    ASSERT_TRUE(strcmp(settings_name(13), "soft_max") == 0, "p13 name");
    ASSERT_TRUE(strcmp(settings_name(-1), "?") == 0, "out of range name");
    return true;
}

/* ================================================================
 * Test runner
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

    /* Ensure clean start */
    settings_reset_defaults();

    ESP_LOGI(TAG, "Default values:");
    RUN_TEST(test_default_values_spot_check);

    ESP_LOGI(TAG, "Set/get round-trip (14 params):");
    RUN_TEST(test_set_get_p0_microstep);
    RUN_TEST(test_set_get_p1_full_steps);
    RUN_TEST(test_set_get_p2_units_per_rev);
    RUN_TEST(test_set_get_p3_run_current);
    RUN_TEST(test_set_get_p4_hold_current);
    RUN_TEST(test_set_get_p5_default_feed);
    RUN_TEST(test_set_get_p6_default_accel);
    RUN_TEST(test_set_get_p7_max_feed);
    RUN_TEST(test_set_get_p8_max_accel);
    RUN_TEST(test_set_get_p9_report_hz);
    RUN_TEST(test_set_get_p10_invert_dir);
    RUN_TEST(test_set_get_p11_uart_baud);
    RUN_TEST(test_set_get_p12_soft_min);
    RUN_TEST(test_set_get_p13_soft_max);

    ESP_LOGI(TAG, "Invalid value matrix:");
    RUN_TEST(test_invalid_p0_values);
    RUN_TEST(test_invalid_p1_full_steps);
    RUN_TEST(test_invalid_p2_units_per_rev);
    RUN_TEST(test_invalid_p3_run_current);
    RUN_TEST(test_crosscheck_p4_gt_p3);
    RUN_TEST(test_crosscheck_p3_lt_p4);
    RUN_TEST(test_invalid_p5_default_feed);
    RUN_TEST(test_crosscheck_p6_gt_p8);
    RUN_TEST(test_invalid_p9_report_hz);
    RUN_TEST(test_invalid_p10_invert_dir);
    RUN_TEST(test_invalid_p11_uart_baud);
    RUN_TEST(test_crosscheck_p12_ge_p13);

    ESP_LOGI(TAG, "Index range:");
    RUN_TEST(test_unknown_idx);

    ESP_LOGI(TAG, "Persistence & reset:");
    RUN_TEST(test_persistence_set_reload);
    RUN_TEST(test_load_forward_reference);
    RUN_TEST(test_reset_defaults);

    ESP_LOGI(TAG, "Notifications:");
    RUN_TEST(test_notification_on_set);
    RUN_TEST(test_notification_on_reset);

    ESP_LOGI(TAG, "Derived values:");
    RUN_TEST(test_derived_steps_per_mm);

    ESP_LOGI(TAG, "Edge cases:");
    RUN_TEST(test_bad_nvs_value_fallback);
    RUN_TEST(test_count_and_name);

    /* Leave board clean */
    settings_reset_defaults();

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");

    return s_pass;
}
