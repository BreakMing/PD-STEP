/*
 * telemetry — 上板自测
 *
 * CONFIG_TELEMETRY_SELF_TEST=y 时编入。覆盖组帧向量与换算；
 * 行率（200 Hz±1）、x_um 漂移、手推方向一致性需从端侧/USB 串口
 * 观察，归入最终上板清单。
 */

#include "telemetry.h"
#include "as5600.h"

#include "esp_log.h"
#include <string.h>

static const char *TAG = "telemetry_selftest";

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

/* 1. 组帧向量（与宿主测试同源） */
static bool test_format_vectors(void)
{
    char buf[64];
    int n = telemetry_format_state(buf, sizeof(buf), 123456, -3141592, -200, 4500);
    CHECK(n > 0 && strcmp(buf, "STATE 123456 -3141592 -200 4500") == 0, "基本格式");
    CHECK(strchr(buf, '\n') == NULL, "不含换行");

    telemetry_format_state(buf, sizeof(buf), 0, 0, 0, 0);
    CHECK(strcmp(buf, "STATE 0 0 0 0") == 0, "零值");

    char small[16];
    CHECK(telemetry_format_state(small, sizeof(small), 1, 1, 1, 1) < 0, "小缓冲负值");
    return true;
}

/* 2. counts→µm 换算 */
static bool test_counts_to_um(void)
{
    CHECK(as5600_counts_to_um(64) == 625, "64 → 625");
    CHECK(as5600_counts_to_um(-64) == -625, "−64 → −625");
    CHECK(as5600_counts_to_um(4096) == 40000, "一圈 → 40000");
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

void telemetry_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Telemetry Self-Test");
    ESP_LOGI(TAG, "========================================");

    RUN_TEST(test_format_vectors);
    RUN_TEST(test_counts_to_um);

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
