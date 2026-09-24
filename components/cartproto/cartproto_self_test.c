/*
 * cartproto — 上板自测
 *
 * CONFIG_CARTPROTO_SELF_TEST=y 时编入，cartproto_start 之后运行。
 * 覆盖解析向量（与宿主测试同源）+ 应答原语冒烟（真实发到 UART1，
 * 供 USB 串口环回人工核对）。用户清单另有全形态命令环回项。
 */

#include "cartproto.h"

#include "esp_log.h"

static const char *TAG = "cartproto_selftest";

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

static bool test_parse_legal(void)
{
    cart_req_t r;
    CHECK(cartproto_parse_line("VEL 200", &r) && r.cmd == CART_CMD_VEL
          && r.v_mm_s == 200, "VEL 200");
    CHECK(cartproto_parse_line("VEL -200", &r) && r.v_mm_s == -200, "VEL -200");
    CHECK(cartproto_parse_line("HOME", &r) && r.cmd == CART_CMD_HOME, "HOME");
    CHECK(cartproto_parse_line("RESET", &r) && r.cmd == CART_CMD_RESET, "RESET");
    CHECK(cartproto_parse_line("QUERY", &r) && r.cmd == CART_CMD_QUERY, "QUERY");
    CHECK(cartproto_parse_line("CONFIG?", &r)
          && r.cmd == CART_CMD_CONFIG_DUMP, "CONFIG?");
    CHECK(cartproto_parse_line("CONFIG X_LIMIT 250000", &r)
          && r.cmd == CART_CMD_CONFIG_SET && r.value == 250000,
          "CONFIG X_LIMIT");
    return true;
}

static bool test_parse_illegal(void)
{
    cart_req_t r;
    CHECK(!cartproto_parse_line("", &r), "空行");
    CHECK(!cartproto_parse_line("vel 200", &r), "小写");
    CHECK(!cartproto_parse_line("VEL 1.5", &r), "浮点");
    CHECK(!cartproto_parse_line("VEL 32768", &r), "超界");
    CHECK(!cartproto_parse_line("CONFIG VEL_MAX", &r), "缺值");
    CHECK(!cartproto_parse_line("CONFIG? extra", &r), "CONFIG? 带参");
    CHECK(!cartproto_parse_line("HOME NOW", &r), "HOME 带参");
    return true;
}

/* 应答原语冒烟：三行真实发出，供环回核对 */
static bool test_send_smoke(void)
{
    cartproto_send_mode("DISABLED", "NONE");
    cartproto_send_ok("VEL_MAX", 1000);
    cartproto_send_err("VEL_MAX");
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

void cartproto_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " cartproto Self-Test");
    ESP_LOGI(TAG, "========================================");

    RUN_TEST(test_parse_legal);
    RUN_TEST(test_parse_illegal);
    RUN_TEST(test_send_smoke);

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
