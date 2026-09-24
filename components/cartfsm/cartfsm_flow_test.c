/*
 * cartfsm — 上板流程自测（CONFIG_CARTFSM_FLOW_TEST=y）
 *
 * 注入式验证回零流程与故障路径（计划任务 8 步骤 1 的用例）：
 *   1. HOME → 注入堵转 → READY，且 as5600 原点重置为 −265000 µm；
 *   2. HOME → 无堵转 → 超时 FAULT/HOME_TIMEOUT（临时 1 s 超时）；
 *   3. HOMING 中注入 AS5600 失联 → FAULT/AS5600 优先于超时；
 *   4. RESET → DISABLED，之后可重新 HOME。
 * 会开 EN；有 12V 时小车在超时用例中以 −HOME_VEL 真实运动 ≤1 s。
 * 机械台位在位的真实堵转回零散布验收见 docs/reviews/T8-acceptance.md
 * （上板清单 T8-③，人工执行）。
 */

#include "cartfsm.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "as5600.h"

static const char *TAG = "cartfsm_flowtest";

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

static bool en_is_off(void)
{
    return gpio_get_level(CONFIG_BSP_EN_GPIO) == 1;   /* ENN 高 = 不使能 */
}

static bool wait_mode(cm_mode_t want, int timeout_ms)
{
    for (int t = 0; t < timeout_ms; t += 5) {
        if (cartfsm_mode() == want) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

static bool wait_fault_settled(int timeout_ms)
{
    /* FAULT 停机次序完成 = EN 关 */
    for (int t = 0; t < timeout_ms; t += 5) {
        if (cartfsm_mode() == CM_FAULT && en_is_off()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

static void to_disabled(void)
{
    cart_req_t req = { .cmd = CART_CMD_RESET };
    cartfsm_test_exec(&req);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* 1. HOME → 注入堵转 → READY，原点重置 */
static bool test_home_stall_to_ready(void)
{
    to_disabled();

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    cartfsm_report_stall(true);          /* 注入堵转（完成信号） */
    CHECK(wait_mode(CM_READY, 500), "堵转后应 READY");

    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t x = as5600_position_um();
    CHECK(x > -266000 && x < -264000, "原点应重置为 −265000 µm 附近");

    /* 收尾：经 RESET 回 DISABLED */
    to_disabled();
    CHECK(cartfsm_mode() == CM_DISABLED, "RESET 回 DISABLED");
    CHECK(en_is_off(), "DISABLED 时 EN 关");
    return true;
}

/* 2. HOME → 超时 → FAULT/HOME_TIMEOUT（临时 1 s 超时） */
static bool test_home_timeout(void)
{
    to_disabled();
    CHECK(fw_params_set("HOME_TIMEOUT", 1000), "临时收紧超时 1 s");

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    /* 无堵转、无编码器失联 → 只能超时 */
    CHECK(wait_mode(CM_FAULT, 3000), "1 s 超时后应 FAULT");
    CHECK(wait_fault_settled(2000), "FAULT 停机次序完成（EN 关）");

    fw_params_set("HOME_TIMEOUT", 30000);
    to_disabled();
    return true;
}

/* 3. HOMING 中注入 AS5600 失联 → FAULT/AS5600 优先于超时 */
static bool test_homing_encoder_fail_priority(void)
{
    to_disabled();

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    cartfsm_test_inject_encoder_fail(1);  /* AS5600 失联 */
    CHECK(wait_mode(CM_FAULT, 500), "失联后应立即 FAULT（先于超时）");
    cartfsm_test_inject_encoder_fail(0);
    CHECK(wait_fault_settled(2000), "EN 关");

    to_disabled();
    return true;
}

/* 4. FAULT 后 RESET → 可重新 HOME 全流程 */
static bool test_reset_then_rehome(void)
{
    to_disabled();

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "重新 HOME 生效");
    cartfsm_report_stall(true);
    CHECK(wait_mode(CM_READY, 500), "再次回零完成");
    to_disabled();
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

void cartfsm_flow_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " cartfsm 流程自测（注入式，会开 EN）");
    ESP_LOGI(TAG, "========================================");

    RUN_TEST(test_home_stall_to_ready);
    RUN_TEST(test_home_timeout);
    RUN_TEST(test_homing_encoder_fail_priority);
    RUN_TEST(test_reset_then_rehome);

    to_disabled();

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
