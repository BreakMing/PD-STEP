/*
 * cartfsm — 上板流程自测（CONFIG_CARTFSM_FLOW_TEST=y）
 *
 * 注入式验证回零流程与故障路径（homing-spec §6）：
 *   1.  HOME → 注入到位（达速锁存 + 偏差）→ READY，原点重置 −265000 µm；
 *   1b. HOME → 未达速偏差 → FAULT/STEP_LOSS（起点压挡块/卡死/域反向）；
 *   2.  HOME → 超时 → FAULT/HOME_TIMEOUT（理想编码器跟随，不依赖 12V）；
 *   3.  HOMING 中注入 AS5600 失联 → FAULT/AS5600 优先于偏差与超时；
 *   3b. READY 中注入 |E−C| 偏差 → FAULT/STEP_LOSS；
 *   4.  RESET → DISABLED，之后可重新 HOME。
 * 会开 EN；有 12V 时小车在超时用例中以 −HOME_VEL 真实运动 ≤1 s。
 * 判定注入经 follow_cmd（E 以指令位置替代）与 speed_latch 覆写实现，
 * 全部用例在有无 12V 时均确定。机械台位在位的真实回零散布验收见
 * deploy/v2/onboard-checklist.md（C1/C3，人工执行）。
 */

#include "cartfsm.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "as5600.h"
#include "step_engine.h"

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

/* 回到 DISABLED：HOMING 直接 RESET（中止逃生口）；READY 下 RESET
 * 被忽略，先经注入失联制造 FAULT（监督任务完成停机次序、EN 关）
 * 再 RESET。 */
static void to_disabled(void)
{
    if (cartfsm_mode() == CM_HOMING) {
        cart_req_t req = { .cmd = CART_CMD_RESET };
        cartfsm_test_exec(&req);
        for (int t = 0; t < 100 && cartfsm_mode() != CM_DISABLED; t += 5) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    } else if (cartfsm_mode() == CM_READY) {
        cartfsm_test_inject_encoder_fail(1);
        if (!wait_fault_settled(2000)) {
            ESP_LOGE(TAG, "to_disabled：FAULT 停机超时");
        }
        cartfsm_test_inject_encoder_fail(0);
    } else if (cartfsm_mode() == CM_FAULT) {
        if (!wait_fault_settled(2000)) {
            ESP_LOGE(TAG, "to_disabled：FAULT 停机超时");
        }
    }
    cart_req_t req = { .cmd = CART_CMD_RESET };
    cartfsm_test_exec(&req);
    for (int t = 0; t < 100 && cartfsm_mode() != CM_DISABLED; t += 5) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* 1. HOME → 注入到位（达速锁存 + 偏差） → READY，原点重置 */
static bool test_home_done_to_ready(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    cartfsm_test_speed_latch(1);             /* 达速锁存置位 */
    cartfsm_test_inject_loss_bias(10000);    /* 偏差 10 mm > T 3 mm */
    CHECK(wait_mode(CM_READY, 500), "到位后应 READY");
    CHECK(cartfsm_reason() == CR_NONE, "READY 原因为 NONE");
    cartfsm_test_inject_loss_bias(0);

    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t x = as5600_position_um();
    CHECK(x > -266000 && x < -264000, "原点应重置为 −265000 µm 附近");

    to_disabled();
    CHECK(cartfsm_mode() == CM_DISABLED, "RESET 回 DISABLED");
    CHECK(en_is_off(), "DISABLED 时 EN 关");
    return true;
}

/* 1b. HOME → 未达速偏差 → FAULT/STEP_LOSS（起点压挡块/卡死/域反向） */
static bool test_home_loss_before_speed(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    cartfsm_test_speed_latch(0);             /* 锁存强制清零 */
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_FAULT, 500), "未达速偏差应 FAULT");
    CHECK(cartfsm_reason() == CR_STEP_LOSS, "原因为 STEP_LOSS");
    cartfsm_test_inject_loss_bias(0);
    CHECK(wait_fault_settled(2000), "EN 关");

    to_disabled();
    return true;
}

/* 2. HOME → 超时 → FAULT/HOME_TIMEOUT（理想编码器跟随，不依赖 12V） */
static bool test_home_timeout(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);
    CHECK(fw_params_set("HOME_TIMEOUT", 1000), "临时收紧超时 1 s");

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");
    cartfsm_test_follow_cmd(true);    /* E 跟随指令：D 恒 0，只可能超时 */

    /* 回零运动必须持续整个超时窗（监督任务续喂引擎看门狗——曾因
     * 断喂在 100 ms 后减速停车，超时断言无法暴露，此处锚定；
     * 步数为逻辑量，不依赖 12V） */
    vTaskDelay(pdMS_TO_TICKS(300));
    int32_t p1 = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(300));
    int32_t p2 = step_engine_position_steps();
    CHECK(p2 < p1 - 100, "HOMING 应持续运动（监督任务喂狗）");

    /* 无偏差、无编码器失联 → 只能超时 */
    CHECK(wait_mode(CM_FAULT, 3000), "1 s 超时后应 FAULT");
    CHECK(cartfsm_reason() == CR_HOME_TIMEOUT, "原因为 HOME_TIMEOUT");
    CHECK(wait_fault_settled(2000), "FAULT 停机次序完成（EN 关）");

    cartfsm_test_follow_cmd(false);
    fw_params_set("HOME_TIMEOUT", 30000);
    to_disabled();
    return true;
}

/* 3. HOMING 中注入 AS5600 失联 → FAULT/AS5600 优先于偏差与超时 */
static bool test_homing_encoder_fail_priority(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");
    cartfsm_test_follow_cmd(true);

    TickType_t t0 = xTaskGetTickCount();
    cartfsm_test_inject_encoder_fail(1);  /* AS5600 失联 */
    CHECK(wait_mode(CM_FAULT, 100), "失联后应立即 FAULT（先于超时）");
    CHECK(cartfsm_reason() == CR_AS5600, "原因为 AS5600");
    cartfsm_test_inject_encoder_fail(0);
    cartfsm_test_follow_cmd(false);
    CHECK(wait_fault_settled(2000), "EN 关");
    ESP_LOGI(TAG, "  失联→FAULT+EN 关 耗时 %u ms（实测，含调度）",
             (unsigned)(xTaskGetTickCount() - t0));

    to_disabled();
    return true;
}

/* 3b. READY 中注入 |E−C| 偏差 → FAULT/STEP_LOSS */
static bool test_ready_step_loss(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "进入 HOMING");
    cartfsm_test_speed_latch(1);
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_READY, 500), "回零完成进 READY");
    cartfsm_test_inject_loss_bias(0);

    cartfsm_test_inject_loss_bias(10000);   /* 10 mm > STEP_LOSS_UM 3000 */
    CHECK(wait_mode(CM_FAULT, 500), "偏差超阈值应 FAULT");
    CHECK(cartfsm_reason() == CR_STEP_LOSS, "原因为 STEP_LOSS");
    cartfsm_test_inject_loss_bias(0);
    CHECK(wait_fault_settled(2000), "EN 关");

    to_disabled();
    return true;
}

/* 4. FAULT 后 RESET → 可重新 HOME 全流程 */
static bool test_reset_then_rehome(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "重新 HOME 生效");
    cartfsm_test_speed_latch(1);
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_READY, 500), "再次回零完成");
    cartfsm_test_inject_loss_bias(0);
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

    RUN_TEST(test_home_done_to_ready);
    RUN_TEST(test_home_loss_before_speed);
    RUN_TEST(test_home_timeout);
    RUN_TEST(test_homing_encoder_fail_priority);
    RUN_TEST(test_ready_step_loss);
    RUN_TEST(test_reset_then_rehome);

    to_disabled();

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
