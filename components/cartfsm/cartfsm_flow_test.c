/*
 * cartfsm — 上板流程自测（CONFIG_CARTFSM_FLOW_TEST=y）
 *
 * 注入式验证手动两点标定流程与故障路径（calib-spec §7）：
 *   1.  两 MARK + ENABLE → READY，重定基后无 STEP_LOSS 误报；
 *   1b. 无左标 / L 非法：拒且状态保持；
 *   1c. 上电未标定（reset_calib 模拟断电）即 ENABLE：拒；
 *   2.  重新标定：CAL_DONE 后 MARK LEFT 覆盖重开；
 *   3.  READY 注入 |E−C| 偏差 → FAULT/STEP_LOSS；
 *   4.  READY 注入 AS5600 失联 → FAULT/AS5600（既有口径）；
 *   5.  FAULT → RESET 回 DISABLED 后标定仍在，直接 ENABLE 恢复。
 * MARK 的两标位置经一次性 loss_bias 注入提供（「手在别处」的替身，
 * 标记读数消费即清）。会开 EN；有 12V 时小车不会运动（命令流为 0）。
 * 机械台位在位的标定散布验收见 deploy/v2/onboard-checklist.md
 * （C1/C2/C8，人工执行）。
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

/* 回到 DISABLED：READY 下 RESET 被忽略，先经注入失联制造 FAULT
 * （监督任务完成停机次序、EN 关）再 RESET。 */
static void to_disabled(void)
{
    if (cartfsm_mode() == CM_READY) {
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

/* 1. 标定正路径：两 MARK + ENABLE → READY，重定基后无误报 */
static bool test_calib_and_enable(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cartfsm_test_inject_loss_bias(0);

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);                       /* E_left = E + 0 */
    CHECK(cartfsm_calib() == CAL_LEFT, "MARK LEFT 后 CAL_LEFT");

    cartfsm_test_inject_loss_bias(500000);        /* 右标在 +500 mm */
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_DONE, "MARK RIGHT 后 CAL_DONE");

    /* 坐标锚（calib-spec §2）：重定基后右端必须读 +L/2——
     * set_origin 语义是「当前物理位置此后读 v」，右端即当前位
     * 置；读数错位意味着软限位窗口整体偏移（右端不设防） */
    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t x_right = as5600_position_um();
    CHECK(x_right > 249000 && x_right < 251000,
          "重定基后右端应读 +250000 µm 附近");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "ENABLE 后 READY");
    vTaskDelay(pdMS_TO_TICKS(200));
    CHECK(cartfsm_mode() == CM_READY, "重定基后 200 ms 无 STEP_LOSS 误报");
    to_disabled();
    return true;
}

/* 1b. 无左标 / L 非法：拒且状态保持 */
static bool test_calib_invalid(void)
{
    to_disabled();
    cartfsm_test_reset_calib();

    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_NONE, "无左标 MARK RIGHT 拒");

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(0);             /* 两点重合 → L=0 */
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_LEFT, "L 非法保持 CAL_LEFT");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(cartfsm_mode() == CM_DISABLED, "未完成标定 ENABLE 拒");
    to_disabled();
    return true;
}

/* 1c. 断电语义：标定丢失后 ENABLE 拒 */
static bool test_enable_after_reset_calib(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(cartfsm_mode() == CM_DISABLED && cartfsm_calib() == CAL_NONE,
          "上电未标定 ENABLE 拒");
    return true;
}

/* 2. 重新标定：CAL_DONE 后 MARK LEFT 覆盖重开 */
static bool test_recalib(void)
{
    to_disabled();
    cartfsm_test_reset_calib();

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(400000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_DONE, "第一段标定完成");

    cartfsm_test_exec(&ml);                       /* 覆盖重开 */
    CHECK(cartfsm_calib() == CAL_LEFT, "重复 MARK LEFT 重开");
    cartfsm_test_inject_loss_bias(500000);
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_DONE, "第二段标定完成");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "新标定后 ENABLE");
    to_disabled();
    return true;
}

/* 3. READY 注入 |E−C| 偏差 → FAULT/STEP_LOSS（既有保护） */
static bool test_ready_step_loss(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "进入 READY");

    cartfsm_test_inject_loss_bias(10000);   /* 一次性：> STEP_LOSS_UM */
    CHECK(wait_mode(CM_FAULT, 500), "偏差超阈值应 FAULT");
    CHECK(cartfsm_reason() == CR_STEP_LOSS, "原因为 STEP_LOSS");
    CHECK(wait_fault_settled(2000), "EN 关");
    to_disabled();
    return true;
}

/* 4. READY 注入 AS5600 失联 → FAULT/AS5600（复用既有口径） */
static bool test_ready_encoder_fail(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "进入 READY");

    TickType_t t0 = xTaskGetTickCount();
    cartfsm_test_inject_encoder_fail(1);
    CHECK(wait_mode(CM_FAULT, 100), "失联应 FAULT");
    CHECK(cartfsm_reason() == CR_AS5600, "原因为 AS5600");
    cartfsm_test_inject_encoder_fail(0);
    CHECK(wait_fault_settled(2000), "EN 关");
    ESP_LOGI(TAG, "  失联→FAULT+EN 关 耗时 %u ms（实测，含调度）",
             (unsigned)(xTaskGetTickCount() - t0));
    to_disabled();
    return true;
}

/* 5. FAULT → RESET 回 DISABLED 后标定仍在，直接 ENABLE 恢复 */
static bool test_fault_reset_recovery(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "进入 READY");
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_FAULT, 500), "制造 FAULT");
    cartfsm_test_inject_loss_bias(0);

    cart_req_t rs = { .cmd = CART_CMD_RESET };
    cartfsm_test_exec(&rs);
    CHECK(cartfsm_mode() == CM_DISABLED, "RESET 回 DISABLED");
    CHECK(cartfsm_calib() == CAL_DONE, "标定保留");
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "直接 ENABLE 恢复");
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

    RUN_TEST(test_calib_and_enable);
    RUN_TEST(test_calib_invalid);
    RUN_TEST(test_enable_after_reset_calib);
    RUN_TEST(test_recalib);
    RUN_TEST(test_ready_step_loss);
    RUN_TEST(test_ready_encoder_fail);
    RUN_TEST(test_fault_reset_recovery);

    to_disabled();

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
