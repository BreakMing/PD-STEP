/*
 * cartfsm 流程测试——注入式验证手动两点标定流程与故障路径
 * （官方 test/ 子目录约定，仅测试工程编入）
 *
 *   1.  两 MARK + ENABLE → READY，重定基后无 STEP_LOSS 误报；
 *   1b. 无左标 / L 非法：拒且状态保持；
 *   1c. 上电未标定（reset_calib 模拟断电）即 ENABLE：拒；
 *   2.  重新标定：CAL_DONE 后 MARK LEFT 覆盖重开；
 *   3.  READY 注入 |E−C| 偏差 → FAULT/STEP_LOSS；
 *   3c. FAULT settling 吃掉在途目标；
 *   4.  READY 注入 AS5600 失联 → FAULT/AS5600（既有口径）；
 *   5.  FAULT → RESET 回 DISABLED 后标定仍在，直接 ENABLE 恢复。
 *
 * MARK 的两标位置经一次性 loss_bias 注入提供（「手在别处」的替身，
 * 标记读数消费即清）。会开 EN（驱动上电使能）；除 3c 建立运动外
 * 命令流为 0，小车仅 3c 向左小幅移动（<50 mm），机械台位在位时
 * 请保持警戒。
 * 机械台位在位的标定散布验收见 deploy/v2/onboard-checklist.md
 * （C1/C2/C8，人工执行）。
 */

#include "cartfsm.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "as5600.h"
#include "step_engine.h"

static const char *TAG = "cartfsm_flowtest";

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
TEST_CASE("流程：两 MARK + ENABLE → READY 无误报", "[cartfsm][flowtest]")
{
    to_disabled();
    cartfsm_test_reset_calib();
    cartfsm_test_inject_loss_bias(0);

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);                       /* E_left = E + 0 */
    TEST_ASSERT_EQUAL_INT(CAL_LEFT, cartfsm_calib());

    cartfsm_test_inject_loss_bias(500000);        /* 右标在 +500 mm */
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    TEST_ASSERT_EQUAL_INT(CAL_DONE, cartfsm_calib());

    /* 坐标锚：重定基后右端必须读 +L/2——
     * set_origin 语义是「当前物理位置此后读 v」，右端即当前位
     * 置；读数错位意味着软限位窗口整体偏移（右端不设防） */
    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t x_right = as5600_position_um();
    TEST_ASSERT_TRUE_MESSAGE(x_right > 249000 && x_right < 251000,
                             "重定基后右端应读 +250000 µm 附近");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "ENABLE 后 READY");
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_TRUE_MESSAGE(cartfsm_mode() == CM_READY,
                             "重定基后 200 ms 无 STEP_LOSS 误报");
    to_disabled();
}

/* 1b. 无左标 / L 非法：拒且状态保持 */
TEST_CASE("流程：无左标 / L 非法拒且状态保持", "[cartfsm][flowtest]")
{
    to_disabled();
    cartfsm_test_reset_calib();

    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    TEST_ASSERT_EQUAL_INT(CAL_NONE, cartfsm_calib());

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(0);             /* 两点重合 → L=0 */
    cartfsm_test_exec(&mr);
    TEST_ASSERT_EQUAL_INT(CAL_LEFT, cartfsm_calib());

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    TEST_ASSERT_EQUAL_INT(CM_DISABLED, cartfsm_mode());
    to_disabled();
}

/* 1c. 断电语义：标定丢失后 ENABLE 拒 */
TEST_CASE("流程：未标定 ENABLE 拒（断电语义）", "[cartfsm][flowtest]")
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    TEST_ASSERT_TRUE_MESSAGE(cartfsm_mode() == CM_DISABLED
                             && cartfsm_calib() == CAL_NONE,
                             "上电未标定 ENABLE 拒");
}

/* 2. 重新标定：CAL_DONE 后 MARK LEFT 覆盖重开 */
TEST_CASE("流程：重新标定覆盖重开", "[cartfsm][flowtest]")
{
    to_disabled();
    cartfsm_test_reset_calib();

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(400000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    TEST_ASSERT_EQUAL_INT(CAL_DONE, cartfsm_calib());

    cartfsm_test_exec(&ml);                       /* 覆盖重开 */
    TEST_ASSERT_EQUAL_INT(CAL_LEFT, cartfsm_calib());
    cartfsm_test_inject_loss_bias(500000);
    cartfsm_test_exec(&mr);
    TEST_ASSERT_EQUAL_INT(CAL_DONE, cartfsm_calib());

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "新标定后 ENABLE");
    to_disabled();
}

/* 3. READY 注入 |E−C| 偏差 → FAULT/STEP_LOSS（既有保护） */
TEST_CASE("流程：READY 偏差注入 → FAULT/STEP_LOSS", "[cartfsm][flowtest]")
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
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "进入 READY");

    cartfsm_test_inject_loss_bias(10000);   /* 一次性：> STEP_LOSS_UM */
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_FAULT, 500), "偏差超阈值应 FAULT");
    TEST_ASSERT_EQUAL_INT(CR_STEP_LOSS, cartfsm_reason());
    TEST_ASSERT_TRUE_MESSAGE(wait_fault_settled(2000), "EN 关");
    to_disabled();
}

/* 3c. FAULT settling 期间晚到的非零目标：监督每拍重发 halt，
 * 不依赖 WDT 兜底即完成停机次序（EN 关）。
 * 模拟 A3 场景：exec 锁外 vel_target 晚于 enter_fault 的 halt。
 *
 * 时序论证：必须先建立运动再注入失联——静止态 vel_now==0 立
 * 即成立，settling 在 FAULT 置位后一个监督拍内完成，5 ms 粒度
 * 的 wait_mode 返回时 EN 已关，目标注入晚于 settling 结束，旧实
 * 现下用例也绿（无法证伪）。运动态下 WDT_DEC 档减速需 ~100 ms
 * （500→0 @5000 mm/s²），注入落在减速窗内；WDT_MS 拉满 1000 ms
 * 后旧实现（无 halt 重发）目标持续存活、引擎运动至 WDT 过期，
 * 300 ms 断言窗内 EN 不关（必红）；新实现每拍 halt 吃掉写入，
 * 减速完成即 EN 关（绿，边距约 200 ms）。
 * 注意：本用例小车自右端向左小幅移动（<50 mm），台位在位时保持
 * 警戒。 */
TEST_CASE("流程：FAULT settling 吃掉在途目标（WDT 不兜底）", "[cartfsm][flowtest]")
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
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "进入 READY");

    cart_req_t wdt = { .cmd = CART_CMD_CONFIG_SET, .name = "WDT_MS",
                       .value = 1000 };
    cartfsm_test_exec(&wdt);          /* READY 下 CONFIG SET 允许 */

    /* 建立运动：自右端向左（远离软限位边界） */
    cart_req_t vel = { .cmd = CART_CMD_VEL, .v_mm_s = -500 };
    cartfsm_test_exec(&vel);
    bool moving = false;
    for (int t = 0; t < 200; t += 5) {
        if (step_engine_vel_now_q16() != 0) {
            moving = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    TEST_ASSERT_TRUE_MESSAGE(moving, "运动建立（vel_now != 0）");

    cartfsm_test_inject_encoder_fail(1);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_FAULT, 100), "失联应 FAULT");

    /* 在途写入：减速窗内注入引擎目标（A3 穿透形态） */
    step_engine_vel_target(200 << 16);
    TEST_ASSERT_TRUE_MESSAGE(wait_fault_settled(300),
                             "settling 应每拍重发 halt，300 ms 内 EN 关");
    cartfsm_test_inject_encoder_fail(0);
    to_disabled();
}

/* 4. READY 注入 AS5600 失联 → FAULT/AS5600（复用既有口径） */
TEST_CASE("流程：编码器失联注入 → FAULT/AS5600", "[cartfsm][flowtest]")
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
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "进入 READY");

    TickType_t t0 = xTaskGetTickCount();
    cartfsm_test_inject_encoder_fail(1);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_FAULT, 100), "失联应 FAULT");
    TEST_ASSERT_EQUAL_INT(CR_AS5600, cartfsm_reason());
    cartfsm_test_inject_encoder_fail(0);
    TEST_ASSERT_TRUE_MESSAGE(wait_fault_settled(2000), "EN 关");
    ESP_LOGI(TAG, "失联→FAULT+EN 关 耗时 %u ms（实测，含调度）",
             (unsigned)(xTaskGetTickCount() - t0));
    to_disabled();
}

/* 5. FAULT → RESET 回 DISABLED 后标定仍在，直接 ENABLE 恢复 */
TEST_CASE("流程：FAULT 后 RESET 保留标定直接恢复", "[cartfsm][flowtest]")
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
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "进入 READY");
    cartfsm_test_inject_loss_bias(10000);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_FAULT, 500), "制造 FAULT");
    cartfsm_test_inject_loss_bias(0);

    cart_req_t rs = { .cmd = CART_CMD_RESET };
    cartfsm_test_exec(&rs);
    TEST_ASSERT_EQUAL_INT(CM_DISABLED, cartfsm_mode());
    TEST_ASSERT_EQUAL_INT(CAL_DONE, cartfsm_calib());
    cartfsm_test_exec(&en);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "直接 ENABLE 恢复");
    to_disabled();
}
