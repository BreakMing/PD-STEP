/*
 * cartfsm 单元测试——核心向量（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 与宿主测试同源：转换表/软限位/标定合法性/配置矩阵抽查，不驱动电机。
 * 流程级注入用例见 test_cartfsm_flow.c。
 */

#include "cartfsm.h"

#include "unity.h"

TEST_CASE("状态转换抽查", "[cartfsm]")
{
    cm_mode_t m = CM_DISABLED;
    cm_reason_t r = CR_NONE;
    TEST_ASSERT_TRUE_MESSAGE(cm_step(&m, &r, CE_CMD_ENABLE) && m == CM_READY
                             && r == CR_NONE, "ENABLE→READY");
    TEST_ASSERT_TRUE_MESSAGE(cm_step(&m, &r, CE_STEP_LOSS) && m == CM_FAULT
                             && r == CR_STEP_LOSS, "丢步→FAULT");
    TEST_ASSERT_TRUE_MESSAGE(cm_step(&m, &r, CE_CMD_RESET) && m == CM_DISABLED,
                             "RESET→DISABLED");
    TEST_ASSERT_TRUE_MESSAGE(cm_step(&m, &r, CE_CMD_ENABLE) && m == CM_READY,
                             "再次 ENABLE");
}

TEST_CASE("软限位钳位", "[cartfsm]")
{
    TEST_ASSERT_EQUAL_INT(0, cm_soft_limit_clamp(260000, -250000, 250000, 100));
    TEST_ASSERT_EQUAL_INT(100, cm_soft_limit_clamp(-260000, -250000, 250000, 100));
}

TEST_CASE("标定长度 L 合法域", "[cartfsm]")
{
    TEST_ASSERT_FALSE(cm_calib_valid(99999));
    TEST_ASSERT_TRUE(cm_calib_valid(100000));
    TEST_ASSERT_TRUE(cm_calib_valid(1000000));
    TEST_ASSERT_FALSE(cm_calib_valid(1000001));
    TEST_ASSERT_FALSE(cm_calib_valid(-1));
}

TEST_CASE("配置矩阵抽查", "[cartfsm]")
{
    TEST_ASSERT_TRUE(cm_config_set_allowed(CM_READY));
    TEST_ASSERT_FALSE(cm_config_set_allowed(CM_FAULT));
    TEST_ASSERT_TRUE(fw_params_set("X_MARGIN_UM", 1000));
    TEST_ASSERT_FALSE(fw_params_set("X_MARGIN_UM", 999));
    TEST_ASSERT_TRUE(fw_params_set("X_MARGIN_UM", 10000));
    TEST_ASSERT_EQUAL_INT(10000, fw_params()->x_margin_um);
}
