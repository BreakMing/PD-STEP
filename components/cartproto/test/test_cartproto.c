/*
 * cartproto 单元测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 解析向量（与宿主测试同源）+ 应答原语冒烟（真实发到 UART1，
 * 供 USB 串口环回人工核对）。用户清单另有全形态命令环回项。
 */

#include "cartproto.h"

#include "unity.h"

TEST_CASE("解析：合法命令行", "[cartproto]")
{
    cart_req_t r;
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("VEL 200", &r)
                             && r.cmd == CART_CMD_VEL && r.v_mm_s == 200,
                             "VEL 200");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("VEL -200", &r)
                             && r.v_mm_s == -200, "VEL -200");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("ENABLE", &r)
                             && r.cmd == CART_CMD_ENABLE, "ENABLE");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("MARK LEFT", &r)
                             && r.cmd == CART_CMD_MARK_LEFT, "MARK LEFT");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("MARK RIGHT", &r)
                             && r.cmd == CART_CMD_MARK_RIGHT, "MARK RIGHT");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("RESET", &r)
                             && r.cmd == CART_CMD_RESET, "RESET");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("QUERY", &r)
                             && r.cmd == CART_CMD_QUERY, "QUERY");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("CONFIG?", &r)
                             && r.cmd == CART_CMD_CONFIG_DUMP, "CONFIG?");
    TEST_ASSERT_TRUE_MESSAGE(cartproto_parse_line("CONFIG X_MARGIN_UM 10000", &r)
                             && r.cmd == CART_CMD_CONFIG_SET && r.value == 10000,
                             "CONFIG X_MARGIN_UM");
}

TEST_CASE("解析：非法命令行拒绝", "[cartproto]")
{
    cart_req_t r;
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("", &r), "空行");
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("vel 200", &r), "小写");
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("VEL 1.5", &r), "浮点");
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("VEL 32768", &r), "超界");
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("CONFIG VEL_MAX", &r), "缺值");
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("CONFIG? extra", &r),
                              "CONFIG? 带参");
    TEST_ASSERT_FALSE_MESSAGE(cartproto_parse_line("MARK UP", &r),
                              "MARK 未知方向词");
}

/* 应答原语冒烟：三行真实发出，供环回核对 */
TEST_CASE("应答原语冒烟（发 UART1 供环回核对）", "[cartproto]")
{
    cartproto_send_mode("DISABLED", "NONE");
    cartproto_send_ok("VEL_MAX", 1000);
    cartproto_send_err("VEL_MAX");
}
