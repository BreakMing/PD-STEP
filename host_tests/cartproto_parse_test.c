/*
 * cartproto_parse_line 宿主测试 — 解析契约（protocol.md §命令）
 *
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include <string.h>
#include "cartproto.h"

static int s_fail = 0;
static int s_run = 0;

#define CHECK(cond, msg) do { \
    s_run++; \
    if (!(cond)) { \
        s_fail++; \
        printf("FAIL %s:%d  %s\n", __func__, __LINE__, msg); \
    } \
} while (0)

static cart_req_t req;

static bool p(const char *line)
{
    memset(&req, 0, sizeof(req));
    return cartproto_parse_line(line, &req);
}

/* ---- 合法集 ---- */
static void test_legal(void)
{
    CHECK(p("VEL 200") && req.cmd == CART_CMD_VEL && req.v_mm_s == 200,
          "VEL 200");
    CHECK(p("VEL -200") && req.cmd == CART_CMD_VEL && req.v_mm_s == -200,
          "VEL -200");
    CHECK(p("VEL 0") && req.cmd == CART_CMD_VEL && req.v_mm_s == 0,
          "VEL 0");
    CHECK(p("VEL 32767") && req.v_mm_s == 32767, "VEL 上边界 32767");
    CHECK(p("VEL -32767") && req.v_mm_s == -32767, "VEL 下边界 -32767");
    CHECK(p("HOME") && req.cmd == CART_CMD_HOME, "HOME");
    CHECK(p("RESET") && req.cmd == CART_CMD_RESET, "RESET");
    CHECK(p("ENABLE") && req.cmd == CART_CMD_ENABLE, "ENABLE");
    CHECK(p("MARK LEFT") && req.cmd == CART_CMD_MARK_LEFT, "MARK LEFT");
    CHECK(p("MARK RIGHT") && req.cmd == CART_CMD_MARK_RIGHT, "MARK RIGHT");
    CHECK(p("QUERY") && req.cmd == CART_CMD_QUERY, "QUERY");
    CHECK(p("CONFIG?") && req.cmd == CART_CMD_CONFIG_DUMP, "CONFIG?");
    CHECK(p("CONFIG X_LIMIT 250000") && req.cmd == CART_CMD_CONFIG_SET
          && strcmp(req.name, "X_LIMIT") == 0 && req.value == 250000,
          "CONFIG X_LIMIT 250000");
    CHECK(p("CONFIG VEL_MAX 800") && req.value == 800, "CONFIG VEL_MAX 800");
    CHECK(p("CONFIG HOME_VEL 50") && req.value == 50, "CONFIG HOME_VEL 50");
    CHECK(p("CONFIG POLE_ZERO_URAD -3141592") && req.value == -3141592,
          "负值 CONFIG");

    /* 行尾仅容空白 */
    CHECK(p("HOME "), "行尾空格容忍");
    CHECK(p("VEL 200  ") && req.v_mm_s == 200, "行尾多空格容忍");
    CHECK(p("CONFIG VEL_MAX 800 \t"), "行尾制表符容忍");
}

/* ---- 非法集（静默丢弃） ---- */
static void test_illegal(void)
{
    CHECK(!p(""), "空行");
    CHECK(!p("   "), "纯空白行");
    CHECK(!p("vel 200"), "小写命令");
    CHECK(!p("Vel 200"), "混合大小写");
    CHECK(!p("VEL 1.5"), "VEL 浮点");
    CHECK(!p("VEL abc"), "VEL 非数字");
    CHECK(!p("VEL 200 300"), "VEL 多余参数");
    CHECK(!p("VEL "), "VEL 缺参数");
    CHECK(!p("VEL"), "VEL 无参数");
    CHECK(!p("VEL +200"), "VEL 加号（仅容负号）");
    CHECK(!p("VEL 32768"), "VEL 超上界 32768");
    CHECK(!p("VEL -32768"), "VEL 超下界 -32768");
    CHECK(!p("VEL 200x"), "VEL 尾随垃圾");
    CHECK(!p("VEL 0x10"), "VEL 十六进制");
    CHECK(!p("HOME NOW"), "HOME 带参数");
    CHECK(!p("QUERY 1"), "QUERY 带参数");
    CHECK(!p("RESET ALL"), "RESET 带参数");
    CHECK(!p("CONFIG x 1"), "NAME 小写");
    CHECK(!p("CONFIG VEL_MAX"), "缺值");
    CHECK(!p("CONFIG VEL_MAX 800 900"), "多余值");
    CHECK(!p("CONFIG VEL_MAX abc"), "值非数字");
    CHECK(!p("CONFIG? extra"), "CONFIG? 带参数");
    CHECK(!p(" CONFIG VEL_MAX 800"), "行首空格");
    CHECK(!p("CONFIG  VEL_MAX 800"), "双空格分隔");
    CHECK(!p("CONFIG VEL_MAX\r"), "行尾 \\r");
    CHECK(!p("VEL 200\r"), "VEL 行尾 \\r");
    CHECK(!p("CONFIG\n"), "行中控制字符");
    CHECK(!p("UNKNOWN 1"), "未知命令");
    CHECK(!p("CONFIG ABCDEFGHIJKLMNOP 1"), "NAME 超 15 字符");
    CHECK(!p("MARK"), "MARK 缺方向词");
    CHECK(!p("MARK UP"), "MARK 未知方向词");
    CHECK(!p("MARK LEFT extra"), "MARK 多余参数");
    CHECK(!p("MARK  LEFT"), "MARK 双空格分隔");
    CHECK(!p("MARKLEFT"), "MARK 粘连");
    CHECK(!p("ENABLE 1"), "ENABLE 带参数");
}

/* NAME 合法字符集 [A-Z0-9_]{1,15} */
static void test_name_charset(void)
{
    CHECK(p("CONFIG A_1 5") && strcmp(req.name, "A_1") == 0,
          "字母数字下划线");
    CHECK(!p("CONFIG A-1 5"), "连字符非法");
    CHECK(!p("CONFIG A.B 5"), "点非法");
    CHECK(!p("CONFIG VEL_MAX! 5"), "叹号非法");
}

int main(void)
{
    test_legal();
    test_illegal();
    test_name_charset();

    printf("cartproto_parse_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
