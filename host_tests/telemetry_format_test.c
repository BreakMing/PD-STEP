/*
 * telemetry_format_state 宿主测试 — 组帧契约
 *
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include <string.h>
#include "telemetry.h"
#include "as5600.h"

static int s_fail = 0;
static int s_run = 0;

#define CHECK(cond, msg) do { \
    s_run++; \
    if (!(cond)) { \
        s_fail++; \
        printf("FAIL %s:%d  %s\n", __func__, __LINE__, msg); \
    } \
} while (0)

static void test_basic_format(void)
{
    char buf[64];
    int n = telemetry_format_state(buf, sizeof(buf), 123456, -3141592, -200, 4500);
    CHECK(n > 0, "正常缓冲应返回正字节数");
    CHECK(strcmp(buf, "STATE 123456 -3141592 -200 4500") == 0, "格式应为四个空格分隔整数");
    CHECK(n == (int)strlen(buf), "返回值应等于字符串长度（不含 NUL）");
    CHECK(strchr(buf, '\n') == NULL, "不含换行（发送方补）");
}

static void test_zero_and_negative(void)
{
    char buf[64];
    telemetry_format_state(buf, sizeof(buf), 0, 0, 0, 0);
    CHECK(strcmp(buf, "STATE 0 0 0 0") == 0, "零值");

    telemetry_format_state(buf, sizeof(buf), -265000, 3141592, -1000, -30000);
    CHECK(strcmp(buf, "STATE -265000 3141592 -1000 -30000") == 0, "正负号字段");

    /* int32 极端值不溢出 */
    telemetry_format_state(buf, sizeof(buf), -2147483647, 2147483647, 0, 0);
    CHECK(strcmp(buf, "STATE -2147483647 2147483647 0 0") == 0, "int32 极值");
}

static void test_buffer_too_small(void)
{
    char buf[39];   /* 最长合法行 54 字节 > 39 */
    CHECK(telemetry_format_state(buf, sizeof(buf), -2147483647, -2147483647,
                                 -2147483647, -2147483647) < 0,
          "缓冲不足 40 字节应返回负值");
    CHECK(telemetry_format_state(buf, 0, 1, 1, 1, 1) < 0, "零长度应返回负值");
    CHECK(telemetry_format_state(NULL, 64, 1, 1, 1, 1) < 0, "NULL 缓冲应返回负值");
}

/* as5600 counts → µm 换算（内联于 as5600.h） */
static void test_counts_to_um(void)
{
    CHECK(as5600_counts_to_um(64) == 625, "64 count → 625 µm");
    CHECK(as5600_counts_to_um(-64) == -625, "−64 count → −625 µm");
    CHECK(as5600_counts_to_um(4096) == 40000, "一圈 4096 count → 40000 µm");
    CHECK(as5600_counts_to_um(0) == 0, "零");
    CHECK(as5600_counts_to_um(61440) == 600000, "全程 600 mm");
}

int main(void)
{
    test_basic_format();
    test_zero_and_negative();
    test_buffer_too_small();
    test_counts_to_um();

    printf("telemetry_format_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
