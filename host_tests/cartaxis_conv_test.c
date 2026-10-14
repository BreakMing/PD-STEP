/*
 * cartaxis_conv 宿主测试 — 手算精确向量逐条断言
 *
 * 编译：gcc -std=c11 -Wall -Wextra -Icomponents/cartaxis/include \
 *       host_tests/cartaxis_conv_test.c components/cartaxis/cartaxis_conv.c -lm
 * 退出码 = 失败断言数（0 = 全过）。
 */
#include <stdio.h>
#include "cartaxis_conv.h"

static int s_fail = 0;
static int s_run = 0;

#define CHECK_EQ(expr, expect) do { \
    long long _e = (long long)(expect); \
    long long _a = (long long)(expr); \
    s_run++; \
    if (_e != _a) { \
        s_fail++; \
        printf("FAIL %s:%d  %s = %lld, 期望 %lld\n", \
               __func__, __LINE__, #expr, _a, _e); \
    } \
} while (0)

int main(void)
{
    /* steps_per_mm = 40（默认 $0=8、$1=200、$2=40） */
    CHECK_EQ(cartaxis_mmps_to_steps(0,    40.0), 0);
    CHECK_EQ(cartaxis_mmps_to_steps(1000, 40.0), 40000);
    CHECK_EQ(cartaxis_mmps_to_steps(-250, 40.0), -10000);
    /* VEL_MAX 注册表上限 2000 mm/s → 80000 steps/s */
    CHECK_EQ(cartaxis_mmps_to_steps(2000, 40.0), 80000);
    /* lround 半数远离零 */
    CHECK_EQ(cartaxis_mmps_to_steps(1, 0.4), 0);
    CHECK_EQ(cartaxis_mmps_to_steps(1, 0.5), 1);
    CHECK_EQ(cartaxis_mmps_to_steps(-1, 0.5), -1);

    CHECK_EQ(cartaxis_accel_to_steps(0,    40.0), 0);
    CHECK_EQ(cartaxis_accel_to_steps(4000, 40.0), 160000);

    CHECK_EQ(cartaxis_steps_to_um(0,      40.0), 0);
    CHECK_EQ(cartaxis_steps_to_um(160000, 40.0), 4000000);
    /* 舍入边界：62.5 → 63（llround 半数远离零），负向对称 */
    CHECK_EQ(cartaxis_steps_to_um(1,  16.0), 63);
    CHECK_EQ(cartaxis_steps_to_um(-1, 16.0), -63);

    CHECK_EQ(cartaxis_um_to_steps(4000000, 40.0), 160000);
    CHECK_EQ(cartaxis_um_to_steps(25, 40.0), 1);  /* 1.0 精确 */
    CHECK_EQ(cartaxis_um_to_steps(13, 40.0), 1);  /* 0.52 → 1 */
    CHECK_EQ(cartaxis_um_to_steps(12, 40.0), 0);  /* 0.48 → 0 */
    CHECK_EQ(cartaxis_um_to_steps(1,  40.0), 0);  /* 0.04 → 0 */

    printf("%s: %d 用例, %d 失败\n", s_fail ? "FAIL" : "PASS", s_run, s_fail);
    return s_fail;
}
