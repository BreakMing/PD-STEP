/*
 * mt6835_crc8 宿主测试 — 帧校验（T4-brief：poly 0x07、init 0、
 * 24 bit 输入 = ANGLE[20:0]<<3 | STATUS[2:0]，MSB 先行）
 *
 * 手算向量：
 *   (angle=0, status=0)：三块 0x00,0x00,0x00 → CRC 0x00
 *   (angle=1, status=0)：块 0x00,0x00,0x08 → 手算 0x38
 * 退出码 = 失败断言数。
 */

#include <stdio.h>
#include "mt6835_urad.h"

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
    CHECK_EQ(mt6835_crc8(0, 0), 0x00);
    CHECK_EQ(mt6835_crc8(1, 0), 0x38);

    printf("mt6835_crc_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
