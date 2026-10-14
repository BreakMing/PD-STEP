/*
 * mt6835_urad 宿主测试 — 手算精确向量
 *
 * 退出码 = 失败断言数（0 = 全过）。
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

/* 环绕换算 */
static void test_raw_to_urad(void)
{
    CHECK_EQ(mt6835_raw_to_urad(0, 0, false), 0);
    /* 半圈 = +π（边界保持正） */
    CHECK_EQ(mt6835_raw_to_urad(1048576u, 0, false), 3141592);
    /* 过半圈 1 count：floor(3141595.5)=3141595 → wrap → −3141590 */
    CHECK_EQ(mt6835_raw_to_urad(1048577u, 0, false), -3141590);
    /* 最大读数：floor(6283182.0)=6283182 → wrap → 6283182−6283185 = −3 */
    CHECK_EQ(mt6835_raw_to_urad(2097151u, 0, false), -3);
    /* 四分之一圈：524288×6283185/2097152 = 1570796（floor 1570796.25） */
    CHECK_EQ(mt6835_raw_to_urad(524288u, 0, false), 1570796);
}

/* zero 平移 */
static void test_zero_offset(void)
{
    CHECK_EQ(mt6835_raw_to_urad(0, 100000, false), -100000);
    CHECK_EQ(mt6835_raw_to_urad(524288u, 1570796, false), 0);
    /* 平移越过 +π 需要 wrap：1570796 + 2000000 = 3570796 → −2712389 */
    CHECK_EQ(mt6835_raw_to_urad(524288u, -2000000, false), -2712389);
}

/* invert 翻转（wrap 之后） */
static void test_invert(void)
{
    CHECK_EQ(mt6835_raw_to_urad(524288u, 0, true), -1570796);
    CHECK_EQ(mt6835_raw_to_urad(0, 0, true), 0);
    /* wrap 后再翻转：raw=524288, zero=−2000000 → −2712389 → 翻转 +2712389 */
    CHECK_EQ(mt6835_raw_to_urad(524288u, -2000000, true), 2712389);
}

/* 多圈差分 */
static void test_unwrap_diff(void)
{
    CHECK_EQ(mt6835_unwrap_diff(10, 0), 10);
    CHECK_EQ(mt6835_unwrap_diff(0, 10), -10);
    /* 跨 0 边界正转：0 ← 2097151 → +1 */
    CHECK_EQ(mt6835_unwrap_diff(0, 2097151u), 1);
    /* 跨 0 边界反转：2097151 ← 0 → −1 */
    CHECK_EQ(mt6835_unwrap_diff(2097151u, 0), -1);
    /* 半圈差（歧义边界，取负） */
    CHECK_EQ(mt6835_unwrap_diff(0, 1048576u), -1048576);
}

int main(void)
{
    test_raw_to_urad();
    test_zero_offset();
    test_invert();
    test_unwrap_diff();

    printf("mt6835_urad_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
