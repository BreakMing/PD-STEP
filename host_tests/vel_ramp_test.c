/*
 * vel_ramp 宿主测试 — 手算精确向量逐条断言
 *
 * 编译：gcc -std=c11 -Wall -Wextra -I<component dir> vel_ramp_test.c vel_ramp.c
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include "vel_ramp.h"

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

/* 便捷：一次调用取全输出 */
static struct {
    uint64_t v;
    uint32_t dt;
    bool     dchg;
    bool     ret;
} r_;

static bool call(uint64_t v, bool dir, uint64_t target, bool tdir,
                 uint32_t accel, uint32_t decel, bool wdt)
{
    r_.v = 0xDEADBEEF;
    r_.dt = 0xDEADBEEF;
    r_.dchg = false;
    r_.ret = vel_ramp_next(v, dir, target, tdir, accel, decel, wdt,
                           &r_.v, &r_.dt, &r_.dchg);
    return r_.ret;
}

#define STEPS(n) ((uint64_t)(n) * VEL_RAMP_Q16_ONE)

/* ① 加速段：v=100 steps/s、a=2000 → dv=20，v'=120，dt=8333 */
static void test_accel_step(void)
{
    CHECK_EQ(call(STEPS(100), true, STEPS(200), true, 2000, 5000, false), 1);
    CHECK_EQ(r_.v, STEPS(120));
    CHECK_EQ(r_.dt, 8333);          /* 1e6*65536/7864320 = 8333.33 */
    CHECK_EQ(r_.dchg, 0);
}

/* ② 减速到零：v=5、a=2000 → dv=20 ≥ 5 → v'=0，返回 false */
static void test_decel_to_zero(void)
{
    CHECK_EQ(call(STEPS(5), true, 0, true, 2000, 5000, false), 0);
    CHECK_EQ(r_.v, 0);
}

/* ③ 加速钳位：v=100、target=110、dv=20 → v'=110 恰在目标 */
static void test_accel_clamp(void)
{
    CHECK_EQ(call(STEPS(100), true, STEPS(110), true, 2000, 5000, false), 1);
    CHECK_EQ(r_.v, STEPS(110));
    CHECK_EQ(r_.dt, 9090);          /* 1e6*65536/7208960 = 9090.9 */
}

/* ④ 看门狗超时：用 decel（500）而非 accel（2000）。
 * v=100、decel=500 → dv=5 → v'=95，dt=10526 */
static void test_wdt_uses_decel(void)
{
    CHECK_EQ(call(STEPS(100), true, STEPS(200), true, 2000, 500, true), 1);
    CHECK_EQ(r_.v, STEPS(95));
    CHECK_EQ(r_.dt, 10526);         /* 1e6*65536/6225920 = 10526.3 */
    CHECK_EQ(r_.dchg, 0);
}

/* ⑤ 零速起步（换向方向）：dt0 = isqrt(2e12/2000) = 31622 µs，
 * v1 = 2000*31622*65536/1e6 = 4144758（= 63.244 steps/s），out_dir_change = true */
static void test_start_from_zero_reversed(void)
{
    CHECK_EQ(call(0, true, STEPS(200), false, 2000, 5000, false), 1);
    CHECK_EQ(r_.dchg, 1);
    CHECK_EQ(r_.dt, 31622);
    CHECK_EQ(r_.v, 4144758LL);
}

/* ⑤b 零速起步（同向）：无换向 */
static void test_start_from_zero_same_dir(void)
{
    CHECK_EQ(call(0, true, STEPS(200), true, 2000, 5000, false), 1);
    CHECK_EQ(r_.dchg, 0);
    CHECK_EQ(r_.dt, 31622);
    CHECK_EQ(r_.v, 4144758LL);
}

/* ⑤c 零速且目标零：停表 */
static void test_zero_zero(void)
{
    CHECK_EQ(call(0, true, 0, true, 2000, 5000, false), 0);
    CHECK_EQ(r_.v, 0);
}

/* ⑥ 同向降速：v=200、target=100、a=2000 → dv=10 → v'=190 */
static void test_same_dir_slowdown(void)
{
    CHECK_EQ(call(STEPS(200), true, STEPS(100), true, 2000, 5000, false), 1);
    CHECK_EQ(r_.v, STEPS(190));
    CHECK_EQ(r_.dt, 5263);          /* 1e6*65536/12451840 = 5263.1 */
}

/* ⑦ 换向经零速（运行中）：v=5 向正、target=−200 → 到零立即换向起步 */
static void test_reversal_through_zero(void)
{
    CHECK_EQ(call(STEPS(5), true, STEPS(200), false, 2000, 5000, false), 1);
    CHECK_EQ(r_.dchg, 1);
    CHECK_EQ(r_.dt, 31622);
    CHECK_EQ(r_.v, 4144758LL);
}

/* ⑦b 换向减速中途：v=100 向正、target=−200、a=2000 → dv=20 → v'=80 */
static void test_reversal_decel_midway(void)
{
    CHECK_EQ(call(STEPS(100), true, STEPS(200), false, 2000, 5000, false), 1);
    CHECK_EQ(r_.dchg, 0);
    CHECK_EQ(r_.v, STEPS(80));
    CHECK_EQ(r_.dt, 12500);         /* 1e6*65536/(80*65536) = 12500 精确 */
}

/* ⑦c 换向尾端吸附：v=15 向正（低于吸附阈值的邻域）、target=−200
 * → 直接吸附归零并换向起步（不产生 <10 steps/s 的中间步） */
static void test_reversal_snap(void)
{
    CHECK_EQ(call(STEPS(15), true, STEPS(200), false, 2000, 5000, false), 1);
    CHECK_EQ(r_.dchg, 1);
    CHECK_EQ(r_.dt, 31622);
}

/* ⑦d 停止尾端吸附：v=15、target=0 → 吸附归零，返回 false */
static void test_stop_snap(void)
{
    CHECK_EQ(call(STEPS(15), true, 0, true, 2000, 5000, false), 0);
    CHECK_EQ(r_.v, 0);
}

/* ⑧ 稳态：v == target → dt = 1e6/200 = 5000 精确 */
static void test_steady(void)
{
    CHECK_EQ(call(STEPS(200), true, STEPS(200), true, 2000, 5000, false), 1);
    CHECK_EQ(r_.v, STEPS(200));
    CHECK_EQ(r_.dt, 5000);
}

/* ⑨ vel_ramp_start_dt 手算值 */
static void test_start_dt_values(void)
{
    CHECK_EQ(vel_ramp_start_dt(2000), 31622);      /* isqrt(1e9)     */
    CHECK_EQ(vel_ramp_start_dt(4000), 22360);      /* isqrt(5e8)     */
    CHECK_EQ(vel_ramp_start_dt(2000000), 1000);    /* isqrt(1e6)     */
    CHECK_EQ(vel_ramp_start_dt(4000 * 40), 3535);  /* isqrt(1.25e7)=3535.5 */
}

/* ⑩ 64 位范围：v = 80000 steps/s（2000 mm/s 上限，超出 32 位 Q16）。
 * accel=200000：dv = 200000/80000 = 2.5 steps/s = 163840 Q16，
 * v+dv = 5243043840 > target(5242881000) → 钳位到目标 */
static void test_wide_range(void)
{
    CHECK_EQ(call((uint64_t)80000 * 65536u, true,
                  (uint64_t)80000 * 65536u + 1000, true, 200000, 200000, false), 1);
    CHECK_EQ(r_.v, (uint64_t)80000 * 65536u + 1000);
}

int main(void)
{
    test_accel_step();
    test_decel_to_zero();
    test_accel_clamp();
    test_wdt_uses_decel();
    test_start_from_zero_reversed();
    test_start_from_zero_same_dir();
    test_zero_zero();
    test_same_dir_slowdown();
    test_reversal_through_zero();
    test_reversal_decel_midway();
    test_reversal_snap();
    test_stop_snap();
    test_steady();
    test_start_dt_values();
    test_wide_range();

    printf("vel_ramp_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
