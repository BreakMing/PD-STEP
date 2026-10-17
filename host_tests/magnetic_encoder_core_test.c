/*
 * magnetic_encoder_core 宿主测试 — 骨架纯函数（失败计数状态机 +
 * 16 ms 速度窗）
 *
 * 失败语义锚定契约：连续 ≥10 次读失败 NOT OK，一次成功恢复；
 * 速度语义锚定契约：17 样本窗位置差 × 62.5（量子 62.5 counts/s），
 * 窗口未满为 0。
 *
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include "magnetic_encoder_core.h"

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

/* ================= 失败计数状态机 ================= */

static void test_fail_init_and_healthy(void)
{
    magnetic_encoder_fail_t f;
    magnetic_encoder_fail_reset(&f);

    CHECK_EQ(f.ok, 1);
    CHECK_EQ(f.streak, 0);

    /* 持续成功：无事件、保持健康 */
    for (int i = 0; i < 100; i++) {
        CHECK_EQ(magnetic_encoder_fail_step(&f, true), 0);
    }
    CHECK_EQ(f.ok, 1);
    CHECK_EQ(f.streak, 0);
}

static void test_fail_trip_at_limit(void)
{
    magnetic_encoder_fail_t f;
    magnetic_encoder_fail_reset(&f);

    /* 第 1..9 次失败：无事件、仍 OK */
    for (int i = 1; i <= 9; i++) {
        CHECK_EQ(magnetic_encoder_fail_step(&f, false), 0);
        CHECK_EQ(f.ok, 1);
        CHECK_EQ(f.streak, i);
    }

    /* 第 10 次：进入 NOT OK（事件恰发生一次） */
    CHECK_EQ(magnetic_encoder_fail_step(&f, false), 1);
    CHECK_EQ(f.ok, 0);
    CHECK_EQ(f.streak, 10);

    /* 第 11+ 次：仍 NOT OK 但不再触发事件 */
    for (int i = 0; i < 50; i++) {
        CHECK_EQ(magnetic_encoder_fail_step(&f, false), 0);
        CHECK_EQ(f.ok, 0);
    }
    CHECK_EQ(f.streak, 60);
}

static void test_fail_recover(void)
{
    magnetic_encoder_fail_t f;
    magnetic_encoder_fail_reset(&f);

    for (int i = 0; i < 10; i++) {
        magnetic_encoder_fail_step(&f, false);
    }
    CHECK_EQ(f.ok, 0);

    /* 一次成功即恢复（恢复也是事件，供日志打印） */
    CHECK_EQ(magnetic_encoder_fail_step(&f, true), 1);
    CHECK_EQ(f.ok, 1);
    CHECK_EQ(f.streak, 0);

    /* 恢复后继续成功：无事件 */
    CHECK_EQ(magnetic_encoder_fail_step(&f, true), 0);
}

static void test_fail_below_limit_no_trip(void)
{
    magnetic_encoder_fail_t f;
    magnetic_encoder_fail_reset(&f);

    /* 9 次失败后成功：从未 NOT OK，无事件 */
    for (int i = 0; i < 9; i++) {
        magnetic_encoder_fail_step(&f, false);
    }
    CHECK_EQ(magnetic_encoder_fail_step(&f, true), 0);
    CHECK_EQ(f.ok, 1);
    CHECK_EQ(f.streak, 0);

    /* streak 归零后需重新累计 10 次才触发 */
    for (int i = 0; i < 9; i++) {
        magnetic_encoder_fail_step(&f, false);
    }
    CHECK_EQ(magnetic_encoder_fail_step(&f, false), 1);
    CHECK_EQ(f.ok, 0);
}

/* ================= 16 ms 速度窗 ================= */

/* 喂 n 个样本（从 0 开始每个 +step）并返回最后一次速度 */
static int32_t feed_ramp(magnetic_encoder_vel_win_t *w, int64_t start,
                         int n, int64_t step)
{
    int32_t v = 0;
    for (int i = 0; i < n; i++) {
        v = magnetic_encoder_vel_push(w, start + (int64_t)(i + 1) * step);
    }
    return v;
}

static void test_vel_warmup_zero(void)
{
    magnetic_encoder_vel_win_t w;
    magnetic_encoder_vel_win_reset(&w);

    /* 窗未满（<17 样本）恒 0 */
    for (int i = 1; i <= 16; i++) {
        CHECK_EQ(magnetic_encoder_vel_push(&w, i * 100), 0);
    }
    /* 第 17 个样本起出速度：16 ms 窗差 1600 counts → 100000 counts/s */
    CHECK_EQ(magnetic_encoder_vel_push(&w, 17 * 100), 100000);
}

static void test_vel_constant_rate(void)
{
    magnetic_encoder_vel_win_t w;
    magnetic_encoder_vel_win_reset(&w);

    /* 恒速 +1 count/ms → 1000 counts/s */
    int32_t v = feed_ramp(&w, 0, 17, 1);
    CHECK_EQ(v, 1000);
    CHECK_EQ(magnetic_encoder_vel_push(&w, 18), 1000);

    /* 恒速 −1 → −1000 */
    magnetic_encoder_vel_win_reset(&w);
    v = feed_ramp(&w, 0, 17, -1);
    CHECK_EQ(v, -1000);
}

static void test_vel_quantum_62_5(void)
{
    magnetic_encoder_vel_win_t w;
    magnetic_encoder_vel_win_reset(&w);

    /* 静止填满窗后单步 +1：delta=1 → 1×62.5 → llround=63 */
    for (int i = 0; i < 17; i++) {
        magnetic_encoder_vel_push(&w, 0);
    }
    CHECK_EQ(magnetic_encoder_vel_push(&w, 1), 63);
}

static void test_vel_slide_and_zero(void)
{
    magnetic_encoder_vel_win_t w;
    magnetic_encoder_vel_win_reset(&w);

    /* 匀速 100/ms 填满 → 100000 counts/s；停住后 oldest 逐样本推过
     * 运动样本，速度每步衰减 6250，第 16 个静止样本起窗全静止为 0 */
    int32_t v = feed_ramp(&w, 0, 17, 100);
    CHECK_EQ(v, 100000);
    for (int k = 1; k <= 16; k++) {
        v = magnetic_encoder_vel_push(&w, 17 * 100);
        CHECK_EQ(v, 6250 * (16 - k));
    }
}

int main(void)
{
    test_fail_init_and_healthy();
    test_fail_trip_at_limit();
    test_fail_recover();
    test_fail_below_limit_no_trip();
    test_vel_warmup_zero();
    test_vel_constant_rate();
    test_vel_quantum_62_5();
    test_vel_slide_and_zero();

    printf("%s：%d 例，失败 %d\n",
           s_fail == 0 ? "PASS" : "FAIL", s_run, s_fail);
    return s_fail;
}
