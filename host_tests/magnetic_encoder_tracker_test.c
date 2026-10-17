/*
 * magnetic_encoder_tracker 宿主测试 — 参数化多圈累计纯逻辑
 *
 * 覆盖：种子语义、正向/反向跨界、边沿抖动退化为 ±1、非对称阈值
 * 全值域恰好覆盖一次（4096 与 2^21 两种 counts/rev）、多圈累计与
 * int64 域。语义承接 as5600_tracker（A 组向量）并统一 mt6835 域。
 *
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include <string.h>
#include "magnetic_encoder_tracker.h"

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

#define AS_CPR   4096
#define MT_CPR   2097152

/* 推进模拟轴 step counts × steps 次，raw 按 cpr 取模回绕（同真实传感器） */
static int64_t sim_run(magnetic_encoder_tracker_t *t, uint32_t *raw,
                       int32_t cpr, int steps, int32_t step)
{
    for (int i = 0; i < steps; i++) {
        int64_t next = ((int64_t)*raw + step) % cpr;
        if (next < 0) {
            next += cpr;
        }
        *raw = (uint32_t)next;
        magnetic_encoder_tracker_update(t, *raw, cpr);
    }
    return t->pos_counts;
}

/* ---- 种子与静止 ---- */

static void test_seed_and_still(void)
{
    magnetic_encoder_tracker_t t;
    magnetic_encoder_tracker_reset(&t);

    /* 首样本只播种：位置为 0，与 raw 初值无关 */
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 100, AS_CPR), 0);
    CHECK_EQ(t.seeded, 1);
    CHECK_EQ(t.pos_counts, 0);
    CHECK_EQ(t.last_raw, 100);

    /* 静止：相同 raw → 0 */
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 100, AS_CPR), 0);

    /* reset 后重新播种 */
    magnetic_encoder_tracker_reset(&t);
    CHECK_EQ(t.seeded, 0);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 3000, AS_CPR), 0);
}

/* ---- 基本计数与跨界（承接 as5600 A 组向量） ---- */

static void test_basic_and_edges(void)
{
    magnetic_encoder_tracker_t t;
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 100, AS_CPR);
    magnetic_encoder_tracker_update(&t, 200, AS_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 300, AS_CPR), 200);

    /* 正向跨界 4090 → 10 = +16 */
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 4090, AS_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 10, AS_CPR), 16);

    /* 反向跨界 10 → 4090 = −16 */
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 10, AS_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 4090, AS_CPR), -16);

    /* 非对称阈值边界（cpr=4096；返回值为累计位置，各序列独立种子）：
     * +2048 恰在阈上直通；−2047 恰在阈下直通；+2049 超阈 → −2047 */
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 0, AS_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 2048, AS_CPR), 2048);

    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 2048, AS_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 1, AS_CPR), -2047);

    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 0, AS_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 2049, AS_CPR), -2047);
}

/* ---- 边沿抖动：4095↔0 交替退化为 ±1 ---- */

static void test_edge_dither(void)
{
    magnetic_encoder_tracker_t t;
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, 4095, AS_CPR);

    CHECK_EQ(magnetic_encoder_tracker_update(&t, 0, AS_CPR), 1);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 4095, AS_CPR), 0);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 0, AS_CPR), 1);

    /* 大 cpr 同一规则：cpr−1 ↔ 0 */
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, MT_CPR - 1, MT_CPR);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 0, MT_CPR), 1);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, MT_CPR - 1, MT_CPR), 0);
}

/* ---- 非对称阈值全值域恰好覆盖一次 ----
 * 固定 last_raw，遍历全部 raw：输出 diff 的取值集合大小 == cpr
 * （无丢失），且每个值只出现一次（无重复）——即 ±cpr/2 非对称
 * 阈值对全部 cpr 个输入构成双射。
 */
static bool s_seen[MT_CPR];

static void check_bijective(int32_t cpr, uint32_t last_raw)
{
    memset(s_seen, 0, (size_t)cpr);
    magnetic_encoder_tracker_t t;

    for (uint32_t raw = 0; raw < (uint32_t)cpr; raw++) {
        /* 每个候选 raw 从固定 last_raw 的种子态独立出发 */
        magnetic_encoder_tracker_reset(&t);
        magnetic_encoder_tracker_update(&t, last_raw, cpr);

        int64_t before = t.pos_counts;
        magnetic_encoder_tracker_update(&t, raw, cpr);
        int64_t diff = t.pos_counts - before;
        /* 合法输出域：[−(cpr/2−1), +cpr/2] */
        if (diff < -(cpr / 2 - 1) || diff > cpr / 2) {
            s_fail++;
            s_run++;
            printf("FAIL check_bijective  diff=%lld 超出输出域 (cpr=%d)\n",
                   (long long)diff, cpr);
            return;
        }
        /* 偏移到非负索引：diff + cpr/2 − 1 ∈ [0, cpr−1] */
        int idx = (int)(diff + cpr / 2 - 1);
        if (s_seen[idx]) {
            s_fail++;
            s_run++;
            printf("FAIL check_bijective  diff=%lld 重复出现 (cpr=%d, last_raw=%u)\n",
                   (long long)diff, cpr, last_raw);
            return;
        }
        s_seen[idx] = true;
    }
    s_run++;   /* 一组双射成立记 1 例 */
}

static void test_bijective_full_domain(void)
{
    check_bijective(AS_CPR, 0);
    check_bijective(AS_CPR, 1234);
    check_bijective(MT_CPR, 0);
    check_bijective(MT_CPR, 1234567);
}

/* ---- 多圈累计与整圈守恒 ---- */

static void test_multiturn(void)
{
    magnetic_encoder_tracker_t t;
    uint32_t raw = 100;

    /* 正向 3 整圈 + 50 counts（cpr=4096） */
    magnetic_encoder_tracker_reset(&t);
    magnetic_encoder_tracker_update(&t, raw, AS_CPR);
    sim_run(&t, &raw, AS_CPR, 3 * AS_CPR + 50, 1);
    CHECK_EQ(t.pos_counts, 3 * AS_CPR + 50);

    /* 对称往返净零 */
    sim_run(&t, &raw, AS_CPR, 3 * AS_CPR + 50, -1);
    CHECK_EQ(t.pos_counts, 0);

    /* 大 cpr 整圈与 int64 域：一圈 2^21 已是 int32 的 1/1024，
     * 每样本半圈推进 4096 步 = 2048 圈 = 2^32 counts，超 int32 仍精确 */
    magnetic_encoder_tracker_reset(&t);
    raw = 0;
    magnetic_encoder_tracker_update(&t, raw, MT_CPR);
    sim_run(&t, &raw, MT_CPR, 4096, MT_CPR / 2);
    CHECK_EQ(t.pos_counts, 4294967296LL);
    CHECK_EQ(t.pos_counts > INT32_MAX, 1);
}

/* ---- 高位清零：raw 输入超出 cpr 位宽时按掩码处理 ---- */

static void test_raw_masked(void)
{
    magnetic_encoder_tracker_t t;
    magnetic_encoder_tracker_reset(&t);
    /* cpr=4096 只看低 12 bit：0x1ABC 的 raw 与 0xABC 等价 */
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 0x1ABC, AS_CPR), 0);
    CHECK_EQ(t.last_raw, 0xABC);
    CHECK_EQ(magnetic_encoder_tracker_update(&t, 0xABD, AS_CPR), 1);
}

int main(void)
{
    test_seed_and_still();
    test_basic_and_edges();
    test_edge_dither();
    test_bijective_full_domain();
    test_multiturn();
    test_raw_masked();

    printf("%s：%d 例，失败 %d\n",
           s_fail == 0 ? "PASS" : "FAIL", s_run, s_fail);
    return s_fail;
}
