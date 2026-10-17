/*
 * ch224q 宿主测试 — 0x09 状态寄存器位解析（CH224DS1 V2.0）
 *
 * 位语义：bit0 BC1.2 握手成功、bit1 QC2.0、bit2 QC3.0、bit3 PD
 * 握手成功、bit4 EPR 已激活、bit5 EPR 存在（>100W）、bit6 AVS 存在。
 *
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include "ch224q_status.h"

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

static void test_decode_all_clear(void)
{
    ch224q_status_t st;
    ch224q_decode_status(0x00, &st);
    CHECK_EQ(st.bc12, 0);
    CHECK_EQ(st.qc2, 0);
    CHECK_EQ(st.qc3, 0);
    CHECK_EQ(st.pd, 0);
    CHECK_EQ(st.epr_active, 0);
    CHECK_EQ(st.epr_present, 0);
    CHECK_EQ(st.avs_present, 0);
}

static void test_decode_pd_success(void)
{
    /* rev2 关注态：PD 协商成功（bit3） */
    ch224q_status_t st;
    ch224q_decode_status(0x08, &st);
    CHECK_EQ(st.pd, 1);
    CHECK_EQ(st.bc12, 0);
    CHECK_EQ(st.qc2, 0);
    CHECK_EQ(st.qc3, 0);
}

static void test_decode_each_bit(void)
{
    /* raw → 依次仅置 bc12/qc2/qc3/pd/epr_active/epr_present/avs_present */
    uint8_t v[7] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40 };
    for (int i = 0; i < 7; i++) {
        ch224q_status_t st;
        ch224q_decode_status(v[i], &st);
        CHECK_EQ(((char *)&st)[i], 1);
        /* 其余位全零 */
        for (int j = 0; j < 7; j++) {
            if (j != i) {
                CHECK_EQ(((char *)&st)[j], 0);
            }
        }
    }
}

static void test_decode_null_safe(void)
{
    ch224q_decode_status(0xFF, NULL);   /* 不得崩溃 */
}

static void test_decode_all_set(void)
{
    ch224q_status_t st;
    ch224q_decode_status(0x7F, &st);
    CHECK_EQ(st.bc12, 1);
    CHECK_EQ(st.qc2, 1);
    CHECK_EQ(st.qc3, 1);
    CHECK_EQ(st.pd, 1);
    CHECK_EQ(st.epr_active, 1);
    CHECK_EQ(st.epr_present, 1);
    CHECK_EQ(st.avs_present, 1);
}

int main(void)
{
    test_decode_all_clear();
    test_decode_pd_success();
    test_decode_each_bit();
    test_decode_null_safe();
    test_decode_all_set();

    printf("%s：%d 例，失败 %d\n",
           s_fail == 0 ? "PASS" : "FAIL", s_run, s_fail);
    return s_fail;
}
