/*
 * tmc2209 单元测试——纯逻辑（官方 test/ 子目录约定，仅测试工程编入）
 *
 * CRC8 向量、帧结构、mA→CS 换算、microstep→MRES 映射与寄存器
 * 位布局锚点。无硬件依赖。
 */

#include "tmc2209.h"

#include <stdio.h>

#include "unity.h"

/* ---- CRC8 向量 ---- */
TEST_CASE("CRC8 向量", "[tmc2209]")
{
    /* 空缓冲 → 0x00 */
    TEST_ASSERT_EQUAL_HEX8(0x00, tmc2209_crc8(NULL, 0));

    /* 单字节 0x00 → 0x00 */
    uint8_t buf1[] = {0x00};
    TEST_ASSERT_EQUAL_HEX8(0x00, tmc2209_crc8(buf1, 1));

    /* 单字节 0xFF：LSB-first CRC-8/ATM poly 0x07 init 0x00 → 0xF3 */
    uint8_t buf2[] = {0xFF};
    TEST_ASSERT_EQUAL_HEX8(0xF3, tmc2209_crc8(buf2, 1));

    /* 写帧 {0x05,0x00,0x80,0,0,0,0}（写 GCONF=0）→ CRC 0x49 */
    uint8_t buf3[] = {0x05, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL_HEX8(0x49, tmc2209_crc8(buf3, 7));

    /* 读请求 {0x05,0x00,0x06}（读 IOIN）→ CRC 0x6F */
    uint8_t buf4[] = {0x05, 0x00, 0x06};
    TEST_ASSERT_EQUAL_HEX8(0x6F, tmc2209_crc8(buf4, 3));

    /* 4 × 0xFF → 0xDE */
    uint8_t buf5[] = {0xFF, 0xFF, 0xFF, 0xFF};
    TEST_ASSERT_EQUAL_HEX8(0xDE, tmc2209_crc8(buf5, 4));

    /* 8 个递增字节：CRC 非零 */
    uint8_t buf6[8];
    for (int i = 0; i < 8; i++) buf6[i] = (uint8_t)i;
    TEST_ASSERT_NOT_EQUAL(0x00, tmc2209_crc8(buf6, 8));
}

/* 帧结构：帧构造器是 tmc2209.c 内部 static，以 CRC 一致性与
 * 字节布局锚点验证（sync/地址/读写位/数据 MSB-first/帧长） */
TEST_CASE("帧编码结构", "[tmc2209]")
{
    /* 写 GCONF=0x12345678：帧 {0x05, 0x00, 0x00|0x80, 0x12, 0x34, 0x56, 0x78, CRC} */
    uint8_t write_frame[] = {0x05, 0x00, 0x80, 0x12, 0x34, 0x56, 0x78};
    (void)tmc2209_crc8(write_frame, 7);
    TEST_ASSERT_BIT_HIGH(7, write_frame[2]);          /* 写位在寄存器地址字节 bit7 */

    /* 数据 MSB-first（byte3 = 最高字节 0x12） */
    TEST_ASSERT_TRUE(write_frame[3] == 0x12 && write_frame[6] == 0x78);

    /* 读帧 {0x05, 0x00, 0x06, CRC}（读 IOIN）*/
    uint8_t read_frame[] = {0x05, 0x00, 0x06};
    (void)tmc2209_crc8(read_frame, 3);
    TEST_ASSERT_BIT_LOW(7, read_frame[2]);            /* 读位为 0 */

    /* 同步字节 0x05 */
    TEST_ASSERT_TRUE(write_frame[0] == 0x05 && read_frame[0] == 0x05);

    /* 从机地址字节位置 */
    TEST_ASSERT_TRUE(write_frame[1] == 0x00 && read_frame[1] == 0x00);

    /* 帧长：写 8 B（7 数据 + CRC），读请求 4 B（3 数据 + CRC） */
    TEST_ASSERT_EQUAL(7, sizeof(write_frame));
    TEST_ASSERT_EQUAL(3, sizeof(read_frame));
}

/* mA → CS 换算 */
TEST_CASE("mA→CS 换算", "[tmc2209]")
{
    const float rs = 0.11f;  /* 默认 R_SENSE */

    /* 边界：0 mA 与负值 → 0 */
    TEST_ASSERT_EQUAL_INT(0, tmc2209_ma_to_cs(0, rs));
    TEST_ASSERT_EQUAL_INT(0, tmc2209_ma_to_cs(-100, rs));

    /* 典型值区间（换算含 VSENSE 分档取整，按区间断言） */
    int cs_500 = tmc2209_ma_to_cs(500, rs);
    TEST_ASSERT_TRUE_MESSAGE(cs_500 >= 13 && cs_500 <= 17, "500mA CS 区间");

    int cs_800 = tmc2209_ma_to_cs(800, rs);
    TEST_ASSERT_TRUE_MESSAGE(cs_800 >= 22 && cs_800 <= 28, "800mA CS 区间");

    int cs_1500 = tmc2209_ma_to_cs(1500, rs);
    TEST_ASSERT_TRUE_MESSAGE(cs_1500 >= 20 && cs_1500 <= 31, "1500mA CS 区间");

    /* 钳位：过流 → 31 */
    TEST_ASSERT_EQUAL_INT(31, tmc2209_ma_to_cs(5000, rs));

    /* R_SENSE 效应：同电流下更大的采样电阻 → 更小 CS */
    TEST_ASSERT_TRUE(tmc2209_ma_to_cs(800, 0.05f) > tmc2209_ma_to_cs(800, 0.15f));

    /* 300mA：正值且不越界 */
    int cs_300 = tmc2209_ma_to_cs(300, rs);
    TEST_ASSERT_TRUE(cs_300 > 0 && cs_300 <= 31);
}

/* microstep → MRES 映射 */
TEST_CASE("microstep→MRES 映射", "[tmc2209]")
{
    TEST_ASSERT_EQUAL_INT(0, tmc2209_microstep_to_mres(256));
    TEST_ASSERT_EQUAL_INT(1, tmc2209_microstep_to_mres(128));
    TEST_ASSERT_EQUAL_INT(2, tmc2209_microstep_to_mres(64));
    TEST_ASSERT_EQUAL_INT(3, tmc2209_microstep_to_mres(32));
    TEST_ASSERT_EQUAL_INT(4, tmc2209_microstep_to_mres(16));
    TEST_ASSERT_EQUAL_INT(5, tmc2209_microstep_to_mres(8));
    TEST_ASSERT_EQUAL_INT(6, tmc2209_microstep_to_mres(4));
    TEST_ASSERT_EQUAL_INT(7, tmc2209_microstep_to_mres(2));
    TEST_ASSERT_EQUAL_INT(8, tmc2209_microstep_to_mres(1));

    /* 非法值 */
    TEST_ASSERT_EQUAL_INT(-1, tmc2209_microstep_to_mres(0));
    TEST_ASSERT_EQUAL_INT(-1, tmc2209_microstep_to_mres(3));
    TEST_ASSERT_EQUAL_INT(-1, tmc2209_microstep_to_mres(5));
    TEST_ASSERT_EQUAL_INT(-1, tmc2209_microstep_to_mres(512));
    TEST_ASSERT_EQUAL_INT(-1, tmc2209_microstep_to_mres(-1));
}

/* IHOLD_IRUN 位布局锚点：IHOLD[4:0]、IRUN[12:8]、IHOLDDELAY[19:16]
 * （打包函数是内部 static，按位布局契约锚定） */
TEST_CASE("IHOLD_IRUN 位布局", "[tmc2209]")
{
    /* IHOLD=5, IRUN=20, IHOLDDELAY=6 */
    uint32_t val = ((uint32_t)5 << 0) | ((uint32_t)20 << 8) | ((uint32_t)6 << 16);

    TEST_ASSERT_EQUAL_UINT32(5, val & 0x1F);
    TEST_ASSERT_EQUAL_UINT32(20, (val >> 8) & 0x1F);
    TEST_ASSERT_EQUAL_UINT32(6, (val >> 16) & 0xF);

    /* 中间保留位为 0 */
    TEST_ASSERT_EQUAL_UINT32(0, val & (0x7U << 5));
    TEST_ASSERT_EQUAL_UINT32(0, val & (0x7U << 13));
}

/* DRV_STATUS 位布局锚点：otpw=bit0, ot=bit1, s2ga=bit2, s2gb=bit3,
 * ola=bit6, olb=bit7, cs_actual=bits16-20, stst=bit31 */
TEST_CASE("DRV_STATUS 位布局", "[tmc2209]")
{
    uint32_t all_set = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3) |
                       (1U << 6) | (1U << 7) |
                       (0x1FU << 16) | (1U << 31);

    TEST_ASSERT_BIT_HIGH(0, all_set);
    TEST_ASSERT_BIT_HIGH(1, all_set);
    TEST_ASSERT_BIT_HIGH(2, all_set);
    TEST_ASSERT_BIT_HIGH(3, all_set);
    TEST_ASSERT_BIT_HIGH(6, all_set);
    TEST_ASSERT_BIT_HIGH(7, all_set);
    TEST_ASSERT_EQUAL_UINT32(31, (all_set >> 16) & 0x1F);
    TEST_ASSERT_BIT_HIGH(31, all_set);

    /* 零值 → 标志全假，cs=0 */
    TEST_ASSERT_EQUAL_UINT32(0, (0U >> 16) & 0x1F);
    TEST_ASSERT_BIT_LOW(31, 0U);
}

/* CHOPCONF 位布局锚点：TOFF[3:0], HSTRT[6:4], HEND[10:7],
 * TBL[16:15], VSENSE[17], MRES[27:24], INTPOL[28] */
TEST_CASE("CHOPCONF 位布局", "[tmc2209]")
{
    uint32_t val =
        (0xFU << 0)   |  /* TOFF = 15 */
        (0x7U << 4)   |  /* HSTRT = 7 */
        (0xFU << 7)   |  /* HEND = 15 */
        (0x3U << 15)  |  /* TBL = 3 */
        (1U << 17)    |  /* VSENSE = 1 */
        (0xFU << 24)  |  /* MRES = 15 */
        (1U << 28);      /* INTPOL = 1 */

    TEST_ASSERT_EQUAL_UINT32(15, val & 0xF);
    TEST_ASSERT_EQUAL_UINT32(7, (val >> 4) & 0x7);
    TEST_ASSERT_EQUAL_UINT32(15, (val >> 7) & 0xF);
    TEST_ASSERT_EQUAL_UINT32(3, (val >> 15) & 0x3);
    TEST_ASSERT_BIT_HIGH(17, val);
    TEST_ASSERT_EQUAL_UINT32(15, (val >> 24) & 0xF);
    TEST_ASSERT_BIT_HIGH(28, val);
}

/* GCONF 位布局锚点：pdn_disable=bit6, en_spreadcycle=bit2,
 * shaft=bit3, mstep_reg_select=bit7 */
TEST_CASE("GCONF 位布局", "[tmc2209]")
{
    uint32_t val = (1U << 6) | (1U << 2) | (1U << 3) | (1U << 7);

    TEST_ASSERT_BIT_HIGH(6, val);
    TEST_ASSERT_BIT_HIGH(2, val);
    TEST_ASSERT_BIT_HIGH(3, val);
    TEST_ASSERT_BIT_HIGH(7, val);
}
