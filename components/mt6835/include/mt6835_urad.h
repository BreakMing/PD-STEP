/*
 * mt6835_urad — 21-bit 原始角 → ±π µrad 环绕换算（纯函数，宿主可测）
 *
 * 量纲：µrad 取 π = 3141592（2π = 6283185，floor）；全整数 u64 中间量。
 * pipeline：raw → µrad → 减 zero → wrap 到 ±π →（可选）符号翻转。
 * invert 在 wrap 之后翻转符号，结果仍落在 ±π。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT6835_COUNTS_PER_REV  2097152u   /* 2^21 */
#define MT6835_PI_URAD   3141592   /* floor(π × 1e6) */
#define MT6835_2PI_URAD  6283185   /* floor(2π × 1e6) */

/**
 * @brief 21-bit 原始角 → ±π 环绕 µrad。
 *
 * raw ∈ [0, 2^21)；半圈（raw = 2^20）映射 +π；raw ∈ (2^20, 2^21)
 * 映射负角。zero 为机械零位偏置（µrad，标定量 POLE_ZERO_URAD 的值）；
 * invert 为符号翻转（POLE_INV）。
 */
int32_t mt6835_raw_to_urad(uint32_t raw21, int32_t zero_urad, bool invert);

/**
 * @brief 两次 21-bit 读数的多圈差分（处理 0/2^21 边界跨越）。
 *
 * 返回 a − b 的带符号圈数展开值；|真差| < 半圈时结果正确。
 */
int32_t mt6835_unwrap_diff(uint32_t a, uint32_t b);

/**
 * @brief 角度帧 CRC8（T4-brief：poly X⁸+X²+X+1 = 0x07、init 0x00、
 *        输入 24 bit = ANGLE[20:0]<<3 | STATUS[2:0]，MSB 先行）。
 *
 * 与器件回传的寄存器 0x006 比较，不等视为读失败（拔线时 MISO
 * 悬空常读 0x00，全零帧一并拒绝）。
 */
uint8_t mt6835_crc8(uint32_t angle21, uint8_t status);

#ifdef __cplusplus
}
#endif
