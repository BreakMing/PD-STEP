/*
 * ch224q_status — CH224Q 0x09 状态寄存器位解析（纯逻辑，宿主可测）
 *
 * 位语义（CH224DS1 V2.0）：bit0 BC1.2 握手成功、bit1 QC2.0、
 * bit2 QC3.0、bit3 PD 握手成功、bit4 EPR 已激活、bit5 EPR 存在
 * （>100W）、bit6 AVS 存在。bit7 保留（解析时忽略）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool bc12;          /* bit0：BC1.2 握手成功 */
    bool qc2;           /* bit1：QC2.0 */
    bool qc3;           /* bit2：QC3.0 */
    bool pd;            /* bit3：PD 握手成功（rev2 关注态：20V 协商成立） */
    bool epr_active;    /* bit4：EPR 已激活 */
    bool epr_present;   /* bit5：EPR 存在（>100W） */
    bool avs_present;   /* bit6：AVS 存在 */
} ch224q_status_t;

/**
 * @brief 解析 0x09 状态寄存器原文。
 *
 * st 为 NULL 时直接返回（不崩溃）；保留位（bit7）忽略。
 */
void ch224q_decode_status(uint8_t raw, ch224q_status_t *st);

#ifdef __cplusplus
}
#endif
