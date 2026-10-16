/*
 * cartpos — 小车位置解释组件（编码器实测位置）
 *
 * 持有 magnetic_encoder 句柄，把原始 counts 解释为 mm 域小车位置：
 * 比例随每转行程（units_per_rev）订阅刷新，原点偏移存 µm 整数。
 * x_um = round(counts × 每转行程 × 1000 ÷ counts_per_rev) − 原点偏移。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 绑定编码器实例并订阅每转行程（比例随变更刷新）。 */
void cartpos_init(magnetic_encoder_handle_t enc);

/** 连续位置（多圈、带符号、含回零原点偏移），µm 整数；首样本前为 0。 */
int64_t cartpos_position_um(void);

/** 16 ms 窗平滑速度 mm/s；窗口未满为 0。 */
float cartpos_velocity_mm_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool cartpos_is_ok(void);

/** 声明当前位置为 new_pos_mm（纯偏移建立，不产生运动）。 */
void cartpos_set_origin_mm(float new_pos_mm);

#ifdef __cplusplus
}
#endif
