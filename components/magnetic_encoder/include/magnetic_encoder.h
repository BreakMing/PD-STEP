/*
 * magnetic_encoder — 绝对式磁编码器类别契约（句柄式多实例）
 *
 * 原生单位原则：契约只含 counts 与每转计数，不出现机械单位
 * （mm、µrad）与已标定角度——换算归解释层（cartpos / poleang）。
 * 命名与 count / accumulator 词汇参考 espp/magnetic_encoder。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct magnetic_encoder_s *magnetic_encoder_handle_t;

/** 每转原始计数（分辨率）：AS5600 为 4096，MT6835 为 2097152。 */
int32_t magnetic_encoder_counts_per_rev(magnetic_encoder_handle_t enc);

/**
 * @brief 多圈累计位置（accumulator：原始 counts，带符号，原生方向，
 *        自 init 累计）；首样本前为 0。
 *
 * int64 承载：连续单方向旋转不溢出。混叠上界：unwrap 以
 * ±每转计数/2 为阈值，单个采样周期内转过超过半转即混叠——本值
 * 与速度估计都不可信（AS5600 @1 kHz 上界约 122 rev/s，MT6835
 * 约 403 rev/s，远高于本台位物理速度）。
 */
int64_t magnetic_encoder_position_counts(magnetic_encoder_handle_t enc);

/** 角速度估计（counts/s，16 窗均值）。量子 62.5 counts/s；窗口未满为 0。 */
int32_t magnetic_encoder_velocity_counts_per_s(magnetic_encoder_handle_t enc);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool magnetic_encoder_is_ok(magnetic_encoder_handle_t enc);

#ifdef __cplusplus
}
#endif
