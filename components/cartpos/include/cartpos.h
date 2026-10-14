/*
 * cartpos — 小车位置域契约（编码器实测位置）
 *
 * 链接期绑定：本头文件只含接口声明，实现源文件在同组件内
 * （当前实现 cartpos_as5600.c，经 Kconfig choice 选择）。
 * 语义：docs/superpowers/specs/2026-10-14-device-decoupling-design.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 连续位置（多圈、带符号、含回零原点偏移），µm 整数精确；首样本前为 0。 */
int64_t cartpos_position_um(void);

/** 16 ms 窗口平滑速度 mm/s；窗口未满为 0。 */
float cartpos_velocity_mm_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool cartpos_is_ok(void);

/** 声明当前位置为 new_pos_mm（纯偏移建立，不产生运动）。 */
void cartpos_set_origin_mm(float new_pos_mm);

#ifdef __cplusplus
}
#endif
