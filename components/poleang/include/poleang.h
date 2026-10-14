/*
 * poleang — 摆角解释组件（已标定摆角）
 *
 * 持有 magnetic_encoder 句柄，把原始角量解释为已标定摆角：
 * 零位与符号（POLE_ZERO_URAD / POLE_INV，来自 cartparams 注册表）
 * 在读取时应用。消费方读到的是标定后角度。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 绑定编码器实例。 */
void poleang_init(magnetic_encoder_handle_t enc);

/** 已标定摆角（±π 环绕 µrad）；首样本前为 0。 */
int32_t poleang_theta_urad(void);

/** 已标定角速度（mrad/s，符号随 POLE_INV 翻转）；窗口未满为 0。 */
int32_t poleang_vth_mrad_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool poleang_is_ok(void);

#ifdef __cplusplus
}
#endif
