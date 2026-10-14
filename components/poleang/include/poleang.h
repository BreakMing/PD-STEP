/*
 * poleang — 摆角域契约（已标定摆角）
 *
 * 链接期绑定：实现源文件在同组件内（当前实现 poleang_mt6835.c，
 * 经 Kconfig choice 选择）。零位/符号标定（POLE_ZERO_URAD / POLE_INV）
 * 在实现内应用——消费方读到的是标定后角度。
 * 语义：docs/superpowers/specs/2026-10-14-device-decoupling-design.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 已标定摆角（±π 环绕 µrad）；首样本前为 0。 */
int32_t poleang_theta_urad(void);

/** 已标定角速度（mrad/s，符号随 POLE_INV 翻转）；窗口未满为 0。 */
int32_t poleang_vth_mrad_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool poleang_is_ok(void);

#ifdef __cplusplus
}
#endif
