/*
 * vel_ramp — 速度模式每步递推（纯函数，无硬件依赖，宿主可测）
 *
 * 物理模型（与旧段引擎一致）：每发出一步后更新速度，再以 1/v
 * 排下一拍。速度带符号，本模块拆成「方向 dir + 幅值 v_q16」。
 *
 * 量纲与类型：
 *   v_q16 / target_q16 — Q16.16 steps/s 幅值（非负）。
 *   accel / decel      — steps/s²。
 *   全部用 64 位：注册表 VEL_MAX 上限 2000 mm/s = 80000 steps/s，
 *   在 Q16.16 下超出 32 位表示范围。
 *
 * 减速到零的尾端吸附：|v| 低于 VEL_RAMP_STOP_Q16（10 steps/s）时
 * 直接归零。离散步进无法表示低于一步一秒的速率，尾端吸附最多
 * 截断 0.25 mm/s 的残余速度；换向场景在吸附点立即换向起步。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VEL_RAMP_Q16_ONE  (65536u)

/* 减速到零的尾端吸附阈值（Q16.16 steps/s） */
#define VEL_RAMP_STOP_Q16 (10u * VEL_RAMP_Q16_ONE)

/**
 * @brief 零速起步的第一步间隔。
 *
 * dt0 = sqrt(2/a) 秒 → dt0_us = sqrt(2e12 / accel)。
 * accel == 0 时按 1 处理（防御；注册表保证加速度有正的下限）。
 */
uint32_t vel_ramp_start_dt(uint32_t accel);

/**
 * @brief 每步速度递推。
 *
 * 输入 v_q16 为「刚发出的那一步」的速率幅值，输出为下一步。
 *  - 同方向：向目标幅值加速/减速（钳位，不过冲/不下穿），用 accel；
 *  - 目标为零或方向相反：向零减速（换向经零速），用 accel；
 *  - wdt_expired：有效目标按 0、速率用 decel、方向不变（看门狗
 *    是瞬态保护；命令恢复即下次调用回到正常追踪）；
 *  - 到零且有效目标非零反向：out_dir_change = true，out_dt_us 为
 *    dt0 语义（调用方换 DIR 并自行加 2 µs setup）。
 *
 * @return true 继续运动（out_* 有效）；false 已到零速且无换向目标
 *         （调用方停表）。
 */
bool vel_ramp_next(uint64_t v_q16, bool dir, uint64_t target_q16, bool target_dir,
                   uint32_t accel, uint32_t decel, bool wdt_expired,
                   uint64_t *out_v_q16, uint32_t *out_dt_us, bool *out_dir_change);

#ifdef __cplusplus
}
#endif
