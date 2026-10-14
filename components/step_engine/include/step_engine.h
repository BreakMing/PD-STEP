/*
 * Step Engine — GPTimer ISR 速度模式脉冲发生器
 *
 * 消费带符号目标速度（Q16.16 steps/s），ISR 内每步递推
 * （vel_ramp 纯函数），one-shot alarm 排下一拍。换向经零速。
 * 命令看门狗：超时未喂（vel_target）则按 decel 减速到 0 并停表，
 * 引擎只管停，模式语义（保持 READY）由 cartfsm 保证。
 *
 * 急停（step_engine_stop）：停脉冲、清目标与递推状态、冻结位置计数。
 * 所有公开函数可在任务上下文调用；内部与 ISR 以自旋锁互斥。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 公开 API ---- */

/**
 * @brief 初始化：GPTimer @1 MHz、one-shot alarm；中断装在调用 init
 * 的核（app_main 于 CPU0 启动，CPU0 布局由此保证）。
 *
 * STEP/DIR GPIO 不重复初始化（bsp 已配 OUT LOW），仅记录引脚号。
 * 上电默认：加速度/减速度 4000 steps/s²、看门狗关闭、目标 0。
 * cartfsm 启动后按参数注册表覆盖。
 */
esp_err_t step_engine_init(int step_gpio, int dir_gpio);

/**
 * @brief 急停 — 停脉冲、清一切递推状态与目标，冻结位置计数。
 *
 * 调用返回后保证不再发出 STEP 脉冲。任务上下文调用，
 * 与 ISR 以临界区同步。
 */
void step_engine_stop(void);

/* ---- 速度模式 ---- */

/**
 * @brief 设置追踪加速度（ACC_MAX 换算到 steps/s²）。0 按 1 防御。
 * 任务上下文。立即生效于后续递推。
 */
void step_engine_vel_set_accel(uint32_t accel_steps_s2);

/**
 * @brief 设置看门狗减速用减速度（WDT_DEC 换算到 steps/s²）。
 */
void step_engine_vel_set_decel(uint32_t decel_steps_s2);

/**
 * @brief 设置命令看门狗超时（µs）。0 = 关闭。
 *
 * 超时判定在 ISR 内做：定时器原始计数 − 最近一次 vel_target 的
 * 时间戳 > wdt_us 即视为超时，有效目标按 0、速率用 decel。
 */
void step_engine_vel_set_wdt_us(uint32_t wdt_us);

/**
 * @brief 设定目标速度（带符号 Q16.16 steps/s）并喂狗。
 *
 * 目标立即生效；从静止设定非零目标会以冻结计数种子法启动定时器
 * （必要时换 DIR，含 2 µs setup）。64 位类型：注册表 VEL_MAX 上限
 * 2000 mm/s = 80000 steps/s，超出 32 位 Q16.16 表示范围。
 */
void step_engine_vel_target(int64_t v_q16);

/**
 * @brief 目标清 0（斜坡停，非急停）。不喂狗。
 */
void step_engine_vel_halt(void);

/**
 * @brief 当前速度快照（带符号 Q16.16 steps/s，物理域——含方向
 * 反转，与 step_engine_position_steps 同域）。
 *
 * 持锁快照（s_spinlock 内一次读齐 stopped/v/dir/invert）；语义上
 * 用于判零与符号。
 */
int64_t step_engine_vel_now_q16(void);

/* ---- 遥测快照 ---- */

/**
 * @brief 指令位置（带符号累计 steps，物理域）。32 位对齐读原子。
 */
int32_t step_engine_position_steps(void);

/**
 * @brief 重定基指令位置（带符号累计 steps，物理域）。
 *
 * 供回零完成时与编码器原点（cartpos_set_origin_mm）同帧重定基：
 * E/C 域对齐后 |E−C| 丢步判定才有意义。必须在引擎停止
 * （step_engine_stop 之后、无脉冲）时调用。
 */
void step_engine_set_position(int32_t steps);

/**
 * @brief 设置方向反转（X_INV）。
 *
 * 输出层 XOR 生效：DIR 物理电平与位置计数同时翻转。
 * 运动中调用：后续脉冲生效（换向减速仍按原方向完成）。
 */
void step_engine_set_dir_invert(bool invert);

/* ---- 测试钩子插桩（仅 CONFIG_STEP_ENGINE_TEST_HOOKS=y；测试工程启用） ---- */

#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS

/** 间隔采样缓冲最大样本数 */
#define STEP_SELFTEST_MAX_SAMPLES  256

/** 开启 ISR 间隔采样（存相邻发步 alarm 的间隔 µs）。 */
void step_selftest_start_sampling(uint32_t *buf, int max, volatile int *count);

/** 停止间隔采样。 */
void step_selftest_stop_sampling(void);

/** ISR 最大执行周期数（自上次复位）。 */
uint32_t step_selftest_max_isr_cycles(void);

/** ISR 分段最大周期数：脉冲段 / 递推段 / 排拍段（归因诊断用）。 */
uint32_t step_selftest_max_cycles_pulse(void);
uint32_t step_selftest_max_cycles_ramp(void);
uint32_t step_selftest_max_cycles_alarm(void);

/** ISR 分段平均周期数（发步调用次数上的均值；典型耗时，用于估算
 *  可持续步频上限，与上面的最大值对照）。 */
uint32_t step_selftest_avg_cycles_pulse(void);
uint32_t step_selftest_avg_cycles_ramp(void);
uint32_t step_selftest_avg_cycles_alarm(void);

/** 复位 ISR 周期计数。 */
void step_selftest_reset_isr_cycles(void);

#endif /* CONFIG_STEP_ENGINE_TEST_HOOKS */

#ifdef __cplusplus
}
#endif
