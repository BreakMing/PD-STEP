/*
 * cartfsm_core — 模式机纯核心（无硬件依赖，宿主可测）
 *
 * 转换表完整分类（protocol.md §固件模式机 + §命令×模式矩阵）。
 * 回零完成信号是位置偏差判据（达速锁存后 |E−C| 越限，homing-spec.md）；
 * 未达速的偏差越限是故障。
 * 纯核心只做状态判定与数值钳位；EN 控制、MODE 行推送、回零动作
 * 由 cartfsm.c 的外壳按核心判定结果执行。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { CM_DISABLED, CM_HOMING, CM_READY, CM_FAULT } cm_mode_t;

typedef enum {
    CR_NONE, CR_STEP_LOSS, CR_AS5600, CR_MT6835, CR_HOME_TIMEOUT,
} cm_reason_t;

typedef enum {
    CE_CMD_HOME = 1,      /* DISABLED 才生效 */
    CE_CMD_RESET,         /* FAULT 清除；HOMING 中止（逃生口） */
    CE_HOME_DONE,         /* HOMING 到位判定（达速后 |E−C| 越限）→ 完成 */
    CE_HOME_TIMEOUT,
    CE_AS5600_FAIL,
    CE_MT6835_FAIL,
    CE_STEP_LOSS,         /* READY 丢步；HOMING 未达速偏差 */
} cm_event_t;

/**
 * @brief 处理一个事件，按转换表更新 mode/reason。
 *
 * @return true = 发生模式转换（外壳需执行伴随动作并推送 MODE 行）；
 *         false = 事件在当前态被忽略。
 */
bool cm_step(cm_mode_t *mode, cm_reason_t *reason, cm_event_t ev);

/** CONFIG SET 允许的模式：仅 DISABLED / READY（HOMING、FAULT 拒绝）。 */
bool cm_config_set_allowed(cm_mode_t mode);

/**
 * @brief 软限位钳位：|x| 越界时与越界同号的目标分量替换为 0，
 *        向内不受限（回零后停在 −265 mm 处于限外，回程不受阻）。
 */
long cm_soft_limit_clamp(long x_um, long x_limit_um, long v_mm_s);

/** 丢步判定（µm 域）：|E−C| 超过阈值即丢步。 */
bool cm_step_loss(long e_um, long c_um, long step_loss_um);

/* ---- HOMING 到位判定（位置偏差判据，homing-spec §2） ---- */

/** HOMING 每拍判定输入（监督任务 1 kHz 组装）。 */
typedef struct {
    bool a5600_ok;      /* AS5600 读取成功（E 有效性；false 最先判） */
    bool m6835_ok;      /* MT6835 读取成功 */
    bool speed_latch;   /* 达速锁存已置位（实测曾达 −0.8×HOME_VEL） */
    long e_um;          /* 实测位置 µm（含测试偏置） */
    long c_um;          /* 指令位置 µm */
    long step_loss_um;  /* 偏差阈值 T */
    bool timed_out;     /* 已超过 HOME_TIMEOUT */
} cm_homing_in_t;

/**
 * @brief HOMING 每拍事件判定（优先序决策表）：
 *        AS5600 失败 > MT6835 失败 > 偏差越限（按达速锁存分类：
 *        达速=到位 CE_HOME_DONE，未达速=CE_STEP_LOSS）> 超时。
 *        E 无效时不得用于任何判定。
 * @return 触发的事件；0 = 本拍无事件。
 */
cm_event_t cm_homing_event(const cm_homing_in_t *in);

/** 达速判据：实测速度达到回零方向 80%（v_meas ≤ −0.8×home_vel，含符号）。 */
bool cm_speed_reached(long v_meas_mm_s, long home_vel_mm_s);

#ifdef __cplusplus
}
#endif
