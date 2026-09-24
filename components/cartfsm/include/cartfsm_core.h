/*
 * cartfsm_core — 模式机纯核心（无硬件依赖，宿主可测）
 *
 * 转换表完整分类（protocol.md §固件模式机 + §命令×模式矩阵）。
 * 堵转在 HOMING 是预期的完成信号，在 READY 跟踪中才是故障。
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
    CR_NONE, CR_STEP_LOSS, CR_STALL, CR_AS5600, CR_MT6835, CR_HOME_TIMEOUT,
} cm_reason_t;

typedef enum {
    CE_CMD_HOME = 1,      /* DISABLED 才生效 */
    CE_CMD_RESET,         /* FAULT 才生效 */
    CE_STALL,             /* HOMING=完成；READY=故障 */
    CE_HOME_TIMEOUT,
    CE_AS5600_FAIL,
    CE_MT6835_FAIL,
    CE_STEP_LOSS,
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

#ifdef __cplusplus
}
#endif
