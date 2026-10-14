/*
 * cartfsm_core — 模式机纯核心（无硬件依赖，宿主可测）
 *
 * 转换表完整分类（三态 DISABLED/READY/FAULT）。
 * 坐标由手动两点标定建立：MARK LEFT/MARK RIGHT
 * 完成后中点为 x=0；ENABLE 是 DISABLED→READY 的唯一路径。
 * 纯核心只做状态判定与数值钳位；EN 控制、MODE 行推送、标定动作
 * 由 cartfsm.c 按核心判定结果执行。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { CM_DISABLED, CM_READY, CM_FAULT } cm_mode_t;

typedef enum {
    CR_NONE, CR_STEP_LOSS, CR_AS5600, CR_MT6835,
} cm_reason_t;

typedef enum {
    CE_CMD_ENABLE = 1,    /* DISABLED 才生效（标定完成后） */
    CE_CMD_RESET,         /* FAULT 清除 */
    CE_AS5600_FAIL,
    CE_MT6835_FAIL,
    CE_STEP_LOSS,         /* READY 丢步 */
} cm_event_t;

/**
 * @brief 处理一个事件，按转换表更新 mode/reason。
 *
 * @return true = 发生模式转换（cartfsm.c 需执行伴随动作并推送 MODE 行）；
 *         false = 事件在当前态被忽略。
 */
bool cm_step(cm_mode_t *mode, cm_reason_t *reason, cm_event_t ev);

/** CONFIG SET 允许的模式：仅 DISABLED / READY（FAULT 拒绝）。 */
bool cm_config_set_allowed(cm_mode_t mode);

/**
 * @brief 软限位窗口钳位：x 越出 [x_min, x_max] 时，
 *        向外分量置 0，向内不受限；界内不受限。
 */
long cm_soft_limit_clamp(long x_um, long x_min_um, long x_max_um,
                         long v_mm_s);

/** 丢步判定（µm 域）：编码器实测位置与指令位置的偏差超过阈值即丢步。 */
bool cm_step_loss(long e_um, long c_um, long step_loss_um);

/**
 * @brief 标定行程 L 合法性：
 *        100000 ≤ L ≤ 1000000 µm 且为正（域反向拒绝）。
 */
bool cm_calib_valid(long l_um);

#ifdef __cplusplus
}
#endif
