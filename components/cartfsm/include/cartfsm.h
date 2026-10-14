/*
 * cartfsm — 模式机：监督任务与命令执行（纯核心 cartfsm_core + 域接口作用点）
 *
 * 1 kHz 监督任务（CPU1、优先级 4）：手动两点标定状态与使能、编码器
 * 失联、丢步（编码器实测位置与指令位置的偏差）、软限位窗口持续钳制、
 * FAULT 停机次序（目标斜坡置 0 → 速度精确为零 → EN 关 → 推 MODE）。
 * 命令执行回调（cartproto exec，CPU0）：命令×模式矩阵 + CONFIG
 * 注册表作用点。上电 DISABLED、EN 关。
 */

#pragma once

#include "esp_err.h"
#include "cartfsm_core.h"
#include "params.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 标定状态（RAM，断电回 CAL_NONE，RESET 不清除）。 */
typedef enum { CAL_NONE, CAL_LEFT, CAL_DONE } calib_state_t;

/**
 * @brief 启动模式机：按注册表默认值配置引擎、注册 cartproto exec、
 *        创建监督任务、确认 EN 关。幂等。
 */
esp_err_t cartfsm_start(void);

/** 当前模式（枚举原子读）。 */
cm_mode_t cartfsm_mode(void);

/** 当前原因（枚举原子读；流程自测核对 reason 用）。 */
cm_reason_t cartfsm_reason(void);

/** 当前标定状态（READY 进入前提是 CAL_DONE）。 */
calib_state_t cartfsm_calib(void);

/* ---- 流程测试注入钩子（CONFIG_CARTFSM_TEST_HOOKS，测试工程启用） ---- */
#ifdef CONFIG_CARTFSM_TEST_HOOKS
#include "cartproto.h"

/** 直接注入一条已解析命令（绕过串口）。 */
void cartfsm_test_exec(const cart_req_t *req);

/** 编码器失联注入：0 无、1 AS5600、2 MT6835。 */
void cartfsm_test_inject_encoder_fail(int which);

/** 丢步判定注入：给 |E−C| 判据的 E 侧加偏置（µm）。 */
void cartfsm_test_inject_loss_bias(long bias_um);

/** 标定状态回 CAL_NONE（模拟断电丢 RAM 标定；测试用例隔离）。 */
void cartfsm_test_reset_calib(void);
#endif

#ifdef __cplusplus
}
#endif
