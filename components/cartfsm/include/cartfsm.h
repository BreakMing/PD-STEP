/*
 * cartfsm — cartpole 模式机外壳（纯核心 cartfsm_core + 硬件作用点）
 *
 * 1 kHz 监督任务（CPU1、优先级 4）：回零完成/超时、编码器失联、
 * 丢步（µm 域 |E−C|）、软限位持续钳制、FAULT 停机次序
 * （目标斜坡置 0 → vel_now==0 → EN 关 → 推 MODE）。
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

/**
 * @brief 启动模式机：按注册表默认值配置引擎、注册 cartproto exec、
 *        创建监督任务、确认 EN 关。幂等。
 */
esp_err_t cartfsm_start(void);

/** 当前模式（枚举原子读）。 */
cm_mode_t cartfsm_mode(void);

/**
 * @brief 堵转报告（任务 8 的 DIAG1 轮询接入点）。
 *
 * HOMING 中 = 回零完成信号；READY 中 = 堵转故障。监督任务消费。
 */
void cartfsm_report_stall(bool stalled);

/* ---- 流程自测钩子（CONFIG_CARTFSM_FLOW_TEST，上板注入用） ---- */
#ifdef CONFIG_CARTFSM_FLOW_TEST
#include "cartproto.h"

/** 直接注入一条已解析命令（绕过串口）。 */
void cartfsm_test_exec(const cart_req_t *req);

/** 编码器失联注入：0 无、1 AS5600、2 MT6835。 */
void cartfsm_test_inject_encoder_fail(int which);
#endif

/* ---- 上板自测 ---- */
#ifdef CONFIG_CARTFSM_SELF_TEST
void cartfsm_self_test_run(void);
#endif
#ifdef CONFIG_CARTFSM_FLOW_TEST
void cartfsm_flow_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
