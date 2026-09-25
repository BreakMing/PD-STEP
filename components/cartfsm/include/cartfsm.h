/*
 * cartfsm — cartpole 模式机外壳（纯核心 cartfsm_core + 硬件作用点）
 *
 * 1 kHz 监督任务（CPU1、优先级 4）：回零到位判定（位置偏差+达速
 * 锁存）/超时、编码器失联、丢步（µm 域 |E−C|）、软限位持续钳制、
 * FAULT 停机次序（目标斜坡置 0 → vel_now==0 → EN 关 → 推 MODE）。
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

/** 当前原因（枚举原子读；流程自测核对 reason 用）。 */
cm_reason_t cartfsm_reason(void);

/* ---- 流程自测钩子（CONFIG_CARTFSM_FLOW_TEST，上板注入用） ---- */
#ifdef CONFIG_CARTFSM_FLOW_TEST
#include "cartproto.h"

/** 直接注入一条已解析命令（绕过串口）。 */
void cartfsm_test_exec(const cart_req_t *req);

/** 编码器失联注入：0 无、1 AS5600、2 MT6835。 */
void cartfsm_test_inject_encoder_fail(int which);

/** 丢步判定注入：给 |E−C| 判据的 E 侧加偏置（µm）。 */
void cartfsm_test_inject_loss_bias(long bias_um);

/** 判定用 E 以指令位置替代（理想编码器替身；用例不依赖 12V）。 */
void cartfsm_test_follow_cmd(bool enable);

/** 达速锁存测试覆写：-1 自然判据（默认）；0 强制清零；1 强制置位。 */
void cartfsm_test_speed_latch(int state);
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
