/*
 * telemetry — 200 Hz STATE 状态流（protocol.md §数据面）
 *
 * 所有固件模式（DISABLED/READY/FAULT）无条件发送——数据面
 * 常开是契约。组帧经 cartproto_send_raw 上 UART1（TX 互斥共用）。
 *
 * 字段来源：x_um ← cartpos_position_um()（含回零原点）；theta_urad ←
 * poleang_theta_urad()（域实现内应用标定）；vx_mm_s ← cartpos
 * 16 ms 窗口速度取整；vth_mrad_s ← poleang。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 200 Hz 组帧任务（CPU1、优先级 2、vTaskDelayUntil 5 ms）。
 * @return ESP_OK 或任务创建错误。
 */
esp_err_t telemetry_start(void);

/**
 * @brief 组帧纯函数（自测目标）：写入 "STATE <x> <theta> <vx> <vth>"，
 *        不含 '\n'。
 *
 * @param buf  输出缓冲。
 * @param len  缓冲大小（不足 40 字节返回负值，内容未定义）。
 * @return 写入字节数（不含 NUL）；缓冲不足返回负值。
 */
int telemetry_format_state(char *buf, size_t len, int32_t x_um,
                           int32_t theta_urad, int32_t vx_mm_s,
                           int32_t vth_mrad_s);

#ifdef __cplusplus
}
#endif
