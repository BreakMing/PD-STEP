/*
 * cartproto — cartpole 线协议通讯栈（protocol.md 的 UART1 载体）
 *
 * 职责（任务 1 骨架范围）：
 *   - UART1 事件队列驱动的接收：行组装（'\n' 结尾，容忍 '\r'）、
 *     超长与超时不完整行静默丢弃（protocol.md：非法行静默丢弃）；
 *   - 每凑成一行回调一次注册的行处理器（行内不含 '\r'/'\n'）；
 *   - 发送原语 cartproto_send_raw：单一 TX 互斥保护，460800 8N1。
 *
 * 命令解析与应答原语（MODE/OK/ERR）在任务 6 填充；本骨架未注册
 * 行处理器时收到的行仅丢弃，不产生任何输出。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 行处理器回调类型。
 *
 * 每收到一个完整 '\n' 行调用一次；line 以 NUL 结尾，不含 '\r'/'\n'，
 * 仅在回调期间有效（可写缓冲）。
 */
typedef void (*cartproto_line_cb_t)(const char *line);

/**
 * @brief 启动通讯任务。
 *
 * 创建 comm_task（CPU0、优先级 5、4KB 栈），订阅任务看门狗。
 * UART1 以事件队列重新初始化（bsp 先装了无队列驱动），波特率取
 * CONFIG_CARTPROTO_UART_BAUD，引脚取 BSP Kconfig。
 *
 * @return ESP_OK 成功；ESP_ERR_INVALID_STATE 已启动；否则为错误码。
 */
esp_err_t cartproto_start(void);

/**
 * @brief 注册行处理器。
 *
 * 传入 NULL 恢复默认（丢弃所有行）。替换此前注册的处理器。
 */
void cartproto_register_line_handler(cartproto_line_cb_t cb);

/**
 * @brief 原始发送（TX 互斥保护）。
 *
 * 不追加任何结尾；调用方自行携带 '\n'。len <= 0 或 data 为空时直接返回。
 */
void cartproto_send_raw(const char *data, int len);

#ifdef __cplusplus
}
#endif
