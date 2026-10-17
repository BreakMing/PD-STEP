/*
 * cartproto — cartpole 线协议通讯栈（protocol.md 的 UART0 载体）
 *
 * 职责：
 *   - 协议通道 UART0 事件队列驱动的接收：行组装（'\n' 结尾，容忍 '\r'）、
 *     超长与超时不完整行静默丢弃（protocol.md：非法行静默丢弃）；
 *   - 每凑成一行回调一次注册的行处理器（行内不含 '\r'/'\n'）；
 *   - 发送原语 cartproto_send_raw：单一 TX 互斥保护，460800 8N1。
 *
 * 命令解析与应答原语（MODE/OK/ERR）由本组件提供；未注册
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
 * 初始化协议通道 UART0（事件队列驱动，cartproto 是其唯一所有者），波特率取
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

/* ================================================================
 * 命令解析与应答
 * ================================================================ */

typedef enum {
    CART_CMD_VEL,            /* VEL <v_mm_s> */
    CART_CMD_MARK_LEFT,      /* MARK LEFT */
    CART_CMD_MARK_RIGHT,     /* MARK RIGHT */
    CART_CMD_ENABLE,
    CART_CMD_RESET,
    CART_CMD_QUERY,
    CART_CMD_CONFIG_SET,     /* CONFIG <NAME> <value> */
    CART_CMD_CONFIG_DUMP,    /* CONFIG? */
} cart_cmd_t;

typedef struct {
    cart_cmd_t cmd;
    int32_t v_mm_s;          /* VEL：整数 mm/s；|v| > 32767 判非法丢弃
                              *（防溢出，正常钳位 ±VEL_MAX 在 cartfsm） */
    char name[16];           /* CONFIG：NAME ∈ [A-Z0-9_]{1,15} */
    long value;              /* CONFIG：十进制 long */
} cart_req_t;

/**
 * @brief 纯解析（自测目标）。合法行填充 *out 返回 true；非法返回
 *        false（调用方静默丢弃，不影响后续行）。
 *
 * 词法契约（protocol.md §命令）：token 以单个空格分隔、命令词全
 * 大写；行尾仅容空白（空格/制表符）；VEL 整数仅容负号；
 * CONFIG 的 NAME ∈ [A-Z0-9_]{1,15}、value 为十进制 long。
 */
bool cartproto_parse_line(const char *line, cart_req_t *out);

/** 应答原语（cartfsm/telemetry 共用；均经 TX 互斥） */
void cartproto_send_mode(const char *mode, const char *reason);  /* "MODE <mode> <reason>\n" */
void cartproto_send_ok(const char *name, long value);            /* "OK <name> <value>\n" */
void cartproto_send_err(const char *name);                       /* "ERR <name>\n" */

/** 命令执行回调（cartfsm 注册；未注册时合法行仅丢弃） */
typedef void (*cartproto_exec_cb_t)(const cart_req_t *req);
void cartproto_register_exec(cartproto_exec_cb_t cb);

#ifdef __cplusplus
}
#endif
