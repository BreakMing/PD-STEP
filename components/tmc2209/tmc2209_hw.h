/*
 * tmc2209_hw — UART 帧与寄存器读写原语（驱动核心层，组件私有）
 *
 * 仅同组件业务壳 tmc2209.c 包含本头文件；不进入 include/。
 * 层内约束：参数全部由 tmc2209_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务；不触碰 GPIO（EN 等板级操作
 * 由业务壳负责）。寄存器地址与位域语义由业务壳组装，本层只按
 * 地址提供通用 32-bit 寄存器读写。
 *
 * public API（tmc2209.h 声明）中的 tmc2209_read_reg / tmc2209_write_reg /
 * tmc2209_crc8 实现于本层。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      uart_num;     /* UART port number, e.g. UART_NUM_2 */
    int      tx_pin;       /* TX GPIO (through 1kΩ to PDN_UART) */
    int      rx_pin;       /* RX GPIO (direct from PDN_UART) */
    int      baud;         /* Baud rate */
    uint8_t  slave_addr;   /* Slave address */
} tmc2209_hw_config_t;

/**
 * @brief 安装 UART 驱动并建立读写原语。
 *
 * 失败直接返回错误码（与旧 tmc2209_init 的 UART 建立段一致）；
 * 成功后 tmc2209_read_reg / tmc2209_write_reg 可用。
 */
esp_err_t tmc2209_hw_init(const tmc2209_hw_config_t *cfg);

#ifdef __cplusplus
}
#endif
