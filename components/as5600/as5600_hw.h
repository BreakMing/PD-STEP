/*
 * as5600_hw — AS5600 I2C 读层（驱动核心层，组件私有）
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   ZPOS / MPOS / CONF / MANG 是一次性可编程（OTP）：写错即永久
 *   损坏芯片。本层不包含任何寄存器写：唯一总线操作是
 *   as5600_hw_read_raw()，使用 i2c_master_transmit_receive()
 *   （寄存器地址出 + 数据入，单事务 = 读）。不得在此添加
 *   i2c_master_transmit() 或任何写辅助。
 *
 * 层内约束：参数全部由 as5600_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务。方向镜像（invert_dir，
 * 磁铁安装标定）发生在本读层，设备组件与纯逻辑 tracker 只见
 * 0..4095 的已定向读数。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;     /* 已初始化的 I2C 总线（组装点从 bsp 获取） */
    uint16_t                dev_addr;     /* 7-bit 从机地址 */
    uint32_t                scl_speed_hz; /* 总线时钟 */
    bool                    invert_dir;   /* true = 镜像 raw 角（安装方向标定） */
} as5600_hw_config_t;

/**
 * @brief 在 config 给出的总线上添加 AS5600 设备。
 *
 * @return ESP_OK；cfg 为 NULL 或 bus 为 NULL 返回
 *         ESP_ERR_INVALID_ARG；其余错误码与
 *         i2c_master_bus_add_device 一致。
 */
esp_err_t as5600_hw_init(const as5600_hw_config_t *cfg);

/**
 * @brief 读 12-bit 原始角（RAW ANGLE 0x0C/0x0D，单次 2 字节突发）。
 *
 * 两次独立单字节读可能跨器件内部角度更新拼接出不存在的值，
 * 因此必须单事务突发读。
 *
 * @return true 成功（*raw ∈ 0..4095，已按 invert_dir 镜像）；
 *         false 事务失败（*raw 不变）。
 */
bool as5600_hw_read_raw(uint16_t *raw);

#ifdef __cplusplus
}
#endif
