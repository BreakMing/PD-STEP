/*
 * cartpos_as5600 — cartpos 域的 AS5600 实现（init 与接线 config）
 *
 * 接线参数（总线、地址、时钟、方向镜像）由组装点读取 Kconfig 宏
 * 与 bsp 总线句柄后经本 config 注入。
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
    bool                    invert_dir;   /* 方向镜像（安装标定） */
} cartpos_as5600_config_t;

/**
 * @brief 内部完成 as5600 设备初始化（含 1 kHz 采样任务）。幂等。
 *
 * @return ESP_OK；cfg 为 NULL 返回 ESP_ERR_INVALID_ARG；
 *         其余透传 as5600_init 返回值。
 */
esp_err_t cartpos_as5600_init(const cartpos_as5600_config_t *cfg);

/** 方向镜像的 Kconfig 默认值（折叠 AS5600_INVERT_DIR；组装点使用）。 */
bool cartpos_as5600_invert_dir_default(void);

#ifdef __cplusplus
}
#endif
