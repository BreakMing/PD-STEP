/*
 * magnetic_encoder_as5600 — magnetic_encoder 类别的 AS5600 实现
 * （init 与接线 config + 器件自测面）
 *
 * 接线参数（总线、地址、时钟、方向镜像）由组装点读取 Kconfig 宏
 * 与 bsp 总线句柄后经本 config 注入。方向镜像是磁铁安装属性，
 * 在读层（as5600_hw）应用于 12-bit 原始角，不出现在类别契约。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;     /* 已初始化的 I2C 总线（组装点从 bsp 获取） */
    uint16_t                dev_addr;     /* 7-bit 从机地址 */
    uint32_t                scl_speed_hz; /* 总线时钟 */
    bool                    invert_dir;   /* 方向镜像（安装标定） */
    const char             *task_name;    /* 采样任务名与日志标签（NULL = "me_as5600"） */
} magnetic_encoder_as5600_config_t;

/**
 * @brief 注入接线参数并启动 1 kHz 采样任务。每次调用各出独立实例。
 *
 * @return ESP_OK；cfg 或 handle_out 为 NULL、bus 为 NULL 返回
 *         ESP_ERR_INVALID_ARG；内存不足返回 ESP_ERR_NO_MEM；
 *         其余错误码与迁移前一致（I2C / 任务创建错误）。
 */
esp_err_t magnetic_encoder_as5600_init(const magnetic_encoder_as5600_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out);

/** 最新 12-bit 原始角 0..4095（器件自测：静止噪声 / 链路健康）。 */
uint16_t magnetic_encoder_as5600_last_raw(magnetic_encoder_handle_t enc);

/** 方向镜像的 Kconfig 默认值（折叠 CONFIG_AS5600_INVERT_DIR；组装点使用）。 */
bool magnetic_encoder_as5600_invert_dir_default(void);

#ifdef __cplusplus
}
#endif
