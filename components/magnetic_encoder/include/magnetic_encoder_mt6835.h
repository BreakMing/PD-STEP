/*
 * magnetic_encoder_mt6835 — magnetic_encoder 类别的 MT6835 实现
 * （init 与接线 config + 器件自测面）
 *
 * 接线参数（SPI 主机、时钟、CS）由组装点读取 Kconfig 宏与 bsp
 * 主机号后经本 config 注入。STATUS 位诊断与失败分诊快照见本头。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t host;         /* 已初始化的 SPI 主机（组装点从 bsp 获取） */
    int               clock_speed_hz;
    int               spics_io_num;
    const char       *task_name;    /* 采样任务名与日志标签（双实例台位区分，如 me_cart/me_pole；NULL = "me_mt6835"） */
} magnetic_encoder_mt6835_config_t;

/**
 * @brief 注入接线参数并启动 1 kHz 采样任务。每次调用各出独立实例
 *        （rev2 双 MT6835 台位：板载小车 + 外接摆角）。
 *
 * @return ESP_OK；cfg 或 handle_out 为 NULL、host 为负返回
 *         ESP_ERR_INVALID_ARG；内存不足返回 ESP_ERR_NO_MEM；
 *         其余错误码与迁移前一致（SPI 设备挂载 / 任务创建错误）。
 */
esp_err_t magnetic_encoder_mt6835_init(const magnetic_encoder_mt6835_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out);

/** 最新原始 21-bit 读数（器件自测）。 */
uint32_t magnetic_encoder_mt6835_last_raw(magnetic_encoder_handle_t enc);

/** 最新 STATUS[2:0]（bit0 过速 bit1 磁弱 bit2 欠压；器件自测）。 */
uint8_t magnetic_encoder_mt6835_last_status(magnetic_encoder_handle_t enc);

#ifdef __cplusplus
}
#endif
