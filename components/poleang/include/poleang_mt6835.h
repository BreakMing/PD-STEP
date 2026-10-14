/*
 * poleang_mt6835 — poleang 域的 MT6835 实现（init 与接线 config）
 */
#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t host;         /* 已初始化的 SPI 主机（组装点从 bsp 获取） */
    int               clock_speed_hz;
    int               spics_io_num;
} poleang_mt6835_config_t;

/** 幂等。内部完成 mt6835 设备初始化（含 1 kHz 采样任务）。 */
esp_err_t poleang_mt6835_init(const poleang_mt6835_config_t *cfg);

#ifdef __cplusplus
}
#endif
