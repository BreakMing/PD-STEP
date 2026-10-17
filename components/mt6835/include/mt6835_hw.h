/*
 * mt6835_hw — SPI 角度帧层（驱动核心层；公共头，magnetic_encoder
 * 类别实现消费）
 *
 * 48-bit 单次事务（MODE3、MSB 先行；帧格式抄录与 CRC8 见
 * mt6835_urad.h 与 docs/reviews/T4-brief.md）：
 *   TX: [0]=OP_ANGLE(0b1010)<<4, [1]=ANGLE1 地址 0x03, [2..5]=0
 *   RX: [2]=ANGLE[20:13] [3]=ANGLE[12:5]
 *       [4]=ANGLE[4:0]<<3 | STATUS[2:0]  [5]=器件 CRC8
 * 帧校验：全零帧（MISO 悬空）与 CRC 不符均判失败，并留存帧原文
 * 供设备组件分诊日志。
 *
 * 层内约束：参数全部由 mt6835_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MT6835_HW_FRAME_BYTES 6

/* 失败分类（分诊字符串由设备组件维护） */
typedef enum {
    MT6835_HW_FAIL_SPI_ERR = 0,  /* SPI 事务错误 */
    MT6835_HW_FAIL_ALL_ZERO,     /* 全零帧（MISO 悬空/器件未应答） */
    MT6835_HW_FAIL_CRC,          /* CRC 不符 */
} mt6835_hw_fail_t;

typedef struct {
    spi_host_device_t host;        /* SPI 主机编号（如 SPI3_HOST；组装点从 bsp 获取，须已初始化） */
    int               clock_speed_hz;
    int               spics_io_num;
} mt6835_hw_config_t;

/* 实例句柄（定义在 mt6835_hw.c）：SPI 设备与失败分诊快照全部进实例，
 * 同一总线可 init 多个实例互不影响（rev2 双编码器各持一份）。 */
typedef struct mt6835_hw_dev_t *mt6835_hw_handle_t;

/**
 * @brief 在 config 给出的总线上添加 MT6835 设备（MODE3），创建实例。
 *
 * @param handle_out 出参，返回新实例句柄。
 * @return ESP_OK；cfg/handle_out 为 NULL 或 host < 0 返回
 *         ESP_ERR_INVALID_ARG；内存不足返回 ESP_ERR_NO_MEM；
 *         其余错误码与 spi_bus_add_device 一致（失败时已释放实例）。
 */
esp_err_t mt6835_hw_init(const mt6835_hw_config_t *cfg,
                         mt6835_hw_handle_t *handle_out);

/**
 * @brief 读一次 21-bit 原始角与 STATUS[2:0]（帧校验含全零/CRC）。
 *
 * @return false = 句柄为 NULL、事务或帧校验失败（*raw21 / *status
 *         不变）；失败详情随后用 mt6835_hw_last_failure() 取快照。
 */
bool mt6835_hw_read_raw(mt6835_hw_handle_t h, uint32_t *raw21, uint8_t *status);

/**
 * @brief 最近一次失败的分类与 6 字节帧原文（无失败时返回上次值）。
 *
 * kind / frame 各自可为 NULL，为 NULL 时跳过该项；句柄为 NULL 时不写任何出参。
 */
void mt6835_hw_last_failure(mt6835_hw_handle_t h,
                            mt6835_hw_fail_t *kind,
                            uint8_t frame[MT6835_HW_FRAME_BYTES]);

#ifdef __cplusplus
}
#endif
