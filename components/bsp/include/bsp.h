/*
 * BSP - Board Support Package public header
 *
 * Two-stage initialization:
 *   bsp_early_init()  — MUST be the first line of app_main.
 *                       Only pulls TMC2209 EN high (disable).
 *   bsp_init()        — Full peripheral init (GPIO, I2C bus + scan,
 *                       dual SPI hosts for the two encoder slots).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"     // for i2c_master_bus_handle_t
#include "driver/spi_master.h"     // for spi_host_device_t

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Early BSP init — MUST be called as the very first line of app_main.
 *
 * The only action: configure EN (TMC2209 ENN, active low) as output and
 * drive it HIGH, which disables the TMC2209 driver.
 *
 * Rationale: during reset the GPIO may float; if it floats low the driver
 * would be accidentally enabled.  Pulling it high as early as possible
 * prevents that.
 */
void bsp_early_init(void);

/**
 * @brief Full BSP initialization — called after the startup banner.
 *
 * Initializes:
 *   - 协议通道 UART0 不在此初始化：cartproto 是其唯一所有者
 *     （引脚/波特率的 Kconfig 定义仍留在 BSP；console 走原生 USB）。
 *   - GPIO: STEP output LOW, DIR output LOW.
 *   - I2C master bus (using new i2c_master API from ESP-IDF v6).
 *     Scans the bus for devices in the 0x08–0x77 range.
 *     If nothing responds, the bus is recreated with internal pullups
 *     and rescanned (the board may lack external pullups — T0 unknown).
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bsp_init(void);

/**
 * @brief Get the I2C master bus handle (for device config assembly
 *        at the assembly point).
 *
 * @return Bus handle, or NULL if bus not initialized.
 */
i2c_master_bus_handle_t bsp_get_i2c_bus(void);

/**
 * @brief 小车位编码器 SPI 主机（板载 MT6835，组装点注入 mt6835 config）。
 *
 * @return Host id (SPI2_HOST), or -1 if bus not initialized.
 */
spi_host_device_t bsp_get_cart_enc_spi_host(void);

/**
 * @brief 摆角位编码器 SPI 主机（外接 MT6835，组装点注入 mt6835 config）。
 *
 * @return Host id (SPI3_HOST), or -1 if bus not initialized.
 */
spi_host_device_t bsp_get_pole_enc_spi_host(void);

#ifdef __cplusplus
}
#endif
