/*
 * BSP - Board Support Package public header
 *
 * Two-stage initialization:
 *   bsp_early_init()  — MUST be the first line of app_main.
 *                       Only pulls TMC2209 EN high (disable).
 *   bsp_init()        — Full peripheral init (UART1, GPIO, I2C bus + scan).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"  // for i2c_master_bus_handle_t

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
 *   - UART1 (protocol channel) at Kconfig-configured pins / baud rate,
 *     with RX buffer >= 1024 bytes.
 *   - GPIO: STEP output LOW, DIR output LOW, DIAG input.
 *   - I2C master bus (using new i2c_master API from ESP-IDF v6).
 *     Scans the bus for devices in the 0x08–0x77 range.
 *     If nothing responds, the bus is recreated with internal pullups
 *     and rescanned (the board may lack external pullups — T0 unknown).
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bsp_init(void);

/**
 * @brief Get the I2C master bus handle (for use by other components, e.g. AS5600).
 *
 * @return Bus handle, or NULL if bus not initialized.
 */
i2c_master_bus_handle_t bsp_get_i2c_bus(void);

#ifdef __cplusplus
}
#endif
