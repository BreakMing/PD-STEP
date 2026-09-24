/*
 * BSP - Board Support Package implementation
 *
 * Two-stage init:
 *   bsp_early_init() — pull EN HIGH (disable TMC2209), as first thing.
 *   bsp_init()       — UART1, GPIO, I2C bus + scan.
 */

#include "bsp.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_check.h"

static const char *TAG = "bsp";

/* ---- Kconfig pin/config constants ---- */
#define BSP_PIN_UART1_TX      CONFIG_BSP_UART1_TX_GPIO
#define BSP_PIN_UART1_RX      CONFIG_BSP_UART1_RX_GPIO
#define BSP_UART1_BAUD        CONFIG_CARTPROTO_UART_BAUD
#define BSP_UART1_PORT_NUM    UART_NUM_1
#define BSP_UART1_RX_BUF_SIZE 1024
#define BSP_UART1_TX_BUF_SIZE 512

#define BSP_PIN_STEP          CONFIG_BSP_STEP_GPIO
#define BSP_PIN_DIR           CONFIG_BSP_DIR_GPIO
#define BSP_PIN_EN            CONFIG_BSP_EN_GPIO
#define BSP_PIN_DIAG          CONFIG_BSP_DIAG_GPIO

#define BSP_PIN_I2C_SDA       CONFIG_BSP_I2C_SDA_GPIO
#define BSP_PIN_I2C_SCL       CONFIG_BSP_I2C_SCL_GPIO
#define BSP_I2C_FREQ_HZ       CONFIG_BSP_I2C_FREQ_HZ
#define BSP_I2C_PORT_NUM      I2C_NUM_0
#define BSP_AS5600_ADDR       CONFIG_BSP_AS5600_I2C_ADDR

/* I2C scan range (7-bit addresses) */
#define I2C_SCAN_ADDR_FIRST   0x08
#define I2C_SCAN_ADDR_LAST    0x77
#define I2C_PROBE_TIMEOUT_MS  50

/* ---- Internal state ---- */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static bool s_i2c_internal_pullup = false;

/* ================================================================
 * bsp_early_init()
 * ================================================================ */
void bsp_early_init(void)
{
    /* TMC2209 ENN is active low.
     * Pull HIGH immediately to disable the driver, preventing
     * accidental enable from a floating pin during reset.
     * This MUST be the first peripheral action in the firmware. */
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << BSP_PIN_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_cfg);
    gpio_set_level(BSP_PIN_EN, 1);  /* HIGH = disabled */
}

/* ================================================================
 * UART1 init
 * ================================================================ */
static esp_err_t bsp_uart1_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = BSP_UART1_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(
            BSP_UART1_PORT_NUM,
            BSP_UART1_RX_BUF_SIZE,
            BSP_UART1_TX_BUF_SIZE,
            0, NULL, 0),
        TAG, "uart_driver_install failed");

    ESP_RETURN_ON_ERROR(
        uart_param_config(BSP_UART1_PORT_NUM, &uart_cfg),
        TAG, "uart_param_config failed");

    ESP_RETURN_ON_ERROR(
        uart_set_pin(
            BSP_UART1_PORT_NUM,
            BSP_PIN_UART1_TX,
            BSP_PIN_UART1_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE),
        TAG, "uart_set_pin failed");

    ESP_LOGI(TAG, "UART1: TX=%d RX=%d @ %d baud 8N1",
             BSP_PIN_UART1_TX, BSP_PIN_UART1_RX, BSP_UART1_BAUD);

    return ESP_OK;
}

/* ================================================================
 * GPIO init (STEP / DIR / DIAG)
 * ================================================================ */
static esp_err_t bsp_gpio_init(void)
{
    /* STEP and DIR as outputs, initial LOW */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << BSP_PIN_STEP) | (1ULL << BSP_PIN_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "gpio_config (out) failed");

    gpio_set_level(BSP_PIN_STEP, 0);
    gpio_set_level(BSP_PIN_DIR, 0);

    /* DIAG as input */
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << BSP_PIN_DIAG),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&in_cfg), TAG, "gpio_config (in) failed");

    ESP_LOGI(TAG, "GPIO: STEP=%d (OUT LOW), DIR=%d (OUT LOW), DIAG=%d (IN), EN=%d (OUT HIGH=disabled)",
             BSP_PIN_STEP, BSP_PIN_DIR, BSP_PIN_DIAG, BSP_PIN_EN);

    return ESP_OK;
}

/* ================================================================
 * I2C bus init + scan
 * ================================================================
 * Strategy:
 *   1. Create bus WITHOUT internal pullup (assume external pullups).
 *   2. Scan 0x08–0x77.
 *   3. If zero devices found: delete bus, recreate WITH internal pullup, rescan.
 *   4. Log which address(es) replied. If 0x36 replies, print AS5600 detected.
 */
static esp_err_t bsp_i2c_create_bus(bool enable_internal_pullup)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BSP_I2C_PORT_NUM,
        .sda_io_num = BSP_PIN_I2C_SDA,
        .scl_io_num = BSP_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = enable_internal_pullup,
        },
    };

    return i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
}

static int bsp_i2c_scan(uint8_t *found_addrs, int max_found)
{
    int count = 0;
    for (int addr = I2C_SCAN_ADDR_FIRST; addr <= I2C_SCAN_ADDR_LAST; addr++) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, (uint16_t)addr, I2C_PROBE_TIMEOUT_MS);
        if (err == ESP_OK) {
            if (count < max_found) {
                found_addrs[count] = (uint8_t)addr;
            }
            count++;
            ESP_LOGI(TAG, "I2C scan: device found at 0x%02x", addr);
        }
        /* ESP_ERR_NOT_FOUND is expected for empty addresses, skip silently */
    }
    return count;
}

static esp_err_t bsp_i2c_init(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "I2C: SDA=%d SCL=%d @ %d Hz (no internal pullup)",
             BSP_PIN_I2C_SDA, BSP_PIN_I2C_SCL, BSP_I2C_FREQ_HZ);

    /* Pass 1: no internal pullup */
    err = bsp_i2c_create_bus(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus (no pullup): %s", esp_err_to_name(err));
        return err;
    }
    s_i2c_internal_pullup = false;

    uint8_t found[16];
    int count = bsp_i2c_scan(found, sizeof(found));

    if (count == 0) {
        ESP_LOGW(TAG, "I2C scan found 0 devices (no pullup). "
                 "Board may lack external pullups; trying internal pullup fallback.");

        /* Delete bus and recreate with internal pullups enabled */
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;

        err = bsp_i2c_create_bus(true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C bus (internal pullup): %s",
                     esp_err_to_name(err));
            return err;
        }
        s_i2c_internal_pullup = true;
        ESP_LOGI(TAG, "I2C bus recreated with internal pullups enabled");

        count = bsp_i2c_scan(found, sizeof(found));

        if (count == 0) {
            ESP_LOGE(TAG, "I2C scan still found 0 devices even with internal pullups. "
                     "Check wiring / power / device presence.");
            /* Keep the bus alive; higher layers can decide what to do. */
        } else {
            ESP_LOGI(TAG, "I2C scan with internal pullup: %d device(s) found", count);
        }
    } else {
        ESP_LOGI(TAG, "I2C scan: %d device(s) found (no internal pullup)", count);
        ESP_LOGI(TAG, "Conclusion: board has external I2C pullups.");
    }

    /* Look for the AS5600 specifically */
    bool as5600_found = false;
    for (int i = 0; i < count && i < (int)sizeof(found); i++) {
        if (found[i] == BSP_AS5600_ADDR) {
            as5600_found = true;
            break;
        }
    }
    if (as5600_found) {
        ESP_LOGI(TAG, "AS5600 detected at 0x%02x", BSP_AS5600_ADDR);
    } else {
        ESP_LOGW(TAG, "AS5600 NOT detected at 0x%02x", BSP_AS5600_ADDR);
    }

    return ESP_OK;
}

/* ================================================================
 * bsp_init()
 * ================================================================ */
esp_err_t bsp_init(void)
{
    esp_err_t err;

    /* UART1 (protocol channel) */
    err = bsp_uart1_init();
    if (err != ESP_OK) {
        return err;
    }

    /* GPIO (STEP / DIR / DIAG) */
    err = bsp_gpio_init();
    if (err != ESP_OK) {
        return err;
    }

    /* I2C bus + scan */
    err = bsp_i2c_init();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "BSP initialization complete");
    return ESP_OK;
}

/* ================================================================
 * Accessors
 * ================================================================ */
i2c_master_bus_handle_t bsp_get_i2c_bus(void)
{
    return s_i2c_bus;
}

/* ================================================================
 * TMC2209 EN control
 * ================================================================ */
void bsp_tmc_en_set(bool enable)
{
    /* ENN is active low: enable=true → GPIO low */
    gpio_set_level(BSP_PIN_EN, enable ? 0 : 1);
}
