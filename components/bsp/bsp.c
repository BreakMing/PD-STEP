/*
 * BSP - Board Support Package implementation
 *
 * Two-stage init:
 *   bsp_early_init() — pull EN HIGH (disable TMC2209), as first thing.
 *   bsp_init()       — GPIO, I2C bus + scan, SPI.
 *
 * UART1（协议通道）不在此初始化：cartproto 是其唯一所有者
 * （引脚与波特率 Kconfig 仍定义于 BSP）。
 */

#include "bsp.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_check.h"

static const char *TAG = "bsp";

/* ---- Kconfig pin/config constants ---- */

#define BSP_PIN_STEP          CONFIG_BSP_STEP_GPIO
#define BSP_PIN_DIR           CONFIG_BSP_DIR_GPIO
#define BSP_PIN_EN            CONFIG_BSP_EN_GPIO

#define BSP_PIN_I2C_SDA       CONFIG_BSP_I2C_SDA_GPIO
#define BSP_PIN_I2C_SCL       CONFIG_BSP_I2C_SCL_GPIO
#define BSP_I2C_FREQ_HZ       CONFIG_BSP_I2C_FREQ_HZ
#define BSP_I2C_PORT_NUM      I2C_NUM_0

/* MT6835 SPI（MOSI=40 MISO=46 SCK=39，CS 由 mt6835 设备侧持有） */
#define BSP_SPI_HOST          SPI3_HOST
#define BSP_PIN_SPI_MOSI      CONFIG_BSP_SPI_MOSI_GPIO
#define BSP_PIN_SPI_MISO      CONFIG_BSP_SPI_MISO_GPIO
#define BSP_PIN_SPI_SCLK      CONFIG_BSP_SPI_SCLK_GPIO

/* I2C scan range (7-bit addresses) */
#define I2C_SCAN_ADDR_FIRST   0x08
#define I2C_SCAN_ADDR_LAST    0x77
#define I2C_PROBE_TIMEOUT_MS  50

/* ---- Internal state ---- */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static bool s_i2c_internal_pullup = false;
static bool s_spi_initialized = false;

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
 * GPIO init (STEP / DIR / EN)
 * ================================================================ */
static esp_err_t bsp_gpio_init(void)
{
    /* STEP/DIR：input-output——输出功能不受影响，输入缓冲打开后
     * gpio_get_level 可回读真实电平（自测与流程测试依赖回读） */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << BSP_PIN_STEP) | (1ULL << BSP_PIN_DIR),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "gpio_config (out) failed");

    gpio_set_level(BSP_PIN_STEP, 0);
    gpio_set_level(BSP_PIN_DIR, 0);

    /* EN 已在 bsp_early_init 配过（OUTPUT + 拉高）；这里只补开输入
     * 缓冲供回读——用 set_direction 避免重复 gpio_config 的冲突告警 */
    gpio_set_direction((gpio_num_t)BSP_PIN_EN, GPIO_MODE_INPUT_OUTPUT);

    ESP_LOGI(TAG, "GPIO: STEP=%d (OUT LOW), DIR=%d (OUT LOW), EN=%d (OUT HIGH=disabled)",
             BSP_PIN_STEP, BSP_PIN_DIR, BSP_PIN_EN);

    return ESP_OK;
}

/* ================================================================
 * I2C bus init + scan
 * ================================================================
 * Strategy:
 *   1. Create bus WITHOUT internal pullup (assume external pullups).
 *   2. Scan 0x08–0x77.
 *   3. If zero devices found: delete bus, recreate WITH internal pullup, rescan.
 *   4. Log which address(es) replied. 器件在场性与健康判定归
 *      magnetic_encoder 的首读与失败计数，本层不按地址识别器件。
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

static int bsp_i2c_scan(void)
{
    int count = 0;
    for (int addr = I2C_SCAN_ADDR_FIRST; addr <= I2C_SCAN_ADDR_LAST; addr++) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, (uint16_t)addr, I2C_PROBE_TIMEOUT_MS);
        if (err == ESP_OK) {
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

    int count = bsp_i2c_scan();

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

        count = bsp_i2c_scan();

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

    return ESP_OK;
}

/* ================================================================
 * SPI bus init (MT6835)
 * ================================================================ */
static esp_err_t bsp_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BSP_PIN_SPI_MOSI,
        .miso_io_num = BSP_PIN_SPI_MISO,
        .sclk_io_num = BSP_PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,   /* 当前最大单事务 6 字节，8 留余量 */
    };

    esp_err_t err = spi_bus_initialize(BSP_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI 总线初始化失败（host=%d）：%s",
                 BSP_SPI_HOST, esp_err_to_name(err));
        return err;
    }

    s_spi_initialized = true;
    ESP_LOGI(TAG, "SPI: host=%d MOSI=%d MISO=%d SCLK=%d",
             BSP_SPI_HOST, BSP_PIN_SPI_MOSI, BSP_PIN_SPI_MISO, BSP_PIN_SPI_SCLK);
    return ESP_OK;
}

/* ================================================================
 * bsp_init()
 * ================================================================ */
esp_err_t bsp_init(void)
{
    esp_err_t err;

    /* GPIO (STEP / DIR / EN) */
    err = bsp_gpio_init();
    if (err != ESP_OK) {
        return err;
    }

    /* I2C bus + scan */
    err = bsp_i2c_init();
    if (err != ESP_OK) {
        return err;
    }

    /* SPI bus (MT6835) */
    err = bsp_spi_init();
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

spi_host_device_t bsp_get_spi_host(void)
{
    return s_spi_initialized ? BSP_SPI_HOST : (spi_host_device_t)-1;
}
