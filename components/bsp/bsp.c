/*
 * BSP - Board Support Package implementation
 *
 * Two-stage init:
 *   bsp_early_init() — pull EN HIGH (disable TMC2209), as first thing.
 *   bsp_init()       — GPIO, I2C bus + scan, SPI.
 *
 * 协议通道 UART0 不在此初始化：cartproto 是其唯一所有者
 * （引脚与波特率 Kconfig 仍定义于 BSP；console 走原生 USB）。
 */

#include "bsp.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp";

/* ---- Kconfig pin/config constants ---- */

#define BSP_PIN_STEP          CONFIG_BSP_STEP_GPIO
#define BSP_PIN_DIR           CONFIG_BSP_DIR_GPIO
#define BSP_PIN_EN            CONFIG_BSP_EN_GPIO

#define BSP_PIN_I2C_SDA       CONFIG_BSP_I2C_SDA_GPIO
#define BSP_PIN_I2C_SCL       CONFIG_BSP_I2C_SCL_GPIO
#define BSP_I2C_FREQ_HZ       CONFIG_BSP_I2C_FREQ_HZ
#define BSP_I2C_PORT_NUM      I2C_NUM_0

/* 双 SPI 主机（rev2）：小车位=板载 MT6835（SPI2），摆角位=外接
 * MT6835（SPI3）；CS 由各 mt6835 设备侧持有 */
#define BSP_CART_ENC_HOST     SPI2_HOST
#define BSP_POLE_ENC_HOST     SPI3_HOST
#define BSP_CART_PIN_SCLK     CONFIG_BSP_CART_ENC_SCK_GPIO
#define BSP_CART_PIN_MOSI     CONFIG_BSP_CART_ENC_MOSI_GPIO
#define BSP_CART_PIN_MISO     CONFIG_BSP_CART_ENC_MISO_GPIO
#define BSP_POLE_PIN_SCLK     CONFIG_BSP_POLE_ENC_SCK_GPIO
#define BSP_POLE_PIN_MOSI     CONFIG_BSP_POLE_ENC_MOSI_GPIO
#define BSP_POLE_PIN_MISO     CONFIG_BSP_POLE_ENC_MISO_GPIO

/* ---- rev2 新增外设：vbus ADC（io4 分压）、LED、按键 ---- */
#define BSP_PIN_VBUS_ADC      CONFIG_BSP_VBUS_ADC_GPIO
#define BSP_VBUS_DIV_R_UPPER  CONFIG_BSP_VBUS_DIV_R_UPPER_OHM
#define BSP_VBUS_DIV_R_LOWER  CONFIG_BSP_VBUS_DIV_R_LOWER_OHM
#define BSP_PIN_LED           CONFIG_BSP_LED_GPIO
#define BSP_BTN_COUNT         3

/* LED 心跳：500 ms 翻转 = 1 Hz 闪烁 */
#define LED_HB_PERIOD_MS      500

/* I2C scan range (7-bit addresses) */
#define I2C_SCAN_ADDR_FIRST   0x08
#define I2C_SCAN_ADDR_LAST    0x77
#define I2C_PROBE_TIMEOUT_MS  50

/* ---- Internal state ---- */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static bool s_i2c_internal_pullup = false;
static bool s_cart_enc_spi_ready = false;
static bool s_pole_enc_spi_ready = false;

/* vbus ADC（oneshot + line-fitting 校准；cali 失败则读取返回 -1） */
static adc_oneshot_unit_handle_t s_adc_unit = NULL;
static adc_cali_handle_t          s_adc_cali = NULL;
static adc_channel_t              s_vbus_chan;

static const int s_btn_gpio[BSP_BTN_COUNT] = {
    CONFIG_BSP_BTN1_GPIO, CONFIG_BSP_BTN2_GPIO, CONFIG_BSP_BTN3_GPIO,
};

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
 * SPI bus init（双主机：小车位 + 摆角位）
 * ================================================================ */
static esp_err_t bsp_spi_add_host(spi_host_device_t host,
                                  int mosi, int miso, int sclk,
                                  const char *slot)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,   /* 当前最大单事务 6 字节，8 留余量 */
    };

    esp_err_t err = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI 总线初始化失败（%s，host=%d）：%s",
                 slot, host, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SPI %s: host=%d MOSI=%d MISO=%d SCLK=%d",
             slot, host, mosi, miso, sclk);
    return ESP_OK;
}

static esp_err_t bsp_spi_init(void)
{
    ESP_RETURN_ON_ERROR(
        bsp_spi_add_host(BSP_CART_ENC_HOST,
                         BSP_CART_PIN_MOSI, BSP_CART_PIN_MISO, BSP_CART_PIN_SCLK,
                         "小车位编码器"),
        TAG, "cart enc SPI host init failed");
    s_cart_enc_spi_ready = true;

    ESP_RETURN_ON_ERROR(
        bsp_spi_add_host(BSP_POLE_ENC_HOST,
                         BSP_POLE_PIN_MOSI, BSP_POLE_PIN_MISO, BSP_POLE_PIN_SCLK,
                         "摆角位编码器"),
        TAG, "pole enc SPI host init failed");
    s_pole_enc_spi_ready = true;

    return ESP_OK;
}

/* ================================================================
 * vbus ADC（io4 分压回读）
 * ================================================================ */
static esp_err_t bsp_vbus_init(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    ESP_RETURN_ON_ERROR(
        adc_oneshot_io_to_channel(BSP_PIN_VBUS_ADC, &unit, &s_vbus_chan),
        TAG, "vbus io→channel 失败");

    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&ucfg, &s_adc_unit),
                        TAG, "vbus ADC unit 创建失败");

    adc_oneshot_chan_cfg_t ccfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,     /* 20V→2.38V 需 12 dB 量程 */
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_unit, s_vbus_chan, &ccfg),
                        TAG, "vbus ADC 通道配置失败");

    /* ESP32-S3 校准方案为 curve fitting（line fitting 是经典 ESP32） */
    adc_cali_curve_fitting_config_t cf = {
        .unit_id = unit,
        .chan = s_vbus_chan,
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cf, &s_adc_cali);
    if (err != ESP_OK) {
        /* 校准建立失败不阻断启动：读取返回 -1（可见、不假装 0V） */
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "vbus ADC 校准建立失败（%s）——电压读取将返回 -1",
                 esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "vbus ADC: io=%d 分压 %d/%d Ω（系数 %.3f）",
             BSP_PIN_VBUS_ADC, BSP_VBUS_DIV_R_UPPER, BSP_VBUS_DIV_R_LOWER,
             (double)(BSP_VBUS_DIV_R_UPPER + BSP_VBUS_DIV_R_LOWER)
                 / (double)BSP_VBUS_DIV_R_LOWER);
    return ESP_OK;
}

float bsp_vbus_read_volt(void)
{
    if (s_adc_cali == NULL) {
        return -1.0f;
    }
    int cali_mv = 0;
    if (adc_oneshot_get_calibrated_result(s_adc_unit, s_adc_cali,
                                          s_vbus_chan, &cali_mv) != ESP_OK) {
        return -1.0f;
    }
    /* 分压还原：vbus = adc 端电压 × (上臂+下臂)/下臂 */
    return (float)cali_mv / 1000.0f
         * (float)(BSP_VBUS_DIV_R_UPPER + BSP_VBUS_DIV_R_LOWER)
         / (float)BSP_VBUS_DIV_R_LOWER;
}

/* ================================================================
 * LED 心跳（io15，500 ms 翻转）与按键（消抖读取）
 * ================================================================ */
static void led_heartbeat_task(void *arg)
{
    (void)arg;
    bool on = false;
    while (1) {
        on = !on;
        gpio_set_level(BSP_PIN_LED, on ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(LED_HB_PERIOD_MS));
    }
}

static esp_err_t bsp_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BSP_PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "LED gpio_config 失败");
    gpio_set_level(BSP_PIN_LED, 0);

    BaseType_t ok = xTaskCreate(led_heartbeat_task, "led_hb",
                                2048, NULL, 1, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "LED 心跳任务创建失败");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t bsp_button_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < BSP_BTN_COUNT; i++) {
        mask |= (1ULL << s_btn_gpio[i]);
    }
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    /* 按下接地 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

bool bsp_button_read(int idx)
{
    if (idx < 0 || idx >= BSP_BTN_COUNT) {
        return false;
    }
    /* 消抖：连续 3 次采样一致才返回（低电平 = 按下） */
    int level = 1;
    for (int i = 0; i < 3; i++) {
        int cur = gpio_get_level((gpio_num_t)s_btn_gpio[idx]);
        if (i == 0) {
            level = cur;
        } else if (cur != level) {
            return false;      /* 抖动期：视为未按下 */
        }
        if (i < 2) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    return level == 0;
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

    /* SPI 双主机（小车位板载 + 摆角位外接 MT6835） */
    err = bsp_spi_init();
    if (err != ESP_OK) {
        return err;
    }

    /* rev2 新增：vbus ADC、LED 心跳、按键 */
    err = bsp_vbus_init();
    if (err != ESP_OK) {
        return err;
    }
    err = bsp_led_init();
    if (err != ESP_OK) {
        return err;
    }
    err = bsp_button_init();
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

spi_host_device_t bsp_get_cart_enc_spi_host(void)
{
    return s_cart_enc_spi_ready ? BSP_CART_ENC_HOST : (spi_host_device_t)-1;
}

spi_host_device_t bsp_get_pole_enc_spi_host(void)
{
    return s_pole_enc_spi_ready ? BSP_POLE_ENC_HOST : (spi_host_device_t)-1;
}
