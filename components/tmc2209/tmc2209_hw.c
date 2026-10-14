/*
 * tmc2209_hw — TMC2209 UART 帧与寄存器读写原语（驱动核心层）
 *
 * UART frame format (verified against TMCStepper + datasheet):
 *   Write:  [SYNC=0x05][slave_addr][reg_addr | 0x80][data3..0][CRC]  (8 bytes)
 *   Read:   [SYNC=0x05][slave_addr][reg_addr | 0x00][CRC]            (4 bytes sent)
 *   Reply:  [SYNC=0x05][slave_addr][reg_addr][data3..0][CRC]          (8 bytes received)
 *
 * Echo handling: TX is connected through 1kΩ to the PDN_UART node,
 * and RX is directly connected to the same node.  Every byte we
 * transmit appears back on our RX.  For reads: send 4-byte request,
 * discard 4-byte echo, then read 8-byte response.  For writes: send
 * 8-byte request, discard 8-byte echo (no response from slave).
 *
 * 层内约束：参数全部由 tmc2209_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务；不触碰 GPIO（EN 等板级操作
 * 由 tmc2209.c 负责）。寄存器地址与位域语义也由 tmc2209.c 组装，
 * 本层只按地址提供通用 32-bit 寄存器读写。
 */

#include "tmc2209_hw.h"

#include <string.h>

#include "tmc2209.h"     /* public 声明：read_reg / write_reg / crc8 在本层实现 */
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "tmc2209";

/* ================================================================
 * Constants
 * ================================================================ */

#define TMC_SYNC_BYTE       0x05
#define TMC_WRITE_FLAG      0x80
#define TMC_READ_FLAG       0x00

/* Timing */
#define READ_RESPONSE_TIMEOUT_MS  5   /* Slave response timeout */

/* ================================================================
 * Internal state
 * ================================================================ */

static tmc2209_hw_config_t s_cfg;
static bool s_initialized = false;
static SemaphoreHandle_t s_uart_mutex = NULL;

/* ================================================================
 * Pure functions (public)
 * ================================================================ */

uint8_t tmc2209_crc8(const uint8_t *data, unsigned len)
{
    uint8_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

/* ================================================================
 * Internal helpers
 * ================================================================ */

/* Build a write datagram (8 bytes: sync + addr + reg|WRITE + 4 data + CRC) */
static void build_write_dgram(uint8_t *buf, uint8_t slave_addr,
                              uint8_t reg, uint32_t val)
{
    buf[0] = TMC_SYNC_BYTE;
    buf[1] = slave_addr;
    buf[2] = reg | TMC_WRITE_FLAG;
    buf[3] = (uint8_t)(val >> 24);
    buf[4] = (uint8_t)(val >> 16);
    buf[5] = (uint8_t)(val >> 8);
    buf[6] = (uint8_t)(val >> 0);
    buf[7] = tmc2209_crc8(buf, 7);
}

/* Build a read request datagram (4 bytes: sync + addr + reg|READ + CRC) */
static void build_read_dgram(uint8_t *buf, uint8_t slave_addr, uint8_t reg)
{
    buf[0] = TMC_SYNC_BYTE;
    buf[1] = slave_addr;
    buf[2] = reg | TMC_READ_FLAG;
    buf[3] = tmc2209_crc8(buf, 3);
}

/* Parse a response datagram (8 bytes) → register value.
 * Returns true if CRC matches. */
static bool parse_response_dgram(const uint8_t *buf, uint32_t *val)
{
    uint8_t crc = tmc2209_crc8(buf, 7);
    if (crc != buf[7]) {
        return false;
    }
    *val = ((uint32_t)buf[3] << 24) |
           ((uint32_t)buf[4] << 16) |
           ((uint32_t)buf[5] << 8)  |
           ((uint32_t)buf[6] << 0);
    return true;
}

/* ================================================================
 * UART low-level (mutex-protected)
 * ================================================================ */

static bool uart_write_reg_locked(uint8_t reg, uint32_t val)
{
    uint8_t dgram[8];
    build_write_dgram(dgram, s_cfg.slave_addr, reg, val);

    /* Flush any stale RX data first */
    uart_flush_input(s_cfg.uart_num);

    /* Send write datagram (8 bytes) */
    int written = uart_write_bytes(s_cfg.uart_num, dgram, 8);
    if (written != 8) {
        ESP_LOGW(TAG, "write_reg: uart_write_bytes returned %d", written);
        return false;
    }

    /* Wait for TX to finish + echo to arrive in RX */
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(10));

    /* Discard echo (8 bytes) — we don't need to verify it, just drain */
    uint8_t echo_buf[8];
    int rx = uart_read_bytes(s_cfg.uart_num, echo_buf, 8, pdMS_TO_TICKS(10));
    if (rx < 8) {
        /* Not necessarily an error — some bytes may have been missed */
        ESP_LOGD(TAG, "write_reg: echo short read %d/8", rx);
    }

    /* TMC2209 does not send a response for write operations */
    return true;
}

static bool uart_read_reg_locked(uint8_t reg, uint32_t *val)
{
    uint8_t tx_dgram[4];
    build_read_dgram(tx_dgram, s_cfg.slave_addr, reg);

    /* Flush RX */
    uart_flush_input(s_cfg.uart_num);

    /* Send read request (4 bytes) */
    int written = uart_write_bytes(s_cfg.uart_num, tx_dgram, 4);
    if (written != 4) {
        ESP_LOGW(TAG, "read_reg: uart_write_bytes returned %d", written);
        return false;
    }

    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(10));

    /* Discard echo of the 4-byte request */
    uint8_t echo_buf[4];
    int echo_rx = uart_read_bytes(s_cfg.uart_num, echo_buf, 4, pdMS_TO_TICKS(10));
    if (echo_rx < 4) {
        ESP_LOGD(TAG, "read_reg: echo short %d/4", echo_rx);
        /* Continue anyway — maybe we started reading mid-echo */
    }

    /* Read 8-byte response from slave */
    uint8_t resp_buf[8];
    int resp_rx = uart_read_bytes(s_cfg.uart_num, resp_buf, 8,
                                  pdMS_TO_TICKS(READ_RESPONSE_TIMEOUT_MS));
    if (resp_rx != 8) {
        ESP_LOGD(TAG, "read_reg(0x%02x): response short %d/8", reg, resp_rx);
        return false;
    }

    return parse_response_dgram(resp_buf, val);
}

/* ================================================================
 * Register layer (public, mutex-protected)
 * ================================================================ */

bool tmc2209_read_reg(uint8_t reg, uint32_t *val)
{
    if (!s_initialized) return false;
    if (val == NULL) return false;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "read_reg: mutex timeout");
        return false;
    }
    bool ok = uart_read_reg_locked(reg, val);
    xSemaphoreGive(s_uart_mutex);
    return ok;
}

bool tmc2209_write_reg(uint8_t reg, uint32_t val)
{
    if (!s_initialized) return false;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "write_reg: mutex timeout");
        return false;
    }
    bool ok = uart_write_reg_locked(reg, val);
    xSemaphoreGive(s_uart_mutex);
    return ok;
}

/* ================================================================
 * Init
 * ================================================================ */

esp_err_t tmc2209_hw_init(const tmc2209_hw_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_initialized) return ESP_OK;

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* Create UART mutex */
    s_uart_mutex = xSemaphoreCreateMutex();
    if (s_uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Install UART driver */
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* RX/TX buffers must be STRICTLY greater than the UART FIFO length
     * (128 on ESP32-S3) — uart_driver_install(128, ...) fails with
     * "uart rx buffer length error".  256 gives headroom for the
     * two-wire echo (8-byte TX echoes back on RX). */
    esp_err_t err = uart_driver_install(cfg->uart_num, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(cfg->uart_num, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d: TX=%d RX=%d @ %d baud 8N1",
             cfg->uart_num, cfg->tx_pin, cfg->rx_pin, cfg->baud);

    s_initialized = true;
    return ESP_OK;
}
