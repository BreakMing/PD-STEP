/*
 * cartproto — 通讯栈实现
 *
 * comm_task（CPU0、优先级 5、4KB 栈）：
 *   - UART1 事件队列驱动接收；
 *   - 行组装：128 字节缓冲，'\n' 分行，'\r' 容忍剥离；
 *   - 超长（>127 字符无换行）与不完整行超时（Kconfig，默认 100 ms）
 *     均静默丢弃，不回任何错误行（protocol.md：非法行静默丢弃）；
 *   - TX 单一互斥保护。
 *
 * 未注册行处理器时，收到的完整行直接丢弃。
 */

#include "cartproto.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"

#include "bsp.h"

static const char *TAG = "cartproto";

/* ---- 配置常量 ---- */
#define UART_PORT_NUM           UART_NUM_1
#define UART_RX_BUF_SIZE        1024   /* RX 环形缓冲大小 */
#define UART_TX_BUF_SIZE        512    /* TX 环形缓冲大小 */
#define UART_QUEUE_SIZE         10     /* 事件队列深度 */

#define LINE_BUF_SIZE           128    /* 行缓冲大小（含 NUL） */
#define LINE_MAX_CHARS          (LINE_BUF_SIZE - 1)  /* 最多 127 字符 */

#define COMM_TASK_STACK_SIZE    4096
#define COMM_TASK_PRIORITY      5
#define COMM_TASK_CORE_ID       0      /* 钉在 CPU0 */

/* 不完整行超时（Kconfig 毫秒值换算为 tick） */
#define LINE_TIMEOUT_TICKS      pdMS_TO_TICKS(CONFIG_CARTPROTO_LINE_TIMEOUT_MS)

/* ---- 内部状态 ---- */
static TaskHandle_t s_comm_task_handle = NULL;
static QueueHandle_t s_uart_queue = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;

/* 行组装缓冲 */
static char s_line_buf[LINE_BUF_SIZE];
static int  s_line_len = 0;
static TickType_t s_line_start_tick = 0;

/* 行处理器（NULL = 丢弃） */
static cartproto_line_cb_t s_line_cb = NULL;

/* ================================================================
 * TX（全部经 s_tx_mutex 保护）
 * ================================================================ */

void cartproto_send_raw(const char *data, int len)
{
    if (len <= 0 || data == NULL) {
        return;
    }
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(UART_PORT_NUM, data, (size_t)len);
    xSemaphoreGive(s_tx_mutex);
}

/* ================================================================
 * 行组装
 * ================================================================ */

/* 行完成、出错或超时后复位组装状态。 */
static void line_reset(void)
{
    s_line_len = 0;
    s_line_start_tick = 0;
}

/* 处理单个接收字符（RX 字节循环调用）。
 *
 * '\r' 静默剥离（'\n' 与 '\r\n' 都能正确分行）；'\n' 触发分发行。
 * 返回 true 表示该行已分发。 */
static bool line_handle_char(char c)
{
    /* 行首记录 tick，用于超时判定 */
    if (s_line_len == 0) {
        s_line_start_tick = xTaskGetTickCount();
    }

    if (c == '\r') {
        return false;
    }

    if (c == '\n') {
        s_line_buf[s_line_len] = '\0';
        if (s_line_cb != NULL) {
            s_line_cb(s_line_buf);
        }
        line_reset();
        return true;
    }

    /* 超长行：静默丢弃并复位（protocol.md：非法行静默丢弃） */
    if (s_line_len >= LINE_MAX_CHARS) {
        ESP_LOGW(TAG, "行超长（>%d 字符），静默丢弃", LINE_MAX_CHARS);
        line_reset();
        return false;
    }

    s_line_buf[s_line_len++] = c;
    return false;
}

/* 事件队列超时（一段时间无新数据）时检查不完整行是否超时，
 * 超时则静默丢弃。 */
static void line_check_timeout(void)
{
    if (s_line_len == 0) {
        return;
    }

    TickType_t elapsed = xTaskGetTickCount() - s_line_start_tick;
    if (elapsed >= LINE_TIMEOUT_TICKS) {
        ESP_LOGW(TAG, "不完整行超时（%d ms），静默丢弃 %d 字符",
                 CONFIG_CARTPROTO_LINE_TIMEOUT_MS, s_line_len);
        line_reset();
    }
}

/* ================================================================
 * comm_task — 通讯主任务
 * ================================================================ */
static void comm_task(void *pvParameters)
{
    ESP_LOGI(TAG, "comm_task 已启动（CPU0，优先级 %d，栈 %d）",
             COMM_TASK_PRIORITY, COMM_TASK_STACK_SIZE);

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uart_event_t event;

    for (;;) {
        esp_task_wdt_reset();

        /* 在 UART 事件队列上阻塞，超时取行超时值 */
        BaseType_t got = xQueueReceive(s_uart_queue, &event, LINE_TIMEOUT_TICKS);

        if (got == pdFALSE) {
            line_check_timeout();
            continue;
        }

        switch (event.type) {
        case UART_DATA: {
            uint8_t buf[128];
            int len;
            do {
                len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), 0);
                if (len > 0) {
                    for (int i = 0; i < len; i++) {
                        line_handle_char((char)buf[i]);
                    }
                }
            } while (len > 0);

            if (event.timeout_flag) {
                line_check_timeout();
            }
            break;
        }

        case UART_BUFFER_FULL:
        case UART_FIFO_OVF: {
            /* RX 溢出：冲掉全部输入并复位行组装，静默处理 */
            ESP_LOGW(TAG, "UART RX 溢出（type=%d），冲刷输入", event.type);
            uart_flush_input(UART_PORT_NUM);
            xQueueReset(s_uart_queue);
            line_reset();
            break;
        }

        default:
            break;
        }
    }

    /* 不可达 */
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

/* ================================================================
 * UART1 事件队列重初始化
 *
 * bsp 安装的 UART1 驱动不带事件队列（queue_size=0）；这里删除后
 * 重装。引脚与波特率来源：BSP Kconfig 引脚 + CONFIG_CARTPROTO_UART_BAUD。
 * ================================================================ */
static esp_err_t uart1_reinit_with_queue(void)
{
    uart_driver_delete(UART_PORT_NUM);

    uart_config_t uart_cfg = {
        .baud_rate = CONFIG_CARTPROTO_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        UART_PORT_NUM,
        UART_RX_BUF_SIZE,
        UART_TX_BUF_SIZE,
        UART_QUEUE_SIZE,
        &s_uart_queue,
        0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install 失败：%s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(UART_PORT_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config 失败：%s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(
        UART_PORT_NUM,
        CONFIG_BSP_UART1_TX_GPIO,
        CONFIG_BSP_UART1_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin 失败：%s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART1 已带事件队列重初始化（TX=%d RX=%d @ %d baud）",
             CONFIG_BSP_UART1_TX_GPIO,
             CONFIG_BSP_UART1_RX_GPIO,
             CONFIG_CARTPROTO_UART_BAUD);

    return ESP_OK;
}

/* ================================================================
 * cartproto_start()
 * ================================================================ */
esp_err_t cartproto_start(void)
{
    if (s_comm_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "创建 TX 互斥失败");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = uart1_reinit_with_queue();
    if (err != ESP_OK) {
        return err;
    }

    line_reset();

    BaseType_t ret = xTaskCreatePinnedToCore(
        comm_task,
        "comm_task",
        COMM_TASK_STACK_SIZE,
        NULL,
        COMM_TASK_PRIORITY,
        &s_comm_task_handle,
        COMM_TASK_CORE_ID);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 comm_task 失败");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "cartproto 已启动（行超时 %d ms）",
             CONFIG_CARTPROTO_LINE_TIMEOUT_MS);
    return ESP_OK;
}

/* ================================================================
 * 回调注册
 * ================================================================ */

void cartproto_register_line_handler(cartproto_line_cb_t cb)
{
    s_line_cb = cb;
}
