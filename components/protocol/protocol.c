/*
 * Protocol - Communication Stack implementation
 *
 * comm_task (pinned to CPU0, prio 5, 4KB stack):
 *   - UART1 event queue driven RX
 *   - Line assembly with 128B buffer, \n or \r\n terminator
 *   - Real-time chars '?' and '!' intercepted in-line (before line end)
 *   - Line timeout (Kconfig configurable, default 100ms) → error:3
 *   - Overflow (>127 chars without newline) → error:3
 *   - TX protected by a single mutex
 *
 * Placeholder callbacks (to be replaced by T4 / T6 / T9):
 *   - Line handler   : empty/comment → ok; anything else → error:1
 *   - Status callback: sends <Idle|C:0.00|E:0.00>
 *   - Stop callback  : logs only (no actual stop — no step engine yet)
 */

#include "protocol.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"

/* BSP Kconfig — we reuse pin/baud configuration from BSP */
#include "bsp.h"

static const char *TAG = "protocol";

/* ---- Configuration constants ---- */
#define UART_PORT_NUM           UART_NUM_1
#define UART_RX_BUF_SIZE        1024   /* RX ring buffer size */
#define UART_TX_BUF_SIZE        512    /* TX ring buffer size */
#define UART_QUEUE_SIZE         10     /* Event queue depth */

#define LINE_BUF_SIZE           128    /* Max line length incl. NUL */
#define LINE_MAX_CHARS          (LINE_BUF_SIZE - 1)  /* 127 chars max */

#define COMM_TASK_STACK_SIZE    4096
#define COMM_TASK_PRIORITY      5
#define COMM_TASK_CORE_ID       0      /* Pinned to CPU0 */

/* Line timeout in ticks (from Kconfig ms value) */
#define LINE_TIMEOUT_TICKS      pdMS_TO_TICKS(CONFIG_PROTOCOL_LINE_TIMEOUT_MS)

/* ---- Real-time character definitions ---- */
#define RT_CHAR_STATUS          '?'   /* Immediate status query */
#define RT_CHAR_STOP            '!'   /* Immediate stop */

/* ---- Internal state ---- */
static TaskHandle_t s_comm_task_handle = NULL;
static QueueHandle_t s_uart_queue = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;

/* Line assembly buffer */
static char s_line_buf[LINE_BUF_SIZE];
static int  s_line_len = 0;
static TickType_t s_line_start_tick = 0;

/* Callbacks (placeholder implementations by default) */
static protocol_line_handler_t s_line_handler = NULL;
static protocol_status_cb_t    s_status_cb    = NULL;
static protocol_stop_cb_t      s_stop_cb      = NULL;

/* ================================================================
 * TX helpers (all protected by s_tx_mutex)
 * ================================================================ */

void protocol_send_raw(const char *data, int len)
{
    if (len <= 0 || data == NULL) {
        return;
    }
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(UART_PORT_NUM, data, (size_t)len);
    xSemaphoreGive(s_tx_mutex);
}

void protocol_send_ok(void)
{
    const char msg[] = "ok\r\n";
    protocol_send_raw(msg, sizeof(msg) - 1);
}

void protocol_send_error(int code)
{
    char buf[24];
    int n = snprintf(buf, sizeof(buf), "error:%d\r\n", code);
    if (n > 0 && n < (int)sizeof(buf)) {
        protocol_send_raw(buf, n);
    }
}

/* ================================================================
 * Placeholder status callback (T9 will replace with real values)
 * ================================================================
 * Sends a minimal status frame in the format specified by PLAN §4.5:
 *   <STATE|C:<pos>|E:<pos>|V:<vel>|T:<flag>>
 *
 * Placeholder uses Idle state and zero values.
 * T9 monitor component will register a real status_cb that reads
 * actual position / velocity / TMC status.
 */
static void placeholder_status_cb(void)
{
    /* Placeholder status frame: Idle, all zeros.
     * Format per PLAN §4.5: <Idle|C:0.00|E:0.00|V:0.00|T:OK> */
    const char frame[] = "<Idle|C:0.00|E:0.00|V:0.00|T:OK>\r\n";
    /* TX is mutex-protected inside protocol_send_raw */
    protocol_send_raw(frame, sizeof(frame) - 1);
}

/* ================================================================
 * Placeholder stop callback (T6 will replace with real stop logic)
 * ================================================================
 * Just logs the event.  T6 step_engine will register a real stop_cb
 * that actually clears the segment queue and halts step pulses.
 */
static void placeholder_stop_cb(void)
{
    ESP_LOGW(TAG, "'!' stop character received — placeholder handler, "
             "no motion to stop yet (T6 step_engine not present)");
}

/* ================================================================
 * Placeholder line handler (T4 G-code parser will replace this)
 * ================================================================
 * Rules:
 *   - Empty line or line starting with ';' (comment) → "ok"
 *   - Anything else → "error:1" (unknown / unsupported command)
 *
 * Error code reference (PLAN §4.6):
 *   1 = unknown/unsupported command
 *   3 = line too long / malformed
 */
static void placeholder_line_handler(const char *line)
{
    /* Skip leading whitespace for the check */
    const char *p = line;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }

    /* Empty line → ok */
    if (*p == '\0') {
        protocol_send_ok();
        return;
    }

    /* Comment line (';' to end of line) → ok */
    if (*p == ';') {
        protocol_send_ok();
        return;
    }

    /* Everything else → unknown command error */
    protocol_send_error(1);
}

/* ================================================================
 * Line assembly helpers
 * ================================================================ */

/* Reset the line buffer (after line completion, error, or timeout). */
static void line_reset(void)
{
    s_line_len = 0;
    s_line_start_tick = 0;
}

/* Handle a single incoming character (called from the RX byte loop).
 *
 * Real-time chars ('?' and '!') are intercepted immediately — even in the
 * middle of a line — and do NOT become part of the line buffer.
 *
 * '\r' is silently stripped (so both \n and \r\n work as line terminators).
 * '\n' triggers line dispatch.
 *
 * Returns true if the line was dispatched (caller should not add more).
 */
static bool line_handle_char(char c)
{
    /* --- Real-time character intercept ---
     * These are processed immediately, regardless of line position.
     * They do NOT go into the line buffer.
     * NOTE: This is checked BEFORE line-end handling, so '?' or '!'
     *       arriving in the middle of a line trigger their action
     *       right away without waiting for \n. */
    if (c == RT_CHAR_STATUS) {
        if (s_status_cb != NULL) {
            s_status_cb();
        }
        return false;
    }
    if (c == RT_CHAR_STOP) {
        if (s_stop_cb != NULL) {
            s_stop_cb();
        }
        return false;
    }

    /* Start-of-line: record tick for timeout tracking */
    if (s_line_len == 0) {
        s_line_start_tick = xTaskGetTickCount();
    }

    /* Carriage return — strip silently (handles both \n and \r\n) */
    if (c == '\r') {
        return false;
    }

    /* Newline — dispatch the complete line */
    if (c == '\n') {
        s_line_buf[s_line_len] = '\0';
        if (s_line_handler != NULL) {
            s_line_handler(s_line_buf);
        }
        line_reset();
        return true;
    }

    /* Regular character — check for overflow */
    if (s_line_len >= LINE_MAX_CHARS) {
        /* Line too long → error:3 and discard */
        ESP_LOGW(TAG, "Line overflow (>%d chars), discarding", LINE_MAX_CHARS);
        protocol_send_error(3);
        line_reset();
        return false;
    }

    s_line_buf[s_line_len++] = c;
    return false;
}

/* Check if the current partial line has timed out.
 * Called when the event queue hits its timeout (no new data for a while).
 * If a partial line exists and is older than LINE_TIMEOUT_TICKS,
 * discard it and send error:3.
 */
static void line_check_timeout(void)
{
    if (s_line_len == 0) {
        return;  /* No partial line, nothing to check */
    }

    TickType_t elapsed = xTaskGetTickCount() - s_line_start_tick;
    if (elapsed >= LINE_TIMEOUT_TICKS) {
        ESP_LOGW(TAG, "Line timeout (%d ms), %d chars discarded",
                 CONFIG_PROTOCOL_LINE_TIMEOUT_MS, s_line_len);
        protocol_send_error(3);
        line_reset();
    }
}

/* ================================================================
 * comm_task — main communication task
 * ================================================================ */
static void comm_task(void *pvParameters)
{
    ESP_LOGI(TAG, "comm_task started (CPU0, prio %d, stack %d)",
             COMM_TASK_PRIORITY, COMM_TASK_STACK_SIZE);

    /* Subscribe to task watchdog — must be fed on every loop iteration */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uart_event_t event;

    for (;;) {
        /* Feed the watchdog at the top of each loop */
        esp_task_wdt_reset();

        /* Block on UART event queue with a timeout = line timeout.
         * The timeout lets us detect partial lines that never complete. */
        BaseType_t got = xQueueReceive(s_uart_queue, &event, LINE_TIMEOUT_TICKS);

        if (got == pdFALSE) {
            /* Queue timeout — no UART events for a while.
             * Check if there's a partial line that has timed out. */
            line_check_timeout();
            continue;
        }

        switch (event.type) {
        case UART_DATA: {
            /* Data available — read all available bytes and process. */
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

            /* After processing a burst, also check for timeout.
             * If the burst contained a partial line and the next burst
             * doesn't come soon, the queue timeout will catch it.
             * But we also check here in case UART_DATA keeps firing
             * with tiny trickles of data (unlikely but defensive). */
            if (event.timeout_flag) {
                line_check_timeout();
            }
            break;
        }

        case UART_BUFFER_FULL:
        case UART_FIFO_OVF: {
            /* RX buffer / FIFO overflow — flush everything, report error:3 */
            ESP_LOGW(TAG, "UART RX overflow (type=%d), flushing", event.type);
            uart_flush_input(UART_PORT_NUM);
            xQueueReset(s_uart_queue);
            line_reset();
            protocol_send_error(3);
            break;
        }

        default:
            /* Other events (PATTERN_DETECT, BREAK, etc.) — ignore */
            break;
        }
    }

    /* Should never reach here */
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

/* ================================================================
 * UART re-init with event queue
 * ================================================================
 * BSP installs the UART1 driver without an event queue (queue_size=0,
 * uart_queue=NULL).  We need an event queue for event-driven RX, so
 * we delete the driver and re-install with a queue.
 *
 * Pin configuration and baud rate come from BSP Kconfig values
 * (CONFIG_BSP_UART1_TX_GPIO / RX / BAUD), keeping a single source of truth.
 */
static esp_err_t uart1_reinit_with_queue(void)
{
    /* Remove the driver instance that bsp installed */
    uart_driver_delete(UART_PORT_NUM);

    /* UART config (same parameters as bsp: 8N1, same baud rate) */
    uart_config_t uart_cfg = {
        .baud_rate = CONFIG_BSP_UART1_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Install driver WITH event queue */
    esp_err_t err = uart_driver_install(
        UART_PORT_NUM,
        UART_RX_BUF_SIZE,
        UART_TX_BUF_SIZE,
        UART_QUEUE_SIZE,
        &s_uart_queue,
        0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(UART_PORT_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(
        UART_PORT_NUM,
        CONFIG_BSP_UART1_TX_GPIO,
        CONFIG_BSP_UART1_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART1 re-initialized with event queue "
             "(TX=%d RX=%d @ %d baud, queue depth %d)",
             CONFIG_BSP_UART1_TX_GPIO,
             CONFIG_BSP_UART1_RX_GPIO,
             CONFIG_BSP_UART1_BAUD,
             UART_QUEUE_SIZE);

    return ESP_OK;
}

/* ================================================================
 * protocol_start()
 * ================================================================ */
esp_err_t protocol_start(void)
{
    /* Already started? */
    if (s_comm_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Create TX mutex */
    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Set placeholder callbacks (may be overridden later via register_* calls) */
    if (s_line_handler == NULL) {
        s_line_handler = placeholder_line_handler;
    }
    if (s_status_cb == NULL) {
        s_status_cb = placeholder_status_cb;
    }
    if (s_stop_cb == NULL) {
        s_stop_cb = placeholder_stop_cb;
    }

    /* Re-initialize UART1 with event queue.
     * BSP already installed the driver but without a queue; we need
     * the queue for event-driven RX + timeout detection. */
    esp_err_t err = uart1_reinit_with_queue();
    if (err != ESP_OK) {
        return err;
    }

    /* Reset line assembly state */
    line_reset();

    /* Create comm_task pinned to CPU0 */
    BaseType_t ret = xTaskCreatePinnedToCore(
        comm_task,
        "comm_task",
        COMM_TASK_STACK_SIZE,
        NULL,
        COMM_TASK_PRIORITY,
        &s_comm_task_handle,
        COMM_TASK_CORE_ID);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create comm_task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "protocol stack started (line timeout %d ms)",
             CONFIG_PROTOCOL_LINE_TIMEOUT_MS);
    return ESP_OK;
}

/* ================================================================
 * Callback registration
 * ================================================================ */

void protocol_register_line_handler(protocol_line_handler_t cb)
{
    if (cb != NULL) {
        s_line_handler = cb;
    } else {
        s_line_handler = placeholder_line_handler;
    }
}

void protocol_register_status_cb(protocol_status_cb_t cb)
{
    if (cb != NULL) {
        s_status_cb = cb;
    } else {
        s_status_cb = placeholder_status_cb;
    }
}

void protocol_register_stop_cb(protocol_stop_cb_t cb)
{
    if (cb != NULL) {
        s_stop_cb = cb;
    } else {
        s_stop_cb = placeholder_stop_cb;
    }
}
