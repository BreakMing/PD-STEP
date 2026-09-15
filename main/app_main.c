/*
 * ESP32-S3 Single-Axis G-code Motion Controller
 * T4: G-code Parser - app_main entry point
 *
 * app_main calls bsp_early_init() as its very first line (to pull
 * TMC2209 EN high and disable the driver), then prints the startup
 * banner, then calls bsp_init() for full peripheral init, then
 * starts the protocol comm task, then registers the G-code parser
 * as the line handler (replacing the T3 placeholder dispatcher).
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

#include "bsp.h"
#include "protocol.h"
#include "gcode_parser.h"

static const char *TAG = "app_main";

/* ---- G-code parser instance & placeholder callbacks ----
 *
 * The parser is created in app_main after protocol starts.
 * Placeholder callbacks respond with "ok" for commands that succeed
 * and simple placeholder text for queries.  These will be replaced
 * by T6 (on_move / on_motor_enable), T11 (on_set_param / on_get_param
 * / on_list_params) in later milestones.
 */

static gcode_parser_t *s_parser = NULL;

/* --- Placeholder: move command ---
 * T6 step_engine will replace this with planner_enqueue().
 * Placeholder: no-op.  Line handler sends "ok" on PARSER_OK. */
static void cb_on_move(void *ctx, float target_mm, float feed_mm_min,
                       float accel_mm_s2, bool rapid)
{
    (void)ctx;
    (void)target_mm;
    (void)feed_mm_min;
    (void)accel_mm_s2;
    (void)rapid;
}

static void cb_on_set_origin(void *ctx, float new_pos_mm)
{
    (void)ctx;
    (void)new_pos_mm;
    /* T8 AS5600 + planner will handle coordinate offset. */
}

static void cb_on_set_absolute(void *ctx)
{
    (void)ctx;
    /* T7 planner will track mode. */
}

static void cb_on_set_relative(void *ctx)
{
    (void)ctx;
    /* T7 planner will track mode. */
}

static void cb_on_motor_enable(void *ctx, bool enable)
{
    (void)ctx;
    (void)enable;
    /* T5 TMC2209 driver will replace this with real EN control. */
}

static void cb_on_list_params(void *ctx)
{
    (void)ctx;
    /* T11 settings component will replace this with full parameter list.
     * Placeholder: send a header line (line handler sends "ok" after). */
    const char *msg = "$$ (placeholder \xe2\x80\x94 T11 not yet integrated)\r\n";
    protocol_send_raw(msg, strlen(msg));
}

static void cb_on_set_param(void *ctx, int idx, float value)
{
    (void)ctx;
    (void)idx;
    (void)value;
    /* T11 settings will replace this with NVS write. */
}

static void cb_on_get_param(void *ctx, int idx)
{
    (void)ctx;
    /* T11 settings will replace this with real value read from NVS.
     * Placeholder: echo "$<idx>=0.00" (line handler sends "ok" after). */
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "$%d=0.00\r\n", idx);
    if (n > 0) {
        protocol_send_raw(buf, n);
    }
}

/* ---- Line handler — bridges protocol → parser ----
 *
 * Protocol: parse success → ok; error → error:<code>.
 * Query callbacks (on_list_params / on_get_param) send their data
 * first; line handler always appends "ok" on PARSER_OK.
 * This matches PLAN §4.2 closed-ack protocol (one response per line). */
static void parser_line_handler(const char *line)
{
    parser_status_t st = gcode_parser_feed(s_parser, line);

    switch (st) {
    case PARSER_OK:
        protocol_send_ok();
        break;
    case PARSER_ERR_UNKNOWN_CMD:
        protocol_send_error(1);
        break;
    case PARSER_ERR_RANGE:
        protocol_send_error(2);
        break;
    case PARSER_ERR_BAD_PARAM:
        protocol_send_error(4);
        break;
    default:
        protocol_send_error(1);
        break;
    }
}

/* Placeholder task - feeds the watchdog and does nothing else.
 * Will be replaced by real component tasks in later milestones. */
static void placeholder_task(void *pvParameters)
{
    ESP_LOGI(TAG, "placeholder_task started");

    /* Subscribe this task to the task watchdog */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        /* Feed the watchdog every 2 seconds */
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Should never reach here */
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* ---------------------------------------------------------------
     * EARLY INIT — must be the VERY first thing app_main does.
     * Pulls TMC2209 EN HIGH (disabled) to prevent accidental enable
     * while the pin might have floated during reset.
     * --------------------------------------------------------------- */
    bsp_early_init();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-S3 Single-Axis Motion Controller");
    ESP_LOGI(TAG, " T4 G-code Parser - Line Parsing");
    ESP_LOGI(TAG, " Target: ESP32-S3-WROOM-1-N16R8");
    ESP_LOGI(TAG, "========================================");

    /* Print chip info */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: ESP32-S3, CPU cores: %d, Silicon revision: %d",
             chip_info.cores, chip_info.revision);
    ESP_LOGI(TAG, "Flash: 16MB Quad (external), PSRAM: 8MB Octal");

    /* Full BSP initialization (UART1, GPIO, I2C + scan) */
    esp_err_t bsp_err = bsp_init();
    if (bsp_err != ESP_OK) {
        ESP_LOGE(TAG, "BSP init failed: %s", esp_err_to_name(bsp_err));
    }

#ifdef CONFIG_GCODE_PARSER_SELF_TEST
    /* On-target self-test: run all 30 parser test cases before
     * starting the protocol stack.  Results are printed to ESP_LOG
     * and the final summary line is "SELF-TEST DONE: <n>/30 PASS".
     * This is gated by CONFIG_GCODE_PARSER_SELF_TEST (default n). */
    gcode_parser_self_test_run();
#endif

    /* Start communication protocol stack */
    if (bsp_err == ESP_OK) {
        esp_err_t proto_err = protocol_start();
        if (proto_err != ESP_OK) {
            ESP_LOGE(TAG, "Protocol start failed: %s", esp_err_to_name(proto_err));
        } else {
            /* ---- G-code parser init ----
             * Creates the parser with placeholder output callbacks and
             * registers it as the line handler, replacing the T3
             * placeholder dispatcher (empty/comment→ok, else→error:1).
             *
             * Integration points for later milestones:
             *   on_move         → T6 step_engine (planner_enqueue)
             *   on_set_origin   → T8 AS5600 + planner (coordinate offset)
             *   on_set_absolute / on_set_relative → T7 planner (mode)
             *   on_motor_enable → T5 TMC2209 (EN pin control)
             *   on_list_params / on_set_param / on_get_param → T11 settings
             */
            parser_output_iface_t iface = {
                .on_move         = cb_on_move,
                .on_set_origin   = cb_on_set_origin,
                .on_set_absolute = cb_on_set_absolute,
                .on_set_relative = cb_on_set_relative,
                .on_motor_enable = cb_on_motor_enable,
                .on_list_params  = cb_on_list_params,
                .on_set_param    = cb_on_set_param,
                .on_get_param    = cb_on_get_param,
                .ctx             = NULL,
            };
            s_parser = gcode_parser_create(&iface);
            if (s_parser != NULL) {
                protocol_register_line_handler(parser_line_handler);
                ESP_LOGI(TAG, "G-code parser registered as line handler");
            } else {
                ESP_LOGE(TAG, "Failed to create G-code parser");
            }
        }
    }

    /* Task watchdog is already enabled via sdkconfig.defaults
     * (CONFIG_ESP_TASK_WDT_INIT=y). The Idle task is subscribed
     * automatically on each core. */

    /* Create placeholder task at low priority */
    xTaskCreate(placeholder_task, "placeholder", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Startup complete. System running.");
}
