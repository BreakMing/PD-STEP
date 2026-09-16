/*
 * ESP32-S3 Single-Axis G-code Motion Controller
 * T11: Settings & NVS - app_main entry point
 *
 * Startup order:
 *   bsp_early_init → settings_init → banner → bsp_init →
 *   [selftest if enabled] → TMC2209 init (T5) → protocol_start →
 *   step_engine_init + stop_cb (T6) →
 *   gcode_parser_create(settings_default_feed, settings_default_accel) →
 *   register as line handler.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_log_write.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

#include "bsp.h"
#include "protocol.h"
#include "gcode_parser.h"
#include "settings.h"
#include "step_engine.h"
#include "tmc2209.h"
#include "driver/uart.h"

#ifdef CONFIG_APP_JOINT_TEST
void app_joint_test_run(void);
#endif

static const char *TAG = "app_main";

static gcode_parser_t *s_parser = NULL;

/* ---- Dual-port log mirror (test builds only) ----
 * When any TMC2209 test build is enabled, the board may be powered
 * from a PD wall charger (12V for the motor, no USB data link).
 * Mirror every ESP_LOG line to UART1 (TX=GPIO41, 115200) so a
 * USB-serial adapter + VOFA+ can capture self-test and bring-up
 * evidence while motor supply is on.  UART0 keeps its normal
 * console output via stdout.  Hook lives until reboot; the normal
 * firmware (all test switches off) is compiled without it. */
#if defined(CONFIG_TMC2209_SELF_TEST) || \
    defined(CONFIG_TMC2209_BRINGUP) || \
    defined(CONFIG_TMC2209_SPIN_TEST) || \
    defined(CONFIG_APP_JOINT_TEST)
static int log_mirror_vprintf(const char *fmt, va_list ap)
{
    char buf[384];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n > 0) {
        if (n >= (int)sizeof(buf)) {
            n = (int)sizeof(buf) - 1;
        }
        uart_write_bytes(UART_NUM_1, buf, n);
        fputs(buf, stdout);
    }
    return n;
}

static void install_log_mirror(void)
{
    esp_log_set_vprintf(log_mirror_vprintf);
    ESP_LOGI(TAG, "log mirror active: all logs also on UART1 "
                  "(TX=GPIO41, 115200)");
}
#endif

/* ---- Placeholder callbacks ----
 *
 * on_move / on_set_origin / on_set_absolute / on_set_relative
 * remain no-op placeholders (T7/T8 later).
 *
 * on_motor_enable → TMC2209 (T5 done).
 * on_stop (protocol) → step_engine (T6 done below).
 *
 * on_list_params / on_set_param / on_restore_defaults are wired to
 * real settings component (T11 done here).
 */

static void cb_on_move(void *ctx, float target_mm, float feed_mm_min,
                       float accel_mm_s2, bool rapid)
{
    (void)ctx; (void)target_mm; (void)feed_mm_min;
    (void)accel_mm_s2; (void)rapid;
    /* T7 step planner will replace this. */
}

static void cb_on_set_origin(void *ctx, float new_pos_mm)
{
    (void)ctx; (void)new_pos_mm;
    /* T8 AS5600 + planner. */
}

static void cb_on_set_absolute(void *ctx)
{
    (void)ctx;
    /* T7 planner. */
}

static void cb_on_set_relative(void *ctx)
{
    (void)ctx;
    /* T7 planner. */
}

static void cb_on_motor_enable(void *ctx, bool enable)
{
    (void)ctx;
    tmc2209_enable(enable);
}

/* --- T6 step_engine: immediate stop callback ---
 * Registered as protocol stop_cb; replaces the default "log only"
 * placeholder with actual step_engine_stop() (PLAN T6 spec). */
static void cb_on_stop(void)
{
    step_engine_stop();
}

/* --- T11 real: list all parameters ($$ / M503) ---
 * Sends 14 lines "$<n>=<value>\r\n" in %.3f format (PLAN §5).
 * Line handler sends "ok" afterwards. */
static void cb_on_list_params(void *ctx)
{
    (void)ctx;
    char buf[32];
    for (int i = 0; i < settings_count(); i++) {
        int n = snprintf(buf, sizeof(buf), "$%d=%.3f\r\n", i, (double)settings_get(i));
        if (n > 0) {
            protocol_send_raw(buf, n);
        }
    }
}

/* --- T11 real: set parameter ($<n>=<v>) ---
 * Returns: 0=ok, 4=invalid, 5=storage failure.
 * Parser propagates as error:4 / error:5. */
static int cb_on_set_param(void *ctx, int idx, float value)
{
    (void)ctx;
    settings_status_t st = settings_set(idx, value);
    switch (st) {
    case SETTINGS_OK:       return 0;
    case SETTINGS_INVALID:  return 4;
    case SETTINGS_STORAGE:  return 5;
    default:                return 4;
    }
}

/* --- T11 real: restore factory defaults (M502) ---
 * Returns: 0=ok, 5=storage failure. */
static int cb_on_restore_defaults(void *ctx)
{
    (void)ctx;
    settings_status_t st = settings_reset_defaults();
    return (st == SETTINGS_OK) ? 0 : 5;
}

/* ---- Line handler — bridges protocol → parser ----
 *
 * Parse success → ok; error → error:<code>.
 * Query callbacks (on_list_params) send their data first;
 * line handler always appends "ok" on PARSER_OK.
 * One response per line (PLAN §4.2). */
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
    case PARSER_ERR_STORAGE:
        protocol_send_error(5);
        break;
    default:
        protocol_send_error(1);
        break;
    }
}

/* Placeholder task — feeds watchdog, replaced later by real component tasks. */
static void placeholder_task(void *pvParameters)
{
    ESP_LOGI(TAG, "placeholder_task started");
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* ---- EARLY INIT — must be first (TMC2209 EN high = disabled) ---- */
    bsp_early_init();

    /* ---- Settings init (before banner — fast, NVS load) ---- */
    esp_err_t settings_err = settings_init();
    if (settings_err != ESP_OK) {
        ESP_LOGE(TAG, "settings_init failed: %s", esp_err_to_name(settings_err));
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-S3 Single-Axis Motion Controller");
    ESP_LOGI(TAG, " T5+T6 Integrated — TMC2209 + Step Engine");
    ESP_LOGI(TAG, " Target: ESP32-S3-WROOM-1-N16R8");
    ESP_LOGI(TAG, "========================================");

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

    /* Mirror logs to UART1 as soon as the UART1 driver exists
     * (bsp_init installed it) — before any self-test output. */
#if defined(CONFIG_TMC2209_SELF_TEST) || \
    defined(CONFIG_TMC2209_BRINGUP) || \
    defined(CONFIG_TMC2209_SPIN_TEST) || \
    defined(CONFIG_APP_JOINT_TEST)
    if (bsp_err == ESP_OK) {
        install_log_mirror();
    }
#endif

    /* ---- Self-tests (gated by Kconfig, default off) ---- */
#ifdef CONFIG_SETTINGS_SELF_TEST
    settings_self_test_run();
#endif
#ifdef CONFIG_GCODE_PARSER_SELF_TEST
    gcode_parser_self_test_run();
#endif
#ifdef CONFIG_STEP_ENGINE_SELF_TEST
    step_engine_self_test_run();
#endif
#ifdef CONFIG_TMC2209_SELF_TEST
    tmc2209_self_test_run();
#endif

    /* ---- TMC2209 init (after BSP, before protocol) ---- */
    if (bsp_err == ESP_OK) {
        tmc2209_config_t tmc_cfg = {
            .uart_num   = UART_NUM_2,
            .tx_pin     = CONFIG_BSP_TMC_UART_TX_GPIO,
            .rx_pin     = CONFIG_BSP_TMC_UART_RX_GPIO,
            .baud       = 115200,
            .slave_addr = 0,
            .r_sense    = CONFIG_TMC2209_RSENSE,
        };
        esp_err_t tmc_err = tmc2209_init(&tmc_cfg);
        if (tmc_err != ESP_OK) {
            ESP_LOGE(TAG, "TMC2209 init failed: %s", esp_err_to_name(tmc_err));
        }

        /* Layer 2: hardware bring-up test (after TMC init) */
#ifdef CONFIG_TMC2209_BRINGUP
        tmc2209_bringup_run();
#endif
    }

    /* ---- Protocol stack ---- */
    if (bsp_err == ESP_OK) {
        esp_err_t proto_err = protocol_start();
        if (proto_err != ESP_OK) {
            ESP_LOGE(TAG, "Protocol start failed: %s", esp_err_to_name(proto_err));
        } else {
            /* ---- Step engine init (T6) ----
             * STEP/DIR GPIOs already configured by bsp_init (OUT LOW).
             * We just pass pin numbers; engine does NOT re-init GPIOs
             * and does NOT touch EN pin (TMC stays disabled until
             * M17 / joint test enables it). */
            esp_err_t se_err = step_engine_init(CONFIG_BSP_STEP_GPIO,
                                                CONFIG_BSP_DIR_GPIO);
            if (se_err != ESP_OK) {
                ESP_LOGE(TAG, "Step engine init failed: %s",
                         esp_err_to_name(se_err));
            }

            /* Register immediate stop callback (replaces protocol's
             * default "log only" placeholder — PLAN T6 spec). */
            protocol_register_stop_cb(cb_on_stop);

            /* ---- G-code parser init ----
             * Modal F/A initial values come from NVS settings (PLAN §4.3:
             * "startup initial value comes from NVS").
             * Registers as line handler, replacing T3 placeholder.
             *
             * Integration points:
             *   on_move → T7 step planner
             *   on_set_origin → T8 AS5600 + planner
             *   on_set_absolute/relative → T7 planner
             *   on_motor_enable → T5 TMC2209 (done)
             *   on_list_params/on_set_param/on_restore_defaults → T11 (done)
             */
            parser_output_iface_t iface = {
                .on_move             = cb_on_move,
                .on_set_origin       = cb_on_set_origin,
                .on_set_absolute     = cb_on_set_absolute,
                .on_set_relative     = cb_on_set_relative,
                .on_motor_enable     = cb_on_motor_enable,
                .on_list_params      = cb_on_list_params,
                .on_set_param        = cb_on_set_param,
                .on_restore_defaults = cb_on_restore_defaults,
                .ctx                 = NULL,
            };
            s_parser = gcode_parser_create(settings_default_feed(),
                                            settings_default_accel(),
                                            &iface);
            if (s_parser != NULL) {
                protocol_register_line_handler(parser_line_handler);
                ESP_LOGI(TAG, "G-code parser registered (feed=%.0f, accel=%.0f)",
                         (double)settings_default_feed(),
                         (double)settings_default_accel());
            } else {
                ESP_LOGE(TAG, "Failed to create G-code parser");
            }
        }
    }

    /* ---- Joint motion test (T5+T6, gated by Kconfig) ---- */
#ifdef CONFIG_APP_JOINT_TEST
    app_joint_test_run();
#endif

    /* Task watchdog: Idle task auto-subscribed per sdkconfig.defaults */
    xTaskCreate(placeholder_task, "placeholder", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Startup complete. System running.");
}
