/* UART0 空载测流入口：复用收发载体，由本任务串行访问电机与测量硬件。 */
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "bsp.h"
#include "cartproto.h"
#include "ch224q.h"
#include "magnetic_encoder_mt6835.h"
#include "step_engine.h"
#include "tmc2209.h"
#include "current_sequence.h"
#include "current_pd.h"

enum {
    REG_GCONF = 0x00, REG_IFCNT = 0x02, REG_IOIN = 0x06,
    REG_MSCNT = 0x6A, REG_MSCURACT = 0x6B, REG_CHOPCONF = 0x6C,
    HEALTH_PERIOD_US = 100000, FEED_PERIOD_US = 25000,
    MIN_VBUS_MV = 8000, MAX_VBUS_MV = 21500,
};

typedef enum {
    ERROR_COMMAND, ERROR_QUEUE, ERROR_BUSY, ERROR_INIT, ERROR_VBUS,
    ERROR_ENCODER, ERROR_TMC_READ, ERROR_TMC_FAULT, ERROR_TMC_CONFIG,
    ERROR_ENABLE, ERROR_PD_READ, ERROR_PD_HANDSHAKE, ERROR_PD_WRITE,
    ERROR_PD_TIMEOUT, ERROR_PD_UNVERIFIED,
} current_error_t;

static const char *const s_errors[] = {
    [ERROR_COMMAND] = "COMMAND", [ERROR_QUEUE] = "QUEUE",
    [ERROR_BUSY] = "BUSY", [ERROR_INIT] = "INIT", [ERROR_VBUS] = "VBUS",
    [ERROR_ENCODER] = "ENCODER", [ERROR_TMC_READ] = "TMC_READ",
    [ERROR_TMC_FAULT] = "TMC_FAULT", [ERROR_TMC_CONFIG] = "TMC_CONFIG",
    [ERROR_ENABLE] = "ENABLE", [ERROR_PD_READ] = "PD_READ",
    [ERROR_PD_HANDSHAKE] = "PD_HANDSHAKE", [ERROR_PD_WRITE] = "PD_WRITE",
    [ERROR_PD_TIMEOUT] = "PD_TIMEOUT", [ERROR_PD_UNVERIFIED] = "PD_UNVERIFIED",
};

static QueueHandle_t s_commands;
static current_sequence_t s_sequence;
static current_pd_t s_pd_check;
static int64_t s_last_pd_poll;
static magnetic_encoder_handle_t s_encoder;
static ch224q_handle_t s_pd;
static bool s_tmc_ready, s_step_ready;
static int64_t s_start_counts, s_last_feed, s_last_health;

static void reply(const char *format, ...)
{
    char buf[512];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    assert(n >= 0 && (size_t)n < sizeof(buf));
    cartproto_send_raw(buf, n);
}

static const char *phase_name(void)
{
    if (s_pd_check.state == CURRENT_PD_WAITING) return "PD_WAIT";
    switch (s_sequence.phase) {
    case CURRENT_RUN: return "RUN";
    case CURRENT_HOLD: return "HOLD";
    default: return "IDLE";
    }
}

static const char *pd_state_name(void)
{
    switch (s_pd_check.state) {
    case CURRENT_PD_WAITING: return "WAITING";
    case CURRENT_PD_VERIFIED: return "VERIFIED";
    case CURRENT_PD_UNCERTAIN: return "UNCERTAIN";
    default: return "UNCHANGED";
    }
}

static int vbus_mv(void)
{
    float v = bsp_vbus_read_volt();
    return v < 0 ? -1 : (int)(v * 1000.0f + 0.5f);
}

static bool ready(void)
{
    return s_tmc_ready && s_step_ready && s_encoder != NULL;
}

static void stop_outputs(void)
{
    if (s_step_ready) step_engine_stop();
    gpio_set_level(CONFIG_BSP_EN_GPIO, 1);
}

static void error_reply(current_error_t error)
{
    reply("CURRENT ERROR %s EN=%d\n", s_errors[error],
          gpio_get_level(CONFIG_BSP_EN_GPIO));
}

static void fault_stop(current_error_t error)
{
    stop_outputs();
    current_sequence_stop(&s_sequence);
    current_pd_cancel(&s_pd_check);
    xQueueReset(s_commands);
    error_reply(error);
}

/* 只有完整、明确的测量命令进入队列；STOP 清理旧请求并优先处理。 */
static void on_line(const char *line)
{
    current_command_t command = current_command_parse(line);
    if (command == CURRENT_CMD_INVALID) {
        error_reply(ERROR_COMMAND);
    } else if (command == CURRENT_CMD_STOP) {
        xQueueReset(s_commands);
        if (xQueueSendToFront(s_commands, &command, 0) != pdTRUE)
            error_reply(ERROR_QUEUE);
    } else if (xQueueSend(s_commands, &command, 0) != pdTRUE) {
        error_reply(ERROR_QUEUE);
    }
}

/* 成功不修改 error；失败必须由调用者关闭电机。 */
static bool healthy(current_error_t *error)
{
    if (!ready()) { *error = ERROR_INIT; return false; }
    int mv = vbus_mv();
    if (mv < MIN_VBUS_MV || mv > MAX_VBUS_MV) {
        *error = ERROR_VBUS; return false;
    }
    if (!magnetic_encoder_is_ok(s_encoder) ||
        magnetic_encoder_mt6835_last_status(s_encoder) != 0) {
        *error = ERROR_ENCODER; return false;
    }
    tmc2209_status_t st;
    if (!tmc2209_read_status(&st)) { *error = ERROR_TMC_READ; return false; }
    /* 包括温度告警、短接 GND 和短接电源；低电流下开路标志只记录。 */
    if (st.ot || st.otpw || st.s2ga || st.s2gb || (st.raw & (3U << 4))) {
        *error = ERROR_TMC_FAULT; return false;
    }
    return true;
}

static void info(void)
{
    reply("CURRENT INFO PHASE=%s EN=%d READY=%d VBUS_MV=%d "
          "RUN_MA=%d HOLD_MA=%d MICROSTEP=%d SPEED_STEPS_S=%d RUN_MS=%d HOLD_MS=%d "
          "PD_STATE=%s TARGET_V=%d\n",
          phase_name(), gpio_get_level(CONFIG_BSP_EN_GPIO), ready(), vbus_mv(),
          CURRENT_RUN_MA, CURRENT_HOLD_MA, CURRENT_MICROSTEP, CURRENT_SPEED_STEPS_S,
          CURRENT_RUN_US / 1000, CURRENT_HOLD_US / 1000,
          pd_state_name(), (int)s_pd_check.target);
    if (s_encoder) {
        reply("CURRENT ENCODER OK=%d STATUS=%u RAW=%" PRIu32 " COUNTS=%" PRId64 "\n",
              magnetic_encoder_is_ok(s_encoder),
              (unsigned)magnetic_encoder_mt6835_last_status(s_encoder),
              magnetic_encoder_mt6835_last_raw(s_encoder),
              magnetic_encoder_position_counts(s_encoder));
    }
    if (s_tmc_ready) {
        uint32_t ioin, gconf, chop, ifcnt, mscnt, mscuract;
        tmc2209_status_t st;
        if (tmc2209_read_reg(REG_IOIN, &ioin) &&
            tmc2209_read_reg(REG_GCONF, &gconf) &&
            tmc2209_read_reg(REG_CHOPCONF, &chop) &&
            tmc2209_read_reg(REG_IFCNT, &ifcnt) &&
            tmc2209_read_reg(REG_MSCNT, &mscnt) &&
            tmc2209_read_reg(REG_MSCURACT, &mscuract) &&
            tmc2209_read_status(&st)) {
            reply("CURRENT TMC IOIN=%08" PRIx32 " GCONF=%08" PRIx32
                  " CHOPCONF=%08" PRIx32 " IFCNT=%" PRIu32
                  " DRV_STATUS=%08" PRIx32 " CS_ACTUAL=%d MSCNT=%" PRIu32
                  " MSCURACT=%08" PRIx32 "\n",
                  ioin, gconf, chop, ifcnt, st.raw, st.cs_actual, mscnt, mscuract);
        } else {
            fault_stop(ERROR_TMC_READ);
        }
    }
    if (s_pd) {
        ch224q_status_t st;
        if (ch224q_get_status(s_pd, &st) == ESP_OK)
            reply("CURRENT PD PD=%d QC3=%d QC2=%d\n", st.pd, st.qc3, st.qc2);
        else
            reply("CURRENT PD READ_ERROR\n");
    } else {
        reply("CURRENT PD UNAVAILABLE\n");
    }
    reply("CURRENT INFO_END\n");
}

static bool configure_driver(void)
{
    uint32_t before, after, ioin, gconf, chop;
    if (!tmc2209_read_reg(REG_IFCNT, &before)) return false;
    if (tmc2209_set_current(CURRENT_RUN_MA, CURRENT_HOLD_MA) != ESP_OK ||
        tmc2209_set_microstep(CURRENT_MICROSTEP) != ESP_OK) return false;
    if (!tmc2209_read_reg(REG_IFCNT, &after) ||
        !tmc2209_read_reg(REG_IOIN, &ioin) ||
        !tmc2209_read_reg(REG_GCONF, &gconf) ||
        !tmc2209_read_reg(REG_CHOPCONF, &chop)) return false;
    /* IHOLD_IRUN 只写；IFCNT 增量证明至少两个配置写帧已被器件接收。 */
    if ((uint8_t)(after - before) < 2) return false;
    if ((ioin >> 24) != 0x21 || (gconf & 0xC7U) != 0xC4U ||
        ((chop >> 24) & 0xFU) != 5 || (chop & 0xFU) == 0 ||
        (chop & (1U << 17)) == 0) return false;
    reply("CURRENT CONFIG_OK IFCNT_BEFORE=%" PRIu32 " IFCNT_AFTER=%" PRIu32
          " GCONF=%08" PRIx32 " CHOPCONF=%08" PRIx32 "\n",
          before, after, gconf, chop);
    return true;
}

static void start(void)
{
    if (s_sequence.phase != CURRENT_IDLE || s_pd_check.state == CURRENT_PD_WAITING) {
        error_reply(ERROR_BUSY); return;
    }
    if (!current_pd_start_allowed(&s_pd_check)) {
        fault_stop(ERROR_PD_UNVERIFIED); return;
    }
    stop_outputs();
    current_error_t error;
    if (!healthy(&error)) { fault_stop(error); return; }
    if (!configure_driver()) { fault_stop(ERROR_TMC_CONFIG); return; }
    step_engine_set_position(0);
    s_start_counts = magnetic_encoder_position_counts(s_encoder);
    if (tmc2209_enable(true) != ESP_OK || gpio_get_level(CONFIG_BSP_EN_GPIO) != 0) {
        fault_stop(ERROR_ENABLE); return;
    }
    int64_t now = esp_timer_get_time();
    if (!current_sequence_start(&s_sequence, now)) {
        fault_stop(ERROR_BUSY); return;
    }
    s_last_feed = now;
    s_last_health = now;
    step_engine_vel_target((int64_t)CURRENT_SPEED_STEPS_S * 65536);
    reply("CURRENT RUN EN=0 DURATION_MS=%d RUN_MA=%d\n",
          CURRENT_RUN_US / 1000, CURRENT_RUN_MA);
}

static void request_pd(current_pd_target_t target)
{
    if (s_sequence.phase != CURRENT_IDLE || s_pd_check.state == CURRENT_PD_WAITING) {
        error_reply(ERROR_BUSY); return;
    }
    if (gpio_get_level(CONFIG_BSP_EN_GPIO) != 1) {
        fault_stop(ERROR_ENABLE); return;
    }
    ch224q_status_t st;
    esp_err_t err = ch224q_get_status(s_pd, &st);
    if (err != ESP_OK) { fault_stop(ERROR_PD_READ); return; }
    if (!st.pd) { fault_stop(ERROR_PD_HANDSHAKE); return; }
    int mv = vbus_mv();
    if (mv < MIN_VBUS_MV || mv > MAX_VBUS_MV) { fault_stop(ERROR_VBUS); return; }
    stop_outputs();
    if (!current_pd_begin(&s_pd_check, target, s_sequence.phase == CURRENT_IDLE,
                          gpio_get_level(CONFIG_BSP_EN_GPIO) == 1, esp_timer_get_time())) {
        fault_stop(ERROR_ENABLE); return;
    }
    xQueueReset(s_commands);
    ch224q_vsel_t vsel = target == CURRENT_PD_20V ? CH224Q_VSEL_20V : CH224Q_VSEL_9V;
    err = ch224q_set_voltage(s_pd, vsel);
    if (err != ESP_OK) {
        reply("CURRENT PD_WRITE_ERROR TARGET_V=%d ERR=%s\n", (int)target, esp_err_to_name(err));
        fault_stop(ERROR_PD_WRITE); return;
    }
    s_last_pd_poll = esp_timer_get_time() - CURRENT_PD_SAMPLE_US;
    reply("CURRENT PD_REQUEST TARGET_V=%d EN=%d VBUS_BEFORE_MV=%d\n",
          (int)target, gpio_get_level(CONFIG_BSP_EN_GPIO), mv);
}

static void advance_pd(void)
{
    if (s_pd_check.state != CURRENT_PD_WAITING) return;
    if (gpio_get_level(CONFIG_BSP_EN_GPIO) != 1) { fault_stop(ERROR_ENABLE); return; }
    int64_t now = esp_timer_get_time();
    if (now - s_last_pd_poll < CURRENT_PD_SAMPLE_US) return;
    s_last_pd_poll = now;
    int mv = vbus_mv();
    ch224q_status_t st;
    esp_err_t err = ch224q_get_status(s_pd, &st);
    bool pd_valid = err == ESP_OK && st.pd;
    reply("CURRENT PD_SAMPLE TARGET_V=%d VBUS_MV=%d PD=%d READ_OK=%d EN=%d\n",
          (int)s_pd_check.target, mv, pd_valid, err == ESP_OK,
          gpio_get_level(CONFIG_BSP_EN_GPIO));
    if (mv > MAX_VBUS_MV) { fault_stop(ERROR_VBUS); return; }
    current_pd_result_t result = current_pd_sample(&s_pd_check, now, pd_valid, mv);
    if (result == CURRENT_PD_TIMEOUT) {
        fault_stop(ERROR_PD_TIMEOUT);
    } else if (result == CURRENT_PD_SUCCESS) {
        stop_outputs();
        xQueueReset(s_commands);
        reply("CURRENT PD_DONE TARGET_V=%d VBUS_MV=%d PD=%d EN=%d ELAPSED_MS=%" PRId64 "\n",
              (int)s_pd_check.target, mv, pd_valid, gpio_get_level(CONFIG_BSP_EN_GPIO),
              (now - s_pd_check.started_us) / 1000);
    }
}

static void advance(void)
{
    int64_t now = esp_timer_get_time();
    current_action_t action = current_sequence_tick(&s_sequence, now);
    if (action == CURRENT_ACTION_DISABLE) {
        stop_outputs();
        xQueueReset(s_commands);
        reply("CURRENT DONE EN=%d STEPS=%" PRId32 " ENCODER_DELTA=%" PRId64
              " ELAPSED_MS=%" PRId64 "\n", gpio_get_level(CONFIG_BSP_EN_GPIO),
              step_engine_position_steps(),
              magnetic_encoder_position_counts(s_encoder) - s_start_counts,
              (now - s_sequence.started_us) / 1000);
    } else if (action == CURRENT_ACTION_HOLD) {
        step_engine_stop();
        reply("CURRENT HOLD EN=%d DURATION_MS=%d HOLD_MA=%d\n",
              gpio_get_level(CONFIG_BSP_EN_GPIO), CURRENT_HOLD_US / 1000, CURRENT_HOLD_MA);
    }
    if (s_sequence.phase == CURRENT_IDLE) return;
    if (s_sequence.phase == CURRENT_RUN && now - s_last_feed >= FEED_PERIOD_US) {
        step_engine_vel_target((int64_t)CURRENT_SPEED_STEPS_S * 65536);
        s_last_feed = now;
    }
    if (now - s_last_health >= HEALTH_PERIOD_US) {
        current_error_t error;
        if (!healthy(&error)) fault_stop(error);
        s_last_health = now;
    }
}

void app_main(void)
{
    bsp_early_init();
    ESP_ERROR_CHECK(bsp_init());
    s_commands = xQueueCreate(4, sizeof(current_command_t));
    ESP_ERROR_CHECK(s_commands ? ESP_OK : ESP_ERR_NO_MEM);
    cartproto_register_line_handler(on_line);
    ESP_ERROR_CHECK(cartproto_start());

    tmc2209_config_t cfg = {
        .uart_num = UART_NUM_2,
        .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
        .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
        .en_gpio = CONFIG_BSP_EN_GPIO,
        .baud = 115200, .slave_addr = 0,
        .r_sense = CONFIG_TMC2209_RSENSE,
        .init_microstep = CURRENT_MICROSTEP,
        .init_run_ma = CURRENT_RUN_MA, .init_hold_ma = CURRENT_HOLD_MA,
    };
    s_tmc_ready = tmc2209_init(&cfg) == ESP_OK;
    s_step_ready = step_engine_init(CONFIG_BSP_STEP_GPIO, CONFIG_BSP_DIR_GPIO) == ESP_OK;
    if (s_step_ready) {
        step_engine_vel_set_accel(10000);
        step_engine_vel_set_decel(10000);
        step_engine_vel_set_wdt_us(100000);
    }
    magnetic_encoder_mt6835_config_t encoder_cfg = {
        .host = bsp_get_cart_enc_spi_host(),
        .clock_speed_hz = CONFIG_BSP_CART_ENC_SPI_HZ,
        .spics_io_num = CONFIG_BSP_CART_ENC_CS_GPIO, .task_name = "me_cart",
    };
    if (magnetic_encoder_mt6835_init(&encoder_cfg, &s_encoder) != ESP_OK)
        s_encoder = NULL;
    ch224q_config_t pd_cfg = {
        .bus = bsp_get_i2c_bus(), .dev_addr = CONFIG_CH224Q_I2C_ADDR,
    };
    if (ch224q_init(&pd_cfg, &s_pd) != ESP_OK) s_pd = NULL;
    stop_outputs();
    xQueueReset(s_commands);
    reply("CURRENT READY EN=%d INIT_OK=%d\n",
          gpio_get_level(CONFIG_BSP_EN_GPIO), ready());
    info();
    while (true) {
        advance();
        advance_pd();
        current_command_t command;
        if (xQueueReceive(s_commands, &command, pdMS_TO_TICKS(5)) != pdTRUE) continue;
        switch (command) {
        case CURRENT_CMD_INFO: info(); break;
        case CURRENT_CMD_START: start(); break;
        case CURRENT_CMD_PD20: request_pd(CURRENT_PD_20V); break;
        case CURRENT_CMD_PD9: request_pd(CURRENT_PD_9V); break;
        case CURRENT_CMD_STOP:
            stop_outputs();
            current_sequence_stop(&s_sequence);
            current_pd_cancel(&s_pd_check);
            xQueueReset(s_commands);
            reply("CURRENT STOPPED EN=%d PD_STATE=%s TARGET_V=%d\n",
                  gpio_get_level(CONFIG_BSP_EN_GPIO), pd_state_name(), (int)s_pd_check.target);
            break;
        default: error_reply(ERROR_COMMAND); break;
        }
    }
}
