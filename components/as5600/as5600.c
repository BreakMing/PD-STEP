/*
 * AS5600 magnetic encoder — 业务壳：1 kHz 采样任务 + 标定 + settings 订阅
 *
 * !! READ-ONLY DEVICE — RED LINE（红线细则与寄存器图见 as5600_hw.c）!!
 *   本组件对器件的唯一总线操作是只读的 as5600_hw_read_raw()；
 *   任何情况下不得向 ZPOS / MPOS / CONF / MANG（OTP）写入。
 *
 * Data path:
 *   as5600_task (1 kHz, prio 3, CPU1)
 *     → read RAW ANGLE 0x0C/0x0D as one 2-byte burst  (as5600_hw)
 *     → as5600_tracker_update()       (pure logic, multi-turn counts)
 *     → as5600_publish()              (atomic snapshot)
 *
 * Why CPU1: the I2C transaction blocks ~100–200 µs at 400 kHz.  Pinning
 * the task to CPU1 keeps that off CPU0, where the STEP-pulse ISR lives
 * (the ISR is timing sensitive).
 *
 * Why a 2-byte burst: two independent single-byte reads can straddle an
 * internal angle update and splice together a value that never existed.
 *
 * Direction calibration: the raw angle's sense depends on the magnet's
 * mounting orientation.  CONFIG_AS5600_INVERT_DIR (default y on this
 * build) mirrors the angle so a +X motor move increases the position —
 * without it the commanded position (C) and encoder position (E) carry
 * opposite signs and cannot be compared.  Mirroring happens in the read
 * layer (as5600_hw); see the Kconfig help.
 */

#include "as5600.h"
#include "as5600_hw.h"
#include "bsp.h"
#include "settings.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

/* Kconfig 布尔在 n 时不定义，统一折算为可用的编译期常量 */
#ifdef CONFIG_AS5600_INVERT_DIR
#define AS5600_DIR_INVERTED 1
#else
#define AS5600_DIR_INVERTED 0
#endif

static const char *TAG = "as5600";

/* ---- Timing / limits ---- */
#define AS5600_SAMPLE_PERIOD_MS 1      /* 1 kHz */
#define AS5600_FAIL_LIMIT       10     /* consecutive failures → not OK */
#define AS5600_TASK_PRIO        3
#define AS5600_TASK_CORE        1
#define AS5600_TASK_STACK       3072

/* ---- Velocity: mean over the last 16 × 1 ms differences (16 ms) ----
 * Raw 1 ms differences carry ±1 count ≈ ±9.77 mm/s of quantisation
 * noise; averaging 16 of them brings that down to ≈ ±2.4 mm/s, which is
 * plenty for "report the order of magnitude" telemetry.
 * 17 samples are kept so the difference spans exactly 16 ms. */
#define AS5600_SPEED_WINDOW     16
#define AS5600_HIST_LEN         (AS5600_SPEED_WINDOW + 1)

#define AS5600_DEFAULT_UPR      40.0f  /* fallback if $2 is unusable */

/* ---- Published state (guarded by s_lock) ---- */

typedef struct {
    int32_t  pos_counts;                    /* continuous position (truth) */
    int32_t  hist[AS5600_HIST_LEN];         /* ring of 1 ms position samples */
    int      hist_head;                     /* newest sample index */
    int      hist_fill;                     /* valid samples, saturates at LEN */
    float    vel_mm_s;                      /* smoothed velocity */
    float    offset_mm;                     /* 原点偏移（回零建立） */
    float    mm_per_count;                  /* $2 / 4096 */
    uint16_t last_raw;                      /* newest raw angle */
} as5600_state_t;

static portMUX_TYPE   s_lock = portMUX_INITIALIZER_UNLOCKED;
static as5600_state_t s_state;

static as5600_tracker_t        s_tracker;
static volatile bool           s_ok = true;    /* false = I2C failing */
static bool                    s_initialized = false;

/* ================================================================
 * State publication
 * ================================================================ */

/* Store one new sample and refresh the smoothed velocity.
 * Called from the sampling task only. */
static void as5600_publish(int32_t pos_counts, uint16_t raw)
{
    portENTER_CRITICAL(&s_lock);

    s_state.pos_counts = pos_counts;
    s_state.last_raw   = raw;

    s_state.hist_head = (s_state.hist_head + 1) % AS5600_HIST_LEN;
    s_state.hist[s_state.hist_head] = pos_counts;
    if (s_state.hist_fill < AS5600_HIST_LEN) {
        s_state.hist_fill++;
    }

    if (s_state.hist_fill == AS5600_HIST_LEN) {
        int     oldest = (s_state.hist_head + 1) % AS5600_HIST_LEN;
        int32_t delta  = pos_counts - s_state.hist[oldest];
        /* counts over 16 ms → mm/s */
        s_state.vel_mm_s = (float)delta * s_state.mm_per_count
                           * (1000.0f / (float)AS5600_SPEED_WINDOW);
    } else {
        s_state.vel_mm_s = 0.0f;   /* window not filled yet */
    }

    portEXIT_CRITICAL(&s_lock);
}

/* Recompute mm/count from $2 (units per revolution). */
static void as5600_refresh_scale(void)
{
    float upr = settings_units_per_rev();
    if (!(upr > 0.0f)) {
        ESP_LOGW(TAG, "invalid $2 units_per_rev=%.3f — falling back to %.1f mm/rev",
                 (double)upr, (double)AS5600_DEFAULT_UPR);
        upr = AS5600_DEFAULT_UPR;
    }

    float mpc = upr / (float)AS5600_COUNTS_PER_REV;

    portENTER_CRITICAL(&s_lock);
    s_state.mm_per_count = mpc;
    portEXIT_CRITICAL(&s_lock);
}

/* Settings notification — only $2 changes the counts → mm scale. */
static void as5600_on_settings_change(int idx, float new_value)
{
    (void)new_value;
    if (idx == 2) {
        as5600_refresh_scale();
        ESP_LOGI(TAG, "$2 changed — mm_per_count refreshed (%.6f mm/count)",
                 (double)(settings_units_per_rev() / (float)AS5600_COUNTS_PER_REV));
    }
}

/* ================================================================
 * Sampling task (1 kHz, CPU1)
 * ================================================================ */
static void as5600_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int        fail_streak = 0;

    /* Task watchdog: the 1 kHz cadence is far shorter than the 30 s
     * timeout, so subscribe and kick the WDT every cycle.  A failed
     * subscribe is not fatal — log it and keep sampling. */
    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(TAG, "task WDT subscribe failed — sampling task not monitored");
    }

    while (1) {
        uint16_t raw = 0;

        if (as5600_hw_read_raw(&raw)) {
            if (fail_streak >= AS5600_FAIL_LIMIT) {
                ESP_LOGI(TAG, "I2C recovered after %d consecutive failures",
                         fail_streak);
            }
            fail_streak = 0;
            s_ok = true;

            int32_t pos = as5600_tracker_update(&s_tracker, raw);
            as5600_publish(pos, raw);
        } else {
            fail_streak++;
            if (fail_streak == AS5600_FAIL_LIMIT) {
                s_ok = false;
                ESP_LOGE(TAG, "I2C read failed %d times in a row — encoder marked NOT OK",
                         fail_streak);
            }
        }

        if (wdt_ok) {
            esp_task_wdt_reset();
        }

        /* Fixed 1 kHz cadence; never busy-wait (watchdog). */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(AS5600_SAMPLE_PERIOD_MS));
    }
}

/* ================================================================
 * Lifecycle
 * ================================================================ */
esp_err_t as5600_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus = bsp_get_i2c_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized (bsp_get_i2c_bus() == NULL)");
        return ESP_ERR_INVALID_STATE;
    }

    /* 读层（驱动核心）：地址/时钟/方向镜像全由 config 注入 */
    as5600_hw_config_t hw = {
        .bus          = bus,
        .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
        .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
        .invert_dir   = AS5600_DIR_INVERTED,
    };
    ESP_RETURN_ON_ERROR(as5600_hw_init(&hw),
                        TAG, "as5600_hw_init failed");

    as5600_tracker_reset(&s_tracker);
    as5600_refresh_scale();

    if (!settings_subscribe(as5600_on_settings_change)) {
        ESP_LOGW(TAG, "settings subscribe failed (slots full) — "
                      "$2 changes will not be picked up at runtime");
    }

    /* Direction calibration status — surface it so the C/E sign
     * question can be answered from the boot log alone. */
    ESP_LOGI(TAG, "counting direction: %s",
             AS5600_DIR_INVERTED
                 ? "INVERTED (magnet mounted opposite; +X move counts up)"
                 : "normal (not inverted)");

    /* First read: seeds the tracker (position 0 here) and proves the
     * bus works before the task starts hammering it. */
    uint16_t raw = 0;
    if (as5600_hw_read_raw(&raw)) {
        int32_t pos = as5600_tracker_update(&s_tracker, raw);
        as5600_publish(pos, raw);
        ESP_LOGI(TAG, "AS5600 online at 0x%02x — first raw angle=%u (seeded, E=0)",
                 CONFIG_BSP_AS5600_I2C_ADDR, raw);
    } else {
        ESP_LOGW(TAG, "first RAW ANGLE read at 0x%02x failed — task will retry",
                 CONFIG_BSP_AS5600_I2C_ADDR);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(as5600_task, "as5600",
                                            AS5600_TASK_STACK, NULL,
                                            AS5600_TASK_PRIO, NULL,
                                            AS5600_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create sampling task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "sampling task up: %d Hz, prio %d, CPU%d, %.6f mm/count "
                  "(%.3f mm/rev ÷ %d counts)",
             (int)(1000 / AS5600_SAMPLE_PERIOD_MS), AS5600_TASK_PRIO,
             AS5600_TASK_CORE,
             (double)(settings_units_per_rev() / (float)AS5600_COUNTS_PER_REV),
             (double)settings_units_per_rev(), AS5600_COUNTS_PER_REV);
    return ESP_OK;
}

/* ================================================================
 * Telemetry getters
 * ================================================================ */

float as5600_position_mm(void)
{
    float mm;
    portENTER_CRITICAL(&s_lock);
    mm = (float)s_state.pos_counts * s_state.mm_per_count - s_state.offset_mm;
    portEXIT_CRITICAL(&s_lock);
    return mm;
}

int32_t as5600_position_counts(void)
{
    int32_t counts;
    portENTER_CRITICAL(&s_lock);
    counts = s_state.pos_counts;
    portEXIT_CRITICAL(&s_lock);
    return counts;
}

int64_t as5600_position_um(void)
{
    portENTER_CRITICAL(&s_lock);
    int32_t counts = s_state.pos_counts;
    float offset_mm = s_state.offset_mm;
    portEXIT_CRITICAL(&s_lock);

    /* 原点偏移取整到 µm：调用方（回零）传入整数 mm，转换精确 */
    int64_t offset_um = (int64_t)llround((double)offset_mm * 1000.0);
    return as5600_counts_to_um(counts) - offset_um;
}

float as5600_velocity_mm_s(void)
{
    float v;
    portENTER_CRITICAL(&s_lock);
    v = s_state.vel_mm_s;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

uint16_t as5600_last_raw(void)
{
    uint16_t raw;
    portENTER_CRITICAL(&s_lock);
    raw = s_state.last_raw;
    portEXIT_CRITICAL(&s_lock);
    return raw;
}

bool as5600_is_ok(void)
{
    return s_ok;
}

/* ================================================================
 * 原点偏移（回零建立）
 * ================================================================ */
void as5600_set_origin(float new_pos_mm)
{
    float offset;
    float raw_mm;

    portENTER_CRITICAL(&s_lock);
    raw_mm = (float)s_state.pos_counts * s_state.mm_per_count;
    s_state.offset_mm = raw_mm - new_pos_mm;
    offset = s_state.offset_mm;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "原点已重置：%.3f mm（原始 %.3f mm，偏移 %.3f mm）",
             (double)new_pos_mm, (double)raw_mm, (double)offset);
}
