/*
 * magnetic_encoder_as5600 — magnetic_encoder 契约的 AS5600 实现
 *
 * 吸收自原 as5600.c 设备组件：1 kHz 采样任务（CPU1、优先级 3，
 * I2C 阻塞事务不扰 CPU0 的 STEP ISR）、连续失败计数（≥10 次
 * NOT OK）、16 样本速度窗。只读红线与寄存器图见 as5600_hw.c；
 * 多圈 tracker 为纯逻辑（as5600_tracker.c，宿主可测）。
 *
 * 发布语义：契约值在 s_lock 内原子发布；速度 = 16 ms 窗位置差
 * × 62.5（量子 62.5 counts/s）。2 字节突发读避免跨内部角度更新
 * 的拼接（见 as5600_hw）。
 */
#include "magnetic_encoder_as5600.h"
#include "as5600.h"
#include "as5600_hw.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

static const char *TAG = "me_as5600";

#define ME_AS5600_SAMPLE_PERIOD_MS  1      /* 1 kHz */
#define ME_AS5600_FAIL_LIMIT        10
#define ME_AS5600_TASK_PRIO         3
#define ME_AS5600_TASK_CORE         1
#define ME_AS5600_TASK_STACK        3072

/* ---- 速度：16 ms 窗（17 个 1 ms 位置样本） ---- */
#define ME_AS5600_SPEED_WINDOW     16
#define ME_AS5600_HIST_LEN         (ME_AS5600_SPEED_WINDOW + 1)

/* ---- 发布状态（s_lock 保护） ---- */
typedef struct magnetic_encoder_s {
    int64_t  pos_counts;             /* 多圈累计（accumulator） */
    int32_t  hist[ME_AS5600_HIST_LEN];
    int      hist_head;
    int      hist_fill;
    int32_t  vel_counts_per_s;
    uint16_t last_raw;
} me_state_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static me_state_t s_state;
static as5600_tracker_t s_tracker;
static volatile bool s_ok = true;        /* false = I2C failing */
static bool s_initialized = false;

/* Store one new sample and refresh the velocity estimate.
 * Called from the sampling task only. */
static void publish(int32_t pos_counts, uint16_t raw)
{
    portENTER_CRITICAL(&s_lock);

    s_state.pos_counts = pos_counts;
    s_state.last_raw   = raw;

    s_state.hist_head = (s_state.hist_head + 1) % ME_AS5600_HIST_LEN;
    s_state.hist[s_state.hist_head] = pos_counts;
    if (s_state.hist_fill < ME_AS5600_HIST_LEN) {
        s_state.hist_fill++;
    }

    if (s_state.hist_fill == ME_AS5600_HIST_LEN) {
        int     oldest = (s_state.hist_head + 1) % ME_AS5600_HIST_LEN;
        int32_t delta  = pos_counts - s_state.hist[oldest];
        /* counts over 16 ms → counts/s（量子 62.5） */
        s_state.vel_counts_per_s =
            (int32_t)llround((double)delta * (1000.0 / ME_AS5600_SPEED_WINDOW));
    } else {
        s_state.vel_counts_per_s = 0;   /* window not filled yet */
    }

    portEXIT_CRITICAL(&s_lock);
}

static void me_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int        fail_streak = 0;

    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(TAG, "task WDT subscribe failed — sampling task not monitored");
    }

    while (1) {
        uint16_t raw = 0;

        if (as5600_hw_read_raw(&raw)) {
            if (fail_streak >= ME_AS5600_FAIL_LIMIT) {
                ESP_LOGI(TAG, "I2C recovered after %d consecutive failures",
                         fail_streak);
            }
            fail_streak = 0;
            s_ok = true;

            int32_t pos = as5600_tracker_update(&s_tracker, raw);
            publish(pos, raw);
        } else {
            fail_streak++;
            if (fail_streak == ME_AS5600_FAIL_LIMIT) {
                s_ok = false;
                ESP_LOGE(TAG, "I2C read failed %d times in a row — encoder marked NOT OK",
                         fail_streak);
            }
        }

        if (wdt_ok) {
            esp_task_wdt_reset();
        }

        /* Fixed 1 kHz cadence; never busy-wait (watchdog). */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ME_AS5600_SAMPLE_PERIOD_MS));
    }
}

/* ---- 契约函数 ---- */

int32_t magnetic_encoder_counts_per_rev(magnetic_encoder_handle_t enc)
{
    (void)enc;
    return AS5600_COUNTS_PER_REV;
}

int64_t magnetic_encoder_position_counts(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    int64_t p = s_state.pos_counts;
    portEXIT_CRITICAL(&s_lock);
    return p;
}

int32_t magnetic_encoder_velocity_counts_per_s(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    int32_t v = s_state.vel_counts_per_s;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

bool magnetic_encoder_is_ok(magnetic_encoder_handle_t enc)
{
    (void)enc;
    return s_ok;
}

uint16_t magnetic_encoder_as5600_last_raw(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    uint16_t r = s_state.last_raw;
    portEXIT_CRITICAL(&s_lock);
    return r;
}

bool magnetic_encoder_as5600_invert_dir_default(void)
{
#ifdef CONFIG_AS5600_INVERT_DIR
    return true;
#else
    return false;
#endif
}

/* ---- 生命周期 ---- */

esp_err_t magnetic_encoder_as5600_init(const magnetic_encoder_as5600_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out)
{
    if (s_initialized) {
        *handle_out = (magnetic_encoder_handle_t)&s_state;
        return ESP_OK;
    }
    if (cfg == NULL || handle_out == NULL || cfg->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 读层（驱动核心）：地址/时钟/方向镜像全由 config 注入 */
    as5600_hw_config_t hw = {
        .bus          = cfg->bus,
        .dev_addr     = cfg->dev_addr,
        .scl_speed_hz = cfg->scl_speed_hz,
        .invert_dir   = cfg->invert_dir,
    };
    ESP_RETURN_ON_ERROR(as5600_hw_init(&hw),
                        TAG, "as5600_hw_init failed");

    as5600_tracker_reset(&s_tracker);

    ESP_LOGI(TAG, "counting direction: %s",
             cfg->invert_dir
                 ? "INVERTED (magnet mounted opposite; +X move counts up)"
                 : "normal (not inverted)");

    /* First read: seeds the tracker (position 0 here) and proves the
     * bus works before the task starts hammering it. */
    uint16_t raw = 0;
    if (as5600_hw_read_raw(&raw)) {
        int32_t pos = as5600_tracker_update(&s_tracker, raw);
        publish(pos, raw);
        ESP_LOGI(TAG, "AS5600 online at 0x%02x — first raw angle=%u (seeded, pos=0)",
                 cfg->dev_addr, raw);
    } else {
        ESP_LOGW(TAG, "first RAW ANGLE read at 0x%02x failed — task will retry",
                 cfg->dev_addr);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(me_task, "me_as5600",
                                            ME_AS5600_TASK_STACK, NULL,
                                            ME_AS5600_TASK_PRIO, NULL,
                                            ME_AS5600_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create sampling task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "sampling task up: 1 kHz, prio %d, CPU%d, %d counts/rev",
             ME_AS5600_TASK_PRIO, ME_AS5600_TASK_CORE, AS5600_COUNTS_PER_REV);
    *handle_out = (magnetic_encoder_handle_t)&s_state;
    return ESP_OK;
}
