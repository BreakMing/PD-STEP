/*
 * magnetic_encoder_mt6835 — magnetic_encoder 契约的 MT6835 实现
 *
 * 吸收自原 mt6835.c 设备组件：1 kHz 采样任务（CPU1、优先级 3，
 * SPI 阻塞事务不扰 CPU0 的 STEP ISR）、连续失败计数（≥10 次
 * NOT OK）、16 样本速度窗、STATUS 位变化告警与失败分诊日志。
 * SPI 帧格式与校验（OP_ANGLE/ANGLE1 突发、全零帧与 CRC 拒收、
 * 失败帧原文留存）在 mt6835_hw.c（驱动核心层）；多圈累计为
 * 参数化纯逻辑（magnetic_encoder_tracker.c，宿主可测）。
 *
 * 发布语义：契约值在 s_lock 内原子发布；速度 = 16 ms 窗位置差
 * × 62.5（量子 62.5 counts/s，与 AS5600 实现同一式）。µrad 换算
 * 与零位/符号标定归解释层（poleang），本实现只发布 counts。
 */
#include "magnetic_encoder_mt6835.h"
#include "mt6835_urad.h"
#include "mt6835_hw.h"
#include "magnetic_encoder_tracker.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

static const char *TAG = "me_mt6835";

/* ---- 时序/参数 ---- */
#define ME_MT6835_SAMPLE_PERIOD_MS  1      /* 1 kHz */
#define ME_MT6835_FAIL_LIMIT        10     /* 连续失败 → not OK */
#define ME_MT6835_TASK_PRIO         3
#define ME_MT6835_TASK_CORE         1
#define ME_MT6835_TASK_STACK        3072

/* ---- 速度：16 ms 窗（17 个 1 ms 位置样本） ---- */
#define ME_MT6835_SPEED_WINDOW     16
#define ME_MT6835_HIST_LEN         (ME_MT6835_SPEED_WINDOW + 1)

/* ---- 操作表函数（单实例：直接读写本文件静态状态） ---- */
static int32_t me_counts_per_rev(magnetic_encoder_handle_t enc);
static int64_t  me_position_counts(magnetic_encoder_handle_t enc);
static int32_t  me_velocity_counts_per_s(magnetic_encoder_handle_t enc);
static bool     me_is_ok(magnetic_encoder_handle_t enc);

/* ---- 发布状态（s_lock 保护；操作表为首成员，句柄即其地址） ---- */
typedef struct {
    struct magnetic_encoder_s ops;
    mt6835_hw_handle_t hw;           /* 帧层实例（驱动核心层） */
    magnetic_encoder_tracker_t tracker;   /* 多圈累计（首样本种子） */
    int64_t  pos_counts;             /* 多圈连续位置 */
    int64_t  hist[ME_MT6835_HIST_LEN];   /* 速度窗样本与位置同为 64 位域 */
    int      hist_head;
    int      hist_fill;
    int32_t  vel_counts_per_s;
    uint8_t  status;                 /* 最新 STATUS[2:0] */
} me_state_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static me_state_t s_state = {
    .ops = {
        .counts_per_rev        = me_counts_per_rev,
        .position_counts       = me_position_counts,
        .velocity_counts_per_s = me_velocity_counts_per_s,
        .is_ok                 = me_is_ok,
    },
};

static volatile bool s_ok = true;
static bool s_initialized = false;

static const char *fail_str(mt6835_hw_fail_t k)
{
    switch (k) {
    case MT6835_HW_FAIL_SPI_ERR:   return "SPI 事务错误";
    case MT6835_HW_FAIL_ALL_ZERO:  return "全零帧（MISO 悬空/器件未应答）";
    case MT6835_HW_FAIL_CRC:       return "CRC 不符";
    default:             return "?";
    }
}

/* ================================================================
 * 状态发布（仅采样任务调用）
 * ================================================================ */

static void publish(uint32_t raw21, uint8_t status)
{
    portENTER_CRITICAL(&s_lock);

    /* 多圈累计：首样本种子（pos=0），跨 0/2^21−1 边界按 cpr/2 修正 */
    int64_t pos = magnetic_encoder_tracker_update(&s_state.tracker, raw21,
                                                  (int32_t)MT6835_COUNTS_PER_REV);
    s_state.pos_counts = pos;
    s_state.status = status;

    s_state.hist_head = (s_state.hist_head + 1) % ME_MT6835_HIST_LEN;
    s_state.hist[s_state.hist_head] = s_state.pos_counts;
    if (s_state.hist_fill < ME_MT6835_HIST_LEN) {
        s_state.hist_fill++;
    }

    if (s_state.hist_fill == ME_MT6835_HIST_LEN) {
        int     oldest = (s_state.hist_head + 1) % ME_MT6835_HIST_LEN;
        int64_t delta  = s_state.pos_counts - s_state.hist[oldest];
        /* counts over 16 ms → counts/s（量子 62.5） */
        s_state.vel_counts_per_s =
            (int32_t)llround((double)delta * (1000.0 / ME_MT6835_SPEED_WINDOW));
    } else {
        s_state.vel_counts_per_s = 0;
    }

    portEXIT_CRITICAL(&s_lock);
}

/* ================================================================
 * 采样任务（1 kHz，CPU1）
 * ================================================================ */

static void me_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int fail_streak = 0;
    uint8_t last_status = 0;

    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(TAG, "任务看门狗订阅失败——采样任务不受监控");
    }

    while (1) {
        uint32_t raw = 0;
        uint8_t status = 0;

        if (mt6835_hw_read_raw(s_state.hw, &raw, &status)) {
            if (fail_streak >= ME_MT6835_FAIL_LIMIT) {
                ESP_LOGI(TAG, "SPI 在连续 %d 次失败后恢复", fail_streak);
            }
            fail_streak = 0;
            s_ok = true;

            if (status != last_status && status != 0) {
                ESP_LOGW(TAG, "STATUS 变化：0x%x（bit0 过速 bit1 磁弱 bit2 欠压）",
                         status);
            }
            last_status = status;

            publish(raw, status);
        } else {
            fail_streak++;
            if (fail_streak == ME_MT6835_FAIL_LIMIT) {
                s_ok = false;
                mt6835_hw_fail_t kind;
                uint8_t frame[MT6835_HW_FRAME_BYTES];
                mt6835_hw_last_failure(s_state.hw, &kind, frame);
                ESP_LOGE(TAG, "SPI 读连续失败 %d 次——编码器标记 NOT OK（%s）",
                         fail_streak, fail_str(kind));
                ESP_LOGE(TAG, "末帧 rx = %02x %02x %02x %02x %02x %02x"
                         "（tx = a0 03 00 00 00 00；rx[2..4]=角度/status，"
                         "rx[5]=器件 CRC）",
                         frame[0], frame[1], frame[2],
                         frame[3], frame[4], frame[5]);
            }
        }

        if (wdt_ok) {
            esp_task_wdt_reset();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ME_MT6835_SAMPLE_PERIOD_MS));
    }
}

/* ================================================================
 * 契约操作表函数
 * ================================================================ */

static int32_t me_counts_per_rev(magnetic_encoder_handle_t enc)
{
    (void)enc;
    return (int32_t)MT6835_COUNTS_PER_REV;
}

static int64_t me_position_counts(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    int64_t p = s_state.pos_counts;
    portEXIT_CRITICAL(&s_lock);
    return p;
}

static int32_t me_velocity_counts_per_s(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    int32_t v = s_state.vel_counts_per_s;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

static bool me_is_ok(magnetic_encoder_handle_t enc)
{
    (void)enc;
    return s_ok;
}

/* ---- 器件自测面 ---- */

uint32_t magnetic_encoder_mt6835_last_raw(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    uint32_t r = s_state.tracker.last_raw;
    portEXIT_CRITICAL(&s_lock);
    return r;
}

uint8_t magnetic_encoder_mt6835_last_status(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    uint8_t s = s_state.status;
    portEXIT_CRITICAL(&s_lock);
    return s;
}

/* ================================================================
 * 生命周期
 * ================================================================ */

esp_err_t magnetic_encoder_mt6835_init(const magnetic_encoder_mt6835_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out)
{
    if (s_initialized) {
        *handle_out = (magnetic_encoder_handle_t)&s_state;
        return ESP_OK;
    }
    if (cfg == NULL || handle_out == NULL || cfg->host < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 帧层（驱动核心）：时钟/CS 由 config 注入 */
    mt6835_hw_config_t hw = {
        .host          = cfg->host,
        .clock_speed_hz = cfg->clock_speed_hz,
        .spics_io_num  = cfg->spics_io_num,
    };
    mt6835_hw_handle_t hwdev = NULL;
    ESP_RETURN_ON_ERROR(mt6835_hw_init(&hw, &hwdev), TAG, "mt6835_hw_init 失败");

    /* 状态清零并保留操作表 */
    s_state = (me_state_t){
        .ops = {
            .counts_per_rev        = me_counts_per_rev,
            .position_counts       = me_position_counts,
            .velocity_counts_per_s = me_velocity_counts_per_s,
            .is_ok                 = me_is_ok,
        },
    };
    s_state.hw = hwdev;

    /* 首读：种子多圈位置并验证总线可用 */
    uint32_t raw = 0;
    uint8_t status = 0;
    if (mt6835_hw_read_raw(s_state.hw, &raw, &status)) {
        publish(raw, status);
        ESP_LOGI(TAG, "MT6835 在线——首读 raw21=%u（种子，pos=0）", raw);
    } else {
        mt6835_hw_fail_t kind;
        uint8_t frame[MT6835_HW_FRAME_BYTES];
        mt6835_hw_last_failure(s_state.hw, &kind, frame);
        ESP_LOGW(TAG, "首读失败（%s）——采样任务将继续重试",
                 fail_str(kind));
        ESP_LOGW(TAG, "末帧 rx = %02x %02x %02x %02x %02x %02x",
                 frame[0], frame[1], frame[2],
                 frame[3], frame[4], frame[5]);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(me_task, "me_mt6835",
                                            ME_MT6835_TASK_STACK, NULL,
                                            ME_MT6835_TASK_PRIO, NULL,
                                            ME_MT6835_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建采样任务失败");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "采样任务已启动：1 kHz，优先级 %d，CPU%d，%u counts/rev",
             ME_MT6835_TASK_PRIO, ME_MT6835_TASK_CORE,
             (unsigned)MT6835_COUNTS_PER_REV);
    *handle_out = (magnetic_encoder_handle_t)&s_state;
    return ESP_OK;
}
