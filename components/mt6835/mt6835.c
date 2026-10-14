/*
 * mt6835 设备组件：1 kHz 采样任务 + 多圈位置/速度估计；接线经
 * mt6835_config_t 注入（组装点读 Kconfig 宏与 bsp 主机号）。
 *
 * 纯传感器职责：读原始 21-bit 角、±π 环绕换算（标定前的原始角）、
 * 16 样本窗口角速度估计、连续失败 ≥10 次的 is_ok。零位/符号标定
 * （POLE_ZERO_URAD / POLE_INV）由 poleang 域实现应用，
 * 本组件不读参数。
 *
 * SPI 帧格式与校验（OP_ANGLE/ANGLE1 突发、全零帧与 CRC 拒收、
 * 失败帧原文留存）在 mt6835_hw.c（驱动核心层，参数全由 config
 * 注入，不碰 settings/bsp/任务）。
 *
 * 采样任务：CPU1、优先级 3、1 kHz vTaskDelayUntil（结构照抄 as5600：
 * 失败计数、is_ok、原子发布）。SPI 事务阻塞在 CPU1，不干扰 CPU0
 * 的 STEP ISR。
 */

#include "mt6835.h"
#include "mt6835_hw.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

static const char *TAG = "mt6835";

/* ---- 时序/参数 ---- */
#define MT6835_SAMPLE_PERIOD_MS  1      /* 1 kHz */
#define MT6835_FAIL_LIMIT        10     /* 连续失败 → not OK */
#define MT6835_TASK_PRIO         3
#define MT6835_TASK_CORE         1
#define MT6835_TASK_STACK        3072

/* ---- 速度：16 ms 窗口（17 个 1 ms 位置样本） ---- */
#define MT6835_SPEED_WINDOW     16
#define MT6835_HIST_LEN         (MT6835_SPEED_WINDOW + 1)

/* ---- 发布状态（s_lock 保护） ---- */
typedef struct {
    uint32_t raw21;                        /* 最新原始角 */
    int32_t  theta_urad;                   /* 标定前原始角 */
    int32_t  pos_counts;                   /* 多圈连续位置 */
    int32_t  hist[MT6835_HIST_LEN];
    int      hist_head;
    int      hist_fill;
    int32_t  vth_mrad_s;
    uint8_t  status;                       /* 最新 STATUS[2:0] */
} mt6835_state_t;

static portMUX_TYPE   s_lock = portMUX_INITIALIZER_UNLOCKED;
static mt6835_state_t s_state;

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

static void mt6835_publish(uint32_t raw21, uint8_t status)
{
    portENTER_CRITICAL(&s_lock);

    int32_t diff = mt6835_unwrap_diff(raw21, s_state.raw21);
    s_state.raw21 = raw21;
    s_state.status = status;
    s_state.pos_counts += diff;
    s_state.theta_urad = mt6835_raw_to_urad(raw21, 0, false);

    s_state.hist_head = (s_state.hist_head + 1) % MT6835_HIST_LEN;
    s_state.hist[s_state.hist_head] = s_state.pos_counts;
    if (s_state.hist_fill < MT6835_HIST_LEN) {
        s_state.hist_fill++;
    }

    if (s_state.hist_fill == MT6835_HIST_LEN) {
        int oldest = (s_state.hist_head + 1) % MT6835_HIST_LEN;
        int32_t delta = s_state.pos_counts - s_state.hist[oldest];
        s_state.vth_mrad_s = mt6835_counts_to_mrad_s(delta,
                                                     MT6835_SPEED_WINDOW);
    } else {
        s_state.vth_mrad_s = 0;
    }

    portEXIT_CRITICAL(&s_lock);
}

/* ================================================================
 * 采样任务（1 kHz，CPU1）
 * ================================================================ */

static void mt6835_task(void *arg)
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

        if (mt6835_hw_read_raw(&raw, &status)) {
            if (fail_streak >= MT6835_FAIL_LIMIT) {
                ESP_LOGI(TAG, "SPI 在连续 %d 次失败后恢复", fail_streak);
            }
            fail_streak = 0;
            s_ok = true;

            if (status != last_status && status != 0) {
                ESP_LOGW(TAG, "STATUS 变化：0x%x（bit0 过速 bit1 磁弱 bit2 欠压）",
                         status);
            }
            last_status = status;

            mt6835_publish(raw, status);
        } else {
            fail_streak++;
            if (fail_streak == MT6835_FAIL_LIMIT) {
                s_ok = false;
                mt6835_hw_fail_t kind;
                uint8_t frame[MT6835_HW_FRAME_BYTES];
                mt6835_hw_last_failure(&kind, frame);
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

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MT6835_SAMPLE_PERIOD_MS));
    }
}

/* ================================================================
 * 生命周期
 * ================================================================ */

esp_err_t mt6835_init(const mt6835_config_t *cfg)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (cfg == NULL || cfg->host < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 帧层（驱动核心）：时钟/CS 由 config 注入 */
    mt6835_hw_config_t hw = {
        .host          = cfg->host,
        .clock_speed_hz = cfg->clock_speed_hz,
        .spics_io_num  = cfg->spics_io_num,
    };
    ESP_RETURN_ON_ERROR(mt6835_hw_init(&hw), TAG, "mt6835_hw_init 失败");

    memset(&s_state, 0, sizeof(s_state));

    /* 首读：种子多圈位置并验证总线可用 */
    uint32_t raw = 0;
    uint8_t status = 0;
    if (mt6835_hw_read_raw(&raw, &status)) {
        mt6835_publish(raw, status);
        ESP_LOGI(TAG, "MT6835 在线——首读 raw21=%u（种子，theta=%d µrad）",
                 raw, mt6835_theta_urad());
    } else {
        mt6835_hw_fail_t kind;
        uint8_t frame[MT6835_HW_FRAME_BYTES];
        mt6835_hw_last_failure(&kind, frame);
        ESP_LOGW(TAG, "首读失败（%s）——采样任务将继续重试",
                 fail_str(kind));
        ESP_LOGW(TAG, "末帧 rx = %02x %02x %02x %02x %02x %02x",
                 frame[0], frame[1], frame[2],
                 frame[3], frame[4], frame[5]);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(mt6835_task, "mt6835",
                                            MT6835_TASK_STACK, NULL,
                                            MT6835_TASK_PRIO, NULL,
                                            MT6835_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建采样任务失败");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "采样任务已启动：1 kHz，优先级 %d，CPU%d",
             MT6835_TASK_PRIO, MT6835_TASK_CORE);
    return ESP_OK;
}

/* ================================================================
 * 遥测读取
 * ================================================================ */

int32_t mt6835_theta_urad(void)
{
    int32_t v;
    portENTER_CRITICAL(&s_lock);
    v = s_state.theta_urad;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

int32_t mt6835_vth_mrad_s(void)
{
    int32_t v;
    portENTER_CRITICAL(&s_lock);
    v = s_state.vth_mrad_s;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

bool mt6835_is_ok(void)
{
    return s_ok;
}

uint32_t mt6835_last_raw(void)
{
    uint32_t r;
    portENTER_CRITICAL(&s_lock);
    r = s_state.raw21;
    portEXIT_CRITICAL(&s_lock);
    return r;
}

int32_t mt6835_position_counts(void)
{
    int32_t p;
    portENTER_CRITICAL(&s_lock);
    p = s_state.pos_counts;
    portEXIT_CRITICAL(&s_lock);
    return p;
}
