/*
 * mt6835 — 驱动 + 1 kHz 采样任务实现
 *
 * SPI 角度读取事务（抄录见 docs/reviews/T4-brief.md，来源
 * SimpleFOC Arduino-FOC-drivers MT6835 驱动，与手册 §SSI 一致）：
 *   MODE3、MSB 先行；48 bit（6 字节）单次事务，CS 拉低期间完成：
 *     TX: [0]=OP_ANGLE(0b1010)<<4, [1]=ANGLE1 地址 0x003, [2..5]=0
 *     RX: [2]=ANGLE[20:13]  [3]=ANGLE[12:5]
 *         [4]=ANGLE[4:0]<<3 | STATUS[2:0]  [5]=CRC8
 *   raw21 = (rx[2]<<13) | (rx[3]<<5) | (rx[4]>>3)
 *   STATUS：bit0 过速、bit1 磁场弱、bit2 欠压（仅记录，不上报协议）。
 *   CRC8（poly 0x07、init 0x00）校验可选；首版不启用，上板首读
 *   验证帧格式正确后再评估。
 */

#include "mt6835.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"
#include "driver/spi_master.h"

#include "bsp.h"

static const char *TAG = "mt6835";

/* ---- 帧常量（T4-brief.md 抄录） ---- */
#define MT6835_OP_ANGLE     0x0Au       /* 0b1010 */
#define MT6835_REG_ANGLE1   0x03u
#define MT6835_FRAME_BYTES  6

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

static spi_device_handle_t s_dev = NULL;
static volatile bool s_ok = true;
static bool s_initialized = false;

/* ================================================================
 * SPI 读取
 * ================================================================ */

/* 读一次 21-bit 原始角。返回 false = 事务失败（raw 不变）。 */
static bool mt6835_read_raw(uint32_t *raw_out, uint8_t *status_out)
{
    uint8_t tx[MT6835_FRAME_BYTES] = {
        (uint8_t)(MT6835_OP_ANGLE << 4),
        MT6835_REG_ANGLE1,
        0, 0, 0, 0,
    };
    uint8_t rx[MT6835_FRAME_BYTES] = { 0 };

    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = MT6835_FRAME_BYTES * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    if (err != ESP_OK) {
        return false;
    }

    *raw_out = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5)
             | ((uint32_t)rx[4] >> 3);
    *status_out = rx[4] & 0x07;
    return true;
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

        if (mt6835_read_raw(&raw, &status)) {
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
                ESP_LOGE(TAG, "SPI 读连续失败 %d 次——编码器标记 NOT OK",
                         fail_streak);
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

esp_err_t mt6835_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    spi_host_device_t host = bsp_get_spi_host();
    if (host < 0) {
        ESP_LOGE(TAG, "SPI 总线未初始化（bsp_get_spi_host() < 0）");
        return ESP_ERR_INVALID_STATE;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = CONFIG_BSP_MT6835_SPI_HZ,
        .mode = 3,                          /* CPOL=1 CPHA=2（MODE3） */
        .spics_io_num = CONFIG_BSP_MT6835_CS_GPIO,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(host, &dev_cfg, &s_dev),
                        TAG, "spi_bus_add_device 失败");

    memset(&s_state, 0, sizeof(s_state));

    /* 首读：种子多圈位置并验证总线可用 */
    uint32_t raw = 0;
    uint8_t status = 0;
    if (mt6835_read_raw(&raw, &status)) {
        mt6835_publish(raw, status);
        ESP_LOGI(TAG, "MT6835 在线——首读 raw21=%u（种子，theta=%d µrad）",
                 raw, mt6835_theta_urad());
    } else {
        ESP_LOGW(TAG, "首读失败——采样任务将继续重试");
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
