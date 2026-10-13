/*
 * telemetry — 200 Hz STATE 流实现
 */

#include "telemetry.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "as5600.h"
#include "mt6835.h"
#include "cartproto.h"
#include "params.h"

static const char *TAG = "telemetry";

#define TELEMETRY_PERIOD_MS   5      /* 200 Hz */
#define TELEMETRY_TASK_PRIO   2
#define TELEMETRY_TASK_CORE   1
#define TELEMETRY_TASK_STACK  3072

static void telemetry_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    char buf[64];

    ESP_LOGI(TAG, "STATE 流任务已启动：200 Hz，优先级 %d，CPU%d",
             TELEMETRY_TASK_PRIO, TELEMETRY_TASK_CORE);

    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(TAG, "任务看门狗订阅失败——组帧任务不受监控");
    }

    for (;;) {
        /* 摆角标定作用于组帧点（POLE_ZERO_URAD/POLE_INV）：
         * mt6835 保持纯传感器输出原始角；速度符号随角度翻转。 */
        const fw_params_t *prm = fw_params();
        int32_t theta = mt6835_raw_to_urad(mt6835_last_raw(),
                                           (int32_t)prm->pole_zero_urad,
                                           prm->pole_inv != 0);
        int32_t vth = mt6835_vth_mrad_s();
        if (prm->pole_inv) {
            vth = -vth;
        }

        int64_t x_um = as5600_position_um();
        int32_t vx = (int32_t)lround(as5600_velocity_mm_s());

        int n = telemetry_format_state(buf, sizeof(buf),
                                       (int32_t)x_um, theta, vx, vth);
        if (n > 0) {
            buf[n] = '\n';
            cartproto_send_raw(buf, n + 1);
        } else {
            ESP_LOGE(TAG, "组帧失败（n=%d）——帧被丢弃", n);
        }

        if (wdt_ok) {
            esp_task_wdt_reset();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

esp_err_t telemetry_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(telemetry_task, "telemetry",
                                            TELEMETRY_TASK_STACK, NULL,
                                            TELEMETRY_TASK_PRIO, NULL,
                                            TELEMETRY_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建组帧任务失败");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
