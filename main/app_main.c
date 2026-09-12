/*
 * ESP32-S3 Single-Axis G-code Motion Controller
 * T2: BSP - app_main entry point
 *
 * app_main calls bsp_early_init() as its very first line (to pull
 * TMC2209 EN high and disable the driver), then prints the startup
 * banner, then calls bsp_init() for full peripheral init.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

#include "bsp.h"

static const char *TAG = "app_main";

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
    ESP_LOGI(TAG, " T2 BSP - Pin & Peripheral Init");
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
        /* Continue anyway — placeholder task still runs for watchdog test */
    }

    /* Task watchdog is already enabled via sdkconfig.defaults
     * (CONFIG_ESP_TASK_WDT_INIT=y). The Idle task is subscribed
     * automatically on each core. */

    /* Create placeholder task at low priority */
    xTaskCreate(placeholder_task, "placeholder", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Startup complete. System running.");
}
