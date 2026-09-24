/*
 * cartpole 固件入口（任务 1 最小骨架）
 *
 * 启动次序：bsp_early_init（EN 拉高 = 电机不使能，任何外设动作之前）
 * → settings_init → bsp_init → as5600_init → tmc2209_init →
 * cartproto_start → step_engine_init。
 * 模式机（cartfsm）与遥测（telemetry）在任务 7/5 接入。
 */

#include "esp_log.h"
#include "driver/uart.h"
#include "bsp.h"
#include "settings.h"
#include "tmc2209.h"
#include "step_engine.h"
#include "as5600.h"
#include "mt6835.h"
#include "cartproto.h"

void app_main(void)
{
    bsp_early_init();                    /* EN 高 = 电机不使能 */
    ESP_ERROR_CHECK(settings_init());
#ifdef CONFIG_SETTINGS_SELF_TEST
    settings_self_test_run();
#endif
    esp_err_t bsp_err = bsp_init();
    if (bsp_err == ESP_OK) {
        ESP_ERROR_CHECK(as5600_init());
        ESP_ERROR_CHECK(mt6835_init());
#ifdef CONFIG_MT6835_SELF_TEST
        mt6835_self_test_run();
#endif
        tmc2209_config_t tmc = {
            .uart_num = UART_NUM_2,
            .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
            .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
            .baud = 115200,
            .slave_addr = 0,
            .r_sense = CONFIG_TMC2209_RSENSE,
        };
        ESP_ERROR_CHECK(tmc2209_init(&tmc));
        ESP_ERROR_CHECK(cartproto_start());
        ESP_ERROR_CHECK(step_engine_init(CONFIG_BSP_STEP_GPIO, CONFIG_BSP_DIR_GPIO));
#ifdef CONFIG_STEP_ENGINE_SELF_TEST
        step_engine_self_test_run();
#endif
    }
    ESP_LOGI("app_main", "cartpole 固件已启动（协议骨架）");
}
