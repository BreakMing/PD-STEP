/*
 * cartpole 固件入口
 *
 * 启动次序：bsp_early_init（EN 拉高 = 电机不使能，任何外设动作之前）
 * → settings_init → bsp_init → as5600_init → mt6835_init →
 * tmc2209_init → cartproto_start → step_engine_init → cartfsm_start →
 * telemetry_start。测试代码不编入正式固件（见 test/ 独立测试工程）。
 */

#include "esp_log.h"
#include "driver/uart.h"
#include "bsp.h"
#include "settings.h"
#include "tmc2209.h"
#include "step_engine.h"
#include "as5600.h"
#include "mt6835.h"
#include "telemetry.h"
#include "cartproto.h"
#include "cartfsm.h"

void app_main(void)
{
    bsp_early_init();                    /* EN 高 = 电机不使能 */
    ESP_ERROR_CHECK(settings_init());
    /* bsp 任一外设初始化失败即 panic 重启：静默半启动表现为死链路，
     * 可见可恢复优于带病存活 */
    ESP_ERROR_CHECK(bsp_init());
    {
        ESP_ERROR_CHECK(as5600_init());
        ESP_ERROR_CHECK(mt6835_init());
        tmc2209_config_t tmc = {
            .uart_num = UART_NUM_2,   /* 端口分配：UART0=烧录/日志、UART1=协议、UART2=TMC */
            .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
            .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
            .baud = 115200,
            .slave_addr = 0,
            .r_sense = CONFIG_TMC2209_RSENSE,
        };
        ESP_ERROR_CHECK(tmc2209_init(&tmc));
        ESP_ERROR_CHECK(cartproto_start());
        ESP_ERROR_CHECK(step_engine_init(CONFIG_BSP_STEP_GPIO, CONFIG_BSP_DIR_GPIO));
        ESP_ERROR_CHECK(cartfsm_start());
        ESP_ERROR_CHECK(telemetry_start());
    }
    ESP_LOGI("app_main", "cartpole 固件已启动");
}
