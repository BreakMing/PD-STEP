/*
 * cartpole 固件入口
 *
 * 启动次序：bsp_early_init（EN 拉高 = 电机不使能，任何外设动作之前）
 * → settings_init → bsp_init → magnetic_encoder_as5600_init +
 * cartpos_init → magnetic_encoder_mt6835_init + poleang_init →
 * motor_tmc2209_init + cartaxis_init → 组装点订阅 $3/$4 中转 →
 * cartproto_start → cartfsm_start → telemetry_start。测试代码不编入
 * 正式固件（见 test/ 独立测试工程）。
 */

#include "esp_log.h"
#include "driver/uart.h"
#include "bsp.h"
#include "settings.h"
#include "motor_tmc2209.h"
#include "motor.h"
#include "cartaxis.h"
#include "magnetic_encoder_as5600.h"
#include "cartpos.h"
#include "magnetic_encoder_mt6835.h"
#include "poleang.h"
#include "telemetry.h"
#include "cartproto.h"
#include "cartfsm.h"

static motor_handle_t s_mot;   /* $3/$4 中转用（app_main 单次启动） */

static void on_driver_settings(int idx, float new_value)
{
    (void)new_value;
    if (idx == SET_RUN_MA || idx == SET_HOLD_MA) {
        esp_err_t err = motor_set_current(s_mot, settings_run_current_ma(),
                                          settings_hold_current_ma());
        if (err != ESP_OK) {
            ESP_LOGE("app_main", "电流写入失败（%d）——保持旧值", err);
        }
    }
}

void app_main(void)
{
    bsp_early_init();                    /* EN 高 = 电机不使能 */
    ESP_ERROR_CHECK(settings_init());
    /* bsp 任一外设初始化失败即 panic 重启：静默半启动表现为死链路，
     * 可见可恢复优于带病存活 */
    ESP_ERROR_CHECK(bsp_init());
    {
        magnetic_encoder_handle_t enc_cart;
        magnetic_encoder_as5600_config_t ecfg = {
            .bus          = bsp_get_i2c_bus(),
            .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
            .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
            .invert_dir   = magnetic_encoder_as5600_invert_dir_default(),
        };
        ESP_ERROR_CHECK(magnetic_encoder_as5600_init(&ecfg, &enc_cart));
        cartpos_init(enc_cart);
        magnetic_encoder_handle_t enc_pole;
        magnetic_encoder_mt6835_config_t pcfg = {
            .host          = bsp_get_spi_host(),
            .clock_speed_hz = CONFIG_BSP_MT6835_SPI_HZ,
            .spics_io_num  = CONFIG_BSP_MT6835_CS_GPIO,
        };
        ESP_ERROR_CHECK(magnetic_encoder_mt6835_init(&pcfg, &enc_pole));
        poleang_init(enc_pole);
        motor_handle_t mot;
        motor_tmc2209_config_t mcfg = {
            .tmc = {
                .uart_num = UART_NUM_2,   /* 端口分配：UART0=烧录/日志、UART1=协议、UART2=TMC */
                .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
                .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
                .en_gpio = CONFIG_BSP_EN_GPIO,
                .baud = 115200,
                .slave_addr = 0,
                .r_sense = CONFIG_TMC2209_RSENSE,
                .init_microstep = settings_microstep(),
                .init_run_ma = settings_run_current_ma(),
                .init_hold_ma = settings_hold_current_ma(),
            },
            .step_gpio = CONFIG_BSP_STEP_GPIO,
            .dir_gpio = CONFIG_BSP_DIR_GPIO,
        };
        ESP_ERROR_CHECK(motor_tmc2209_init(&mcfg, &mot));
        cartaxis_init(mot);
        /* $3/$4 中转：纯驱动器参数由组装点推送（返回值接住并告警，
         * 消灭旧订阅回调吞返回值的脱钩窗口） */
        s_mot = mot;
        if (!settings_subscribe(on_driver_settings)) {
            ESP_LOGW("app_main", "settings 订阅失败（槽满）——$3/$4 变更不会生效");
        }
        ESP_ERROR_CHECK(cartproto_start());
        ESP_ERROR_CHECK(cartfsm_start());
        ESP_ERROR_CHECK(telemetry_start());
    }
    ESP_LOGI("app_main", "cartpole 固件已启动");
}
