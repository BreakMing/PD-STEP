/*
 * cartpole 测试工程入口：与正式固件 app_main 相同的初始化次序，
 * 然后进入 Unity 交互菜单（Enter 列出用例，按名/序号/[标签]/'*' 选择）。
 *
 * 标签约定：
 *   [组件名]      常规自测（大多不驱动电机）
 *   [flowtest]    cartfsm 注入式流程自测（会开 EN）
 *   [manual]      人工引导段（手转轴/摆杆，只显式选择时运行）
 *   [bringup]     TMC2209 硬件寄存器核对（[spin] 会实际转轴）
 */

#include "esp_log.h"
#include "driver/uart.h"

#include "unity.h"

#include "bsp.h"
#include "settings.h"
#include "motor_tmc2209.h"
#include "magnetic_encoder_as5600.h"
#include "cartpos.h"
#include "magnetic_encoder_mt6835.h"
#include "poleang.h"
#include "telemetry.h"
#include "cartproto.h"
#include "cartfsm.h"

void app_main(void)
{
    /* 与正式固件相同的启动次序（EN 高 = 电机不使能） */
    bsp_early_init();
    ESP_ERROR_CHECK(settings_init());
    ESP_ERROR_CHECK(bsp_init());
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
    motor_tmc2209_config_t mot = {
        .tmc = {
            .uart_num = UART_NUM_2,
            .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
            .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
            .en_gpio = CONFIG_BSP_EN_GPIO,
            .baud = 115200,
            .slave_addr = 0,
            .r_sense = CONFIG_TMC2209_RSENSE,
        },
        .step_gpio = CONFIG_BSP_STEP_GPIO,
        .dir_gpio = CONFIG_BSP_DIR_GPIO,
    };
    ESP_ERROR_CHECK(motor_tmc2209_init(&mot));
    ESP_ERROR_CHECK(cartproto_start());
    ESP_ERROR_CHECK(cartfsm_start());
    ESP_ERROR_CHECK(telemetry_start());

    ESP_LOGI("test_main", "测试固件就绪：Enter 出菜单，'*' 跑全部；"
             "'[flowtest]' 会开 EN，'[manual]'/'[bringup]' 为人工/硬件段");
    unity_run_menu();   /* 交互菜单，不返回 */
}
