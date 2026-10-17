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
#include "motor.h"
#include "cartaxis.h"
#include "magnetic_encoder_mt6835.h"
#include "cartpos.h"
#include "poleang.h"
#include "telemetry.h"
#include "cartproto.h"
#include "cartfsm.h"

static motor_handle_t s_mot;   /* 运行/保持电流中转用（app_main 单次启动） */

/* 编码器实例（init 不幂等，每台位一次）——测试组件经这对
 * accessor 共享 app_main 组装的句柄 */
static magnetic_encoder_handle_t s_enc_cart;
static magnetic_encoder_handle_t s_enc_pole;

magnetic_encoder_handle_t test_me_cart_enc(void)
{
    return s_enc_cart;
}

magnetic_encoder_handle_t test_me_pole_enc(void)
{
    return s_enc_pole;
}

static void on_driver_settings(int idx, float new_value)
{
    (void)new_value;
    if (idx == SET_RUN_MA || idx == SET_HOLD_MA) {
        esp_err_t err = motor_set_current(s_mot, settings_run_current_ma(),
                                          settings_hold_current_ma());
        if (err != ESP_OK) {
            ESP_LOGE("test_main", "电流写入失败（%d）——保持旧值", err);
        }
    }
}

void app_main(void)
{
    /* 与正式固件相同的启动次序（EN 高 = 电机不使能）；
     * 双 MT6835 台位（rev2）：小车位板载（SPI2）、摆角位外接（SPI3） */
    bsp_early_init();
    ESP_ERROR_CHECK(settings_init());
    ESP_ERROR_CHECK(bsp_init());
    magnetic_encoder_mt6835_config_t ccfg = {
        .host          = bsp_get_cart_enc_spi_host(),
        .clock_speed_hz = CONFIG_BSP_CART_ENC_SPI_HZ,
        .spics_io_num  = CONFIG_BSP_CART_ENC_CS_GPIO,
        .task_name     = "me_cart",
    };
    ESP_ERROR_CHECK(magnetic_encoder_mt6835_init(&ccfg, &s_enc_cart));
    cartpos_init(s_enc_cart);
    magnetic_encoder_mt6835_config_t pcfg = {
        .host          = bsp_get_pole_enc_spi_host(),
        .clock_speed_hz = CONFIG_BSP_POLE_ENC_SPI_HZ,
        .spics_io_num  = CONFIG_BSP_POLE_ENC_CS_GPIO,
        .task_name     = "me_pole",
    };
    ESP_ERROR_CHECK(magnetic_encoder_mt6835_init(&pcfg, &s_enc_pole));
    poleang_init(s_enc_pole);
    motor_handle_t mot;
    motor_tmc2209_config_t mcfg = {
        .tmc = {
            .uart_num = UART_NUM_2,
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
    /* 运行/保持电流中转：纯驱动器参数由组装点推送（返回值接住并告警，
     * 消灭旧订阅回调吞返回值的脱钩窗口） */
    s_mot = mot;
    if (!settings_subscribe(on_driver_settings)) {
        ESP_LOGW("test_main", "settings 订阅失败（槽满）——运行/保持电流变更不会生效");
    }
    ESP_ERROR_CHECK(cartproto_start());
    ESP_ERROR_CHECK(cartfsm_start());
    ESP_ERROR_CHECK(telemetry_start());

    ESP_LOGI("test_main", "测试固件就绪：Enter 出菜单，'*' 跑全部；"
             "'[flowtest]' 会开 EN，'[manual]'/'[bringup]' 为人工/硬件段");
    unity_run_menu();   /* 交互菜单，不返回 */
}
