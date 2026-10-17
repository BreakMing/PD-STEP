/*
 * ch224q 上板用例（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 前两个用例即 hardware.md 检查清单「io4 总线电压回读 ≈20V」与
 * 「I2C 改档后 ADC 回读跟随」的载体；地址 0x22/0x23 变体在首个
 * 用例暴露（0x22 不在位时改 CONFIG_CH224Q_I2C_ADDR 为 0x23 重跑）。
 */

#include "ch224q.h"
#include "bsp.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ch224q_test";

static ch224q_handle_t dev(void)
{
    static ch224q_handle_t h = NULL;
    if (h == NULL) {
        ch224q_config_t cfg = {
            .bus      = bsp_get_i2c_bus(),
            .dev_addr = CONFIG_CH224Q_I2C_ADDR,
        };
        TEST_ASSERT_EQUAL_MESSAGE(
            ESP_OK, ch224q_init(&cfg, &h),
            "CH224Q 挂载失败——若 0x22 不在位，改 Kconfig 地址 0x23 重试");
    }
    return h;
}

/* 在位 + PD 握手成功（CF1 电阻档 20V 已由上电路径请求） */
TEST_CASE("在位与 PD 握手状态", "[ch224q]")
{
    ch224q_status_t st;
    TEST_ASSERT_EQUAL(ESP_OK, ch224q_get_status(dev(), &st));
    ESP_LOGI(TAG, "PD=%d QC3=%d QC2=%d BC12=%d EPR=%d/%d AVS=%d",
             st.pd, st.qc3, st.qc2, st.bc12,
             st.epr_active, st.epr_present, st.avs_present);
    TEST_ASSERT_TRUE_MESSAGE(st.pd, "PD 未握手成功（适配器能力或接线问题）");
}

/* 档最大电流（适配器 20V 档供电能力——加速度实验前评估项） */
TEST_CASE("当前档最大电流读取", "[ch224q]")
{
    uint16_t ma = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ch224q_get_current_ma(dev(), &ma));
    ESP_LOGI(TAG, "当前档最大电流 %u mA", ma);
    TEST_ASSERT_TRUE_MESSAGE(ma >= 1000, "档电流 <1A——档位或读数异常");
}

/* 设档 → vbus ADC 回读闭环（改档后协商建立需数秒） */
TEST_CASE("设 20V → vbus 回读 ≈20V（闭环）", "[ch224q][manual]")
{
    TEST_ASSERT_EQUAL(ESP_OK, ch224q_set_voltage(dev(), CH224Q_VSEL_20V));
    vTaskDelay(pdMS_TO_TICKS(3000));   /* 等 PD 协商重建 */

    float v = bsp_vbus_read_volt();
    ESP_LOGI(TAG, "vbus 回读 %.2f V（期望 20V ±分压与校准误差）", (double)v);
    TEST_ASSERT_TRUE_MESSAGE(v > 19.0f && v < 21.5f,
                             "vbus 回读偏离 20V——分压/校准/协商核查");
}
