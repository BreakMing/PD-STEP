/*
 * tmc2209 硬件 bring-up 验证（官方 test/ 子目录约定，仅测试工程编入）
 *
 * [bringup] 真实寄存器读写核对；[spin] 会实际发送 STEP 脉冲转轴
 * （确认电机可自由转动后再跑）。
 *
 *   1. 读 IOIN → VERSION == 0x21
 *   2. 写 GCONF → 读回一致
 *   3. set_current（300mA/150mA）调用成功
 *   4. set_microstep(8) → CHOPCONF 读回 MRES=5、INTPOL=1
 *   5. 读 DRV_STATUS → 合理值（无 OT，CS 在界内）
 *   6. [spin] EN 低 → 2000 STEP 脉冲 @~500Hz → EN 高
 */

#include "tmc2209.h"
#include "bsp.h"

#include <stdio.h>

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

static const char *TAG = "tmc2209_bringup";

TEST_CASE("BRINGUP：IOIN 读 + VERSION=0x21", "[tmc2209][bringup]")
{
    uint32_t ioin = 0;
    TEST_ASSERT_TRUE_MESSAGE(tmc2209_read_reg(0x06, &ioin), "IOIN 读失败（无响应？）");
    uint8_t version = (uint8_t)(ioin >> 24);

    ESP_LOGI(TAG, "IOIN: 0x%08lx, VERSION=0x%02x", (unsigned long)ioin, version);
    /* TMC2209 数据手册（Rev. 1.06+）：IOIN bits 31:24 VERSION = 0x21；
     * 0x40 属 TMC2240 */
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x21, version, "VERSION 应为 0x21");
}

TEST_CASE("BRINGUP：GCONF 写读回一致", "[tmc2209][bringup]")
{
    uint32_t test_val = (1U << 6) | (1U << 7);  /* pdn_disable + mstep_reg_select */
    TEST_ASSERT_TRUE_MESSAGE(tmc2209_write_reg(0x00, test_val), "GCONF 写");

    uint32_t read_back = 0;
    TEST_ASSERT_TRUE_MESSAGE(tmc2209_read_reg(0x00, &read_back), "GCONF 读");

    /* 只核对关心的位（pdn_disable=bit6, mstep_reg_select=bit7,
     * en_spreadcycle=bit2, shaft=bit3） */
    uint32_t mask = (1U << 6) | (1U << 7) | (1U << 2) | (1U << 3);
    ESP_LOGI(TAG, "GCONF write=0x%08lx read=0x%08lx",
             (unsigned long)test_val, (unsigned long)read_back);
    TEST_ASSERT_EQUAL_UINT32(test_val & mask, read_back & mask);
}

TEST_CASE("BRINGUP：set_current(300, 150)", "[tmc2209][bringup]")
{
    /* IHOLD_IRUN 在 TMC2209 上只写不可读，无法直接读回验证；
     * 间接验证（电机激活时 DRV_STATUS.cs_actual）归入 DRV_STATUS 例 */
    esp_err_t err = tmc2209_set_current(300, 150);
    ESP_LOGI(TAG, "set_current(300, 150): %s", esp_err_to_name(err));
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

TEST_CASE("BRINGUP：set_microstep(8) 读回 MRES/INTPOL", "[tmc2209][bringup]")
{
    esp_err_t err = tmc2209_set_microstep(8);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    uint32_t chopconf = 0;
    TEST_ASSERT_TRUE(tmc2209_read_reg(0x6C, &chopconf));  /* CHOPCONF = 0x6C */

    int mres = (chopconf >> 24) & 0xF;
    int intpol = (chopconf >> 28) & 1;
    ESP_LOGI(TAG, "CHOPCONF: 0x%08lx, MRES=%d（8ustep→MRES=5）, INTPOL=%d",
             (unsigned long)chopconf, mres, intpol ? 1 : 0);
    TEST_ASSERT_EQUAL_INT(5, mres);
    TEST_ASSERT_EQUAL_INT(1, intpol);
}

TEST_CASE("BRINGUP：DRV_STATUS 合理性", "[tmc2209][bringup]")
{
    tmc2209_status_t st;
    TEST_ASSERT_TRUE(tmc2209_read_status(&st));

    ESP_LOGI(TAG, "DRV_STATUS: raw=0x%08lx stst=%d ola=%d olb=%d "
             "s2ga=%d s2gb=%d otpw=%d ot=%d cs_actual=%d",
             (unsigned long)st.raw, st.stst, st.ola, st.olb,
             st.s2ga, st.s2gb, st.otpw, st.ot, st.cs_actual);

    /* 冷启动不应有过温 */
    TEST_ASSERT_FALSE_MESSAGE(st.ot, "OT 不应置位");

    /* CS_ACTUAL 应在 0..31 */
    TEST_ASSERT_TRUE_MESSAGE(st.cs_actual >= 0 && st.cs_actual <= 31,
                             "cs_actual 超出 0..31");

    /* OLA/OLB 电机未接时可能置位——仅提示 */
    if (st.ola || st.olb) {
        ESP_LOGW(TAG, "检测到开路负载（电机未接？）：OLA=%d OLB=%d",
                 st.ola, st.olb);
    }
}

/* Spin：会转轴（~10 整步 = 8 细分下 1/8 圈）。确认电机可自由转动 */
TEST_CASE("BRINGUP spin：2000 STEP @~500Hz 实转", "[tmc2209][bringup][spin]")
{
    ESP_LOGI(TAG, "Spin test: EN 低 → 2000 steps @~500Hz → EN 高");

    /* 使能驱动 */
    tmc2209_enable(true);
    vTaskDelay(pdMS_TO_TICKS(10));

    int step_pin = CONFIG_BSP_STEP_GPIO;
    int dir_pin = CONFIG_BSP_DIR_GPIO;

    gpio_set_level(dir_pin, 0);

    /* ~500 Hz：2000 µs 周期 = 1 µs 高 + ~1999 µs 低 */
    for (int i = 0; i < 2000; i++) {
        gpio_set_level(step_pin, 1);
        esp_rom_delay_us(1);
        gpio_set_level(step_pin, 0);
        esp_rom_delay_us(1999);
    }

    tmc2209_enable(false);

    ESP_LOGI(TAG, "Spin test 完成（已发 2000 步）");
}
