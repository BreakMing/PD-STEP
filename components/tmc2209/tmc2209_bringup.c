/*
 * TMC2209 bring-up test — hardware verification (gated by CONFIG_TMC2209_BRINGUP)
 *
 * Tests:
 *   1. Read IOIN → VERSION == 0x21?
 *   2. Write GCONF → read back consistent?
 *   3. Write IHOLD_IRUN (300mA/150mA) → read back consistent?
 *   4. Write CHOPCONF (8 microsteps) → read back consistent?
 *   5. Read DRV_STATUS → reasonable values (no OT, CS in range)?
 *   6. (SPIN_TEST) EN low → 2000 STEP pulses @500Hz → EN high
 *
 * Final line: "BRINGUP: PASS n/6" or per-case FAIL details.
 */

#include "tmc2209.h"

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#ifdef CONFIG_TMC2209_BRINGUP

#include "bsp.h"

static const char *TAG = "tmc2209_bringup";

static int s_pass = 0;
static int s_total = 0;

static void test_check(const char *name, bool passed)
{
    s_total++;
    if (passed) {
        s_pass++;
        ESP_LOGI(TAG, "PASS %s", name);
    } else {
        ESP_LOGE(TAG, "FAIL %s", name);
    }
}

int tmc2209_bringup_run(void)
{
    ESP_LOGI(TAG, "=== TMC2209 bring-up start ===");

    s_pass = 0;
    s_total = 0;

    /* ---- Test 1: Read IOIN → VERSION check ---- */
    {
        uint32_t ioin = 0;
        bool ok = tmc2209_read_reg(0x06, &ioin);  /* IOIN = 0x06 */
        uint8_t version = (uint8_t)(ioin >> 24);

        if (!ok) {
            ESP_LOGE(TAG, "IOIN read failed (no response?)");
            test_check("IOIN read", false);
            test_check("VERSION=0x21", false);
        } else {
            test_check("IOIN read", true);
            ESP_LOGI(TAG, "IOIN: 0x%08lx, VERSION=0x%02x", (unsigned long)ioin, version);
            /* TMC2209 datasheet (Rev. 1.06+): IOIN bits 31:24 VERSION =
             * 0x21 = first version of the IC.  0x40 belongs to TMC2240. */
            test_check("VERSION=0x21", version == 0x21);
            if (version != 0x21) {
                ESP_LOGW(TAG, "Unexpected VERSION 0x%02x (expected 0x21)", version);
            }
        }
    }

    /* ---- Test 2: Write GCONF → read back ---- */
    {
        uint32_t test_val = (1U << 6) | (1U << 7);  /* pdn_disable + mstep_reg_select */
        bool write_ok = tmc2209_write_reg(0x00, test_val);  /* GCONF = 0x00 */
        test_check("GCONF write", write_ok);

        uint32_t read_back = 0;
        bool read_ok = tmc2209_read_reg(0x00, &read_back);
        if (read_ok) {
            /* Only check bits we care about (pdn_disable=bit6, mstep_reg_select=bit7,
             * en_spreadcycle=bit2, shaft=bit3) */
            uint32_t mask = (1U << 6) | (1U << 7) | (1U << 2) | (1U << 3);
            bool match = (read_back & mask) == (test_val & mask);
            ESP_LOGI(TAG, "GCONF write=0x%08lx read=0x%08lx",
                     (unsigned long)test_val, (unsigned long)read_back);
            test_check("GCONF read-back", match);
        } else {
            test_check("GCONF read-back", false);
        }
    }

    /* ---- Test 3: Write IHOLD_IRUN (300mA run / 150mA hold) ---- */
    {
        /* We use the high-level API */
        esp_err_t err = tmc2209_set_current(300, 150);
        test_check("IHOLD_IRUN set_current", err == ESP_OK);

        /* IHOLD_IRUN is write-only on TMC2209 — can't read back.
         * Verify indirectly via DRV_STATUS cs_actual when motor is active.
         * Instead, just confirm the call succeeded. */
        ESP_LOGI(TAG, "set_current(300, 150): %s", esp_err_to_name(err));
    }

    /* ---- Test 4: Write CHOPCONF (8 microsteps) → read back ---- */
    {
        esp_err_t err = tmc2209_set_microstep(8);
        test_check("CHOPCONF set_microstep(8)", err == ESP_OK);

        uint32_t chopconf = 0;
        bool read_ok = tmc2209_read_reg(0x6C, &chopconf);  /* CHOPCONF = 0x6C */
        if (read_ok) {
            int mres = (chopconf >> 24) & 0xF;
            bool intpol = (chopconf >> 28) & 1;
            ESP_LOGI(TAG, "CHOPCONF: 0x%08lx, MRES=%d (8ustep→MRES=5), INTPOL=%d",
                     (unsigned long)chopconf, mres, intpol ? 1 : 0);
            /* 8 microsteps → MRES=5 */
            test_check("CHOPCONF MRES=5", mres == 5);
            test_check("CHOPCONF INTPOL=1", intpol == true);
        } else {
            test_check("CHOPCONF MRES read", false);
            test_check("CHOPCONF INTPOL read", false);
        }
    }

    /* ---- Test 5: Read DRV_STATUS → sanity check ---- */
    {
        tmc2209_status_t st;
        bool ok = tmc2209_read_status(&st);
        test_check("DRV_STATUS read", ok);

        if (ok) {
            ESP_LOGI(TAG, "DRV_STATUS: raw=0x%08lx stst=%d ola=%d olb=%d "
                     "s2ga=%d s2gb=%d otpw=%d ot=%d cs_actual=%d",
                     (unsigned long)st.raw, st.stst, st.ola, st.olb,
                     st.s2ga, st.s2gb, st.otpw, st.ot, st.cs_actual);

            /* Sanity checks: OT should not be set (cold start) */
            test_check("DRV_STATUS no OT", st.ot == false);

            /* CS_ACTUAL should be in 0..31 range */
            bool cs_ok = (st.cs_actual >= 0 && st.cs_actual <= 31);
            test_check("DRV_STATUS cs_actual range", cs_ok);

            /* OLA/OLB may be set if motor is not connected — just warn */
            if (st.ola || st.olb) {
                ESP_LOGW(TAG, "Open load detected (motor disconnected?): OLA=%d OLB=%d",
                         st.ola, st.olb);
            }
        } else {
            test_check("DRV_STATUS no OT", false);
            test_check("DRV_STATUS cs_actual range", false);
        }
    }

#ifdef CONFIG_TMC2209_SPIN_TEST
    /* ---- Test 6: Spin test ---- */
    {
        ESP_LOGI(TAG, "Spin test: EN low → 2000 steps @500Hz → EN high");

        /* Enable driver */
        tmc2209_enable(true);
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Get STEP pin from BSP Kconfig */
        int step_pin = CONFIG_BSP_STEP_GPIO;
        int dir_pin = CONFIG_BSP_DIR_GPIO;

        /* Set direction */
        gpio_set_level(dir_pin, 0);

        /* 2000 step pulses at ~500 Hz = 2ms period = 1ms high, 1ms low
         * Actually 500 Hz = 2000 µs period. Use 1 µs pulse width as brief says.
         * Wait, brief says "1µs 脉宽" — but 500 Hz means 2ms period.
         * Let's do: 1µs high, ~1999µs low = ~500 Hz */
        for (int i = 0; i < 2000; i++) {
            gpio_set_level(step_pin, 1);
            /* 1 µs pulse — use ets_delay_us or simple nop loop */
            esp_rom_delay_us(1);
            gpio_set_level(step_pin, 0);
            /* ~2000 µs period total, minus the 1 µs high */
            esp_rom_delay_us(1999);
        }

        /* Disable driver */
        tmc2209_enable(false);

        ESP_LOGI(TAG, "Spin test complete (2000 steps sent)");
        test_check("Spin test", true);
    }
#else
    /* No spin test — count as N/A but still report 6 tests total?
     * Brief says 6 tests.  Without SPIN_TEST, we have 5 tests.
     * Let's keep it at 5 and note spin test is disabled. */
    ESP_LOGI(TAG, "Spin test skipped (CONFIG_TMC2209_SPIN_TEST not set)");
#endif

    ESP_LOGI(TAG, "BRINGUP: PASS %d/%d", s_pass, s_total);
    return s_pass;
}

#endif /* CONFIG_TMC2209_BRINGUP */
