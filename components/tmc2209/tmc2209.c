/*
 * tmc2209 设备组件：寄存器配置策略 + $0/$3/$4 订阅；EN 引脚经
 * config 的 en_gpio 注入（引脚方向由 bsp_init 配置，本组件只驱动电平）。
 *
 * UART 帧格式、回声处理与通用寄存器读写原语在 tmc2209_hw.c
 * （驱动核心层：参数全由 config 注入，不碰 settings/bsp/任务）。
 * 本文件持有：寄存器地址与位域语义、上电默认配置策略（GCONF/
 * CHOPCONF/IHOLD_IRUN/TPOWERDOWN/TPWMTHRS/PWMCONF）、电流与细分
 * 功能层（含 VSENSE 自动分档）、EN 使能以及 $0/$3/$4 的
 * settings 订阅。
 */

#include "tmc2209.h"
#include "tmc2209_hw.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "settings.h"

static const char *TAG = "tmc2209";

/* ================================================================
 * Constants
 * ================================================================ */

/* Register addresses (TMC2209) */
#define REG_GCONF           0x00
#define REG_GSTAT           0x01
#define REG_IOIN            0x06
#define REG_IHOLD_IRUN      0x10
#define REG_TPOWERDOWN      0x11
#define REG_TPWMTHRS        0x13
#define REG_VACTUAL         0x22
#define REG_MSCNT           0x6A
#define REG_CHOPCONF        0x6C
#define REG_DRV_STATUS      0x6F
#define REG_PWMCONF         0x70

/* GCONF bits */
#define GCONF_I_SCALE_ANALOG   (1U << 0)
#define GCONF_INTERNAL_RSENSE  (1U << 1)
#define GCONF_EN_SPREADCYCLE   (1U << 2)
#define GCONF_SHAFT            (1U << 3)
#define GCONF_INDEX_OTPW       (1U << 4)
#define GCONF_INDEX_STEP       (1U << 5)
#define GCONF_PDN_DISABLE      (1U << 6)
#define GCONF_MSTEP_REG_SELECT (1U << 7)
#define GCONF_MULTISTEP_FILT   (1U << 8)
#define GCONF_TEST_MODE        (1U << 9)

/* CHOPCONF bit positions */
#define CHOPCONF_TOFF_SHIFT    0
#define CHOPCONF_TOFF_MASK     (0xFU << 0)
#define CHOPCONF_HSTRT_SHIFT   4
#define CHOPCONF_HSTRT_MASK    (0x7U << 4)
#define CHOPCONF_HEND_SHIFT    7
#define CHOPCONF_HEND_MASK     (0xFU << 7)
#define CHOPCONF_TBL_SHIFT     15
#define CHOPCONF_TBL_MASK      (0x3U << 15)
#define CHOPCONF_VSENSE_SHIFT  17
#define CHOPCONF_VSENSE_MASK   (1U << 17)
#define CHOPCONF_MRES_SHIFT    24
#define CHOPCONF_MRES_MASK     (0xFU << 24)
#define CHOPCONF_INTPOL_SHIFT  28
#define CHOPCONF_INTPOL_MASK   (1U << 28)
#define CHOPCONF_DEDGE_SHIFT   29
#define CHOPCONF_DEDGE_MASK    (1U << 29)
#define CHOPCONF_DISS2G_SHIFT  30
#define CHOPCONF_DISS2G_MASK   (1U << 30)
#define CHOPCONF_DISS2VS_SHIFT 31
#define CHOPCONF_DISS2VS_MASK   (1U << 31)

/* IHOLD_IRUN bit positions */
#define IHOLD_IRUN_IHOLD_SHIFT      0
#define IHOLD_IRUN_IHOLD_MASK       (0x1FU << 0)
#define IHOLD_IRUN_IRUN_SHIFT       8
#define IHOLD_IRUN_IRUN_MASK        (0x1FU << 8)
#define IHOLD_IRUN_IHOLDDELAY_SHIFT 16
#define IHOLD_IRUN_IHOLDDELAY_MASK  (0xFU << 16)

/* DRV_STATUS bit positions */
#define DRV_STATUS_OTPW_SHIFT     0
#define DRV_STATUS_OT_SHIFT       1
#define DRV_STATUS_S2GA_SHIFT     2
#define DRV_STATUS_S2GB_SHIFT     3
#define DRV_STATUS_S2VSA_SHIFT    4
#define DRV_STATUS_S2VSB_SHIFT    5
#define DRV_STATUS_OLA_SHIFT      6
#define DRV_STATUS_OLB_SHIFT      7
#define DRV_STATUS_CS_ACTUAL_SHIFT 16
#define DRV_STATUS_CS_ACTUAL_MASK  (0x1FU << 16)
#define DRV_STATUS_STEALTH_SHIFT  30
#define DRV_STATUS_STST_SHIFT     31

/* PWMCONF defaults (StealthChop2) */
#define PWMCONF_PWM_OFS_SHIFT     0
#define PWMCONF_PWM_GRAD_SHIFT    8
#define PWMCONF_PWM_FREQ_SHIFT    16
#define PWMCONF_PWM_AUTOSCALE_SHIFT 18
#define PWMCONF_PWM_AUTOGRAD_SHIFT  19
#define PWMCONF_FREEWHEEL_SHIFT   20
#define PWMCONF_PWM_REG_SHIFT     24
#define PWMCONF_PWM_LIM_SHIFT     28

/* Recommended chopper parameters (StealthChop, datasheet) */
#define CHOPCONF_TOFF_DEFAULT   5   /* Off time: 5 clocks */
#define CHOPCONF_TBL_DEFAULT    2   /* Blanking time: 24 clocks */
#define CHOPCONF_HSTRT_DEFAULT  4   /* Hysteresis start */
#define CHOPCONF_HEND_DEFAULT   0   /* Hysteresis end */
#define IHOLDDELAY_DEFAULT      6   /* Hold delay */

/* Timing */
#define ENABLE_SETTLE_MS          2   /* ≥1ms settle after EN low */

/* ================================================================
 * Internal state
 * ================================================================ */

static tmc2209_config_t s_cfg;      /* 只使用其中的 r_sense 与 en_gpio */
static bool s_initialized = false;
static bool s_vsense = false;   /* Current VSENSE setting (cached) */

/* ================================================================
 * Pure functions (public)
 * ================================================================ */

int tmc2209_ma_to_cs(int ma, float r_sense)
{
    if (ma <= 0) return 0;

    float i_rms = (float)ma / 1000.0f;  /* mA → A */

    /* Try VSENSE=0 first (V_fs = 0.325V) */
    float v_fs = 0.325f;
    float cs_float = 32.0f * sqrtf(2.0f) * i_rms * (r_sense + 0.02f) / v_fs - 1.0f;

    /* If CS < 16, switch to VSENSE=1 (higher sensitivity, V_fs = 0.180V)
     * for better resolution at low currents */
    if (cs_float < 16.0f) {
        v_fs = 0.180f;
        cs_float = 32.0f * sqrtf(2.0f) * i_rms * (r_sense + 0.02f) / v_fs - 1.0f;
    }

    int cs = (int)roundf(cs_float);
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    return cs;
}

int tmc2209_microstep_to_mres(int ms)
{
    switch (ms) {
    case 256: return 0;
    case 128: return 1;
    case 64:  return 2;
    case 32:  return 3;
    case 16:  return 4;
    case 8:   return 5;
    case 4:   return 6;
    case 2:   return 7;
    case 1:   return 8;
    default:  return -1;
    }
}

/* ================================================================
 * Internal helpers
 * ================================================================ */

/* Determine optimal VSENSE for a given current level.
 * Returns true if VSENSE=1 (0.180V, high sensitivity) is better. */
static bool calc_vsense(int ma, float r_sense)
{
    if (ma <= 0) return false;
    float i_rms = (float)ma / 1000.0f;
    float cs_v0 = 32.0f * sqrtf(2.0f) * i_rms * (r_sense + 0.02f) / 0.325f - 1.0f;
    return (cs_v0 < 16.0f);
}

/* ================================================================
 * Functional layer
 * ================================================================ */

esp_err_t tmc2209_set_current(int run_ma, int hold_ma)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (run_ma <= 0 || hold_ma < 0 || hold_ma > run_ma) {
        return ESP_ERR_INVALID_ARG;
    }

    int cs_run = tmc2209_ma_to_cs(run_ma, s_cfg.r_sense);
    int cs_hold = tmc2209_ma_to_cs(hold_ma, s_cfg.r_sense);
    bool new_vsense = calc_vsense(run_ma, s_cfg.r_sense);

    /* If VSENSE changed, update CHOPCONF first */
    if (new_vsense != s_vsense) {
        uint32_t chopconf;
        if (!tmc2209_read_reg(REG_CHOPCONF, &chopconf)) {
            return ESP_FAIL;
        }
        if (new_vsense) {
            chopconf |= CHOPCONF_VSENSE_MASK;
        } else {
            chopconf &= ~CHOPCONF_VSENSE_MASK;
        }
        if (!tmc2209_write_reg(REG_CHOPCONF, chopconf)) {
            return ESP_FAIL;
        }
        s_vsense = new_vsense;
    }

    /* Write IHOLD_IRUN */
    uint32_t val = ((uint32_t)cs_hold << IHOLD_IRUN_IHOLD_SHIFT) |
                   ((uint32_t)cs_run  << IHOLD_IRUN_IRUN_SHIFT)  |
                   ((uint32_t)IHOLDDELAY_DEFAULT << IHOLD_IRUN_IHOLDDELAY_SHIFT);

    if (!tmc2209_write_reg(REG_IHOLD_IRUN, val)) {
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "set_current: run=%dmA(CS=%d) hold=%dmA(CS=%d) VSENSE=%d",
             run_ma, cs_run, hold_ma, cs_hold, new_vsense ? 1 : 0);
    return ESP_OK;
}

esp_err_t tmc2209_set_microstep(int ms)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    int mres = tmc2209_microstep_to_mres(ms);
    if (mres < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t chopconf;
    if (!tmc2209_read_reg(REG_CHOPCONF, &chopconf)) {
        return ESP_FAIL;
    }

    chopconf &= ~CHOPCONF_MRES_MASK;
    chopconf |= ((uint32_t)mres << CHOPCONF_MRES_SHIFT);

    /* Always enable interpolation (256x microstep smoothing) */
    chopconf |= CHOPCONF_INTPOL_MASK;

    if (!tmc2209_write_reg(REG_CHOPCONF, chopconf)) {
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "set_microstep: %d (MRES=%d, INTPOL=1)", ms, mres);
    return ESP_OK;
}

esp_err_t tmc2209_enable(bool en)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    /* ENN 低有效；引脚方向由 bsp_init 配置，这里只驱动电平 */
    gpio_set_level(s_cfg.en_gpio, en ? 0 : 1);

    if (en) {
        /* Datasheet: wait for driver to wake up after EN low */
        vTaskDelay(pdMS_TO_TICKS(ENABLE_SETTLE_MS));
    }

    return ESP_OK;
}

/* ================================================================
 * Status layer
 * ================================================================ */

bool tmc2209_read_status(tmc2209_status_t *st)
{
    if (!s_initialized || st == NULL) return false;

    uint32_t val;
    if (!tmc2209_read_reg(REG_DRV_STATUS, &val)) {
        return false;
    }

    st->raw = val;
    st->stst = (val >> DRV_STATUS_STST_SHIFT) & 0x1;
    st->ola  = (val >> DRV_STATUS_OLA_SHIFT) & 0x1;
    st->olb  = (val >> DRV_STATUS_OLB_SHIFT) & 0x1;
    st->s2ga = (val >> DRV_STATUS_S2GA_SHIFT) & 0x1;
    st->s2gb = (val >> DRV_STATUS_S2GB_SHIFT) & 0x1;
    st->otpw = (val >> DRV_STATUS_OTPW_SHIFT) & 0x1;
    st->ot   = (val >> DRV_STATUS_OT_SHIFT) & 0x1;
    st->cs_actual = (val & DRV_STATUS_CS_ACTUAL_MASK) >> DRV_STATUS_CS_ACTUAL_SHIFT;

    return true;
}

/* ================================================================
 * Settings change callback
 * ================================================================ */

static void settings_change_cb(int idx, float new_value)
{
    if (!s_initialized) return;

    switch (idx) {
    case SET_MICROSTEP:
        tmc2209_set_microstep((int)new_value);
        break;
    case SET_RUN_MA:
    case SET_HOLD_MA:
        tmc2209_set_current(settings_run_current_ma(),
                            settings_hold_current_ma());
        break;
    default:
        break;
    }
}

/* ================================================================
 * Init
 * ================================================================ */

/* 上电默认寄存器配置。关键寄存器（GCONF/CHOPCONF/IHOLD_IRUN）任一
 * 推送失败返回 ESP_FAIL（驱动将停留在出厂配置，电流/细分失控）；
 * 次要寄存器失败仅告警。 */
static esp_err_t apply_default_registers(void)
{
    /* GCONF: pdn_disable=1 (UART mode), mstep_reg_select=1,
     * en_spreadcycle=1（固定 SpreadCycle：全速域统一 chopper，
     * TPWMTHRS 不参与模式切换）, shaft=0 */
    uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT
                   | GCONF_EN_SPREADCYCLE;
    if (!tmc2209_write_reg(REG_GCONF, gconf)) {
        ESP_LOGE(TAG, "init: GCONF write failed (check wiring?)");
        return ESP_FAIL;
    }

    /* CHOPCONF: TOFF, TBL, HSTRT, HEND per datasheet StealthChop recommendation;
     * MRES default from settings; INTPOL=1 */
    int mres = tmc2209_microstep_to_mres(settings_microstep());
    if (mres < 0) mres = 4; /* default 16 microsteps */

    s_vsense = calc_vsense(settings_run_current_ma(), s_cfg.r_sense);

    uint32_t chopconf =
        ((uint32_t)CHOPCONF_TOFF_DEFAULT  << CHOPCONF_TOFF_SHIFT)  |
        ((uint32_t)CHOPCONF_HSTRT_DEFAULT << CHOPCONF_HSTRT_SHIFT) |
        ((uint32_t)CHOPCONF_HEND_DEFAULT  << CHOPCONF_HEND_SHIFT)  |
        ((uint32_t)CHOPCONF_TBL_DEFAULT   << CHOPCONF_TBL_SHIFT)   |
        (s_vsense ? CHOPCONF_VSENSE_MASK : 0)                     |
        ((uint32_t)mres << CHOPCONF_MRES_SHIFT)                    |
        CHOPCONF_INTPOL_MASK;

    if (!tmc2209_write_reg(REG_CHOPCONF, chopconf)) {
        ESP_LOGE(TAG, "init: CHOPCONF write failed (check wiring?)");
        return ESP_FAIL;
    }

    /* IHOLD_IRUN: current from settings */
    int cs_run = tmc2209_ma_to_cs(settings_run_current_ma(), s_cfg.r_sense);
    int cs_hold = tmc2209_ma_to_cs(settings_hold_current_ma(), s_cfg.r_sense);
    uint32_t ihold_irun =
        ((uint32_t)cs_hold << IHOLD_IRUN_IHOLD_SHIFT) |
        ((uint32_t)cs_run  << IHOLD_IRUN_IRUN_SHIFT)  |
        ((uint32_t)IHOLDDELAY_DEFAULT << IHOLD_IRUN_IHOLDDELAY_SHIFT);

    if (!tmc2209_write_reg(REG_IHOLD_IRUN, ihold_irun)) {
        ESP_LOGE(TAG, "init: IHOLD_IRUN write failed (check wiring?)");
        return ESP_FAIL;
    }

    /* TPOWERDOWN: default delay（静止休眠定时，失败仅影响功耗） */
    if (!tmc2209_write_reg(REG_TPOWERDOWN, 20)) {
        ESP_LOGW(TAG, "init: TPOWERDOWN write failed");
    }

    /* TPWMTHRS: 固定 SpreadCycle，此值不参与模式切换
     *（保留写 0 = 寄存器确定状态） */
    if (!tmc2209_write_reg(REG_TPWMTHRS, 0)) {
        ESP_LOGW(TAG, "init: TPWMTHRS write failed");
    }

    /* PWMCONF: StealthChop2 default parameters
     * pwm_autoscale=1, pwm_autograd=1, freewheel=0, pwm_freq=1 */
    uint32_t pwmconf =
        ((uint32_t)34 << PWMCONF_PWM_OFS_SHIFT)    |  /* pwm_ofs */
        ((uint32_t)80 << PWMCONF_PWM_GRAD_SHIFT)   |  /* pwm_grad */
        ((uint32_t)1  << PWMCONF_PWM_FREQ_SHIFT)   |  /* pwm_freq */
        ((uint32_t)1  << PWMCONF_PWM_AUTOSCALE_SHIFT) |  /* pwm_autoscale */
        ((uint32_t)1  << PWMCONF_PWM_AUTOGRAD_SHIFT)  |  /* pwm_autograd */
        ((uint32_t)4  << PWMCONF_PWM_REG_SHIFT)    |  /* pwm_reg */
        ((uint32_t)12 << PWMCONF_PWM_LIM_SHIFT);      /* pwm_lim */
    if (!tmc2209_write_reg(REG_PWMCONF, pwmconf)) {
        ESP_LOGW(TAG, "init: PWMCONF write failed");
    }

    ESP_LOGD(TAG, "apply_default_registers: run=%dmA(CS=%d) hold=%dmA(CS=%d)",
             settings_run_current_ma(), cs_run,
             settings_hold_current_ma(), cs_hold);
    return ESP_OK;
}

esp_err_t tmc2209_init(const tmc2209_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_initialized) return ESP_OK;

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* UART 驱动与寄存器读写原语（驱动核心层） */
    tmc2209_hw_config_t hw = {
        .uart_num   = cfg->uart_num,
        .tx_pin     = cfg->tx_pin,
        .rx_pin     = cfg->rx_pin,
        .baud       = cfg->baud,
        .slave_addr = cfg->slave_addr,
    };
    esp_err_t err = tmc2209_hw_init(&hw);
    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(apply_default_registers(),
                        TAG, "default register config failed");

    s_initialized = true;

    ESP_LOGI(TAG, "initialized: microstep=%d, run=%dmA, hold=%dmA, VSENSE=%d",
             settings_microstep(),
             settings_run_current_ma(),
             settings_hold_current_ma(),
             s_vsense ? 1 : 0);

    /* Subscribe to settings changes ($0, $3, $4) */
    if (!settings_subscribe(settings_change_cb)) {
        ESP_LOGW(TAG, "settings_subscribe failed (slot full)");
    }

    return ESP_OK;
}
