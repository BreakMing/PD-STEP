/*
 * TMC2209 — UART-controlled stepper driver implementation
 *
 * UART frame format (verified against TMCStepper + datasheet):
 *   Write:  [SYNC=0x05][slave_addr][reg_addr | 0x80][data3..0][CRC]  (8 bytes)
 *   Read:   [SYNC=0x05][slave_addr][reg_addr | 0x00][CRC]            (4 bytes sent)
 *   Reply:  [SYNC=0x05][slave_addr][reg_addr][data3..0][CRC]          (8 bytes received)
 *
 * Echo handling: TX is connected through 1kΩ to the PDN_UART node,
 * and RX is directly connected to the same node.  Every byte we
 * transmit appears back on our RX.  For reads: send 4-byte request,
 * discard 4-byte echo, then read 8-byte response.  For writes: send
 * 8-byte request, discard 8-byte echo (no response from slave).
 */

#include "tmc2209.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "bsp.h"
#include "settings.h"

static const char *TAG = "tmc2209";

/* ================================================================
 * Constants
 * ================================================================ */

#define TMC_SYNC_BYTE       0x05
#define TMC_WRITE_FLAG      0x80
#define TMC_READ_FLAG       0x00

/* Register addresses (TMC2209) */
#define REG_GCONF           0x00
#define REG_GSTAT           0x01
#define REG_IOIN            0x06
#define REG_IHOLD_IRUN      0x10
#define REG_TPOWERDOWN      0x11
#define REG_TSTEP           0x12
#define REG_TPWMTHRS        0x13
#define REG_TCOOLTHRS       0x14
#define REG_VACTUAL         0x22
/* SGTHRS=0x74（写入其他地址芯片同样回 ACK，值不会生效）；SG_RESULT
 * 不是独立寄存器——为 DRV_STATUS(0x6F) 的低 10 位 */
#define REG_SGTHRS          0x74
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
#define CHOPCONF_DISS2VS_MASK  (1U << 31)

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
#define READ_RESPONSE_TIMEOUT_MS  5   /* Slave response timeout */
#define ENABLE_SETTLE_MS          2   /* ≥1ms settle after EN low */

/* ================================================================
 * Internal state
 * ================================================================ */

static tmc2209_config_t s_cfg;
static bool s_initialized = false;
static SemaphoreHandle_t s_uart_mutex = NULL;
static bool s_vsense = false;   /* Current VSENSE setting (cached) */

/* ================================================================
 * Pure functions (public)
 * ================================================================ */

uint8_t tmc2209_crc8(const uint8_t *data, unsigned len)
{
    uint8_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

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

/* Build a write datagram (8 bytes: sync + addr + reg|WRITE + 4 data + CRC) */
static void build_write_dgram(uint8_t *buf, uint8_t slave_addr,
                              uint8_t reg, uint32_t val)
{
    buf[0] = TMC_SYNC_BYTE;
    buf[1] = slave_addr;
    buf[2] = reg | TMC_WRITE_FLAG;
    buf[3] = (uint8_t)(val >> 24);
    buf[4] = (uint8_t)(val >> 16);
    buf[5] = (uint8_t)(val >> 8);
    buf[6] = (uint8_t)(val >> 0);
    buf[7] = tmc2209_crc8(buf, 7);
}

/* Build a read request datagram (4 bytes: sync + addr + reg|READ + CRC) */
static void build_read_dgram(uint8_t *buf, uint8_t slave_addr, uint8_t reg)
{
    buf[0] = TMC_SYNC_BYTE;
    buf[1] = slave_addr;
    buf[2] = reg | TMC_READ_FLAG;
    buf[3] = tmc2209_crc8(buf, 3);
}

/* Parse a response datagram (8 bytes) → register value.
 * Returns true if CRC matches. */
static bool parse_response_dgram(const uint8_t *buf, uint32_t *val)
{
    uint8_t crc = tmc2209_crc8(buf, 7);
    if (crc != buf[7]) {
        return false;
    }
    *val = ((uint32_t)buf[3] << 24) |
           ((uint32_t)buf[4] << 16) |
           ((uint32_t)buf[5] << 8)  |
           ((uint32_t)buf[6] << 0);
    return true;
}

/* ================================================================
 * UART low-level (mutex-protected)
 * ================================================================ */

static bool uart_write_reg_locked(uint8_t reg, uint32_t val)
{
    uint8_t dgram[8];
    build_write_dgram(dgram, s_cfg.slave_addr, reg, val);

    /* Flush any stale RX data first */
    uart_flush_input(s_cfg.uart_num);

    /* Send write datagram (8 bytes) */
    int written = uart_write_bytes(s_cfg.uart_num, dgram, 8);
    if (written != 8) {
        ESP_LOGW(TAG, "write_reg: uart_write_bytes returned %d", written);
        return false;
    }

    /* Wait for TX to finish + echo to arrive in RX */
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(10));

    /* Discard echo (8 bytes) — we don't need to verify it, just drain */
    uint8_t echo_buf[8];
    int rx = uart_read_bytes(s_cfg.uart_num, echo_buf, 8, pdMS_TO_TICKS(10));
    if (rx < 8) {
        /* Not necessarily an error — some bytes may have been missed */
        ESP_LOGD(TAG, "write_reg: echo short read %d/8", rx);
    }

    /* TMC2209 does not send a response for write operations */
    return true;
}

static bool uart_read_reg_locked(uint8_t reg, uint32_t *val)
{
    uint8_t tx_dgram[4];
    build_read_dgram(tx_dgram, s_cfg.slave_addr, reg);

    /* Flush RX */
    uart_flush_input(s_cfg.uart_num);

    /* Send read request (4 bytes) */
    int written = uart_write_bytes(s_cfg.uart_num, tx_dgram, 4);
    if (written != 4) {
        ESP_LOGW(TAG, "read_reg: uart_write_bytes returned %d", written);
        return false;
    }

    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(10));

    /* Discard echo of the 4-byte request */
    uint8_t echo_buf[4];
    int echo_rx = uart_read_bytes(s_cfg.uart_num, echo_buf, 4, pdMS_TO_TICKS(10));
    if (echo_rx < 4) {
        ESP_LOGD(TAG, "read_reg: echo short %d/4", echo_rx);
        /* Continue anyway — maybe we started reading mid-echo */
    }

    /* Read 8-byte response from slave */
    uint8_t resp_buf[8];
    int resp_rx = uart_read_bytes(s_cfg.uart_num, resp_buf, 8,
                                  pdMS_TO_TICKS(READ_RESPONSE_TIMEOUT_MS));
    if (resp_rx != 8) {
        ESP_LOGD(TAG, "read_reg(0x%02x): response short %d/8", reg, resp_rx);
        return false;
    }

    return parse_response_dgram(resp_buf, val);
}

/* ================================================================
 * Register layer (public, mutex-protected)
 * ================================================================ */

bool tmc2209_read_reg(uint8_t reg, uint32_t *val)
{
    if (!s_initialized) return false;
    if (val == NULL) return false;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "read_reg: mutex timeout");
        return false;
    }
    bool ok = uart_read_reg_locked(reg, val);
    xSemaphoreGive(s_uart_mutex);
    return ok;
}

bool tmc2209_write_reg(uint8_t reg, uint32_t val)
{
    if (!s_initialized) return false;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "write_reg: mutex timeout");
        return false;
    }
    bool ok = uart_write_reg_locked(reg, val);
    xSemaphoreGive(s_uart_mutex);
    return ok;
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

    bsp_tmc_en_set(en);

    if (en) {
        /* Datasheet: wait for driver to wake up after EN low */
        vTaskDelay(pdMS_TO_TICKS(ENABLE_SETTLE_MS));
    }

    return ESP_OK;
}

esp_err_t tmc2209_set_stall_threshold(uint8_t sgthrs)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    if (!tmc2209_write_reg(REG_SGTHRS, (uint32_t)sgthrs)) {
        ESP_LOGE(TAG, "SGTHRS write failed (0x%02x)", sgthrs);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SGTHRS = %u", sgthrs);
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
    case 0: /* microstep */
        tmc2209_set_microstep((int)new_value);
        break;
    case 3: /* run_current_mA */
    case 4: /* hold_current_mA */
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

esp_err_t tmc2209_init(const tmc2209_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_initialized) return ESP_OK;

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* Create UART mutex */
    s_uart_mutex = xSemaphoreCreateMutex();
    if (s_uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Install UART driver */
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* RX/TX buffers must be STRICTLY greater than the UART FIFO length
     * (128 on ESP32-S3) — uart_driver_install(128, ...) fails with
     * "uart rx buffer length error".  256 gives headroom for the
     * two-wire echo (8-byte TX echoes back on RX). */
    esp_err_t err = uart_driver_install(cfg->uart_num, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(cfg->uart_num, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d: TX=%d RX=%d @ %d baud 8N1",
             cfg->uart_num, cfg->tx_pin, cfg->rx_pin, cfg->baud);

    s_initialized = true;

    /* ---- Apply default register configuration ---- */

    /* GCONF: pdn_disable=1 (UART mode), mstep_reg_select=1,
     * en_spreadcycle=1（StallGuard2 仅在 SpreadCycle 下有效，
     * 强制后 TPWMTHRS 不再参与模式切换；静音让位于堵转检测）,
     * shaft=0 */
    uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT
                   | GCONF_EN_SPREADCYCLE;
    if (!tmc2209_write_reg(REG_GCONF, gconf)) {
        ESP_LOGW(TAG, "init: GCONF write failed (check wiring?)");
    }

    /* CHOPCONF: TOFF, TBL, HSTRT, HEND per datasheet StealthChop recommendation;
     * MRES default from settings; INTPOL=1 */
    int mres = tmc2209_microstep_to_mres(settings_microstep());
    if (mres < 0) mres = 4; /* default 16 microsteps */

    s_vsense = calc_vsense(settings_run_current_ma(), cfg->r_sense);

    uint32_t chopconf =
        ((uint32_t)CHOPCONF_TOFF_DEFAULT  << CHOPCONF_TOFF_SHIFT)  |
        ((uint32_t)CHOPCONF_HSTRT_DEFAULT << CHOPCONF_HSTRT_SHIFT) |
        ((uint32_t)CHOPCONF_HEND_DEFAULT  << CHOPCONF_HEND_SHIFT)  |
        ((uint32_t)CHOPCONF_TBL_DEFAULT   << CHOPCONF_TBL_SHIFT)   |
        (s_vsense ? CHOPCONF_VSENSE_MASK : 0)                     |
        ((uint32_t)mres << CHOPCONF_MRES_SHIFT)                    |
        CHOPCONF_INTPOL_MASK;

    if (!tmc2209_write_reg(REG_CHOPCONF, chopconf)) {
        ESP_LOGW(TAG, "init: CHOPCONF write failed (check wiring?)");
    }

    /* IHOLD_IRUN: current from settings */
    int cs_run = tmc2209_ma_to_cs(settings_run_current_ma(), cfg->r_sense);
    int cs_hold = tmc2209_ma_to_cs(settings_hold_current_ma(), cfg->r_sense);
    uint32_t ihold_irun =
        ((uint32_t)cs_hold << IHOLD_IRUN_IHOLD_SHIFT) |
        ((uint32_t)cs_run  << IHOLD_IRUN_IRUN_SHIFT)  |
        ((uint32_t)IHOLDDELAY_DEFAULT << IHOLD_IRUN_IHOLDDELAY_SHIFT);

    if (!tmc2209_write_reg(REG_IHOLD_IRUN, ihold_irun)) {
        ESP_LOGW(TAG, "init: IHOLD_IRUN write failed (check wiring?)");
    }

    /* TPOWERDOWN: default delay */
    tmc2209_write_reg(REG_TPOWERDOWN, 20);

    /* TPWMTHRS: en_spreadCycle 已强制 SpreadCycle，此值不参与
     * 模式切换（保留写 0 = 寄存器确定状态） */
    tmc2209_write_reg(REG_TPWMTHRS, 0);

    /* TCOOLTHRS: 堵转输出仅在 TSTEP ≤ TCOOLTHRS（速度足够快）时
     * 启用，0 = 永不启用。f_clk≈12 MHz，50 mm/s = 2000 steps/s →
     * TSTEP≈6000；60000 对应 ≥~200 steps/s（约 5 mm/s），覆盖回零
     * 与全部运行速度，低速爬行时自动关闭防误报 */
    tmc2209_write_reg(REG_TCOOLTHRS, 60000);

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
    tmc2209_write_reg(REG_PWMCONF, pwmconf);

    ESP_LOGI(TAG, "initialized: microstep=%d, run=%dmA(CS=%d), hold=%dmA(CS=%d), VSENSE=%d",
             settings_microstep(),
             settings_run_current_ma(), cs_run,
             settings_hold_current_ma(), cs_hold,
             s_vsense ? 1 : 0);

    /* Subscribe to settings changes ($0, $3, $4) */
    if (!settings_subscribe(settings_change_cb)) {
        ESP_LOGW(TAG, "settings_subscribe failed (slot full)");
    }

    return ESP_OK;
}
