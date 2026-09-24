/*
 * TMC2209 — UART-controlled stepper driver
 *
 * Two-wire UART interface (TX through 1kΩ to PDN_UART,
 * RX direct from PDN_UART node).  Echo handling required
 * because TX appears on RX via the shared node.
 *
 * Register map based on TMC2209 datasheet + TMCStepper reference.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Configuration
 * ================================================================ */

typedef struct {
    int      uart_num;     /* UART port number, e.g. UART_NUM_2 */
    int      tx_pin;       /* TX GPIO (through 1kΩ to PDN_UART) */
    int      rx_pin;       /* RX GPIO (direct from PDN_UART) */
    int      baud;         /* Baud rate, default 115200 */
    uint8_t  slave_addr;   /* Slave address, default 0 */
    float    r_sense;      /* Sense resistor value in ohms */
} tmc2209_config_t;

/* ================================================================
 * Lifecycle
 * ================================================================ */

/**
 * @brief Initialize TMC2209 driver.
 *
 * Installs UART driver, applies default register configuration
 * (GCONF, CHOPCONF, IHOLD_IRUN, PWMCONF, TPWMTHRS), and sets
 * initial current / microstep from settings.
 *
 * Also subscribes to settings changes for $0/$3/$4.
 *
 * @param cfg  Configuration structure.
 * @return ESP_OK on success.
 */
esp_err_t tmc2209_init(const tmc2209_config_t *cfg);

/* ================================================================
 * Register layer
 * ================================================================ */

/**
 * @brief Read a 32-bit register via UART.
 *
 * @param reg  Register address.
 * @param val  Pointer to receive the read value.
 * @return true if communication succeeded and CRC matched.
 */
bool tmc2209_read_reg(uint8_t reg, uint32_t *val);

/**
 * @brief Write a 32-bit register via UART.
 *
 * @param reg  Register address.
 * @param val  Value to write.
 * @return true if communication succeeded.
 */
bool tmc2209_write_reg(uint8_t reg, uint32_t val);

/* ================================================================
 * Functional layer
 * ================================================================ */

/**
 * @brief Set run and hold currents in mA.
 *
 * Automatically selects VSENSE (0 or 1) for best resolution.
 *
 * @param run_ma   Run current (mA).
 * @param hold_ma  Hold current (mA).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range.
 */
esp_err_t tmc2209_set_current(int run_ma, int hold_ma);

/**
 * @brief Set microstep resolution.
 *
 * @param ms  Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if illegal value.
 */
esp_err_t tmc2209_set_microstep(int ms);

/**
 * @brief Enable / disable driver via EN pin (active low).
 *
 * en=true → drive EN low, wait ≥1ms for driver to wake up.
 * en=false → drive EN high (disabled).
 *
 * @param en  true = enable, false = disable.
 * @return ESP_OK on success.
 */
esp_err_t tmc2209_enable(bool en);

/**
 * @brief 写 StallGuard 堵转检测阈值 SGTHRS（REG 0x40）。
 *
 * 回零完成信号与 READY 堵转故障共用该阈值（CONFIG 注册表 SGTHRS）。
 * 值越大检测越灵敏（SG_RESULT < 2×SGTHRS 触发）。
 *
 * @param sgthrs  阈值 0–255。
 * @return ESP_OK 成功；ESP_FAIL UART 写失败。
 */
esp_err_t tmc2209_set_stall_threshold(uint8_t sgthrs);

/* ================================================================
 * Status layer
 * ================================================================ */

typedef struct {
    uint32_t raw;           /* Raw DRV_STATUS register value */
    bool     stst;          /* Standstill (motor stopped) */
    bool     ola;           /* Open load phase A */
    bool     olb;           /* Open load phase B */
    bool     s2ga;          /* Short to GND phase A */
    bool     s2gb;          /* Short to GND phase B */
    bool     otpw;          /* Overtemperature pre-warning */
    bool     ot;            /* Overtemperature shutdown */
    int      cs_actual;     /* Actual current scale (0..31) */
} tmc2209_status_t;

/**
 * @brief Read DRV_STATUS and decode flags.
 *
 * @param st  Status structure to fill.
 * @return true if read succeeded.
 */
bool tmc2209_read_status(tmc2209_status_t *st);

/* ================================================================
 * Pure functions (no hardware dependency — self-test targets)
 * ================================================================ */

/**
 * @brief Compute CRC-8 (polynomial 0x07, init 0x00, LSB-first).
 *
 * @param data  Data buffer.
 * @param len   Number of bytes.
 * @return CRC value.
 */
uint8_t tmc2209_crc8(const uint8_t *data, unsigned len);

/**
 * @brief Convert RMS current (mA) to CS register value (0..31).
 *
 * Uses formula: CS = 32 * sqrt(2) * I_rms * (R_sense + 0.02) / V_fs - 1
 * Automatically picks VSENSE based on current level.
 * Result is clamped to [0, 31].
 *
 * @param ma       RMS current in mA.
 * @param r_sense  Sense resistor in ohms.
 * @return CS value [0..31].
 */
int tmc2209_ma_to_cs(int ma, float r_sense);

/**
 * @brief Convert microstep setting to MRES register value.
 *
 * Mapping: 256→0, 128→1, 64→2, 32→3, 16→4, 8→5, 4→6, 2→7, 1→8.
 *
 * @param ms  Microsteps per full step.
 * @return MRES value (0..8), or -1 if invalid.
 */
int tmc2209_microstep_to_mres(int ms);

/* ================================================================
 * Self-test (gated by CONFIG_TMC2209_SELF_TEST)
 * ================================================================ */
#ifdef CONFIG_TMC2209_SELF_TEST
int tmc2209_self_test_run(void);
#endif

/* ================================================================
 * Bring-up test (gated by CONFIG_TMC2209_BRINGUP)
 * ================================================================ */
#ifdef CONFIG_TMC2209_BRINGUP
int tmc2209_bringup_run(void);
#endif

#ifdef __cplusplus
}
#endif
