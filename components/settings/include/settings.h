/*
 * Settings — NVS-backed parameter store (PLAN §5)
 *
 * 14 parameters ($0–$13) with validation, NVS persistence,
 * change notification, and derived value accessors.
 *
 * Thread-safe: internal FreeRTOS mutex protects all reads/writes.
 * No dependency on protocol / gcode_parser / bsp (pure data layer).
 *
 * NVS layout: namespace "settings", keys "p0"–"p13", stored as u32
 * (float values are bit-copied via memcpy to/from uint32_t).
 * Missing keys → default value used silently (not written back until set).
 * Corrupt/out-of-range values → default + ESP_LOGW, key preserved.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_COUNT  14   /* $0..$13 */

/* ---- Status codes ---- */
typedef enum {
    SETTINGS_OK       = 0,
    SETTINGS_INVALID  = 1,   /* value illegal / unknown idx  → error:4 */
    SETTINGS_STORAGE  = 2,   /* NVS write/erase failure       → error:5 */
} settings_status_t;

/* ---- Change notification callback ----
 * Called after a successful set (and during reset_defaults for each
 * parameter that differs from default). */
typedef void (*settings_change_cb_t)(int idx, float new_value);

/* ================================================================
 * Lifecycle
 * ================================================================ */

/**
 * @brief Initialize settings: nvs_flash_init + load all 14 values.
 *
 * Must be called before any other settings_* function.
 * Safe to call multiple times (subsequent calls are no-op).
 *
 * @return ESP_OK on success, error code from nvs_flash_init otherwise.
 */
esp_err_t settings_init(void);

/**
 * @brief Reload all values from NVS (without re-running nvs_flash_init).
 *
 * @return SETTINGS_OK or SETTINGS_STORAGE.
 */
settings_status_t settings_reload(void);

/**
 * @brief Reset all parameters to factory defaults.
 *
 * Erases all NVS keys in the "settings" namespace, loads defaults,
 * and fires change notifications for every parameter.
 *
 * @return SETTINGS_OK or SETTINGS_STORAGE.
 */
settings_status_t settings_reset_defaults(void);

/* ================================================================
 * Core API
 * ================================================================ */

/**
 * @brief Set a parameter value (validates + writes to NVS + notifies).
 *
 * @param idx    Parameter index (0..13).
 * @param value  New value.
 *
 * @return SETTINGS_OK / SETTINGS_INVALID / SETTINGS_STORAGE.
 */
settings_status_t settings_set(int idx, float value);

/**
 * @brief Get current value of a parameter.
 *
 * @param idx  Parameter index (0..13).  Out-of-range returns 0.0f.
 * @return Current value.
 */
float settings_get(int idx);

/**
 * @brief Pure validation — no side effects.
 *
 * @param idx    Parameter index.
 * @param value  Value to check.
 *
 * @return true if value is legal for this parameter (including cross-checks
 *         against current values of other parameters).
 */
bool settings_is_valid(int idx, float value);

/**
 * @brief Number of parameters (= 14).
 */
int settings_count(void);

/**
 * @brief Short parameter name (for logging / $$ display aid).
 *
 * @param idx  Parameter index.  Out-of-range returns "?".
 */
const char *settings_name(int idx);

/* ================================================================
 * Derived / typed accessors
 *
 * Convenience wrappers; values derived per PLAN §4.7 / §5.
 * ================================================================ */

int   settings_microstep(void);
int   settings_full_steps(void);
float settings_units_per_rev(void);
float settings_steps_per_mm(void);     /* full_steps * microstep / units_per_rev */
int   settings_run_current_ma(void);
int   settings_hold_current_ma(void);
float settings_default_feed(void);
float settings_default_accel(void);
float settings_max_feed(void);
float settings_max_accel(void);
int   settings_report_hz(void);
bool  settings_invert_dir(void);
int   settings_uart_baud(void);
float settings_soft_min(void);
float settings_soft_max(void);

/* ================================================================
 * Change notification
 * ================================================================ */

/**
 * @brief Subscribe to parameter changes.
 *
 * Up to 4 subscriber slots.  Returns false if slots are full.
 *
 * @param cb  Callback function.
 * @return true if subscribed successfully.
 */
bool  settings_subscribe(settings_change_cb_t cb);

/* ---- On-target self-test (≥24 cases) ----
 * Only available when CONFIG_SETTINGS_SELF_TEST=y.
 * Prints results via ESP_LOG, final line: "SELF-TEST DONE: <n>/<total> PASS".
 * Cleans up with settings_reset_defaults() at the end. */
#ifdef CONFIG_SETTINGS_SELF_TEST
int settings_self_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
