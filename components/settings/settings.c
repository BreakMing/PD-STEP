/*
 * Settings — NVS-backed parameter store implementation
 *
 * PLAN §5 parameter table ($0–$13) with validation, NVS persistence,
 * change notification, and derived accessors.
 *
 * Thread safety: FreeRTOS mutex (s_mutex) protects s_values[] and
 * all NVS operations.  All public functions acquire the mutex.
 */

#include "settings.h"

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "settings";

/* ---- Parameter metadata ---- */

typedef struct {
    const char *name;
    float       default_val;
    bool        is_integer;
} param_meta_t;

static const param_meta_t s_meta[SETTINGS_COUNT] = {
    [0]  = { "microstep",         16.0f,   true  },  /* 2^n, 1–256 */
    [1]  = { "full_steps_per_rev", 200.0f, true  },  /* ∈{200, 400} */
    [2]  = { "units_per_rev",      40.0f,  false },  /* (0, 500] */
    [3]  = { "run_current_mA",     800.0f, true  },  /* (0, 2000] */
    [4]  = { "hold_current_mA",    400.0f, true  },  /* [0, $3] */
    [5]  = { "default_feed",       600.0f, false },  /* (0, $7] */
    [6]  = { "default_accel",      500.0f, false },  /* (0, $8] */
    [7]  = { "max_feed",           3000.0f, false }, /* (0, 30000] */
    [8]  = { "max_accel",          2000.0f, false }, /* (0, 50000] */
    [9]  = { "report_hz",          10.0f,   true  }, /* [1, 50] */
    [10] = { "invert_dir",         0.0f,    true  }, /* ∈{0, 1} */
    [11] = { "uart_baud",          115200.0f, true },/* baud enum */
    [12] = { "soft_min",           0.0f,    false }, /* [-1000, 1000], < $13 */
    [13] = { "soft_max",           600.0f,  false }, /* [-1000, 1000], > $12 */
};

/* ---- Runtime state ---- */

static float  s_values[SETTINGS_COUNT];
static bool   s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;

#define MAX_SUBSCRIBERS 4
static settings_change_cb_t s_subscribers[MAX_SUBSCRIBERS];
static int s_subscriber_count = 0;

/* ---- Helpers (must be called with mutex held) ---- */

static bool is_power_of_two(int v)
{
    return (v > 0) && ((v & (v - 1)) == 0);
}

static bool is_baud_valid(int baud)
{
    switch (baud) {
    case 9600: case 19200: case 38400: case 57600:
    case 115200: case 230400: case 460800: case 921600:
        return true;
    default:
        return false;
    }
}

/* Validate a single parameter against its domain rules.
 * Cross-checks use current s_values[] (caller must hold mutex). */
static bool validate_locked(int idx, float value)
{
    if (idx < 0 || idx >= SETTINGS_COUNT) {
        return false;
    }

    const param_meta_t *m = &s_meta[idx];

    /* Integer check */
    if (m->is_integer) {
        if (floorf(value) != value) {
            return false;  /* non-integer rejected */
        }
    }

    int iv = (int)value;

    switch (idx) {
    case 0:  /* microstep: 2^n, 1–256, integer */
        if (iv < 1 || iv > 256) return false;
        if (!is_power_of_two(iv)) return false;
        break;
    case 1:  /* full_steps_per_rev: 200 or 400 */
        if (iv != 200 && iv != 400) return false;
        break;
    case 2:  /* units_per_rev: (0, 500] */
        if (value <= 0.0f || value > 500.0f) return false;
        break;
    case 3:  /* run_current_mA: (0, 2000], int */
        if (iv <= 0 || iv > 2000) return false;
        /* Cross-check: $4 must be <= new $3 (both directions, PLAN §5) */
        if (iv < (int)s_values[4]) return false;
        break;
    case 4:  /* hold_current_mA: [0, $3], int */
        if (iv < 0 || iv > (int)s_values[3]) return false;
        break;
    case 5:  /* default_feed: (0, $7] */
        if (value <= 0.0f || value > s_values[7]) return false;
        break;
    case 6:  /* default_accel: (0, $8] */
        if (value <= 0.0f || value > s_values[8]) return false;
        break;
    case 7:  /* max_feed: (0, 30000] */
        if (value <= 0.0f || value > 30000.0f) return false;
        /* Cross-check: $5 must be <= new $7 */
        if (s_values[5] > value) return false;
        break;
    case 8:  /* max_accel: (0, 50000] */
        if (value <= 0.0f || value > 50000.0f) return false;
        /* Cross-check: $6 must be <= new $8 */
        if (s_values[6] > value) return false;
        break;
    case 9:  /* report_hz: [1, 50], int */
        if (iv < 1 || iv > 50) return false;
        break;
    case 10: /* invert_dir: 0 or 1 */
        if (iv != 0 && iv != 1) return false;
        break;
    case 11: /* uart_baud: enum */
        if (!is_baud_valid(iv)) return false;
        break;
    case 12: /* soft_min: [-1000, 1000], < $13 */
        if (value < -1000.0f || value > 1000.0f) return false;
        if (!(value < s_values[13])) return false;
        break;
    case 13: /* soft_max: [-1000, 1000], > $12 */
        if (value < -1000.0f || value > 1000.0f) return false;
        if (!(value > s_values[12])) return false;
        break;
    default:
        return false;
    }

    return true;
}

static void notify_locked(int idx, float new_value)
{
    for (int i = 0; i < s_subscriber_count; i++) {
        if (s_subscribers[i] != NULL) {
            s_subscribers[i](idx, new_value);
        }
    }
}

/* ---- NVS helpers ---- */

static const char *NVS_NAMESPACE = "settings";

static settings_status_t nvs_write_float(const char *key, float value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return SETTINGS_STORAGE;

    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    err = nvs_set_u32(handle, key, raw);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return (err == ESP_OK) ? SETTINGS_OK : SETTINGS_STORAGE;
}

static bool nvs_read_float(const char *key, float *out_val, bool *out_found)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        *out_found = false;
        return true;  /* not found is not an error */
    }

    uint32_t raw = 0;
    err = nvs_get_u32(handle, key, &raw);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out_found = false;
        return true;
    }
    if (err != ESP_OK) {
        return false;  /* real storage error */
    }

    memcpy(out_val, &raw, sizeof(float));
    *out_found = true;
    return true;
}

static settings_status_t nvs_erase_settings_all(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return SETTINGS_STORAGE;

    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return (err == ESP_OK) ? SETTINGS_OK : SETTINGS_STORAGE;
}

/* Load all values from NVS.  Two phases: first fill EVERY slot (missing
 * keys → defaults), only then validate — so cross-checks ($5≤$7, $6≤$8,
 * $12<$13) always see the complete set.  A single pass validated forward
 * references against not-yet-loaded zeros and wrongly discarded any
 * stored $5/$6/$12 on cold boot.  Invalid values → defaults + warning.
 * Caller must hold s_mutex. */
static bool load_all_locked(void)
{
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        char key[8];
        snprintf(key, sizeof(key), "p%d", i);

        float val = 0.0f;
        bool  found = false;
        if (!nvs_read_float(key, &val, &found)) {
            return false;  /* storage error */
        }
        s_values[i] = found ? val : s_meta[i].default_val;
    }

    for (int i = 0; i < SETTINGS_COUNT; i++) {
        if (!validate_locked(i, s_values[i])) {
            ESP_LOGW(TAG, "p%d has bad value %.3f, falling back to default %.3f",
                     i, (double)s_values[i], (double)s_meta[i].default_val);
            s_values[i] = s_meta[i].default_val;
        }
    }
    return true;
}

/* ================================================================
 * Public API
 * ================================================================ */

esp_err_t settings_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Standard NVS init with full-page-erase + new-version retry pattern */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase (%s), erasing and retrying",
                 esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool ok = load_all_locked();
    xSemaphoreGive(s_mutex);

    if (!ok) {
        ESP_LOGE(TAG, "Failed to load settings from NVS");
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "settings initialized (%d parameters, NVS namespace '%s')",
             SETTINGS_COUNT, NVS_NAMESPACE);
    return ESP_OK;
}

settings_status_t settings_reload(void)
{
    if (!s_initialized) return SETTINGS_STORAGE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool ok = load_all_locked();
    xSemaphoreGive(s_mutex);

    return ok ? SETTINGS_OK : SETTINGS_STORAGE;
}

settings_status_t settings_reset_defaults(void)
{
    if (!s_initialized) return SETTINGS_STORAGE;

    settings_status_t st = nvs_erase_settings_all();
    if (st != SETTINGS_OK) {
        return st;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Save old values for notification comparison */
    float old_vals[SETTINGS_COUNT];
    memcpy(old_vals, s_values, sizeof(s_values));

    /* Load defaults (NVS is empty now) */
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        s_values[i] = s_meta[i].default_val;
    }

    /* Notify for every parameter (reset changes all) */
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        if (old_vals[i] != s_values[i]) {
            xSemaphoreGive(s_mutex);
            /* Notify without holding mutex to avoid deadlock if
             * subscriber calls settings_get.  Value is stable because
             * we already set it above. */
            notify_locked(i, s_values[i]);
            xSemaphoreTake(s_mutex, portMAX_DELAY);
        }
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "all parameters reset to defaults");
    return SETTINGS_OK;
}

settings_status_t settings_set(int idx, float value)
{
    if (!s_initialized) return SETTINGS_STORAGE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (!validate_locked(idx, value)) {
        xSemaphoreGive(s_mutex);
        return SETTINGS_INVALID;
    }

    /* Write to NVS first */
    char key[8];
    snprintf(key, sizeof(key), "p%d", idx);
    settings_status_t st = nvs_write_float(key, value);
    if (st != SETTINGS_OK) {
        xSemaphoreGive(s_mutex);
        return SETTINGS_STORAGE;
    }

    /* Update in-memory value */
    s_values[idx] = value;

    xSemaphoreGive(s_mutex);

    /* Notify outside the mutex */
    for (int i = 0; i < s_subscriber_count; i++) {
        if (s_subscribers[i] != NULL) {
            s_subscribers[i](idx, value);
        }
    }

    return SETTINGS_OK;
}

float settings_get(int idx)
{
    if (idx < 0 || idx >= SETTINGS_COUNT) return 0.0f;
    if (!s_initialized) return s_meta[idx].default_val;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float v = s_values[idx];
    xSemaphoreGive(s_mutex);
    return v;
}

bool settings_is_valid(int idx, float value)
{
    if (!s_initialized) return false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool ok = validate_locked(idx, value);
    xSemaphoreGive(s_mutex);
    return ok;
}

int settings_count(void)
{
    return SETTINGS_COUNT;
}

const char *settings_name(int idx)
{
    if (idx < 0 || idx >= SETTINGS_COUNT) return "?";
    return s_meta[idx].name;
}

/* ---- Derived accessors ---- */

int settings_microstep(void)
{
    return (int)settings_get(0);
}

int settings_full_steps(void)
{
    return (int)settings_get(1);
}

float settings_units_per_rev(void)
{
    return settings_get(2);
}

float settings_steps_per_mm(void)
{
    /* steps/mm = full_steps_per_rev * microstep / units_per_rev */
    return (float)settings_full_steps() * (float)settings_microstep()
           / settings_units_per_rev();
}

int settings_run_current_ma(void)
{
    return (int)settings_get(3);
}

int settings_hold_current_ma(void)
{
    return (int)settings_get(4);
}

float settings_default_feed(void)
{
    return settings_get(5);
}

float settings_default_accel(void)
{
    return settings_get(6);
}

float settings_max_feed(void)
{
    return settings_get(7);
}

float settings_max_accel(void)
{
    return settings_get(8);
}

int settings_report_hz(void)
{
    return (int)settings_get(9);
}

bool settings_invert_dir(void)
{
    return settings_get(10) != 0.0f;
}

int settings_uart_baud(void)
{
    return (int)settings_get(11);
}

float settings_soft_min(void)
{
    return settings_get(12);
}

float settings_soft_max(void)
{
    return settings_get(13);
}

/* ---- Subscription ---- */

bool settings_subscribe(settings_change_cb_t cb)
{
    if (cb == NULL) return false;
    if (s_subscriber_count >= MAX_SUBSCRIBERS) return false;

    s_subscribers[s_subscriber_count++] = cb;
    return true;
}
