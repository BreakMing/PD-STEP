/*
 * Settings — NVS 参数存储实现
 *
 * 参数表（$0–$4，机械/驱动层）带校验、NVS 持久化、变更通知
 * 与派生访问器。方向反转由 CONFIG X_INV 承担，不在本表。
 *
 * 线程安全：FreeRTOS 互斥（s_mutex）保护 s_values[] 与全部 NVS
 * 操作。所有公开函数都先取互斥。
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

/* ---- 参数元数据 ---- */

typedef struct {
    const char *name;
    float       default_val;
    bool        is_integer;
} param_meta_t;

static const param_meta_t s_meta[SETTINGS_COUNT] = {
    [SET_MICROSTEP]  = { "microstep",         8.0f,   true  },  /* 2^n, 1–256 */
    [SET_FULL_STEP]  = { "full_steps_per_rev", 200.0f, true  },  /* ∈{200, 400} */
    [SET_MM_PER_REV] = { "mm_per_rev",        40.0f,  false },  /* (0, 500] */
    [SET_RUN_MA]     = { "run_current_mA",    800.0f, true  },  /* (0, 2000] */
    [SET_HOLD_MA]    = { "hold_current_mA",   400.0f, true  },  /* [0, $3] */
};

/* ---- 运行状态 ---- */

static float  s_values[SETTINGS_COUNT];
static bool   s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;

#define MAX_SUBSCRIBERS 4
static settings_change_cb_t s_subscribers[MAX_SUBSCRIBERS];
static int s_subscriber_count = 0;

/* ---- 辅助（须持互斥调用） ---- */

static bool is_power_of_two(int v)
{
    return (v > 0) && ((v & (v - 1)) == 0);
}

/* 对单个参数做域规则校验。交叉检查用当前 s_values[]
 * （调用方须持互斥）。 */
static bool validate_locked(int idx, float value)
{
    if (idx < 0 || idx >= SETTINGS_COUNT) {
        return false;
    }

    const param_meta_t *m = &s_meta[idx];

    if (m->is_integer) {
        if (floorf(value) != value) {
            return false;  /* 非整数拒绝 */
        }
    }

    int iv = (int)value;

    switch (idx) {
    case SET_MICROSTEP:  /* 2^n, 1–256, 整数 */
        if (iv < 1 || iv > 256) return false;
        if (!is_power_of_two(iv)) return false;
        break;
    case SET_FULL_STEP:  /* 200 或 400 */
        if (iv != 200 && iv != 400) return false;
        break;
    case SET_MM_PER_REV:  /* (0, 500] */
        if (value <= 0.0f || value > 500.0f) return false;
        break;
    case SET_RUN_MA:  /* (0, 2000], 整数；交叉检查 $4 ≤ 新 $3 */
        if (iv <= 0 || iv > 2000) return false;
        if (iv < (int)s_values[SET_HOLD_MA]) return false;
        break;
    case SET_HOLD_MA:  /* [0, $3], 整数 */
        if (iv < 0 || iv > (int)s_values[SET_RUN_MA]) return false;
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

/* ---- NVS 辅助 ---- */

static const char *NVS_NAMESPACE = "cart";

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
        return true;  /* 未找到不是错误 */
    }

    uint32_t raw = 0;
    err = nvs_get_u32(handle, key, &raw);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out_found = false;
        return true;
    }
    if (err != ESP_OK) {
        return false;  /* 真实存储错误 */
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

/* 从 NVS 载入全部值。两阶段：先填满每个槽（缺键 → 默认值），
 * 再统一校验——交叉检查（$3/$4）总是看到完整集合。单遍前向
 * 校验会在冷启动时用未载入的零值错误丢弃合法存储值。
 * 调用方须持 s_mutex。 */
static bool load_all_locked(void)
{
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        char key[8];
        snprintf(key, sizeof(key), "p%d", i);

        float val = 0.0f;
        bool  found = false;
        if (!nvs_read_float(key, &val, &found)) {
            return false;  /* 存储错误 */
        }
        s_values[i] = found ? val : s_meta[i].default_val;
    }

    for (int i = 0; i < SETTINGS_COUNT; i++) {
        if (!validate_locked(i, s_values[i])) {
            ESP_LOGW(TAG, "p%d 存储值 %.3f 非法，回落默认 %.3f",
                     i, (double)s_values[i], (double)s_meta[i].default_val);
            s_values[i] = s_meta[i].default_val;
        }
    }
    return true;
}

/* ================================================================
 * 公开 API
 * ================================================================ */

esp_err_t settings_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "创建互斥失败");
        return ESP_ERR_NO_MEM;
    }

    /* 标准 NVS 初始化：无空闲页/新版本时整擦重试 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要擦除（%s），擦除后重试",
                 esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init 失败：%s", esp_err_to_name(err));
        return err;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool ok = load_all_locked();
    xSemaphoreGive(s_mutex);

    if (!ok) {
        ESP_LOGE(TAG, "从 NVS 载入参数失败");
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "settings 已初始化（%d 参数，NVS namespace '%s'）",
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

    float old_vals[SETTINGS_COUNT];
    memcpy(old_vals, s_values, sizeof(s_values));

    for (int i = 0; i < SETTINGS_COUNT; i++) {
        s_values[i] = s_meta[i].default_val;
    }

    for (int i = 0; i < SETTINGS_COUNT; i++) {
        if (old_vals[i] != s_values[i]) {
            xSemaphoreGive(s_mutex);
            /* 不持互斥通知，避免订阅方回调 settings_get 死锁；
             * 值已在上面写入，稳定。 */
            notify_locked(i, s_values[i]);
            xSemaphoreTake(s_mutex, portMAX_DELAY);
        }
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "全部参数已恢复默认值");
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

    char key[8];
    snprintf(key, sizeof(key), "p%d", idx);
    settings_status_t st = nvs_write_float(key, value);
    if (st != SETTINGS_OK) {
        xSemaphoreGive(s_mutex);
        return st;
    }

    s_values[idx] = value;

    xSemaphoreGive(s_mutex);

    /* 互斥外通知 */
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

/* ---- 派生访问器 ---- */

int settings_microstep(void)
{
    return (int)settings_get(SET_MICROSTEP);
}

int settings_full_steps(void)
{
    return (int)settings_get(SET_FULL_STEP);
}

float settings_units_per_rev(void)
{
    return settings_get(SET_MM_PER_REV);
}

float settings_steps_per_mm(void)
{
    /* steps/mm = full_steps_per_rev * microstep / mm_per_rev */
    return (float)settings_full_steps() * (float)settings_microstep()
           / settings_units_per_rev();
}

int settings_run_current_ma(void)
{
    return (int)settings_get(SET_RUN_MA);
}

int settings_hold_current_ma(void)
{
    return (int)settings_get(SET_HOLD_MA);
}

/* ---- 订阅 ---- */

bool settings_subscribe(settings_change_cb_t cb)
{
    if (cb == NULL) return false;
    if (s_subscriber_count >= MAX_SUBSCRIBERS) return false;

    s_subscribers[s_subscriber_count++] = cb;
    return true;
}
