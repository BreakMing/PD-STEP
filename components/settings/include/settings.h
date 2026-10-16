/*
 * Settings — NVS 机械/驱动层参数存储
 *
 * 5 项参数（枚举索引 0–4）带校验、NVS 持久化、变更通知与派生访问器。
 * 方向反转不在此表——由线协议 CONFIG 扩项 X_INV 承担，
 * 避免 NVS 与 CONFIG 两处权威。
 *
 * 线程安全：内部 FreeRTOS 互斥保护全部读写。
 * 不依赖 cartproto / bsp（纯数据层）。
 *
 * NVS 布局：namespace "cart"，键 "p0"–"p4"，u32 存储
 * （float 值经 memcpy 位拷贝为 uint32_t）。
 * 缺键 → 静默使用默认值（直到 set 才写回）。
 * 损坏/超范围值 → 默认值 + ESP_LOGW，键保留。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 参数索引（NVS 键 "p<idx>"） */
enum {
    SET_MICROSTEP   = 0,  /* 细分，2^n，1–256，整数，默认 8 */
    SET_FULL_STEP   = 1,  /* 每转整步数，∈{200,400}，默认 200 */
    SET_MM_PER_REV  = 2,  /* 每转行程 mm，(0,500]，默认 40.0 */
    SET_RUN_MA      = 3,  /* 运行电流 mA，(0,2000]，整数，默认 800 */
    SET_HOLD_MA     = 4,  /* 保持电流 mA，[0, 运行电流]，整数，默认 400 */
    SETTINGS_COUNT  = 5,
};

/* ---- 状态码 ---- */
typedef enum {
    SETTINGS_OK       = 0,
    SETTINGS_INVALID  = 1,   /* 值非法 / 索引未知 */
    SETTINGS_STORAGE  = 2,   /* NVS 写/擦失败 */
} settings_status_t;

/* ---- 变更通知回调 ----
 * set 成功后（以及 reset_defaults 时每个偏离默认值的参数）调用。 */
typedef void (*settings_change_cb_t)(int idx, float new_value);

/* ================================================================
 * 生命周期
 * ================================================================ */

/**
 * @brief 初始化：nvs_flash_init + 载入全部参数。
 *
 * 必须在其他 settings_* 之前调用；重复调用为空操作。
 */
esp_err_t settings_init(void);

/**
 * @brief 从 NVS 重新载入全部参数（不重跑 nvs_flash_init）。
 *
 * @return SETTINGS_OK 或 SETTINGS_STORAGE。
 */
settings_status_t settings_reload(void);

/**
 * @brief 恢复出厂默认值。
 *
 * 擦除 "cart" namespace 全部键，载入默认值，
 * 并对每个参数触发变更通知。
 */
settings_status_t settings_reset_defaults(void);

/* ================================================================
 * 核心 API
 * ================================================================ */

/**
 * @brief 设置参数（校验 + 写 NVS + 通知）。
 *
 * @param idx    参数索引（0..4）。
 * @param value  新值。
 * @return SETTINGS_OK / SETTINGS_INVALID / SETTINGS_STORAGE。
 */
settings_status_t settings_set(int idx, float value);

/**
 * @brief 读取参数当前值。索引超范围返回 0.0f。
 */
float settings_get(int idx);

/**
 * @brief 纯校验，无副作用（含对其他参数当前值的交叉检查）。
 */
bool settings_is_valid(int idx, float value);

/**
 * @brief 参数数量（= 5）。
 */
int settings_count(void);

/**
 * @brief 参数短名（日志用）。索引超范围返回 "?"。
 */
const char *settings_name(int idx);

/* ================================================================
 * 派生访问器
 * ================================================================ */

int   settings_microstep(void);
int   settings_full_steps(void);
float settings_units_per_rev(void);
float settings_steps_per_mm(void);     /* full_steps * microstep / units_per_rev */
int   settings_run_current_ma(void);
int   settings_hold_current_ma(void);

/* ================================================================
 * 变更通知
 * ================================================================ */

/**
 * @brief 订阅参数变更。最多 4 个订阅槽，满则返回 false。
 */
bool  settings_subscribe(settings_change_cb_t cb);

#ifdef __cplusplus
}
#endif
