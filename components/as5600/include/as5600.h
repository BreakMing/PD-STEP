/*
 * AS5600 magnetic encoder — 12-bit angle + multi-turn position
 *
 * Scope: sensor layer only — I2C angle read → multi-turn
 * counting → continuous position (counts / mm) + velocity estimate +
 * origin shift.  NO loss-of-step detection and NO closed-loop
 * compensation (both deliberately deferred to phase 2), no status-frame
 * packing.
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   The AS5600 ZPOS / MPOS / CONF / MANG registers are one-time
 *   programmable (OTP): a wrong write destroys the chip permanently.
 *   This component therefore issues NO register write of any kind —
 *   the business layer never touches the bus directly; its only
 *   read path is as5600_hw_read_raw() (driver core; register-level
 *   red-line details live in as5600_hw.c).  Do not add write helpers.
 *
 * Hardware facts:
 *   7-bit address 0x36 (CONFIG_BSP_AS5600_I2C_ADDR), I2C_NUM_0,
 *   SDA=GPIO8 / SCL=GPIO9 @ 400 kHz, external pullups present.
 *   RAW ANGLE = 0x0C/0x0D, 12 bit → 4096 counts/rev.
 *   Drive: 2GT 20T × 2 mm = 40 mm/rev (D7, settings $2) →
 *   1 count = 40 / 4096 = 9.7656 µm.
 *
 * Sampling: dedicated task at 1 kHz, pinned to CPU1 so
 * the blocking I2C transaction cannot delay the STEP-pulse ISR on CPU0.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Counts per mechanical revolution (12-bit raw angle). */
#define AS5600_COUNTS_PER_REV   4096

/* ================================================================
 * Multi-turn tracker — pure logic, hardware independent (testable)
 * ================================================================ */

typedef struct {
    int32_t  pos_counts;   /* continuous position (counts, multi-turn, signed) */
    uint16_t last_raw;     /* previous raw angle 0..4095 */
    bool     seeded;       /* first sample received? */
} as5600_tracker_t;

/**
 * @brief Drop history — the next sample becomes position 0.
 */
void as5600_tracker_reset(as5600_tracker_t *t);

/**
 * @brief Feed one raw angle, return the updated continuous position.
 *
 * First sample only seeds the tracker (position 0, no displacement).
 * Wraparound correction:
 *   diff = raw - last_raw
 *   diff >  +2048 → diff -= 4096   (backwards across the 0/4095 edge)
 *   diff <  -2047 → diff += 4096   (forwards across the 0/4095 edge)
 * The asymmetric pair of thresholds covers all 4096 diff values with
 * no gap and no overlap.
 *
 * @param t     Tracker state.
 * @param raw   Raw angle 0..4095 (higher bits are masked off).
 * @return Continuous position in counts.
 */
int32_t as5600_tracker_update(as5600_tracker_t *t, uint16_t raw);

/* ================================================================
 * Driver + sampling task
 * ================================================================ */

/* Kconfig 布尔在 n 时不定义，统一折算为编译期常量（组装点使用） */
#ifdef CONFIG_AS5600_INVERT_DIR
#define AS5600_INVERT_DIR_DEFAULT 1
#else
#define AS5600_INVERT_DIR_DEFAULT 0
#endif

/* 接线参数（组装点读取 Kconfig 宏与 bsp 总线句柄后注入） */
typedef struct {
    i2c_master_bus_handle_t bus;
    uint16_t                dev_addr;
    uint32_t                scl_speed_hz;
    bool                    invert_dir;
} as5600_config_t;

/**
 * @brief 注入接线参数并启动 1 kHz 采样任务。幂等。
 *
 * mm/count 来自 settings $2，经一个订阅槽跟踪运行中变更。
 *
 * @return ESP_OK；cfg 为 NULL 或 bus 为 NULL 返回 ESP_ERR_INVALID_ARG；
 *         其余错误码与迁移前一致（I2C / 任务创建错误）。
 */
esp_err_t as5600_init(const as5600_config_t *cfg);

/* ================================================================
 * Telemetry (consumed by T9 monitor)
 * ================================================================ */

/** 连续位置（mm，含回零建立的原点偏移）。 */
float    as5600_position_mm(void);

/** Continuous position in counts (multi-turn, signed) — the truth value. */
int32_t  as5600_position_counts(void);

/**
 * @brief 位置换算 counts → µm（整数精确：40 mm ÷ 4096 = 625/64 µm/count）。
 *
 * 纯换算，不含原点偏移；头文件内联，宿主可测。
 */
static inline int64_t as5600_counts_to_um(int32_t counts)
{
    return (int64_t)counts * 625 / 64;
}

/**
 * @brief 连续位置（µm，整数精确，含回零原点偏移）。
 *
 * = counts × 625/64 − round(offset_mm × 1000)。不经 mm 浮点域。
 */
int64_t  as5600_position_um(void);

/** Smoothed velocity in mm/s (16 ms window); ≈0 when at standstill. */
float    as5600_velocity_mm_s(void);

/** Most recent raw angle 0..4095 (debug / self-test). */
uint16_t as5600_last_raw(void);

/** false = I2C reads have been failing (≥10 consecutive failures). */
bool     as5600_is_ok(void);

/* ================================================================
 * 原点偏移（回零触发点重置时建立）
 * ================================================================ */

/**
 * @brief Declare the current encoder position to be new_pos_mm (no motion).
 *
 * Implementation: offset = raw_position_mm − new_pos_mm;
 * reported position = raw_position_mm − offset.
 *
 * @param new_pos_mm  New reported coordinate in mm.
 */
void     as5600_set_origin(float new_pos_mm);

#ifdef __cplusplus
}
#endif
