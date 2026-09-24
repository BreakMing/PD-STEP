/*
 * vel_ramp — 速度递推纯函数实现（与 vel_ramp.h 的契约对应）
 *
 * 全整数运算，64 位中间量；可上板（ISR）亦可宿主编译。
 */

#include "vel_ramp.h"

/* 整数平方根（逐位法，Hacker's Delight；u64 安全无溢出） */
static uint64_t isqrt64(uint64_t x)
{
    uint64_t r = 0;
    uint64_t bit = 1ULL << 62;
    while (bit > x) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (x >= r + bit) {
            x -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

uint32_t vel_ramp_start_dt(uint32_t accel)
{
    if (accel == 0) {
        accel = 1;   /* 防御：注册表保证加速度有正下限 */
    }
    uint64_t dt = isqrt64(2000000000000ULL / accel);
    if (dt < 1) {
        dt = 1;
    }
    return (uint32_t)dt;
}

bool vel_ramp_next(uint64_t v_q16, bool dir, uint64_t target_q16, bool target_dir,
                   uint32_t accel, uint32_t decel, bool wdt_expired,
                   uint64_t *out_v_q16, uint32_t *out_dt_us, bool *out_dir_change)
{
    *out_dir_change = false;

    /* 看门狗超时：有效目标 0、速率 decel、方向不变 */
    uint64_t eff_target = wdt_expired ? 0 : target_q16;
    bool     eff_dir    = wdt_expired ? dir : target_dir;
    uint32_t rate       = wdt_expired ? decel : accel;
    if (rate == 0) {
        rate = 1;   /* 防御 */
    }

    /* ---- 零速：起步（dt0 语义）或无事可做 ---- */
    if (v_q16 == 0) {
        if (eff_target == 0) {
            *out_v_q16 = 0;
            *out_dt_us = 0;
            return false;
        }
        *out_dir_change = (eff_dir != dir);
        uint32_t dt0 = vel_ramp_start_dt(accel);
        *out_v_q16 = (uint64_t)accel * dt0 * VEL_RAMP_Q16_ONE / 1000000u;
        *out_dt_us = dt0;
        return true;
    }

    /* ---- 向零减速（目标为零，或换向经零速） ---- */
    if (eff_target == 0 || eff_dir != dir) {
        uint64_t dv = ((uint64_t)rate << 32) / v_q16;
        uint64_t v_new = (dv >= v_q16) ? 0 : (v_q16 - dv);

        if (v_new < VEL_RAMP_STOP_Q16) {
            /* 尾端吸附：到零（或低于吸附阈值） */
            if (!wdt_expired && eff_target > 0 && eff_dir != dir) {
                /* 换向起步 */
                *out_dir_change = true;
                uint32_t dt0 = vel_ramp_start_dt(accel);
                *out_v_q16 = (uint64_t)accel * dt0 * VEL_RAMP_Q16_ONE / 1000000u;
                *out_dt_us = dt0;
                return true;
            }
            *out_v_q16 = 0;
            *out_dt_us = 0;
            return false;
        }

        *out_v_q16 = v_new;
        *out_dt_us = (uint32_t)(1000000ULL * VEL_RAMP_Q16_ONE / v_new);
        return true;
    }

    /* ---- 同方向追踪目标 ---- */
    uint64_t dv = ((uint64_t)rate << 32) / v_q16;
    uint64_t v_new;
    if (v_q16 < eff_target) {
        v_new = v_q16 + dv;
        if (v_new > eff_target || v_new < v_q16) {
            v_new = eff_target;   /* 钳位：不过冲、不回绕 */
        }
    } else {
        uint64_t room = v_q16 - eff_target;
        v_new = (dv >= room) ? eff_target : (v_q16 - dv);   /* 钳位：不下穿 */
    }

    *out_v_q16 = v_new;
    *out_dt_us = (uint32_t)(1000000ULL * VEL_RAMP_Q16_ONE / v_new);
    return true;
}
