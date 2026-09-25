/*
 * cartfsm_core — 模式机纯核心实现（转换表见 cartfsm_core.h）
 */

#include "cartfsm_core.h"

#include <stddef.h>

bool cm_step(cm_mode_t *mode, cm_reason_t *reason, cm_event_t ev)
{
    if (mode == NULL || reason == NULL) {
        return false;
    }

    switch (*mode) {
    case CM_DISABLED:
        if (ev == CE_CMD_HOME) {
            *mode = CM_HOMING;
            *reason = CR_NONE;
            return true;
        }
        return false;

    case CM_HOMING:
        switch (ev) {
        case CE_STALL:              /* 堵转 = 回零完成信号 */
            *mode = CM_READY;
            *reason = CR_NONE;
            return true;
        case CE_HOME_DONE:          /* 到位 = 回零完成信号（位置偏差判据） */
            *mode = CM_READY;
            *reason = CR_NONE;
            return true;
        case CE_HOME_TIMEOUT:
            *mode = CM_FAULT;
            *reason = CR_HOME_TIMEOUT;
            return true;
        case CE_AS5600_FAIL:
            *mode = CM_FAULT;
            *reason = CR_AS5600;
            return true;
        case CE_MT6835_FAIL:
            *mode = CM_FAULT;
            *reason = CR_MT6835;
            return true;
        case CE_CMD_RESET:          /* 回零中止：用户逃生口（撞墙不停/
                                     * 异响时立即停下），EN 由外壳关闭 */
            *mode = CM_DISABLED;
            *reason = CR_NONE;
            return true;
        default:
            return false;           /* HOME/STEP_LOSS 忽略 */
        }

    case CM_READY:
        switch (ev) {
        case CE_STEP_LOSS:
            *mode = CM_FAULT;
            *reason = CR_STEP_LOSS;
            return true;
        case CE_STALL:
            *mode = CM_FAULT;
            *reason = CR_STALL;
            return true;
        case CE_AS5600_FAIL:
            *mode = CM_FAULT;
            *reason = CR_AS5600;
            return true;
        case CE_MT6835_FAIL:
            *mode = CM_FAULT;
            *reason = CR_MT6835;
            return true;
        default:
            return false;           /* HOME/RESET/TIMEOUT 忽略 */
        }

    case CM_FAULT:
        if (ev == CE_CMD_RESET) {
            *mode = CM_DISABLED;
            *reason = CR_NONE;
            return true;
        }
        return false;

    default:
        return false;
    }
}

bool cm_config_set_allowed(cm_mode_t mode)
{
    return mode == CM_DISABLED || mode == CM_READY;
}

long cm_soft_limit_clamp(long x_um, long x_limit_um, long v_mm_s)
{
    if (x_um > x_limit_um && v_mm_s > 0) {
        return 0;                   /* 越上界，向外（+）分量置 0 */
    }
    if (x_um < -x_limit_um && v_mm_s < 0) {
        return 0;                   /* 越下界，向外（−）分量置 0 */
    }
    return v_mm_s;                  /* 界内或向内：不受限 */
}

bool cm_step_loss(long e_um, long c_um, long step_loss_um)
{
    long diff = e_um - c_um;
    if (diff < 0) {
        diff = -diff;
    }
    return diff > step_loss_um;
}

bool cm_speed_reached(long v_meas_mm_s, long home_vel_mm_s)
{
    if (home_vel_mm_s <= 0) {
        return false;
    }
    return v_meas_mm_s * 5 <= -home_vel_mm_s * 4;
}

cm_event_t cm_homing_event(const cm_homing_in_t *in)
{
    if (in == NULL) {
        return (cm_event_t)0;
    }
    if (!in->a5600_ok) {
        return CE_AS5600_FAIL;      /* E 无效时不得用于任何判定 */
    }
    if (!in->m6835_ok) {
        return CE_MT6835_FAIL;
    }
    if (cm_step_loss(in->e_um, in->c_um, in->step_loss_um)) {
        return in->speed_latch ? CE_HOME_DONE : CE_STEP_LOSS;
    }
    if (in->timed_out) {
        return CE_HOME_TIMEOUT;
    }
    return (cm_event_t)0;
}
