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
        if (ev == CE_CMD_ENABLE) {
            *mode = CM_READY;
            *reason = CR_NONE;
            return true;
        }
        return false;

    case CM_READY:
        switch (ev) {
        case CE_STEP_LOSS:
            *mode = CM_FAULT;
            *reason = CR_STEP_LOSS;
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
            return false;           /* ENABLE/RESET 忽略 */
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

long cm_soft_limit_clamp(long x_um, long x_min_um, long x_max_um,
                         long v_mm_s)
{
    if (x_um > x_max_um && v_mm_s > 0) {
        return 0;                   /* 越上界，向外（+）分量置 0 */
    }
    if (x_um < x_min_um && v_mm_s < 0) {
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

bool cm_calib_valid(long l_um)
{
    return l_um >= 100000L && l_um <= 1000000L;
}
