/*
 * magnetic_encoder_core — 骨架纯函数实现（契约见同名头）
 */

#include "magnetic_encoder_core.h"

#include <math.h>
#include <string.h>

/* ---- 失败计数状态机 ---- */

void magnetic_encoder_fail_reset(magnetic_encoder_fail_t *f)
{
    if (f == NULL) {
        return;
    }
    f->streak = 0;
    f->ok     = true;
}

bool magnetic_encoder_fail_step(magnetic_encoder_fail_t *f, bool read_ok)
{
    if (f == NULL) {
        return false;
    }

    if (read_ok) {
        f->streak = 0;
        if (!f->ok) {
            f->ok = true;
            return true;        /* NOT OK 期间恢复——事件 */
        }
        return false;
    }

    f->streak++;
    if (f->streak == MAGNETIC_ENCODER_FAIL_LIMIT && f->ok) {
        f->ok = false;
        return true;           /* 首达 LIMIT 进入 NOT OK——事件 */
    }
    return false;
}

/* ---- 16 ms 速度窗 ---- */

void magnetic_encoder_vel_win_reset(magnetic_encoder_vel_win_t *w)
{
    if (w == NULL) {
        return;
    }
    memset(w, 0, sizeof(*w));
    w->head = -1;
}

int32_t magnetic_encoder_vel_push(magnetic_encoder_vel_win_t *w, int64_t pos)
{
    if (w == NULL) {
        return 0;
    }

    w->head = (w->head + 1) % MAGNETIC_ENCODER_HIST_LEN;
    w->hist[w->head] = pos;
    if (w->fill < MAGNETIC_ENCODER_HIST_LEN) {
        w->fill++;
    }

    if (w->fill < MAGNETIC_ENCODER_HIST_LEN) {
        return 0;              /* 窗未满 */
    }

    int oldest = (w->head + 1) % MAGNETIC_ENCODER_HIST_LEN;
    int64_t delta = pos - w->hist[oldest];
    /* counts / 16 ms → counts/s（量子 62.5） */
    return (int32_t)llround((double)delta
                            * (1000.0 / MAGNETIC_ENCODER_SPEED_WINDOW));
}
