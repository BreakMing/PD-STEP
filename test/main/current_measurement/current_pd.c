#include "current_pd.h"

bool current_pd_begin(current_pd_t *s, current_pd_target_t target,
                      bool motor_idle, bool en_disabled, int64_t now_us)
{
    if (!motor_idle || !en_disabled || s->state == CURRENT_PD_WAITING ||
        (target != CURRENT_PD_9V && target != CURRENT_PD_20V)) return false;
    s->state = CURRENT_PD_WAITING;
    s->target = target;
    s->started_us = now_us;
    s->last_sample_us = now_us - CURRENT_PD_SAMPLE_US;
    s->stable_samples = 0;
    return true;
}

current_pd_result_t current_pd_sample(current_pd_t *s, int64_t now_us,
                                     bool pd_valid, int vbus_mv)
{
    if (s->state != CURRENT_PD_WAITING) return CURRENT_PD_PENDING;
    int64_t elapsed = now_us - s->started_us;
    if (elapsed >= CURRENT_PD_TIMEOUT_US) {
        current_pd_cancel(s);
        return CURRENT_PD_TIMEOUT;
    }
    if (elapsed < CURRENT_PD_SETTLE_US ||
        now_us - s->last_sample_us < CURRENT_PD_SAMPLE_US) return CURRENT_PD_PENDING;
    s->last_sample_us = now_us;
    if (pd_valid && vbus_mv >= (int)s->target * 950 &&
        vbus_mv <= (int)s->target * 1050) {
        ++s->stable_samples;
    } else {
        s->stable_samples = 0;
    }
    if (s->stable_samples < CURRENT_PD_STABLE_SAMPLES) return CURRENT_PD_PENDING;
    s->state = CURRENT_PD_VERIFIED;
    return CURRENT_PD_SUCCESS;
}

void current_pd_cancel(current_pd_t *s)
{
    if (s->state == CURRENT_PD_WAITING) s->state = CURRENT_PD_UNCERTAIN;
    s->stable_samples = 0;
}

bool current_pd_start_allowed(const current_pd_t *s)
{
    return s->state == CURRENT_PD_UNCHANGED || s->state == CURRENT_PD_VERIFIED;
}
