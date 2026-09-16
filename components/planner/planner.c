/*
 * Motion Planner implementation (see planner.h for the contract)
 *
 * Pipeline (planner_enqueue):
 *   soft limit → mm clamp ($7/$8) → mm→steps → solve → enqueue → commit
 */

#include "planner.h"
#include "settings.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "planner";

/* ---- Engine-domain limits (T6 fact #2) ----
 * step_engine converts float→Q16.16 as (uint32)(v*65536+0.5f).
 * v == 65536.0f would produce 0x100000000 → wraps to 0, so the
 * inclusive upper bound is 65535.0f.  The lower bound keeps the
 * conversion from rounding down to 0 (ISR divide-by-zero / no accel).
 */
#define PLANNER_ENGINE_V_MAX   65535.0f
#define PLANNER_ENGINE_MIN     1.0e-3f

/* Absurd-travel guard: 1e9 steps at 80 steps/mm = 12500 km (PLAN §4.1
 * step 3).  Anything larger indicates a corrupt target. */
#define PLANNER_MAX_DELTA_STEPS  1.0e9

/* ---- State ---- */

static bool             s_initialized = false;
static volatile int32_t s_pos_steps = 0;    /* commanded position */
static float            s_k = 0.0f;         /* steps per mm */

#ifdef CONFIG_PLANNER_SELF_TEST
static step_seg_t s_last_seg;
#endif

/* ---- Helpers ---- */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Recompute the steps/mm cache from settings ($0/$1/$2). */
static void refresh_k(void)
{
    float k = settings_steps_per_mm();
    if (k > 0.0f) {
        s_k = k;
    }
    ESP_LOGI(TAG, "steps_per_mm k=%.4f", (double)s_k);
}

/* Settings change notification — only $0/$1/$2 affect k. */
static void on_settings_change(int idx, float new_value)
{
    (void)new_value;
    if (idx == 0 || idx == 1 || idx == 2) {
        refresh_k();
    }
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

esp_err_t planner_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_pos_steps = 0;   /* D10: power-up position is 0 */

    if (!settings_subscribe(on_settings_change)) {
        ESP_LOGW(TAG, "settings subscribe failed (slots full) — "
                      "k will not track $0/$1/$2 changes");
    }

    refresh_k();
    if (s_k <= 0.0f) {
        ESP_LOGE(TAG, "invalid steps_per_mm (%.4f) — planner unusable",
                 (double)s_k);
        return ESP_ERR_INVALID_STATE;
    }

    s_initialized = true;
    return ESP_OK;
}

/* ================================================================
 * Pure solver
 * ================================================================ */

bool planner_solve(uint32_t delta_steps, float v_max_steps,
                   float accel_steps, step_seg_t *out_seg)
{
    if (out_seg == NULL || delta_steps == 0) {
        return false;
    }
    if (!(v_max_steps > 0.0f) || !(accel_steps > 0.0f)) {
        return false;   /* also rejects NaN */
    }

    /* Engine-domain clamp (upper = Q16.16 wraparound guard,
     * lower = float_to_q16 rounds-to-zero guard) */
    float v = clampf(v_max_steps, PLANNER_ENGINE_MIN, PLANNER_ENGINE_V_MAX);
    float a = clampf(accel_steps, PLANNER_ENGINE_MIN, PLANNER_ENGINE_V_MAX);

    uint32_t accel_cnt;
    uint32_t decel_cnt;
    uint32_t cruise_cnt;
    float    v_cruise;

    /* Steps required to accelerate from 0 to v */
    float d = (v * v) / (2.0f * a);

    if (2.0f * d <= (float)delta_steps) {
        /* Trapezoid: reaches the requested cruise velocity */
        accel_cnt = (uint32_t)floorf(d);
        decel_cnt = accel_cnt;                    /* symmetric (A4) */
        cruise_cnt = delta_steps - 2u * accel_cnt;
        v_cruise = v;
    } else {
        /* Triangle: distance too short to reach v */
        accel_cnt = delta_steps / 2u;
        decel_cnt = delta_steps - accel_cnt;      /* odd N: +1 to decel */
        cruise_cnt = 0u;
        v_cruise = sqrtf(2.0f * a * (float)accel_cnt);
    }

    /* Floor: N=1 triangle gives sqrtf(0)=0, which fails the engine's
     * "v_cruise > 0" validation. */
    v_cruise = fmaxf(v_cruise, PLANNER_ENGINE_MIN);

    out_seg->steps        = delta_steps;
    out_seg->accel_steps  = accel_cnt;
    out_seg->cruise_steps = cruise_cnt;
    out_seg->decel_steps  = decel_cnt;
    out_seg->accel        = a;
    out_seg->v_cruise     = v_cruise;
    /* out_seg->dir is set by the caller */

    /* Invariant check — a mismatch here is a planner bug (T6 P0#3). */
    if (accel_cnt + cruise_cnt + decel_cnt != delta_steps) {
        ESP_LOGE(TAG, "seg sum mismatch: %lu+%lu+%lu != %lu",
                 (unsigned long)accel_cnt, (unsigned long)cruise_cnt,
                 (unsigned long)decel_cnt, (unsigned long)delta_steps);
        return false;
    }

    return true;
}

/* ================================================================
 * Core planning
 * ================================================================ */

planner_status_t planner_enqueue(float target_mm, float feed_mm_min,
                                 float accel_mm_s2)
{
    if (!s_initialized) {
        /* Defensive: treat as soft-limit rejection rather than moving
         * with an unknown steps/mm scale. */
        ESP_LOGE(TAG, "enqueue before planner_init");
        return PLANNER_ERR_SOFT_LIMIT;
    }

    /* 1. Soft limits — inclusive on both ends */
    float lo = settings_soft_min();
    float hi = settings_soft_max();
    if (target_mm < lo || target_mm > hi) {
        ESP_LOGW(TAG, "soft limit: target %.4f outside [%.4f, %.4f]",
                 (double)target_mm, (double)lo, (double)hi);
        return PLANNER_ERR_SOFT_LIMIT;
    }

    /* 2. mm → steps (round to nearest) */
    float    k = s_k;
    int32_t  target_steps = (int32_t)lroundf(target_mm * k);

    /* 3. Delta + absurd-travel guard */
    int32_t  pos = s_pos_steps;
    int64_t  delta = (int64_t)target_steps - (int64_t)pos;

    if (delta > PLANNER_MAX_DELTA_STEPS || delta < -PLANNER_MAX_DELTA_STEPS) {
        ESP_LOGW(TAG, "delta %ld steps exceeds guard", (long)delta);
        return PLANNER_ERR_SOFT_LIMIT;
    }

    /* 4. No movement — still commit (G92-style re-sync) */
    if (delta == 0) {
        s_pos_steps = target_steps;
        return PLANNER_OK;
    }

    /* 5. mm-domain clamp via $7 / $8, then to step domain */
    float feed = feed_mm_min;
    float max_feed = settings_max_feed();
    if (feed > max_feed) {
        feed = max_feed;
    }

    float accel = accel_mm_s2;
    float max_accel = settings_max_accel();
    if (accel > max_accel) {
        accel = max_accel;
    }

    float v_steps = feed / 60.0f * k;   /* mm/min → mm/s → steps/s */
    float a_steps = accel * k;          /* mm/s²  → steps/s² */

    /* 6. Solve the profile */
    uint32_t dist = (uint32_t)((delta > 0) ? delta : -delta);

    step_seg_t seg;
    memset(&seg, 0, sizeof(seg));
    if (!planner_solve(dist, v_steps, a_steps, &seg)) {
        ESP_LOGE(TAG, "planner_solve failed (dist=%lu v=%.3f a=%.3f)",
                 (unsigned long)dist, (double)v_steps, (double)a_steps);
        return PLANNER_ERR_SOFT_LIMIT;
    }
    seg.dir = (delta > 0);

    /* 7. Blocking enqueue with '!' race handling.
     *
     * enqueue failure ⇒ the 16-deep queue is full ⇒ busy must be true.
     * The busy re-check below happens in the same iteration with no
     * intervening delay (µs scale).  Draining 16 queued segments takes
     * at least 16 steps at the 65535 steps/s ceiling ≈ 240 µs, far
     * longer than that window, so a false "not busy" cannot arise from
     * normal consumption.  The only cause is step_engine_stop() having
     * cleared the queue ('!' immediate stop, D11 — no resume).
     * In that case the move is cancelled: drop the segment, leave the
     * commanded position untouched, and report success so the G-code
     * line still answers "ok". */
    while (!step_engine_enqueue(&seg)) {
        if (!step_engine_busy()) {
            ESP_LOGW(TAG, "move cancelled by '!' stop — segment dropped");
            return PLANNER_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

#ifdef CONFIG_PLANNER_SELF_TEST
    s_last_seg = seg;
#endif

    /* 8. Commit commanded position */
    s_pos_steps = target_steps;

    return PLANNER_OK;
}

void planner_set_origin(float new_pos_mm)
{
    if (!s_initialized) {
        return;
    }
    s_pos_steps = (int32_t)lroundf(new_pos_mm * s_k);
    ESP_LOGI(TAG, "origin set: %.4f mm = %ld steps",
             (double)new_pos_mm, (long)s_pos_steps);
}

/* ================================================================
 * Telemetry
 * ================================================================ */

int32_t planner_position_steps(void)
{
    return s_pos_steps;
}

float planner_position_mm(void)
{
    float k = s_k;
    if (k <= 0.0f) {
        return 0.0f;
    }
    return (float)s_pos_steps / k;
}

/* ================================================================
 * Self-test hooks
 * ================================================================ */

#ifdef CONFIG_PLANNER_SELF_TEST
void planner_selftest_last_seg(step_seg_t *out)
{
    if (out != NULL) {
        *out = s_last_seg;
    }
}
#endif
