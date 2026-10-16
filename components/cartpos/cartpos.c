/*
 * cartpos — 小车位置解释实现
 *
 * 机械比例（每转行程 → µm/count）的唯一权威在本层：位置与速度同源
 * 同值。每转行程非法（≤0）回退 40 mm/rev 并告警（沿袭原 as5600 行为）。
 */
#include "cartpos.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "settings.h"

static const char *TAG = "cartpos";

#define CARTPOS_DEFAULT_UPR  40.0f  /* fallback if units_per_rev is unusable */

static magnetic_encoder_handle_t s_enc;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static long   s_offset_um = 0;     /* 原点偏移（µm 整数） */
static double s_um_per_count = 0.0;

static void refresh_scale(void)
{
    float upr = settings_units_per_rev();
    if (!(upr > 0.0f)) {
        ESP_LOGW(TAG, "invalid units_per_rev=%.3f — falling back to %.1f mm/rev",
                 (double)upr, (double)CARTPOS_DEFAULT_UPR);
        upr = CARTPOS_DEFAULT_UPR;
    }

    int32_t cpr = magnetic_encoder_counts_per_rev(s_enc);
    double  umc = (double)upr * 1000.0 / (double)cpr;

    portENTER_CRITICAL(&s_lock);
    s_um_per_count = umc;
    portEXIT_CRITICAL(&s_lock);
}

/* Settings notification — only units_per_rev changes the counts → mm scale. */
static void on_settings_change(int idx, float new_value)
{
    (void)new_value;
    if (idx == SET_MM_PER_REV) {
        refresh_scale();
        ESP_LOGI(TAG, "units_per_rev changed — scale refreshed (%.6f µm/count)",
                 (double)settings_units_per_rev() * 1000.0
                     / (double)magnetic_encoder_counts_per_rev(s_enc));
    }
}

void cartpos_init(magnetic_encoder_handle_t enc)
{
    s_enc = enc;
    refresh_scale();
    if (!settings_subscribe(on_settings_change)) {
        ESP_LOGW(TAG, "settings 订阅失败（槽满）——每转行程变更不会生效");
    }
}

/* 无偏移原始位置（µm） */
static long raw_um(int64_t counts)
{
    double umc;
    portENTER_CRITICAL(&s_lock);
    umc = s_um_per_count;
    portEXIT_CRITICAL(&s_lock);
    return (long)llround((double)counts * umc);
}

int64_t cartpos_position_um(void)
{
    portENTER_CRITICAL(&s_lock);
    long offset = s_offset_um;
    portEXIT_CRITICAL(&s_lock);
    return raw_um(magnetic_encoder_position_counts(s_enc)) - offset;
}

float cartpos_velocity_mm_s(void)
{
    double umc;
    portENTER_CRITICAL(&s_lock);
    umc = s_um_per_count;
    portEXIT_CRITICAL(&s_lock);
    return (float)((double)magnetic_encoder_velocity_counts_per_s(s_enc)
                   * umc / 1000.0);
}

bool cartpos_is_ok(void)
{
    return magnetic_encoder_is_ok(s_enc);
}

void cartpos_set_origin_mm(float new_pos_mm)
{
    long raw = raw_um(magnetic_encoder_position_counts(s_enc));
    long offset = raw - (long)llround((double)new_pos_mm * 1000.0);

    portENTER_CRITICAL(&s_lock);
    s_offset_um = offset;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "原点已重置：%.3f mm（原始 %.3f mm，偏移 %ld µm）",
             (double)new_pos_mm, (double)raw / 1000.0, offset);
}
