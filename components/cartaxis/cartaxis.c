/*
 * cartaxis — 小车直线轴解释实现
 *
 * $0 单一写者：订阅 $0/$1/$2——$1/$2 只影响换算系数（即读即算，
 * 即时生效）；$0 变更时静止则回调内立即「推细分→按新单位重定基
 * （µm 值不变）」，运动中先斜坡停并挂起，此后任意入口在静止时应用
 * （cartfsm 监督任务 1 kHz 调用读数，滞后 ≤ 数拍；DISABLED 必静止，
 * 回调内即完成）。细分写失败放弃本次并告警，链路恢复后重设 $0 恢复。
 *
 * 换算单一权威：steps_per_mm 用已写入芯片的细分档（s_applied_
 * microstep）与当前 $1/$2 派生。settings 的值提交先于通知到达
 * （settings.c：s_values[idx] 赋值后才 notify），$0 变更回调里
 * settings_steps_per_mm() 已是新档——直接使用会把旧档步数按新系数
 * 换算，µm 不保持；按 applied 档派生则挂起等待与写失败期间读数
 * 都保持旧档换算，应用完成后 µm 无跳变。
 */
#include "cartaxis.h"
#include "cartaxis_conv.h"

#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include "settings.h"
#include "params.h"

static const char *TAG = "cartaxis";

static motor_handle_t s_m;
static int s_applied_microstep;               /* 已写入芯片的细分档 */
static volatile bool s_pending_microstep = false;

/* apply 串行化：settings 回调（通讯任务上下文）与 cartaxis 入口
 * （监督/通讯任务）可并发到达；后到者直接返回，语义由先到者完成 */
static portMUX_TYPE s_apply_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_apply_busy = false;

static double spm(void)
{
    return (double)settings_full_steps() * (double)s_applied_microstep
           / (double)settings_units_per_rev();
}

static long sgn(void)
{
    return (fw_params()->x_inv != 0) ? -1L : 1L;
}

/* 无挂起检查的原始读数（apply_microstep_now 内部使用，防递归） */
static long raw_cmd_um(void)
{
    return sgn() * cartaxis_steps_to_um(motor_position_steps(s_m), spm());
}

static void apply_microstep_now(void)
{
    portENTER_CRITICAL(&s_apply_lock);
    if (s_apply_busy) {
        /* 并发到达的第二条 $0 变更挂起待补推（幂等：下次入口重记 µm
         * → 重推 settings 当前档 → µm 不变），不得静默丢弃 */
        s_pending_microstep = true;
        portEXIT_CRITICAL(&s_apply_lock);
        return;
    }
    s_apply_busy = true;
    portEXIT_CRITICAL(&s_apply_lock);

    s_pending_microstep = false;
    long um = raw_cmd_um();   /* 旧档换算（s_applied_microstep 尚未更新） */

    int ms = settings_microstep();
    esp_err_t err = motor_set_microstep(s_m, ms);
    if (err != ESP_OK) {
        /* 芯片档与换算一并保持旧值（applied 未更新）——读数仍自洽 */
        ESP_LOGE(TAG, "细分写入失败（%d）——保持旧档 %d，换算不变，"
                      "链路恢复后重设 $0 恢复", err, s_applied_microstep);
        portENTER_CRITICAL(&s_apply_lock);
        s_apply_busy = false;
        portEXIT_CRITICAL(&s_apply_lock);
        return;
    }
    s_applied_microstep = ms;
    cartaxis_set_position_um(um);   /* 静止下按新单位重定基，µm 值不变 */

    portENTER_CRITICAL(&s_apply_lock);
    s_apply_busy = false;
    portEXIT_CRITICAL(&s_apply_lock);
}

static void maybe_apply_pending(void)
{
    if (s_pending_microstep && motor_at_rest(s_m)) {
        apply_microstep_now();
    }
}

static void on_settings_change(int idx, float new_value)
{
    (void)new_value;
    if (idx != SET_MICROSTEP) {
        return;   /* $1/$2 影响换算系数，即读即算即时生效 */
    }
    if (motor_at_rest(s_m)) {
        apply_microstep_now();
    } else {
        motor_halt(s_m);
        s_pending_microstep = true;
    }
}

void cartaxis_init(motor_handle_t m)
{
    s_m = m;
    /* tmc2209 上电经 config（组装点从 settings 读取）已推同值 */
    s_applied_microstep = settings_microstep();
    ESP_LOGI(TAG, "细分档 %d 已生效，steps_per_mm=%.1f（换算按已生效档）",
             s_applied_microstep, spm());
    if (!settings_subscribe(on_settings_change)) {
        ESP_LOGW(TAG, "settings 订阅失败（槽满）——$0 变更不会生效");
    }
}

esp_err_t cartaxis_enable(bool en)
{
    return motor_enable(s_m, en);
}

void cartaxis_set_vel_mm_s(long v_mm_s)
{
    maybe_apply_pending();
    motor_set_vel(s_m, sgn() * cartaxis_mmps_to_steps(v_mm_s, spm()));
}

void cartaxis_halt(void)
{
    maybe_apply_pending();
    motor_halt(s_m);
}

void cartaxis_stop(void)
{
    motor_stop(s_m);
}

bool cartaxis_at_rest(void)
{
    maybe_apply_pending();
    return motor_at_rest(s_m);
}

int64_t cartaxis_cmd_position_um(void)
{
    maybe_apply_pending();
    return raw_cmd_um();
}

void cartaxis_set_position_um(long um)
{
    motor_set_position(s_m,
                       (int32_t)(sgn() * cartaxis_um_to_steps(um, spm())));
}

void cartaxis_set_accel_mm_s2(long a_mm_s2)
{
    motor_set_accel(s_m, cartaxis_accel_to_steps(a_mm_s2, spm()));
}

void cartaxis_set_decel_mm_s2(long d_mm_s2)
{
    motor_set_decel(s_m, cartaxis_accel_to_steps(d_mm_s2, spm()));
}

void cartaxis_set_wdt_ms(long t_ms)
{
    motor_set_wdt_ms(s_m, (uint32_t)t_ms);
}
