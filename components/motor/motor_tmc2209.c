/*
 * motor_tmc2209 — motor 域接口的 TMC2209 + GPTimer 实现（链接期绑定）
 */
#include "motor.h"
#include "motor_tmc2209.h"
#include "motor_conv.h"

#include "tmc2209.h"
#include "step_engine.h"
#include "settings.h"

/* 换算即时生效：每次调用读当前派生值（$0/$1/$2 变更后即用新值） */
static double spm(void)
{
    return (double)settings_steps_per_mm();
}

esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = tmc2209_init(&cfg->tmc);
    if (err != ESP_OK) {
        return err;
    }
    return step_engine_init(cfg->step_gpio, cfg->dir_gpio);
}

esp_err_t motor_enable(bool en)
{
    return tmc2209_enable(en);
}

void motor_set_vel_mm_s(long v_mm_s)
{
    step_engine_vel_target(motor_conv_mmps_to_q16(v_mm_s, spm()));
}

void motor_halt(void)
{
    step_engine_vel_halt();
}

void motor_stop(void)
{
    step_engine_stop();
}

bool motor_at_rest(void)
{
    /* 精确判零：不经 mm/s 整数舍入（残余速度不得提前判停） */
    return step_engine_vel_now_q16() == 0;
}

int64_t motor_cmd_position_um(void)
{
    return motor_conv_steps_to_um(step_engine_position_steps(), spm());
}

void motor_set_position_um(long um)
{
    step_engine_set_position(motor_conv_um_to_steps(um, spm()));
}

void motor_set_accel_mm_s2(long a_mm_s2)
{
    step_engine_vel_set_accel(motor_conv_accel_to_steps(a_mm_s2, spm()));
}

void motor_set_decel_mm_s2(long d_mm_s2)
{
    step_engine_vel_set_decel(motor_conv_accel_to_steps(d_mm_s2, spm()));
}

void motor_set_wdt_ms(long t_ms)
{
    step_engine_vel_set_wdt_us((uint32_t)t_ms * 1000u);
}

void motor_set_dir_invert(bool invert)
{
    step_engine_set_dir_invert(invert);
}
