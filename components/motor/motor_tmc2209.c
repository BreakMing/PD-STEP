/*
 * motor_tmc2209 — motor 契约的 TMC2209 + GPTimer 实现（单实例）
 *
 * steps 域直通 step_engine（速度目标以 steps/s×65536 的定点值下发，
 * 整数 steps/s 恰为定点值的 65536 倍，无损）；细分与电流经 tmc2209
 * 功能层（含 VSENSE 分档）。
 */
#include "motor.h"
#include "motor_tmc2209.h"

#include "tmc2209.h"
#include "step_engine.h"

static struct motor_s {
    int unused;   /* 单实例句柄载体 */
} s_inst;

esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg,
                             motor_handle_t *handle_out)
{
    if (cfg == NULL || handle_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = tmc2209_init(&cfg->tmc);
    if (err != ESP_OK) {
        return err;
    }
    err = step_engine_init(cfg->step_gpio, cfg->dir_gpio);
    if (err != ESP_OK) {
        return err;
    }
    *handle_out = &s_inst;
    return ESP_OK;
}

esp_err_t motor_enable(motor_handle_t m, bool en)
{
    (void)m;
    return tmc2209_enable(en);
}

void motor_set_vel(motor_handle_t m, long steps_per_s)
{
    (void)m;
    step_engine_vel_target((int64_t)steps_per_s * 65536);
}

void motor_halt(motor_handle_t m)
{
    (void)m;
    step_engine_vel_halt();
}

void motor_stop(motor_handle_t m)
{
    (void)m;
    step_engine_stop();
}

bool motor_at_rest(motor_handle_t m)
{
    /* 精确判零：引擎内部定点域（残余速度不得提前判停） */
    (void)m;
    return step_engine_vel_now_q16() == 0;
}

int32_t motor_position_steps(motor_handle_t m)
{
    (void)m;
    return step_engine_position_steps();
}

void motor_set_position(motor_handle_t m, int32_t steps)
{
    (void)m;
    step_engine_set_position(steps);
}

void motor_set_accel(motor_handle_t m, uint32_t steps_per_s2)
{
    (void)m;
    step_engine_vel_set_accel(steps_per_s2);
}

void motor_set_decel(motor_handle_t m, uint32_t steps_per_s2)
{
    (void)m;
    step_engine_vel_set_decel(steps_per_s2);
}

void motor_set_wdt_ms(motor_handle_t m, uint32_t ms)
{
    (void)m;
    step_engine_vel_set_wdt_us(ms * 1000u);
}

esp_err_t motor_set_microstep(motor_handle_t m, int microstep)
{
    (void)m;
    return tmc2209_set_microstep(microstep);
}

esp_err_t motor_set_current(motor_handle_t m, int run_ma, int hold_ma)
{
    (void)m;
    return tmc2209_set_current(run_ma, hold_ma);
}
