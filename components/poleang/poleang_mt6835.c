/*
 * poleang_mt6835 — poleang 域接口的 MT6835 实现（链接期绑定）
 *
 * 标定（POLE_ZERO_URAD / POLE_INV，来自 cartparams 注册表）在读取时
 * 应用：theta 经 mt6835_raw_to_urad 纯函数换算，vth 符号随 POLE_INV
 * 翻转。参数写入瞬间 theta 与 vth 可能取到不同快照（两次独立读取
 * 注册表），200 Hz 下一帧自愈——与迁移前单快照的差异仅出现在参数
 * 写入的那一帧，接受。
 */
#include "poleang.h"
#include "poleang_mt6835.h"
#include "mt6835.h"
#include "mt6835_urad.h"
#include "params.h"

esp_err_t poleang_mt6835_init(const poleang_mt6835_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mt6835_config_t dev = {
        .host          = cfg->host,
        .clock_speed_hz = cfg->clock_speed_hz,
        .spics_io_num  = cfg->spics_io_num,
    };
    return mt6835_init(&dev);
}

int32_t poleang_theta_urad(void)
{
    const fw_params_t *p = fw_params();
    return mt6835_raw_to_urad(mt6835_last_raw(),
                              (int32_t)p->pole_zero_urad,
                              p->pole_inv != 0);
}

int32_t poleang_vth_mrad_s(void)
{
    const fw_params_t *p = fw_params();
    int32_t v = mt6835_vth_mrad_s();
    return (p->pole_inv != 0) ? -v : v;
}

bool poleang_is_ok(void)
{
    return mt6835_is_ok();
}
