/*
 * poleang — 摆角解释实现：encoder 角量 → 已标定摆角
 *
 * 零位与符号（POLE_ZERO_URAD / POLE_INV，来自 cartparams 注册表）
 * 在读取时应用。µrad 换算的整数算术与原 mt6835_raw_to_urad 逐位
 * 一致（u64 中间量、π = 3141592 / 2π = 6283185、±π 环绕、wrap 后
 * 符号翻转）；单圈角 = 多圈 counts 对每转计数取模（与原 last_raw
 * 同值，pos ≡ raw (mod cpr)）。参数写入瞬间 theta 与 vth 可能取到
 * 不同快照，200 Hz 下一帧自愈——与迁移前一致，接受。
 */
#include "poleang.h"

#include <math.h>

#include "magnetic_encoder.h"
#include "params.h"

/* floor(π × 1e6) / floor(2π × 1e6)，与 mt6835_urad 同约定 */
#define POLEANG_PI_URAD   3141592
#define POLEANG_2PI_URAD  6283185

static magnetic_encoder_handle_t s_enc;

void poleang_init(magnetic_encoder_handle_t enc)
{
    s_enc = enc;
}

int32_t poleang_theta_urad(void)
{
    const fw_params_t *p = fw_params();
    int32_t cpr = magnetic_encoder_counts_per_rev(s_enc);
    int64_t counts = magnetic_encoder_position_counts(s_enc);
    uint32_t raw_mod = (uint32_t)(((counts % cpr) + cpr) % cpr);

    int64_t a = (int64_t)((uint64_t)raw_mod * (uint64_t)POLEANG_2PI_URAD
                          / (uint64_t)cpr);
    a -= p->pole_zero_urad;

    while (a > POLEANG_PI_URAD) {
        a -= POLEANG_2PI_URAD;
    }
    while (a < -POLEANG_PI_URAD) {
        a += POLEANG_2PI_URAD;
    }

    return (p->pole_inv != 0) ? (int32_t)(-a) : (int32_t)a;
}

int32_t poleang_vth_mrad_s(void)
{
    const fw_params_t *p = fw_params();
    int32_t cpr = magnetic_encoder_counts_per_rev(s_enc);
    int32_t cps = magnetic_encoder_velocity_counts_per_s(s_enc);
    int32_t v = (int32_t)llround((double)cps * (double)POLEANG_2PI_URAD
                                 / (double)cpr / 1000.0);
    return (p->pole_inv != 0) ? -v : v;
}

bool poleang_is_ok(void)
{
    return magnetic_encoder_is_ok(s_enc);
}
