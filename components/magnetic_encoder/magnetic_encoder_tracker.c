/*
 * magnetic_encoder_tracker — 参数化多圈累计实现（契约见同名头）
 *
 * 语义自 as5600_tracker.c 上提参数化（counts_per_rev 为参数），
 * 阈值非对称式与全值域覆盖论证见头文件；MT6835 原对称式
 * unwrap_diff 的差异仅在物理不可达的混叠区（>cpr/2 counts/样本）。
 */

#include "magnetic_encoder_tracker.h"

#include <string.h>

void magnetic_encoder_tracker_reset(magnetic_encoder_tracker_t *t)
{
    if (t == NULL) {
        return;
    }
    memset(t, 0, sizeof(*t));
}

int64_t magnetic_encoder_tracker_update(magnetic_encoder_tracker_t *t,
                                        uint32_t raw, int32_t cpr)
{
    if (t == NULL) {
        return 0;
    }
    if (cpr <= 0) {
        cpr = 1;
    }
    raw %= (uint32_t)cpr;

    if (!t->seeded) {
        /* 首样本：在此定义原点，不产生位移 */
        t->last_raw   = raw;
        t->seeded     = true;
        t->pos_counts = 0;
        return 0;
    }

    int32_t diff = (int32_t)raw - (int32_t)t->last_raw;

    if (diff > cpr / 2) {
        diff -= cpr;        /* 反向越过 0/cpr−1 边界 */
    } else if (diff < -(cpr / 2 - 1)) {
        diff += cpr;        /* 正向越过 0/cpr−1 边界 */
    }

    t->pos_counts += diff;
    t->last_raw    = raw;

    return t->pos_counts;
}
