/*
 * mt6835_urad — 换算纯函数实现（契约见 mt6835_urad.h）
 */

#include "mt6835_urad.h"

int32_t mt6835_raw_to_urad(uint32_t raw21, int32_t zero_urad, bool invert)
{
    /* urad = raw × 2π/2^21，u64 中间量；减零位后 wrap 到 ±π */
    int64_t a = (int64_t)((uint64_t)raw21 * MT6835_2PI_URAD / MT6835_COUNTS_PER_REV);
    a -= zero_urad;

    while (a > MT6835_PI_URAD) {
        a -= MT6835_2PI_URAD;
    }
    while (a < -MT6835_PI_URAD) {
        a += MT6835_2PI_URAD;
    }

    if (invert) {
        a = -a;
    }
    return (int32_t)a;
}

int32_t mt6835_unwrap_diff(uint32_t a, uint32_t b)
{
    int32_t diff = (int32_t)(a - b);   /* 0..2^21−1 域内的无符号差 */
    if (diff > (int32_t)(MT6835_COUNTS_PER_REV / 2)) {
        diff -= (int32_t)MT6835_COUNTS_PER_REV;
    } else if (diff < -(int32_t)(MT6835_COUNTS_PER_REV / 2)) {
        diff += (int32_t)MT6835_COUNTS_PER_REV;
    }
    return diff;
}

int32_t mt6835_counts_to_mrad_s(int64_t delta_counts, uint32_t window_ms)
{
    if (window_ms == 0) {
        return 0;   /* 防御 */
    }
    /* µrad/ms 恰为 mrad/s */
    return (int32_t)(delta_counts * MT6835_2PI_URAD
                     / (int64_t)MT6835_COUNTS_PER_REV / (int64_t)window_ms);
}
