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

uint8_t mt6835_crc8(uint32_t angle21, uint8_t status)
{
    uint32_t payload = (angle21 << 3) | (status & 0x07);
    uint8_t crc = 0x00;
    for (int bit = 23; bit >= 0; bit--) {
        uint8_t msb = (uint8_t)((crc & 0x80) ? 1 : 0);
        uint8_t in = (uint8_t)((payload >> bit) & 1);
        crc = (uint8_t)(crc << 1);
        if (msb ^ in) {
            crc ^= 0x07;
        }
    }
    return crc;
}
