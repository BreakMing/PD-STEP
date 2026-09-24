/*
 * telemetry_format — STATE 行组帧纯函数（无 ESP 依赖，宿主可测）
 */

#include "telemetry.h"

#include <stdio.h>

int telemetry_format_state(char *buf, size_t len, int32_t x_um,
                           int32_t theta_urad, int32_t vx_mm_s,
                           int32_t vth_mrad_s)
{
    if (buf == NULL || len < 40) {
        return -1;
    }
    int n = snprintf(buf, len, "STATE %ld %ld %ld %ld",
                     (long)x_um, (long)theta_urad,
                     (long)vx_mm_s, (long)vth_mrad_s);
    if (n < 0 || (size_t)n >= len) {
        return -1;
    }
    return n;
}
