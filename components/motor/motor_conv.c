#include "motor_conv.h"

#include <math.h>

int64_t motor_conv_mmps_to_q16(long v_mm_s, double steps_per_mm)
{
    return (int64_t)llround((double)v_mm_s * steps_per_mm * 65536.0);
}

uint32_t motor_conv_accel_to_steps(long a_mm_s2, double steps_per_mm)
{
    return (uint32_t)llround((double)a_mm_s2 * steps_per_mm);
}

long motor_conv_steps_to_um(int32_t steps, double steps_per_mm)
{
    return (long)llround((double)steps * 1000.0 / steps_per_mm);
}

int32_t motor_conv_um_to_steps(long um, double steps_per_mm)
{
    return (int32_t)llround((double)um * steps_per_mm / 1000.0);
}
