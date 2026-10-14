#include "cartaxis_conv.h"

#include <math.h>

long cartaxis_mmps_to_steps(long v_mm_s, double steps_per_mm)
{
    return llround((double)v_mm_s * steps_per_mm);
}

uint32_t cartaxis_accel_to_steps(long a_mm_s2, double steps_per_mm)
{
    return (uint32_t)llround((double)a_mm_s2 * steps_per_mm);
}

long cartaxis_steps_to_um(int32_t steps, double steps_per_mm)
{
    return (long)llround((double)steps * 1000.0 / steps_per_mm);
}

int32_t cartaxis_um_to_steps(long um, double steps_per_mm)
{
    return (int32_t)llround((double)um * steps_per_mm / 1000.0);
}
