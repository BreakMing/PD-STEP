/*
 * AS5600 multi-turn tracker — pure logic, no hardware dependency
 *
 * Kept in its own translation unit so the counting rules can be exercised
 * on-target without touching I2C (see as5600_self_test.c group A).
 *
 * The AS5600 reports an absolute single-turn angle (0..4095).  Because
 * the magnet is mounted on the motor shaft (direct drive, 40 mm/rev),
 * a continuous position needs unwrapping across the 0/4095 edge:
 *
 *   diff = raw − last_raw
 *   diff >  +2048 → diff −= 4096   (travelled backwards over the edge)
 *   diff <  −2047 → diff += 4096   (travelled forwards over the edge)
 *
 * Threshold analysis
 * ------------------
 * The two thresholds are deliberately asymmetric (+2048 / −2047) and
 * together cover all 4096 possible diff values exactly once:
 *   raw diffs −2047..+2048 (4096 values) pass through unchanged — this
 *   is the largest interval in which the direction is unambiguous;
 *   diff = ±4095..±2049 are interpreted as a small step the other way.
 * With a symmetric rule one of the two extreme cases would be dropped
 * or counted twice.
 *
 * Noise behaviour at the edge
 * ---------------------------
 * dithering 4095 ↔ 0 after correction yields diff = +1 / −1 instead of
 * ±4095, so no extra dead-band is needed to meet the "≤2 counts at
 * standstill" criterion (acceptance record: docs/reviews/T8-acceptance.md)
 * — the edge dither degrades
 * naturally to ±1 count, same order as the sensor's own LSB noise.
 *
 * Speed ceiling
 * -------------
 * The largest reliably resolvable step per sample is ±2048 counts (half
 * a turn); anything faster is misread as a step the other way.  At the
 * 1 kHz sampling rate that is 2,048,000 counts/s = 500 rev/s =
 * 30,000 rpm.  The design requirement (">4500 rpm still has margin") is
 * 75 rev/s = 307 counts/ms, i.e. 15% of the ceiling — 6.7× margin.
 */

#include "as5600.h"

#include <string.h>

#define AS5600_RAW_MASK          0x0FFF
#define AS5600_HALF_TURN         2048
#define AS5600_FULL_TURN         4096

void as5600_tracker_reset(as5600_tracker_t *t)
{
    if (t == NULL) {
        return;
    }
    memset(t, 0, sizeof(*t));
}

int32_t as5600_tracker_update(as5600_tracker_t *t, uint16_t raw)
{
    if (t == NULL) {
        return 0;
    }

    raw = (uint16_t)(raw & AS5600_RAW_MASK);

    if (!t->seeded) {
        /* First sample: define the origin here, produce no displacement. */
        t->last_raw   = raw;
        t->seeded     = true;
        t->pos_counts = 0;
        return 0;
    }

    int32_t diff = (int32_t)raw - (int32_t)t->last_raw;

    if (diff > AS5600_HALF_TURN) {
        diff -= AS5600_FULL_TURN;        /* backwards across 0/4095 */
    } else if (diff < -(AS5600_HALF_TURN - 1)) {
        diff += AS5600_FULL_TURN;        /* forwards across 0/4095 */
    }

    t->pos_counts += diff;
    t->last_raw    = raw;

    return t->pos_counts;
}
