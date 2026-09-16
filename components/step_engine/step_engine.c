/*
 * Step Engine — GPTimer ISR stepper pulse generator
 *
 * Algorithm (PLAN-specified physical model):
 *   v += a * dt;   dt = 1 / v
 * Accel / cruise / decel phases switched by step-count boundaries
 * pre-computed at enqueue time.
 *
 * Fixed-point format: Q16.16 for velocity (v) and acceleration (a),
 * giving 0.000015 steps/s resolution and max ~65535 steps/s
 * (well above 50 kHz performance threshold).
 *
 * Timer: GPTimer @ 1 MHz resolution, one-shot alarm, ISR reloads
 * next interval.  The counter free-runs: a one-shot alarm disables
 * only the alarm (never the counter), and gptimer_start() does not
 * reset the counter.  Alarm targets are therefore RELATIVE — each
 * next target = target of the alarm that just fired + dt — with a
 * latency guard floor so a target can never land behind the
 * counter.  Intervals > MAX_ALARM_US are split into a chain of
 * ≤ MAX_ALARM_US alarms (chain-wait path for very low speeds).
 *
 * Queue: SPSC ring buffer of depth 16 (power of two).
 *   Task side: write data → memory barrier → update head
 *   ISR side:  read data  → process      → update tail
 * All on CPU0 (v1), so single-core SPSC semantics apply; compiler
 * barriers prevent instruction reordering.
 *
 * STEP pulse: HIGH → esp_rom_delay_us(1) → LOW (≥1 µs width).
 * DIR setup: when direction changes at segment boundary, DIR is
 * set first and 2 µs is added to the first step's dt (no busy-wait
 * in ISR — the extra timer time guarantees setup time).
 */

#include "step_engine.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "step_engine";

/* ================================================================
 * Configuration constants
 * ================================================================ */

#define QUEUE_DEPTH       16     /* must be power of 2 */
#define QUEUE_MASK        (QUEUE_DEPTH - 1)

#define TIMER_RESOLUTION_HZ  1000000  /* 1 MHz → 1 µs per tick */

/* Maximum single-shot alarm value (µs).  Chaining kicks in above
 * this.  Set to 60 ms to safely fit within a hypothetical 16-bit
 * timer @ 1 MHz (65535 µs = 65.5 ms); ESP32-S3 GPTimer is actually
 * 54-bit, but chaining is implemented per spec requirement. */
#define MAX_ALARM_US      60000

/* Minimum headroom (µs) between the counter value latched at ISR
 * entry and the next alarm target.  Covers callback execution
 * (STEP pulse + fixed-point math) between the latch and the alarm
 * register write, so the target can never land in the past. */
#define ALARM_GUARD_US    5

/* Minimum velocity clamp for deceleration end (steps/s).
 * Below this, dt would "explode" (grow very large).  Instead we
 * finish remaining decel steps at this constant low velocity.
 * 10 steps/s → dt = 100 ms, still manageable with chaining. */
#define V_MIN_STEPS_PER_S  10

/* Q16.16 fixed-point scaling */
#define Q16_SHIFT   16
#define Q16_ONE     (1U << Q16_SHIFT)   /* 65536 */

/* Convert float → Q16.16 (task context only) */
static inline uint32_t float_to_q16(float v)
{
    return (uint32_t)(v * (float)Q16_ONE + 0.5f);
}

/* Convert Q16.16 → approximate float (for debug/log only) */
static inline float q16_to_float(uint32_t q)
{
    return (float)q / (float)Q16_ONE;
}

/* ================================================================
 * Internal segment (fixed-point, stored in queue)
 * ================================================================ */

typedef struct {
    uint32_t total_steps;
    bool     dir;
    uint32_t accel_steps;
    uint32_t cruise_steps;
    uint32_t decel_steps;
    uint32_t v_cruise_q16;   /* Q16.16 cruise velocity */
    uint32_t a_q16;          /* Q16.16 acceleration */
    uint32_t dt0_us;         /* first step interval, µs (precomputed) */
} seg_internal_t;

/* ================================================================
 * SPSC ring queue
 * ================================================================ */

static seg_internal_t s_queue[QUEUE_DEPTH];
static volatile uint32_t s_queue_head = 0;  /* written by task */
static volatile uint32_t s_queue_tail = 0;  /* written by ISR */

/* ================================================================
 * ISR state
 * ================================================================ */

/* GPIO pins */
static int s_step_gpio = -1;
static int s_dir_gpio  = -1;

/* Direction inversion ($10) */
static volatile bool s_dir_invert = false;

/* Last output direction (for DIR setup-time detection) */
static bool s_last_out_dir = false;  /* physical pin state last set */

/* Current segment being executed */
static seg_internal_t s_cur_seg;
static uint32_t s_cur_step = 0;      /* steps completed so far in current seg */
static uint8_t  s_cur_phase = 0;     /* 0=accel, 1=cruise, 2=decel */
static uint32_t s_v_q16 = 0;         /* current velocity Q16.16 */
static bool s_has_cur_seg = false;

/* Chain-wait state: remaining µs after the current alarm fires.
 * >0 means the next alarm is a chain wait (no step generated). */
static uint32_t s_chain_remaining_us = 0;

/* Position counter (signed, direction-aware, includes $10 inversion) */
static volatile int32_t s_position = 0;

/* Stopped flag (set by stop(), checked first thing in ISR) */
static volatile bool s_stopped = true;

/* Timer handle */
static gptimer_handle_t s_timer = NULL;
static volatile bool s_timer_running = false;

/* Alarm scheduling base, refreshed at the top of every alarm ISR:
 *   s_cb_alarm_base   — target of the alarm that just fired (exact)
 *   s_cb_guard_floor  — latched counter + ALARM_GUARD_US (latency floor)
 * The enqueue kick-start path (timer stopped) seeds both from the
 * frozen raw counter. */
static uint64_t s_cb_alarm_base = 0;
static uint64_t s_cb_guard_floor = 0;

/* ISR spinlock for critical sections in task-context functions */
static portMUX_TYPE s_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* Init guard for idempotent step_engine_init() */
static bool s_initialized = false;

/* ================================================================
 * Self-test hooks (only active when CONFIG_STEP_ENGINE_SELF_TEST=y)
 * ================================================================ */

#ifdef CONFIG_STEP_ENGINE_SELF_TEST
static uint32_t *s_sample_buf = NULL;
static int       s_sample_max = 0;
static volatile int *s_sample_count = NULL;
static uint32_t  s_last_alarm_ccount = 0;  /* ccount at last alarm */

static volatile uint32_t s_max_isr_cycles = 0;
#endif /* CONFIG_STEP_ENGINE_SELF_TEST */

/* ================================================================
 * Forward declarations
 * ================================================================ */

static bool gptimer_alarm_cb(gptimer_handle_t timer,
                             const gptimer_alarm_event_data_t *edata,
                             void *user_ctx);
static void isr_start_next_step(void);
static bool try_pop_segment(void);
static void compute_first_step_dt(const seg_internal_t *seg, uint32_t *out_dt_us, uint32_t *out_v_q16);

/* ================================================================
 * Helper: apply direction inversion and set DIR pin
 * ================================================================ */
static inline void set_dir_physical(bool logical_dir)
{
    bool physical = logical_dir ^ s_dir_invert;
    gpio_set_level((gpio_num_t)s_dir_gpio, physical ? 1 : 0);
    s_last_out_dir = physical;
}

/* ================================================================
 * Helper: check if direction (physical) changes for next segment
 * ================================================================ */
static inline bool dir_changes(bool next_logical_dir)
{
    bool next_phys = next_logical_dir ^ s_dir_invert;
    return (next_phys != s_last_out_dir);
}

/* ================================================================
 * Queue operations (SPSC)
 * ================================================================ */

/* Task-side enqueue.  Returns false if full. */
static bool queue_push(const seg_internal_t *seg)
{
    uint32_t next_head = (s_queue_head + 1) & QUEUE_MASK;
    if (next_head == s_queue_tail) {
        return false;  /* full */
    }
    s_queue[s_queue_head] = *seg;
    /* Compiler barrier: ensure data write is visible before head update.
     * On single-core (CPU0) where ISR preempts task, this is sufficient
     * because the ISR return acts as a full hardware barrier. */
    asm volatile("" ::: "memory");
    s_queue_head = next_head;
    return true;
}

/* ISR-side pop.  Returns false if empty. */
static bool queue_pop(seg_internal_t *out_seg)
{
    if (s_queue_head == s_queue_tail) {
        return false;  /* empty */
    }
    *out_seg = s_queue[s_queue_tail];
    asm volatile("" ::: "memory");
    s_queue_tail = (s_queue_tail + 1) & QUEUE_MASK;
    return true;
}

/* ================================================================
 * Compute the first step dt and initial velocity update.
 *
 * For the first step from v=0: dt0 = sqrt(2/a) seconds
 *   → dt0_us = sqrt(2 / a) * 1000000
 * After first step: v1 = a * dt0 = sqrt(2*a)
 *
 * Called from ISR context when loading a new segment (first step).
 * Uses pre-computed dt0_us from enqueue, but also computes the
 * velocity after first step.
 * ================================================================ */
static void compute_first_step_dt(const seg_internal_t *seg,
                                  uint32_t *out_dt_us,
                                  uint32_t *out_v_q16)
{
    *out_dt_us = seg->dt0_us;
    /* v = a * dt, where dt in seconds = dt0_us / 1000000
     * v_q16 = a_q16 * dt0_us / 1000000 */
    uint64_t v_q16_64 = (uint64_t)seg->a_q16 * seg->dt0_us / 1000000ULL;
    if (v_q16_64 > 0xFFFFFFFFULL) v_q16_64 = 0xFFFFFFFFULL;
    *out_v_q16 = (uint32_t)v_q16_64;
}

/* ================================================================
 * Arm a one-shot alarm at (base + dt), never behind floor.
 * Pure register scheduling — does not touch chain state; callers
 * manage s_chain_remaining_us.
 * Called from ISR context and from the enqueue kick-start path.
 * ================================================================ */
static void arm_alarm(uint64_t base, uint64_t floor, uint32_t dt_us)
{
    uint64_t target = base + dt_us;
    if (target < floor) {
        target = floor;  /* latency guard: keep target in the future */
    }

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = target,
        .flags = { .auto_reload_on_alarm = false },
    };
    gptimer_set_alarm_action(s_timer, &alarm_cfg);
    gptimer_start(s_timer);  /* no-op if already running */
    s_timer_running = true;
}

/* ================================================================
 * Schedule the next step interval relative to the alarm that just
 * fired (s_cb_alarm_base).  Intervals > MAX_ALARM_US are split
 * into a chain: first ≤ MAX_ALARM_US chunk armed now, remainder
 * re-armed from subsequent chain alarms.
 * Called from ISR context (and the enqueue kick-start path, where
 * s_cb_* are seeded from the frozen counter beforehand).
 * ================================================================ */
static void isr_set_alarm(uint32_t dt_us)
{
    if (dt_us > MAX_ALARM_US) {
        s_chain_remaining_us = dt_us - MAX_ALARM_US;
        arm_alarm(s_cb_alarm_base, s_cb_guard_floor, MAX_ALARM_US);
    } else {
        s_chain_remaining_us = 0;
        arm_alarm(s_cb_alarm_base, s_cb_guard_floor, dt_us);
    }
}

/* ================================================================
 * Try to load the next segment from the queue.
 *
 * Returns true if a new segment was loaded and the first step
 * alarm has been set up.  Returns false if queue is empty.
 * Called from ISR context.
 * ================================================================ */
static bool try_pop_segment(void)
{
    seg_internal_t seg;
    if (!queue_pop(&seg)) {
        return false;
    }

    s_cur_seg = seg;
    s_cur_step = 0;
    s_has_cur_seg = true;

    /* Determine initial phase */
    if (seg.accel_steps > 0) {
        s_cur_phase = 0;  /* accel */
        s_v_q16 = 0;
    } else if (seg.cruise_steps > 0) {
        s_cur_phase = 1;  /* cruise */
        s_v_q16 = seg.v_cruise_q16;
    } else {
        s_cur_phase = 2;  /* decel only (shouldn't normally happen) */
        s_v_q16 = seg.v_cruise_q16;
    }

    /* Handle direction change */
    uint32_t extra_setup_us = 0;
    if (dir_changes(seg.dir)) {
        set_dir_physical(seg.dir);
        extra_setup_us = 2;  /* DIR setup time, added to first step dt */
    }

    /* Compute first step interval */
    uint32_t dt_us;
    if (s_cur_phase == 1) {
        /* Starting at cruise (no accel phase) */
        uint64_t num = 1000000ULL * Q16_ONE;
        dt_us = (uint32_t)(num / seg.v_cruise_q16);
    } else if (s_cur_phase == 0) {
        /* Starting from standstill: use precomputed dt0 */
        uint32_t v_after_q16;
        compute_first_step_dt(&seg, &dt_us, &v_after_q16);
        /* v will be updated after the first step fires, but we need
         * to compute v after step 0→1.  We set s_v_q16 to 0 now
         * and update after step is generated. */
        s_v_q16 = 0;
    } else {
        /* Decel-only: start at cruise velocity */
        s_v_q16 = seg.v_cruise_q16;
        uint64_t num = 1000000ULL * Q16_ONE;
        dt_us = (uint32_t)(num / s_v_q16);
    }

    dt_us += extra_setup_us;

    /* Set up the timer for the first step */
    isr_set_alarm(dt_us);

    return true;
}

/* ================================================================
 * ISR: start next step (compute dt, set alarm).
 * Called after a step has been generated and counters updated.
 * Determines phase and sets up the next alarm.
 * ================================================================ */
static void isr_start_next_step(void)
{
    /* Segment complete (all its steps pulsed): chain directly to
     * the next segment — no wasted inter-segment alarm. */
    if (s_has_cur_seg && s_cur_step >= s_cur_seg.total_steps) {
        s_has_cur_seg = false;
    }

    if (!s_has_cur_seg) {
        /* No current segment — try to pop one */
        if (!try_pop_segment()) {
            /* Queue empty, stop timer */
            gptimer_stop(s_timer);
            s_timer_running = false;
            s_stopped = true;
            return;
        }
        return;  /* try_pop_segment already set up the alarm */
    }

    /* Check for phase transition based on completed steps */
    uint32_t steps_done = s_cur_step;

    /* After step n is done, compute dt to step n+1.
     * Phase for step n+1 depends on steps completed (s_cur_step). */
    if (steps_done < s_cur_seg.accel_steps) {
        s_cur_phase = 0;  /* still accelerating */
    } else if (steps_done < s_cur_seg.accel_steps + s_cur_seg.cruise_steps) {
        if (s_cur_phase != 1) {
            /* Just entered cruise — snap velocity to v_cruise */
            s_cur_phase = 1;
            s_v_q16 = s_cur_seg.v_cruise_q16;
        }
    } else {
        if (s_cur_phase != 2) {
            /* Just entered decel — snap velocity to v_cruise */
            s_cur_phase = 2;
            s_v_q16 = s_cur_seg.v_cruise_q16;
        }
    }

    /* Compute next dt and update velocity */
    uint32_t dt_us;

    if (s_cur_phase == 1) {
        /* Cruise: constant velocity */
        uint64_t num = 1000000ULL * Q16_ONE;
        dt_us = (uint32_t)(num / s_v_q16);
    } else {
        /* Accel or decel: compute dt from current v, then update v */

        if (s_v_q16 == 0 && s_cur_phase == 0) {
            /* First step from standstill just completed.
             * dt0 was the interval step 0→1 (already elapsed).
             * v_1 = a * dt0 (velocity after step 1). */
            uint64_t v1_q16 = (uint64_t)s_cur_seg.a_q16 * s_cur_seg.dt0_us / 1000000ULL;
            if (v1_q16 > 0xFFFFFFFFULL) v1_q16 = 0xFFFFFFFFULL;
            s_v_q16 = (uint32_t)v1_q16;
            /* Fall through: normal path computes dt_1 = 1/v_1 and v_2. */
        }

        /* Compute dt from current velocity (for next step interval) */
        uint64_t num = 1000000ULL * Q16_ONE;
        dt_us = (uint32_t)(num / s_v_q16);

        /* Update velocity: v_{n+1} = v_n + a * dt_n */
        uint64_t delta_q16 = (uint64_t)s_cur_seg.a_q16 * dt_us / 1000000ULL;

        if (s_cur_phase == 0) {
            /* Acceleration */
            uint64_t new_v = (uint64_t)s_v_q16 + delta_q16;
            if (new_v > 0xFFFFFFFFULL) new_v = 0xFFFFFFFFULL;
            s_v_q16 = (uint32_t)new_v;
            if (s_v_q16 > s_cur_seg.v_cruise_q16) {
                s_v_q16 = s_cur_seg.v_cruise_q16;
            }
        } else {
            /* Deceleration */
            if (delta_q16 >= s_v_q16) {
                s_v_q16 = 0;
            } else {
                s_v_q16 -= (uint32_t)delta_q16;
            }
            uint32_t v_min_q16 = V_MIN_STEPS_PER_S * Q16_ONE;
            if (s_v_q16 < v_min_q16) {
                s_v_q16 = v_min_q16;
            }
        }
    }

    /* Set up the alarm */
    isr_set_alarm(dt_us);
}

/* ================================================================
 * GPTimer alarm ISR callback
 * ================================================================ */

static bool gptimer_alarm_cb(gptimer_handle_t timer,
                             const gptimer_alarm_event_data_t *edata,
                             void *user_ctx)
{
    (void)timer;
    (void)user_ctx;

    /* Refresh the alarm scheduling base: alarm_value is the exact
     * counter value this alarm fired at; count_value was latched at
     * ISR entry and bounds how far behind the counter we may aim. */
    s_cb_alarm_base = edata->alarm_value;
    s_cb_guard_floor = edata->count_value + ALARM_GUARD_US;

#ifdef CONFIG_STEP_ENGINE_SELF_TEST
    uint32_t isr_start_ccount = esp_cpu_get_cycle_count();
#endif

    /* Check stop flag FIRST — if stopped, shut down and return */
    if (s_stopped) {
        gptimer_stop(s_timer);
        s_timer_running = false;
        return false;  /* no task yield needed */
    }

    /* Handle chain-wait: if remaining > 0, this was a chain alarm
     * (no step generated).  Re-arm the next chain chunk and return. */
    if (s_chain_remaining_us > 0) {
        uint32_t chunk = (s_chain_remaining_us > MAX_ALARM_US)
                       ? MAX_ALARM_US : s_chain_remaining_us;
        s_chain_remaining_us -= chunk;
        arm_alarm(s_cb_alarm_base, s_cb_guard_floor, chunk);
#ifdef CONFIG_STEP_ENGINE_SELF_TEST
        uint32_t isr_end_ccount = esp_cpu_get_cycle_count();
        uint32_t cycles = isr_end_ccount - isr_start_ccount;
        if (cycles > s_max_isr_cycles) {
            s_max_isr_cycles = cycles;
        }
#endif
        return false;
    }

    /* ---- Generate STEP pulse ---- */

    /* Defensive: normally unreachable — isr_start_next_step chains
     * directly to the next segment on completion.  Kept in case an
     * alarm ever fires with the segment already complete. */
    if (!s_has_cur_seg || s_cur_step >= s_cur_seg.total_steps) {
        s_has_cur_seg = false;
        if (try_pop_segment()) {
            /* New segment loaded; its first-step alarm is armed
             * relative to the alarm that just fired.  No step pulse
             * is generated in this invocation. */
#ifdef CONFIG_STEP_ENGINE_SELF_TEST
            uint32_t isr_end_ccount = esp_cpu_get_cycle_count();
            uint32_t cycles = isr_end_ccount - isr_start_ccount;
            if (cycles > s_max_isr_cycles) {
                s_max_isr_cycles = cycles;
            }
            /* Sample: interval between alarm arrivals */
            if (s_sample_buf && s_sample_count && *s_sample_count < s_sample_max) {
                uint32_t now = esp_cpu_get_cycle_count();
                if (s_last_alarm_ccount != 0) {
                    /* Convert cycles → µs (240 MHz → divide by 240) */
                    uint32_t delta_cc = now - s_last_alarm_ccount;
                    s_sample_buf[*s_sample_count] = delta_cc / 240;
                    (*s_sample_count)++;
                }
                s_last_alarm_ccount = now;
            }
#endif
            return false;
        } else {
            /* No more segments — stop */
            gptimer_stop(s_timer);
            s_timer_running = false;
            s_stopped = true;
#ifdef CONFIG_STEP_ENGINE_SELF_TEST
            uint32_t isr_end_ccount = esp_cpu_get_cycle_count();
            uint32_t cycles = isr_end_ccount - isr_start_ccount;
            if (cycles > s_max_isr_cycles) {
                s_max_isr_cycles = cycles;
            }
#endif
            return false;
        }
    }

    /* Generate the STEP pulse */
    gpio_set_level((gpio_num_t)s_step_gpio, 1);
    esp_rom_delay_us(1);
    gpio_set_level((gpio_num_t)s_step_gpio, 0);

    /* Update position counter.
     * Position sign follows the PHYSICAL direction on the DIR pin
     * (logical ^ $10 inversion) — same mapping as set_dir_physical(),
     * so DIR HIGH = positive. */
    bool forward = s_cur_seg.dir ^ s_dir_invert;
    if (forward) {
        s_position++;
    } else {
        s_position--;
    }

    /* Increment step counter */
    s_cur_step++;

#ifdef CONFIG_STEP_ENGINE_SELF_TEST
    /* Sample: interval between step-generating alarms */
    if (s_sample_buf && s_sample_count && *s_sample_count < s_sample_max) {
        uint32_t now = esp_cpu_get_cycle_count();
        if (s_last_alarm_ccount != 0) {
            uint32_t delta_cc = now - s_last_alarm_ccount;
            s_sample_buf[*s_sample_count] = delta_cc / 240;  /* cycles → µs */
            (*s_sample_count)++;
        }
        s_last_alarm_ccount = now;
    }
#endif

    /* Set up next step (or next segment) */
    isr_start_next_step();

#ifdef CONFIG_STEP_ENGINE_SELF_TEST
    uint32_t isr_end_ccount = esp_cpu_get_cycle_count();
    uint32_t cycles = isr_end_ccount - isr_start_ccount;
    if (cycles > s_max_isr_cycles) {
        s_max_isr_cycles = cycles;
    }
#endif

    return false;  /* no task yield needed from ISR */
}

/* ================================================================
 * Public API implementations
 * ================================================================ */

esp_err_t step_engine_init(int step_gpio, int dir_gpio)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "step_engine already initialized, skipping");
        return ESP_OK;
    }

    s_step_gpio = step_gpio;
    s_dir_gpio  = dir_gpio;

    /* Initialize state */
    s_queue_head = 0;
    s_queue_tail = 0;
    s_position = 0;
    s_stopped = true;
    s_timer_running = false;
    s_has_cur_seg = false;
    s_cur_step = 0;
    s_v_q16 = 0;
    s_chain_remaining_us = 0;

    /* Configure GPTimer */
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,  /* 1 MHz */
    };
    esp_err_t err = gptimer_new_timer(&timer_cfg, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_new_timer failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Register alarm callback */
    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_alarm_cb,
    };
    err = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_register_event_callbacks failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Enable timer */
    err = gptimer_enable(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_enable failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Step engine initialized (STEP=GPIO%d, DIR=GPIO%d, timer @ 1MHz)",
             step_gpio, dir_gpio);

    s_initialized = true;
    return ESP_OK;
}

bool step_engine_enqueue(const step_seg_t *seg)
{
    if (seg == NULL) return false;
    if (seg->steps == 0) return false;
    if (seg->accel_steps + seg->cruise_steps + seg->decel_steps != seg->steps) {
        return false;  /* boundary validation */
    }
    if (seg->accel <= 0.0f || seg->v_cruise <= 0.0f) return false;

    /* Convert to internal fixed-point format */
    seg_internal_t iseg;
    iseg.total_steps  = seg->steps;
    iseg.dir          = seg->dir;
    iseg.accel_steps  = seg->accel_steps;
    iseg.cruise_steps = seg->cruise_steps;
    iseg.decel_steps  = seg->decel_steps;
    iseg.v_cruise_q16 = float_to_q16(seg->v_cruise);
    iseg.a_q16        = float_to_q16(seg->accel);

    /* Pre-compute first step dt: dt0 = sqrt(2 / a) seconds
     * dt0_us = sqrt(2 / a) * 1e6 */
    float dt0_s = sqrtf(2.0f / seg->accel);
    float dt0_us_f = dt0_s * 1000000.0f;
    if (dt0_us_f > 60000000.0f) {
        /* Cap to avoid overflow; chaining will handle it */
        dt0_us_f = 60000000.0f;
    }
    iseg.dt0_us = (uint32_t)(dt0_us_f + 0.5f);
    if (iseg.dt0_us < 1) iseg.dt0_us = 1;

    /* Push to queue (critical section to protect against ISR race) */
    bool ok;
    portENTER_CRITICAL(&s_spinlock);
    ok = queue_push(&iseg);
    portEXIT_CRITICAL(&s_spinlock);

    if (!ok) {
        return false;
    }

    /* If timer is stopped and we were idle, kick-start it */
    portENTER_CRITICAL(&s_spinlock);
    if (s_stopped && !s_timer_running && s_has_cur_seg == false) {
        /* Try to load the first segment and start the timer.
         * We can't call try_pop_segment directly from here because
         * it's ISR-side logic.  Instead, clear stopped flag and
         * set up the initial state, then let the ISR start.
         *
         * Actually simpler: just set s_stopped = false and if the
         * timer isn't running, manually pop the first segment and
         * set up its first alarm. */
        s_stopped = false;

        if (!s_has_cur_seg) {
            /* Pop first segment and set up initial alarm */
            seg_internal_t first_seg;
            if (queue_pop(&first_seg)) {
                s_cur_seg = first_seg;
                s_cur_step = 0;
                s_has_cur_seg = true;

                /* Determine initial phase */
                if (first_seg.accel_steps > 0) {
                    s_cur_phase = 0;
                    s_v_q16 = 0;
                } else if (first_seg.cruise_steps > 0) {
                    s_cur_phase = 1;
                    s_v_q16 = first_seg.v_cruise_q16;
                } else {
                    s_cur_phase = 2;
                    s_v_q16 = first_seg.v_cruise_q16;
                }

                /* Direction */
                uint32_t extra_setup_us = 0;
                if (dir_changes(first_seg.dir)) {
                    set_dir_physical(first_seg.dir);
                    extra_setup_us = 2;
                }

                /* Compute first dt */
                uint32_t dt_us;
                if (s_cur_phase == 0) {
                    dt_us = first_seg.dt0_us;
                } else {
                    uint64_t num = 1000000ULL * Q16_ONE;
                    dt_us = (uint32_t)(num / s_v_q16);
                }
                dt_us += extra_setup_us;

                /* Arm the first-step alarm relative to the frozen
                 * counter (the timer is stopped here, so the raw
                 * count read is exact).  The counter free-runs from
                 * this base for the rest of the motion. */
                uint64_t now_count = 0;
                gptimer_get_raw_count(s_timer, &now_count);
                s_cb_alarm_base = now_count;
                s_cb_guard_floor = now_count + ALARM_GUARD_US;
                isr_set_alarm(dt_us);
            }
        }
    }
    portEXIT_CRITICAL(&s_spinlock);

    return true;
}

void step_engine_stop(void)
{
    portENTER_CRITICAL(&s_spinlock);

    s_stopped = true;

    /* Clear queue */
    s_queue_head = s_queue_tail = 0;

    /* Stop the timer */
    if (s_timer != NULL && s_timer_running) {
        gptimer_stop(s_timer);
        s_timer_running = false;
    }

    /* Clear current segment state */
    s_has_cur_seg = false;
    s_chain_remaining_us = 0;

    portEXIT_CRITICAL(&s_spinlock);
}

int32_t step_engine_position_steps(void)
{
    /* 32-bit aligned read is atomic on Xtensa */
    return s_position;
}

uint32_t step_engine_velocity_steps(void)
{
    /* Approximate: convert Q16.16 → integer steps/s */
    uint32_t v_q16 = s_v_q16;
    if (!s_timer_running && s_stopped) {
        return 0;
    }
    return v_q16 >> Q16_SHIFT;
}

bool step_engine_busy(void)
{
    bool running = s_timer_running;
    bool queue_nonempty = (s_queue_head != s_queue_tail);
    return running || queue_nonempty || s_has_cur_seg;
}

void step_engine_set_dir_invert(bool invert)
{
    /* Takes effect on next segment; current segment unaffected */
    s_dir_invert = invert;
}

/* ================================================================
 * Self-test hook implementations
 * ================================================================ */

#ifdef CONFIG_STEP_ENGINE_SELF_TEST

void step_selftest_start_sampling(uint32_t *buf, int max, volatile int *count)
{
    portENTER_CRITICAL(&s_spinlock);
    s_sample_buf = buf;
    s_sample_max = max;
    s_sample_count = count;
    *count = 0;
    s_last_alarm_ccount = 0;
    portEXIT_CRITICAL(&s_spinlock);
}

void step_selftest_stop_sampling(void)
{
    portENTER_CRITICAL(&s_spinlock);
    s_sample_buf = NULL;
    s_sample_count = NULL;
    portEXIT_CRITICAL(&s_spinlock);
}

uint32_t step_selftest_max_isr_cycles(void)
{
    return s_max_isr_cycles;
}

void step_selftest_reset_isr_cycles(void)
{
    s_max_isr_cycles = 0;
}

#endif /* CONFIG_STEP_ENGINE_SELF_TEST */
