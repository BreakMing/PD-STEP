/*
 * Monitor / Telemetry (T9) — status-frame packing
 *
 * Data sources (all already synchronised internally — no extra lock here):
 *   C  planner_position_mm()      commanded position (32-bit atomic read)
 *   E  as5600_position_mm()       measured position  (portMUX protected)
 *   V  as5600_velocity_mm_s()*60  measured velocity, signed, mm/min
 *   T  tmc2209_read_status()      DRV_STATUS decoded → single flag
 *   STATE step_engine_busy()      RUN / IDLE (T10 replaces this source)
 *
 * report_hz ($9) is read fresh on every frame: the four settings
 * subscriber slots are already taken (tmc2209 / planner / app_main /
 * as5600), so the monitor must not subscribe (PLAN v1.15 T8 note ⑤).
 *
 * Periodic reporting (D12): while the axis is moving a frame goes out
 * every 1000/report_hz ms; while it is standing still nothing is sent
 * spontaneously — '?' answers on demand at any time.
 */

#include "monitor.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "protocol.h"
#include "planner.h"
#include "as5600.h"
#include "step_engine.h"
#include "tmc2209.h"
#include "settings.h"

static const char *TAG = "monitor";

#define MONITOR_TASK_STACK     3072
#define MONITOR_TASK_PRIO      2
#define MONITOR_TASK_CORE      1     /* shares CPU1 with as5600 (prio 3) —
                                      * CPU0 stays free for the pulse ISR */
#define MONITOR_TICK_MS        10
#define MONITOR_FRAME_BUF      128   /* worst case frame ≈ 60 bytes */
#define MONITOR_FLAG_BUF       8
#define MONITOR_FALLBACK_K     80.0f /* only if settings is unusable */

/* ---- State ---- */
static bool     s_initialized = false;
static uint32_t s_frames_sent = 0;   /* single writer: monitor_task */

/* G92 reference: lets the commanded-position field carry an EXECUTED
 * position (see build_status_frame()).  Written by monitor_note_origin()
 * from the G92 callback, read by both reporter paths — hence the lock. */
static portMUX_TYPE s_ref_lock  = portMUX_INITIALIZER_UNLOCKED;
static int32_t      s_ref_steps = 0;      /* step_engine position at last G92 */
static float        s_ref_mm    = 0.0f;   /* coordinate declared at that G92 */

/* ================================================================
 * Pure functions
 * ================================================================ */

void monitor_tmc_flag(const tmc2209_status_t *st, char *buf, size_t len)
{
    const char *flag;

    if (st == NULL) {
        flag = "ERR";          /* DRV_STATUS read failed */
    } else if (st->ot) {
        flag = "OT";           /* overtemperature shutdown     */
    } else if (st->s2ga) {
        flag = "S2GA";         /* short to GND, phase A        */
    } else if (st->s2gb) {
        flag = "S2GB";         /* short to GND, phase B        */
    } else if (st->ola) {
        flag = "OLA";          /* open load, phase A           */
    } else if (st->olb) {
        flag = "OLB";          /* open load, phase B           */
    } else if (st->otpw) {
        flag = "OTPW";         /* overtemperature pre-warning  */
    } else {
        flag = "OK";
    }

    snprintf(buf, len, "%s", flag);
}

int monitor_format_frame(char *buf, size_t len, const char *state,
                         float c_mm, float e_mm, float v_mm_min,
                         const char *flag)
{
    int n = snprintf(buf, len, "<%s|C:%.3f|E:%.3f|V:%.3f|T:%s>",
                     (state != NULL) ? state : "IDLE",
                     (double)c_mm, (double)e_mm, (double)v_mm_min,
                     (flag != NULL) ? flag : "ERR");

    if (n < 0) {
        if (len > 0) {
            buf[0] = '\0';
        }
        return 0;
    }
    if ((size_t)n >= len) {
        return (int)len - 1;    /* truncated */
    }
    return n;
}

/* ================================================================
 * Frame assembly (shared by the '?' path and the periodic path)
 * ================================================================ */

static int build_status_frame(char *buf, size_t len)
{
    tmc2209_status_t st;
    char flag[MONITOR_FLAG_BUF];

    bool ok = tmc2209_read_status(&st);
    monitor_tmc_flag(ok ? &st : NULL, flag, sizeof(flag));

    /* Readings carry their own sync; read them close together so the
     * frame is as coherent a snapshot as the API allows. */
    const char *state = step_engine_busy() ? "RUN" : "IDLE";

    /* C = EXECUTED position: the pulse ISR's step counter mapped into the
     * G92 coordinate frame.
     *
     * Using the planner's target position instead would pin C at the
     * destination for the whole move, so C−E would degenerate into
     * "distance still to travel" and carry no loss-of-step information
     * until the move ended.  PLAN T9 asks for the commanded position to be
     * sampled from the ISR-updated counter, which is this one — with it,
     * C−E is the live loss-of-step amount at every instant. */
    float k = settings_steps_per_mm();
    if (!(k > 0.0f)) {
        k = MONITOR_FALLBACK_K;
    }

    int32_t ref_steps;
    float   ref_mm;
    portENTER_CRITICAL(&s_ref_lock);
    ref_steps = s_ref_steps;
    ref_mm    = s_ref_mm;
    portEXIT_CRITICAL(&s_ref_lock);

    float c_mm     = (float)(step_engine_position_steps() - ref_steps) / k + ref_mm;
    float e_mm     = as5600_position_mm();
    float v_mm_min = as5600_velocity_mm_s() * 60.0f;

    return monitor_format_frame(buf, len, state, c_mm, e_mm, v_mm_min, flag);
}

#ifdef CONFIG_MONITOR_SELF_TEST
int monitor_selftest_build_frame(char *buf, size_t len)
{
    return build_status_frame(buf, len);
}
#endif

/* ================================================================
 * Runtime
 * ================================================================ */

/**
 * @brief Align the executed-position frame with a new coordinate origin.
 *
 * Called from the G92 path (app_main's on_set_origin callback) together
 * with planner_set_origin() / as5600_set_origin(), which is what keeps
 * C and E shifting together (PLAN D10).
 */
void monitor_note_origin(float new_pos_mm)
{
    portENTER_CRITICAL(&s_ref_lock);
    s_ref_steps = step_engine_position_steps();
    s_ref_mm    = new_pos_mm;
    portEXIT_CRITICAL(&s_ref_lock);
}

void monitor_send_status(void)
{
    char buf[MONITOR_FRAME_BUF];

    /* Keep room for the \r\n terminator. */
    int n = build_status_frame(buf, sizeof(buf) - 2);
    buf[n]     = '\r';
    buf[n + 1] = '\n';

    protocol_send_raw(buf, n + 2);   /* TX mutex inside protocol */
}

uint32_t monitor_frames_sent(void)
{
    return s_frames_sent;
}

/* Periodic reporter — D12: only while the axis is moving. */
static void monitor_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   ticks = 0;      /* 10 ms ticks accumulated since last frame */

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MONITOR_TICK_MS));

        if (!step_engine_busy()) {
            ticks = 0;         /* idle → nothing spontaneous (D12) */
            continue;
        }

        /* Read $9 every frame — never cached, never subscribed. */
        int hz = settings_report_hz();
        if (hz < 1) {
            hz = 1;
        } else if (hz > 50) {
            hz = 50;           /* settings validates [1,50]; clamp defensively */
        }

        uint32_t ticks_per_frame = (uint32_t)(1000 / (hz * MONITOR_TICK_MS));
        if (ticks_per_frame < 1) {
            ticks_per_frame = 1;
        }

        if (++ticks >= ticks_per_frame) {
            ticks = 0;
            monitor_send_status();
            s_frames_sent++;   /* periodic frames only, single writer */
        }
    }
}

esp_err_t monitor_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* Power-up position is 0 (D10) — align the executed-position frame
     * before the first frame can be requested. */
    monitor_note_origin(0.0f);

    /* '?' query path — protocol calls this from comm_task context.
     * The callback is short: format + one TMC read + one TX. */
    protocol_register_status_cb(monitor_send_status);

    BaseType_t ok = xTaskCreatePinnedToCore(monitor_task, "monitor",
                                            MONITOR_TASK_STACK, NULL,
                                            MONITOR_TASK_PRIO, NULL,
                                            MONITOR_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create monitor_task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "telemetry up: prio %d, CPU%d, %d ms tick, report_hz=%d",
             MONITOR_TASK_PRIO, MONITOR_TASK_CORE, MONITOR_TICK_MS,
             settings_report_hz());
    return ESP_OK;
}
