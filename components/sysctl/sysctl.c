/*
 * System Control (T10) — implementation (see sysctl.h for the contract)
 *
 * No task, no polling: sysctl_get_state() derives the state from
 * step_engine_busy() whenever it is asked (PLAN §3.2 has no sysctl_task).
 *
 * Writer/reader: s_state is written by comm_task (sysctl_init /
 * sysctl_stop / the cached refresh inside sysctl_get_state) and read by
 * monitor_task.  An enum write is a single aligned 32-bit store on
 * Xtensa, so `volatile` is enough — the worst case is one sample of
 * staleness, which is harmless for a telemetry state field.
 */

#include "sysctl.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "step_engine.h"
#include "planner.h"

static const char *TAG = "sysctl";

static volatile sysctl_state_t s_state = SYSCTL_BOOT;

esp_err_t sysctl_init(void)
{
    if (s_state != SYSCTL_BOOT) {
        return ESP_OK;              /* idempotent */
    }

    s_state = step_engine_busy() ? SYSCTL_RUN : SYSCTL_IDLE;
    ESP_LOGI(TAG, "state machine up: BOOT -> %s (derived, no task)",
             sysctl_state_str());
    return ESP_OK;
}

void sysctl_stop(void)
{
    /* ① Stop the pulses first — only then is the executed position final. */
    step_engine_stop();

    /* ② Rebase the commanded position onto that frozen position, so the
     *    next G command starts where the axis really stopped instead of
     *    from the cancelled target (PLAN §4.4). */
    planner_rebase_to_executed();

    /* ③ */
    s_state = SYSCTL_IDLE;

    ESP_LOGW(TAG, "'!' immediate stop: pulses stopped, queue cleared, "
                  "command position rebased to executed");
}

sysctl_state_t sysctl_get_state(void)
{
    if (s_state == SYSCTL_BOOT) {
        return SYSCTL_BOOT;         /* not initialized yet */
    }

    /* Refresh on demand (the state is never polled by a task). */
    s_state = step_engine_busy() ? SYSCTL_RUN : SYSCTL_IDLE;
    return s_state;
}

const char *sysctl_state_str(void)
{
    switch (sysctl_get_state()) {
    case SYSCTL_BOOT:
        return "BOOT";
    case SYSCTL_RUN:
        return "RUN";
    case SYSCTL_IDLE:
    default:
        return "IDLE";
    }
}
