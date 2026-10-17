/*
 * magnetic_encoder_skeleton — 采样骨架（类别组件内部）
 *
 * 收编自两实现的原各自副本：1 kHz 采样任务（CPU1、优先级 3，
 * 阻塞总线事务不扰 CPU0 的 STEP ISR）、失败计数（≥10 次 NOT OK，
 * 纯函数见 magnetic_encoder_core.c）、16 样本速度窗、s_lock 内
 * 原子发布。实例全部状态进 magnetic_encoder_inst_t（arg=实例）。
 *
 * 层内约束：不 include settings.h；不调用 bsp；总线交互只经
 * impl->read_frame 回调。
 */

#include "magnetic_encoder_impl.h"

#include <string.h>

#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

/* ---- 任务布局（契约不变量：CPU1、优先级 3、1 kHz） ---- */
#define ME_SKELETON_PERIOD_MS   1
#define ME_SKELETON_TASK_PRIO   3
#define ME_SKELETON_TASK_CORE   1
#define ME_SKELETON_TASK_STACK  3072

/* ================================================================
 * 状态发布（仅采样任务调用；首读时任务未起，同样安全）
 * ================================================================ */

static void publish_sample(magnetic_encoder_inst_t *inst,
                           uint32_t raw, uint8_t status)
{
    portENTER_CRITICAL(&inst->lock);

    /* 多圈累计（首样本种子）+ 速度窗推进，一并原子发布 */
    int64_t pos = magnetic_encoder_tracker_update(
        &inst->tracker, raw, inst->impl->counts_per_rev(inst->impl_ctx));
    inst->pos_counts      = pos;
    inst->last_raw        = raw;
    inst->status          = status;
    inst->vel_counts_per_s = magnetic_encoder_vel_push(&inst->vel, pos);

    portEXIT_CRITICAL(&inst->lock);
}

/* 失败计数推进（与契约 is_ok 读同锁，保证 NOT OK 可见性同步） */
static bool fail_step_locked(magnetic_encoder_inst_t *inst, bool read_ok)
{
    portENTER_CRITICAL(&inst->lock);
    bool event = magnetic_encoder_fail_step(&inst->fail, read_ok);
    portEXIT_CRITICAL(&inst->lock);
    return event;
}

static const char *fail_desc_of(const magnetic_encoder_inst_t *inst)
{
    if (inst->impl->fail_desc != NULL) {
        return inst->impl->fail_desc(inst->impl_ctx);
    }
    return "读取失败";
}

/* ================================================================
 * 采样任务（1 kHz，CPU1）
 * ================================================================ */

static void skeleton_task(void *arg)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)arg;

    TickType_t last_wake = xTaskGetTickCount();

    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(inst->name, "任务看门狗订阅失败——采样任务不受监控");
    }

    while (1) {
        uint32_t raw = 0;
        uint8_t status = 0;

        if (inst->impl->read_frame(inst->impl_ctx, &raw, &status)) {
            if (fail_step_locked(inst, true)) {
                ESP_LOGI(inst->name, "读取在连续 %d 次失败后恢复",
                         MAGNETIC_ENCODER_FAIL_LIMIT);
            }

            publish_sample(inst, raw, status);

            if (inst->impl->on_sample != NULL) {
                inst->impl->on_sample(inst->impl_ctx, status);
            }
        } else {
            if (fail_step_locked(inst, false)) {
                ESP_LOGE(inst->name,
                         "连续 %d 次读取失败——编码器标记 NOT OK（%s）",
                         MAGNETIC_ENCODER_FAIL_LIMIT, fail_desc_of(inst));
            }
        }

        if (wdt_ok) {
            esp_task_wdt_reset();
        }

        /* Fixed 1 kHz cadence; never busy-wait (watchdog). */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ME_SKELETON_PERIOD_MS));
    }
}

/* ================================================================
 * 契约操作表的骨架公共实现
 * ================================================================ */

int32_t magnetic_encoder_inst_counts_per_rev(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    return inst->impl->counts_per_rev(inst->impl_ctx);
}

int64_t magnetic_encoder_inst_position_counts(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    portENTER_CRITICAL(&inst->lock);
    int64_t p = inst->pos_counts;
    portEXIT_CRITICAL(&inst->lock);
    return p;
}

int32_t magnetic_encoder_inst_velocity_counts_per_s(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    portENTER_CRITICAL(&inst->lock);
    int32_t v = inst->vel_counts_per_s;
    portEXIT_CRITICAL(&inst->lock);
    return v;
}

bool magnetic_encoder_inst_is_ok(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    portENTER_CRITICAL(&inst->lock);
    bool ok = inst->fail.ok;
    portEXIT_CRITICAL(&inst->lock);
    return ok;
}

/* ================================================================
 * 生命周期
 * ================================================================ */

esp_err_t magnetic_encoder_skeleton_start(magnetic_encoder_inst_t *inst,
                                          const magnetic_encoder_impl_t *impl,
                                          void *impl_ctx,
                                          const char *name)
{
    if (inst == NULL || impl == NULL || name == NULL
        || impl->read_frame == NULL || impl->counts_per_rev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 全量接管实例：清零 + 显式初始化器/初值 */
    memset(inst, 0, sizeof(*inst));
    inst->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    inst->ops = (struct magnetic_encoder_s){
        .counts_per_rev        = magnetic_encoder_inst_counts_per_rev,
        .position_counts       = magnetic_encoder_inst_position_counts,
        .velocity_counts_per_s = magnetic_encoder_inst_velocity_counts_per_s,
        .is_ok                 = magnetic_encoder_inst_is_ok,
    };
    inst->impl     = impl;
    inst->impl_ctx = impl_ctx;
    inst->name     = name;

    magnetic_encoder_tracker_reset(&inst->tracker);
    magnetic_encoder_vel_win_reset(&inst->vel);
    magnetic_encoder_fail_reset(&inst->fail);      /* ok=true 起步 */

    /* 首读：种子多圈位置（pos=0）并验证链路可用 */
    uint32_t raw = 0;
    uint8_t status = 0;
    if (impl->read_frame(impl_ctx, &raw, &status)) {
        publish_sample(inst, raw, status);
        ESP_LOGI(name, "在线——首读 raw=%u（种子，pos=0）", raw);
    } else {
        ESP_LOGW(name, "首读失败（%s）——采样任务将继续重试",
                 fail_desc_of(inst));
    }

    BaseType_t ok = xTaskCreatePinnedToCore(skeleton_task, name,
                                            ME_SKELETON_TASK_STACK, inst,
                                            ME_SKELETON_TASK_PRIO, NULL,
                                            ME_SKELETON_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(name, "创建采样任务失败");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(name, "采样任务已启动：1 kHz，优先级 %d，CPU%d，%d counts/rev",
             ME_SKELETON_TASK_PRIO, ME_SKELETON_TASK_CORE,
             (int)impl->counts_per_rev(impl_ctx));
    return ESP_OK;
}
