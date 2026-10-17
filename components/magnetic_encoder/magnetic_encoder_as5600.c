/*
 * magnetic_encoder_as5600 — magnetic_encoder 契约的 AS5600 薄读层
 *
 * 采样任务、失败计数、速度窗、多圈累计与原子发布全在类别骨架
 * （magnetic_encoder_skeleton.c / magnetic_encoder_tracker.c /
 * magnetic_encoder_core.c）；本实现只提供读帧回调与每转计数，
 * 每次注入接线参数，实现状态全部进实例（calloc）——多次 init
 * 各出独立句柄。只读红线与寄存器图见 as5600_hw.c。
 */
#include "magnetic_encoder_as5600.h"
#include "as5600.h"
#include "as5600_hw.h"
#include "magnetic_encoder_impl.h"

#include <stdlib.h>

#include "esp_check.h"

/* ---- 实现实例：骨架实例 + 读层句柄 ---- */
typedef struct {
    magnetic_encoder_inst_t skel;      /* 骨架实例（首成员，句柄即其地址） */
    as5600_hw_handle_t      hw;
} me_as5600_inst_t;

/* ---- 实现回调（ctx = me_as5600_inst_t*） ---- */

static bool read_frame(void *ctx, uint32_t *raw, uint8_t *status)
{
    me_as5600_inst_t *me = ctx;
    uint16_t r = 0;
    if (!as5600_hw_read_raw(me->hw, &r)) {
        return false;
    }
    *raw = r;
    *status = 0;              /* AS5600 无 STATUS 位 */
    return true;
}

static int32_t counts_per_rev(void *ctx)
{
    (void)ctx;
    return AS5600_COUNTS_PER_REV;
}

static const magnetic_encoder_impl_t s_impl = {
    .read_frame    = read_frame,
    .counts_per_rev = counts_per_rev,
    .fail_desc     = NULL,    /* I2C 失败无帧可 dump——骨架通用文案 */
    .on_sample     = NULL,
};

/* ---- 器件自测面 ---- */

uint16_t magnetic_encoder_as5600_last_raw(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    portENTER_CRITICAL(&inst->lock);
    uint32_t r = inst->last_raw;
    portEXIT_CRITICAL(&inst->lock);
    return (uint16_t)(r & 0x0FFF);
}

bool magnetic_encoder_as5600_invert_dir_default(void)
{
#ifdef CONFIG_AS5600_INVERT_DIR
    return true;
#else
    return false;
#endif
}

/* ---- 生命周期 ---- */

esp_err_t magnetic_encoder_as5600_init(const magnetic_encoder_as5600_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out)
{
    if (cfg == NULL || handle_out == NULL || cfg->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    me_as5600_inst_t *me = calloc(1, sizeof(*me));
    if (me == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 读层（驱动核心）：地址/时钟/方向镜像全由 config 注入 */
    as5600_hw_config_t hw = {
        .bus          = cfg->bus,
        .dev_addr     = cfg->dev_addr,
        .scl_speed_hz = cfg->scl_speed_hz,
        .invert_dir   = cfg->invert_dir,
    };
    esp_err_t err = as5600_hw_init(&hw, &me->hw);
    if (err != ESP_OK) {
        free(me);
        ESP_LOGE("me_as5600", "as5600_hw_init 失败（%d）", err);
        return err;
    }

    ESP_LOGI("me_as5600", "counting direction: %s",
             cfg->invert_dir
                 ? "INVERTED (magnet mounted opposite; +X move counts up)"
                 : "normal (not inverted)");

    err = magnetic_encoder_skeleton_start(&me->skel, &s_impl, me,
                                          cfg->task_name != NULL
                                              ? cfg->task_name
                                              : "me_as5600");
    if (err != ESP_OK) {
        free(me);
        return err;
    }

    *handle_out = (magnetic_encoder_handle_t)&me->skel;
    return ESP_OK;
}
