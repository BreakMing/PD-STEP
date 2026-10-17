/*
 * magnetic_encoder_mt6835 — magnetic_encoder 契约的 MT6835 薄读层
 *
 * 采样任务、失败计数、速度窗、多圈累计与原子发布全在类别骨架
 * （magnetic_encoder_skeleton.c / magnetic_encoder_tracker.c /
 * magnetic_encoder_core.c）；本实现只提供读帧回调、每转计数、
 * 失败分诊（含末帧原文 dump）与 STATUS 位告警，每次注入接线参数，
 * 实现状态全部进实例（calloc）——多次 init 各出独立句柄（rev2
 * 双 MT6835 台位）。SPI 帧格式与校验在 mt6835_hw.c（驱动核心层）。
 * µrad 换算与零位/符号标定归解释层（poleang），本实现只发布 counts。
 */
#include "magnetic_encoder_mt6835.h"
#include "mt6835_urad.h"
#include "mt6835_hw.h"
#include "magnetic_encoder_impl.h"

#include <stdio.h>
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"

/* ---- 实现实例：骨架实例 + 帧层句柄 + STATUS 告警去重 ---- */
typedef struct {
    magnetic_encoder_inst_t skel;      /* 骨架实例（首成员，句柄即其地址） */
    mt6835_hw_handle_t      hw;
    const char             *name;      /* 日志标签（任务名） */
    uint8_t                 last_status;
} me_mt6835_inst_t;

/* ---- 实现回调（ctx = me_mt6835_inst_t*） ---- */

static bool read_frame(void *ctx, uint32_t *raw, uint8_t *status)
{
    me_mt6835_inst_t *me = ctx;
    return mt6835_hw_read_raw(me->hw, raw, status);
}

static int32_t counts_per_rev(void *ctx)
{
    (void)ctx;
    return (int32_t)MT6835_COUNTS_PER_REV;
}

/* 失败分诊：分类 + 末帧 6 字节原文（接线/算法分诊用） */
static void fail_desc(void *ctx, char *buf, size_t len)
{
    me_mt6835_inst_t *me = ctx;
    mt6835_hw_fail_t kind;
    uint8_t frame[MT6835_HW_FRAME_BYTES];
    mt6835_hw_last_failure(me->hw, &kind, frame);

    const char *k;
    switch (kind) {
    case MT6835_HW_FAIL_SPI_ERR:  k = "SPI 事务错误"; break;
    case MT6835_HW_FAIL_ALL_ZERO: k = "全零帧（MISO 悬空/器件未应答）"; break;
    case MT6835_HW_FAIL_CRC:      k = "CRC 不符"; break;
    default:                      k = "?"; break;
    }
    snprintf(buf, len,
             "%s；末帧 rx = %02x %02x %02x %02x %02x %02x"
             "（tx = a0 03 00 00 00 00；rx[2..4]=角度/status，rx[5]=器件 CRC）",
             k, frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
}

/* STATUS 位变化告警（bit0 过速 bit1 磁场弱 bit2 欠压；仅记录，不上报协议） */
static void on_sample(void *ctx, uint8_t status)
{
    me_mt6835_inst_t *me = ctx;
    if (status != me->last_status && status != 0) {
        ESP_LOGW(me->name, "STATUS 变化：0x%x（bit0 过速 bit1 磁弱 bit2 欠压）",
                 status);
    }
    me->last_status = status;
}

static const magnetic_encoder_impl_t s_impl = {
    .read_frame    = read_frame,
    .counts_per_rev = counts_per_rev,
    .fail_desc     = fail_desc,
    .on_sample     = on_sample,
};

/* ---- 器件自测面 ---- */

uint32_t magnetic_encoder_mt6835_last_raw(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    portENTER_CRITICAL(&inst->lock);
    uint32_t r = inst->last_raw;
    portEXIT_CRITICAL(&inst->lock);
    return r;
}

uint8_t magnetic_encoder_mt6835_last_status(magnetic_encoder_handle_t enc)
{
    magnetic_encoder_inst_t *inst = (magnetic_encoder_inst_t *)enc;
    portENTER_CRITICAL(&inst->lock);
    uint8_t s = inst->status;
    portEXIT_CRITICAL(&inst->lock);
    return s;
}

/* ---- 生命周期 ---- */

esp_err_t magnetic_encoder_mt6835_init(const magnetic_encoder_mt6835_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out)
{
    if (cfg == NULL || handle_out == NULL || cfg->host < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    me_mt6835_inst_t *me = calloc(1, sizeof(*me));
    if (me == NULL) {
        return ESP_ERR_NO_MEM;
    }
    me->name = cfg->task_name != NULL ? cfg->task_name : "me_mt6835";

    /* 帧层（驱动核心）：时钟/CS 由 config 注入 */
    mt6835_hw_config_t hw = {
        .host          = cfg->host,
        .clock_speed_hz = cfg->clock_speed_hz,
        .spics_io_num  = cfg->spics_io_num,
    };
    esp_err_t err = mt6835_hw_init(&hw, &me->hw);
    if (err != ESP_OK) {
        ESP_LOGE(me->name, "mt6835_hw_init 失败（%d）", err);
        free(me);
        return err;
    }

    err = magnetic_encoder_skeleton_start(&me->skel, &s_impl, me, me->name);
    if (err != ESP_OK) {
        free(me);
        return err;
    }

    *handle_out = (magnetic_encoder_handle_t)&me->skel;
    return ESP_OK;
}
