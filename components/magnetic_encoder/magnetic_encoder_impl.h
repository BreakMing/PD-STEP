/*
 * magnetic_encoder_impl — 骨架与实现接线（类别组件内部私有头）
 *
 * 骨架（magnetic_encoder_skeleton.c）收编采样任务、失败计数、
 * 速度窗与原子发布；实现（magnetic_encoder_as5600.c /
 * magnetic_encoder_mt6835.c）退化为薄读层：经本回调表提供读帧、
 * 每转计数与失败分诊，不再各自维护任务与发布状态。
 *
 * 实例语义：magnetic_encoder_inst_t 全部状态进实例结构，任务
 * arg=实例指针——同一实现可多实例（rev2 双 MT6835 台位）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "magnetic_encoder.h"
#include "magnetic_encoder_tracker.h"
#include "magnetic_encoder_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 实现回调表：薄读层的全部职责。 */
typedef struct {
    /**
     * 读一帧原始角（阻塞事务，任务上下文调用）。
     * @return true = 出参 raw 与 status 有效；false = 读取失败。
     */
    bool (*read_frame)(void *ctx, uint32_t *raw, uint8_t *status);

    /** 每转原始计数（如 4096 / 2^21），骨架传给 tracker。 */
    int32_t (*counts_per_rev)(void *ctx);

    /**
     * 失败分诊文案（进入 NOT OK 事件的日志用）。可 NULL——
     * 骨架用通用文案。
     */
    const char *(*fail_desc)(void *ctx);

    /**
     * 成功样本钩子（STATUS 位告警等实现侧职责）。可 NULL。
     * 任务上下文调用，publish 之后。
     */
    void (*on_sample)(void *ctx, uint8_t status);
} magnetic_encoder_impl_t;

/** 骨架实例：契约操作表为首成员（句柄即其地址），余为骨架状态。 */
typedef struct magnetic_encoder_inst_s {
    struct magnetic_encoder_s ops;          /* 契约操作表（首成员） */
    const magnetic_encoder_impl_t *impl;    /* 实现回调表 */
    void                        *impl_ctx;  /* 实现上下文（hw 句柄等） */
    const char                  *name;      /* 任务名与日志标签（静态字符串） */

    /* 以下状态由 lock 保护，仅采样任务写、契约面读 */
    portMUX_TYPE                 lock;
    magnetic_encoder_tracker_t   tracker;   /* 多圈累计（首样本种子） */
    int64_t                      pos_counts;
    magnetic_encoder_vel_win_t   vel;       /* 16 ms 速度窗 */
    int32_t                      vel_counts_per_s;
    uint32_t                     last_raw;  /* 最新原始角（器件自测面） */
    uint8_t                      status;    /* 最新 STATUS（器件自测面） */
    magnetic_encoder_fail_t      fail;      /* 失败计数与健康位 */
} magnetic_encoder_inst_t;

/**
 * @brief 启动骨架：接管实例（全量重置）、首读种子、创建采样任务。
 *
 * 任务布局（契约不变量）：1 kHz、CPU1、优先级 3；任务名 = name。
 * inst 由实现层提供存储（静态或堆分配均可）；impl_ctx 生命周期
 * 须长于任务（进程级，无停机路径）。
 *
 * @return ESP_OK；参数为 NULL 或 read_frame/counts_per_rev 缺失
 *         返回 ESP_ERR_INVALID_ARG；任务创建失败返回 ESP_ERR_NO_MEM。
 */
esp_err_t magnetic_encoder_skeleton_start(magnetic_encoder_inst_t *inst,
                                          const magnetic_encoder_impl_t *impl,
                                          void *impl_ctx,
                                          const char *name);

/* 契约四函数的骨架公共实现（实现层把操作表指到这里，enc 即实例） */
int32_t magnetic_encoder_inst_counts_per_rev(magnetic_encoder_handle_t enc);
int64_t  magnetic_encoder_inst_position_counts(magnetic_encoder_handle_t enc);
int32_t  magnetic_encoder_inst_velocity_counts_per_s(magnetic_encoder_handle_t enc);
bool     magnetic_encoder_inst_is_ok(magnetic_encoder_handle_t enc);

#ifdef __cplusplus
}
#endif
