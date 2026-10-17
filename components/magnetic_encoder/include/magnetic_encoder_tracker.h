/*
 * magnetic_encoder_tracker — 参数化多圈累计纯逻辑（类别组件）
 *
 * 单圈绝对角（0..cpr−1）→ 连续多圈位置（counts）。counts_per_rev
 * 为参数：AS5600（4096）与 MT6835（2^21）共用同一规则与实现。
 *
 * 层内约束：纯逻辑，无硬件/RTOS 依赖，宿主可测（见
 * host_tests/magnetic_encoder_tracker_test.c）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t  pos_counts;   /* 连续位置（counts，多圈，带符号） */
    uint32_t last_raw;     /* 上一原始角 0..cpr−1 */
    bool     seeded;       /* 已收到首样本？ */
} magnetic_encoder_tracker_t;

/**
 * @brief 清零历史——下一个样本成为位置原点。
 */
void magnetic_encoder_tracker_reset(magnetic_encoder_tracker_t *t);

/**
 * @brief 喂入一个原始角，返回更新后的连续位置。
 *
 * 首样本只播种（位置 0，不产生位移）。回绕修正（cpr 为
 * counts_per_rev）：
 *   diff = raw − last_raw
 *   diff >  +cpr/2     → diff −= cpr   （反向越过 0 边界）
 *   diff <  −(cpr/2−1) → diff += cpr   （正向越过 0 边界）
 *
 * 非对称阈值对全部 cpr 个 diff 取值恰好覆盖一次（无丢失、无重复，
 * 宿主测试对 4096 与 2^21 全值域验证）：
 *   −(cpr/2−1)..+cpr/2 共 cpr 个取值直通——这是方向无歧义的最大区间；
 *   ±(cpr/2+1)..±(cpr−1) 解释为反方向的小步。
 * 对称式规则会在两个极端取值上丢一个或重一个。
 *
 * 边界抖动：0↔cpr−1 抖动经修正后 diff = ±1（而非 ±(cpr−1)），
 * 静止判据无需额外死区。
 *
 * 速度上限：单样本最大可分辨步长 ±cpr/2（半圈）；1 kHz 采样下
 * 即 cpr/2×1000 counts/s（cpr=4096 → 30,000 rpm；cpr=2^21 →
 * 2.1×10^9 counts/s ≈ 60,000 rpm——均远超设计需求）。
 *
 * @param t    tracker 状态（NULL 时返回 0）。
 * @param raw  原始角，高位按 cpr 归一（等价于 raw % cpr）。
 * @param cpr  每转计数（>0；0 视为 1 防除零）。
 * @return 连续位置（counts）。
 */
int64_t magnetic_encoder_tracker_update(magnetic_encoder_tracker_t *t,
                                        uint32_t raw, int32_t cpr);

#ifdef __cplusplus
}
#endif
