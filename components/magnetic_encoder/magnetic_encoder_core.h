/*
 * magnetic_encoder_core — 骨架纯函数（类别组件内部接口）
 *
 * 失败计数状态机与 16 ms 速度窗：语义锚定公共契约
 * （include/magnetic_encoder.h）不变量 2/3——连续 ≥10 次读失败
 * NOT OK、一次成功恢复；速度 = 17 样本窗位置差 × 62.5（量子
 * 62.5 counts/s），窗口未满为 0。
 *
 * 纯逻辑，无硬件/RTOS 依赖，宿主可测（host_tests/
 * magnetic_encoder_core_test.c）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 连续读失败达到该值 → NOT OK（契约不变量 3） */
#define MAGNETIC_ENCODER_FAIL_LIMIT   10

/* 速度窗：16 ms（17 个 1 ms 位置样本，环形缓冲） */
#define MAGNETIC_ENCODER_SPEED_WINDOW 16
#define MAGNETIC_ENCODER_HIST_LEN     (MAGNETIC_ENCODER_SPEED_WINDOW + 1)

/* ---- 失败计数状态机 ---- */

typedef struct {
    int  streak;   /* 当前连续失败数 */
    bool ok;       /* 契约可见健康位 */
} magnetic_encoder_fail_t;

/** 健康初值：streak=0、ok=true（开机尚未失败即健康）。 */
void magnetic_encoder_fail_reset(magnetic_encoder_fail_t *f);

/**
 * @brief 喂入一次读取结果，推进状态机。
 *
 * @return true = 本步发生状态迁移事件（streak 首达 LIMIT 进入
 *         NOT OK，或 NOT OK 期间首次成功恢复）——调用方借此打
 *         一次日志；平稳步返回 false。
 */
bool magnetic_encoder_fail_step(magnetic_encoder_fail_t *f, bool read_ok);

/* ---- 16 ms 速度窗 ---- */

typedef struct {
    int64_t hist[MAGNETIC_ENCODER_HIST_LEN];   /* 环形位置样本 */
    int     head;                              /* 最新样本下标 */
    int     fill;                              /* 已存样本数（≤ 窗长） */
} magnetic_encoder_vel_win_t;

/** 清空窗口（重新预热）。 */
void magnetic_encoder_vel_win_reset(magnetic_encoder_vel_win_t *w);

/**
 * @brief 推入一个 1 ms 位置样本，返回最新速度估计。
 *
 * 窗未满（首 16 个样本）返回 0；满窗后
 * vel = llround((pos − oldest) × 1000 / SPEED_WINDOW)，
 * 即 16 ms 位置差 × 62.5（量子 62.5 counts/s）。
 */
int32_t magnetic_encoder_vel_push(magnetic_encoder_vel_win_t *w, int64_t pos);

#ifdef __cplusplus
}
#endif
