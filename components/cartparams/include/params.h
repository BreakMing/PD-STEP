/*
 * params — 参数注册表（九项全量）
 *
 * 纯数据层（无 ESP 依赖，宿主可测）：默认值、合法范围、按名设置。
 * 名称/默认值/范围的权威来源：protocol.md 参数注册表。
 * 固件按注册表默认值上电；参数权威在端侧，会话开始时推送覆盖。
 * 运行中收紧限制立即约束当前运动（外壳负责作用点）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 参数索引（注册表顺序，CONFIG? dump 按此序）。
 * 注册表数组以指定初始化器与本枚举锁定一致；作用点分派一律
 * 经 fw_params_find，不得在外壳重复参数名字面量。 */
enum {
    FP_VEL_MAX = 0, FP_ACC_MAX, FP_WDT_MS, FP_WDT_DEC,
    FP_X_MARGIN_UM, FP_POLE_ZERO_URAD, FP_POLE_INV,
    FP_X_INV, FP_STEP_LOSS_UM, FP_COUNT,
};

typedef struct {
    long vel_max, acc_max, wdt_ms, wdt_dec;
    long x_margin_um;                          /* 软限位两端余量 µm */
    long pole_zero_urad, pole_inv, x_inv, step_loss_um;  /* 台位标定项 */
} fw_params_t;

/** 当前参数快照（静态存储；单字段 32 位读原子，跨核一致即可）。 */
const fw_params_t *fw_params(void);

/**
 * @brief 按名设置参数（范围校验）。未知 NAME 或超范围返回 false。
 *        成功即生效于后续读取；硬件作用点由外壳执行。
 */
bool fw_params_set(const char *name, long value);

/**
 * @brief 按名查参数索引。未知 NAME 或 NULL 返回 -1。
 */
int fw_params_find(const char *name);

/** 注册表条目数（CONFIG? dump 用）。 */
int fw_params_count(void);

/**
 * @brief 第 idx 个条目的 NAME（注册表顺序）；越界返回 NULL。
 */
const char *fw_params_name(int idx);

/**
 * @brief 第 idx 个条目的当前值；越界返回 0。
 */
long fw_params_value(int idx);

#ifdef __cplusplus
}
#endif
