/*
 * params — 参数注册表实现（默认值与范围：protocol.md §参数注册表）
 *
 * 纯数据层：fw_params_set 只做范围校验与写值；写硬件的作用点
 * （引擎加速度/看门狗/方向反转）由 cartfsm 外壳执行。
 */

#include "params.h"

#include <stddef.h>
#include <string.h>

/* 注册表顺序（CONFIG? dump 按此顺序） */
static fw_params_t s_params = {
    .vel_max        = 1000,     /* 1 – 2000 mm/s          */
    .acc_max        = 5000,     /* 100 – 30000 mm/s²      */
    .wdt_ms         = 100,      /* 10 – 1000 ms           */
    .wdt_dec        = 5000,     /* 100 – 30000 mm/s²      */
    .x_margin_um    = 10000,    /* 1000 – 50000 µm        */
    .pole_zero_urad = 0,        /* ±3141592 µrad          */
    .pole_inv       = 0,        /* 0 – 1                  */
    .x_inv          = 0,        /* 0 – 1                  */
    .step_loss_um   = 3000,     /* 500 – 20000 µm         */
};

typedef struct {
    const char *name;
    long *field;
    long min;
    long max;
} param_entry_t;

static const param_entry_t s_entries[FP_COUNT] = {
    [FP_VEL_MAX]        = { "VEL_MAX",        &s_params.vel_max,         1,       2000    },
    [FP_ACC_MAX]        = { "ACC_MAX",        &s_params.acc_max,         100,     30000   },
    [FP_WDT_MS]         = { "WDT_MS",         &s_params.wdt_ms,          10,      1000    },
    [FP_WDT_DEC]        = { "WDT_DEC",        &s_params.wdt_dec,         100,     30000   },
    [FP_X_MARGIN_UM]    = { "X_MARGIN_UM",    &s_params.x_margin_um,     1000,    50000   },
    [FP_POLE_ZERO_URAD] = { "POLE_ZERO_URAD", &s_params.pole_zero_urad,  -3141592, 3141592 },
    [FP_POLE_INV]       = { "POLE_INV",       &s_params.pole_inv,        0,       1       },
    [FP_X_INV]          = { "X_INV",          &s_params.x_inv,           0,       1       },
    [FP_STEP_LOSS_UM]   = { "STEP_LOSS_UM",   &s_params.step_loss_um,    500,     20000   },
};

#define ENTRY_COUNT ((int)(sizeof(s_entries) / sizeof(s_entries[0])))

const fw_params_t *fw_params(void)
{
    return &s_params;
}

bool fw_params_set(const char *name, long value)
{
    if (name == NULL) {
        return false;
    }
    for (int i = 0; i < ENTRY_COUNT; i++) {
        if (strcmp(s_entries[i].name, name) == 0) {
            if (value < s_entries[i].min || value > s_entries[i].max) {
                return false;
            }
            *s_entries[i].field = value;
            return true;
        }
    }
    return false;
}

int fw_params_find(const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (int i = 0; i < ENTRY_COUNT; i++) {
        if (strcmp(s_entries[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

int fw_params_count(void)
{
    return ENTRY_COUNT;
}

const char *fw_params_name(int idx)
{
    if (idx < 0 || idx >= ENTRY_COUNT) {
        return NULL;
    }
    return s_entries[idx].name;
}

long fw_params_value(int idx)
{
    if (idx < 0 || idx >= ENTRY_COUNT) {
        return 0;
    }
    return *s_entries[idx].field;
}
