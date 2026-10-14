/*
 * driver/i2c_master 宿主桩 — 仅提供 as5600.h 公开 config 类型所需的
 * 不透明句柄定义（与 ESP-IDF driver/i2c_master.h 的定义一致），
 * 供宿主测试包含器件头时使用。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

#ifdef __cplusplus
}
#endif
