# 三器件域解耦实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 cartpos / poleang / motor 三个域组件，业务组件（cartfsm、telemetry）与器件驱动解耦，换器件型号时业务零改动。

**Architecture:** 域组件 = 契约头（链接期绑定）+ 实现源文件（Kconfig choice 选择，menuconfig 切换）；接线参数全部由 app_main / 测试工程入口读取 Kconfig 宏与 bsp 总线句柄后经实现 init 注入；mm↔steps 换算从 cartfsm 下沉到 motor 域的 motor_conv 纯函数。

**Tech Stack:** ESP-IDF v6.0.2（C、CMake 组件、Kconfig choice、Unity 测试工程）、gcc 宿主纯逻辑测试。

**Spec:** [2026-10-14-device-decoupling-design.md](../specs/2026-10-14-device-decoupling-design.md)（计划从规格推导；执行者实施任何任务前先读规格全文）

## Global Constraints

- 链接期绑定：域契约头只含函数声明，接口函数定义在同组件实现源文件；不引入函数指针注册。
- 依赖规则（规格「依赖规则」五条）：cartfsm 最终 REQUIRES = cartpos poleang motor cartparams cartproto；telemetry = cartpos poleang cartproto；器件驱动组件（as5600/mt6835/tmc2209/step_engine）不 REQUIRES bsp、只被域组件与测试工程引用；main 只 REQUIRES 三个域组件与业务组件；cartproto 现状不动。
- 迁移不变量 1–5（规格）：FAULT 停机次序、重定基时序、锁序、协议字节兼容（STATE 字段与 MODE reason 值 "AS5600"/"MT6835"/"STEP_LOSS" 不变）、测试注入语义。
- 每个任务收尾：主工程与测试工程构建通过；未改纯逻辑源的任务 host 测试不受影响（Task 6 全量复跑）。
- 被改文件的头部注释同步按本设计职责表述更新（清除「业务壳」「外壳」等依赖上下文的旧称谓）；不引入自创缩写（用全称：编码器实测位置、指令位置）。
- 三个 Kconfig choice 各只有一个选项且为默认（`CARTPOS_IMPL_AS5600` / `POLEANG_IMPL_MT6835` / `MOTOR_IMPL_TMC2209`），现有两工程 sdkconfig 无需改动。
- 构建命令：主工程 `idf.py -B build build`；测试工程 `idf.py -C test build`。本机环境经 `idf_env.ps1` 包装：`powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"`（或 ESP-IDF MCP 构建工具）。宿主测试形态同 [vel_ramp_test.c](../../../host_tests/vel_ramp_test.c) 头部注释：`gcc -std=c11 -Wall -Wextra -I<头文件目录> 测试.c 源.c`，退出码 = 失败断言数。

## Review Focus

规格未逐条展开、但最可能咬人的五类条件，及其锚定位置：

1. **换算即时生效**：$0/$1/$2 运行中变更后，后续 motor 调用必须用新 steps_per_mm——Task 3 宿主测试钉死四条公式；Task 4 实现即读即算（每次调用读 settings 派生值），实现里不得出现缓存的换算系数。
2. **精确判零**：FAULT 停机次序依赖速度精确为零，Q16.16 域的微小残余不得经整数 mm/s 舍入提前判停——Task 4 `motor_at_rest` 直连 `step_engine_vel_now_q16() == 0`；Task 6 附录 A 上板 flowtest 验证停机次序。
3. **失败保留最后成功值**：is_ok=false 期间 cartpos/poleang 读函数返回最后一次成功样本值、不清零——器件行为不变，域实现为纯转发；Task 6 附录 A 上板拔线用例验证。
4. **协议字节兼容**：STATE/MODE 输出与迁移前逐字节一致——Task 2 的标定移动不得改变数值（同一纯函数、同一参数）；Task 6 附录 A 端侧联调核对。
5. **Kconfig 默认无感**：三个 choice 默认即现实现，两工程现有 sdkconfig 不改也能构建——Task 1/2/4 各自的构建步骤即验证。

---

### Task 1: cartpos 域组件（AS5600 实现 + 接线注入）

**Files:**
- Create: `components/cartpos/include/cartpos.h`
- Create: `components/cartpos/include/cartpos_as5600.h`
- Create: `components/cartpos/cartpos_as5600.c`
- Create: `components/cartpos/Kconfig`
- Create: `components/cartpos/CMakeLists.txt`
- Modify: `components/as5600/include/as5600.h`
- Modify: `components/as5600/as5600.c`
- Modify: `components/as5600/CMakeLists.txt`
- Modify: `main/app_main.c`
- Modify: `main/CMakeLists.txt`
- Modify: `test/main/test_app_main.c`
- Modify: `test/main/CMakeLists.txt`
- Modify: `components/as5600/test/test_as5600.c:197-202`

**Interfaces:**
- Consumes: `as5600_init` 现行为（幂等、任务创建、$2 订阅）；`bsp_get_i2c_bus()`
- Produces: `cartpos_position_um` / `cartpos_velocity_mm_s` / `cartpos_is_ok` / `cartpos_set_origin_mm`（Task 2/5 消费）；`cartpos_as5600_init(const cartpos_as5600_config_t *)`；`as5600_init(const as5600_config_t *)`（签名变更，本任务内更新全部三个调用点）

- [ ] **Step 1: 新建 cartpos 域组件四文件**

`components/cartpos/include/cartpos.h`：

```c
/*
 * cartpos — 小车位置域契约（编码器实测位置）
 *
 * 链接期绑定：本头文件只含接口声明，实现源文件在同组件内
 * （当前实现 cartpos_as5600.c，经 Kconfig choice 选择）。
 * 语义：docs/superpowers/specs/2026-10-14-device-decoupling-design.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 连续位置（多圈、带符号、含回零原点偏移），µm 整数精确；首样本前为 0。 */
int64_t cartpos_position_um(void);

/** 16 ms 窗口平滑速度 mm/s；窗口未满为 0。 */
float cartpos_velocity_mm_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool cartpos_is_ok(void);

/** 声明当前位置为 new_pos_mm（纯偏移建立，不产生运动）。 */
void cartpos_set_origin_mm(float new_pos_mm);

#ifdef __cplusplus
}
#endif
```

`components/cartpos/include/cartpos_as5600.h`：

```c
/*
 * cartpos_as5600 — cartpos 域的 AS5600 实现（init 与接线 config）
 *
 * 接线参数（总线、地址、时钟、方向镜像）由组装点读取 Kconfig 宏
 * 与 bsp 总线句柄后经本 config 注入。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;     /* 已初始化的 I2C 总线（组装点从 bsp 获取） */
    uint16_t                dev_addr;     /* 7-bit 从机地址 */
    uint32_t                scl_speed_hz; /* 总线时钟 */
    bool                    invert_dir;   /* 方向镜像（安装标定） */
} cartpos_as5600_config_t;

/** 幂等。内部完成 as5600 设备初始化（含 1 kHz 采样任务）。 */
esp_err_t cartpos_as5600_init(const cartpos_as5600_config_t *cfg);

#ifdef __cplusplus
}
#endif
```

`components/cartpos/cartpos_as5600.c`：

```c
/*
 * cartpos_as5600 — cartpos 域接口的 AS5600 实现（链接期绑定）
 *
 * 四个域函数逐条转发 as5600 设备 API；init 注入接线 config。
 */
#include "cartpos.h"
#include "cartpos_as5600.h"
#include "as5600.h"

esp_err_t cartpos_as5600_init(const cartpos_as5600_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    as5600_config_t dev = {
        .bus          = cfg->bus,
        .dev_addr     = cfg->dev_addr,
        .scl_speed_hz = cfg->scl_speed_hz,
        .invert_dir   = cfg->invert_dir,
    };
    return as5600_init(&dev);
}

int64_t cartpos_position_um(void)   { return as5600_position_um(); }
float   cartpos_velocity_mm_s(void) { return as5600_velocity_mm_s(); }
bool    cartpos_is_ok(void)         { return as5600_is_ok(); }
void    cartpos_set_origin_mm(float new_pos_mm) { as5600_set_origin(new_pos_mm); }
```

`components/cartpos/Kconfig`：

```kconfig
menu "cartpos - 小车位置域"

choice CARTPOS_IMPL
    prompt "小车位置传感器实现"
    default CARTPOS_IMPL_AS5600

config CARTPOS_IMPL_AS5600
    bool "AS5600（I2C 磁编码器，4096 counts/rev）"

endchoice

endmenu
```

`components/cartpos/CMakeLists.txt`：

```cmake
# cartpos — 小车位置域组件：契约头 + 实现源文件（Kconfig choice 选择）
set(cartpos_srcs "")
set(cartpos_reqs "")

if(CONFIG_CARTPOS_IMPL_AS5600)
    list(APPEND cartpos_srcs "cartpos_as5600.c")
    list(APPEND cartpos_reqs as5600)
endif()

idf_component_register(SRCS ${cartpos_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES ${cartpos_reqs})
```

- [ ] **Step 2: as5600.h 增加公开 config 与编译期方向默认值**

在 `#include "esp_err.h"` 之后加 `#include "driver/i2c_master.h"`；将 `esp_err_t as5600_init(void);` 及其注释替换为：

```c
/* Kconfig 布尔在 n 时不定义，统一折算为编译期常量（组装点使用） */
#ifdef CONFIG_AS5600_INVERT_DIR
#define AS5600_INVERT_DIR_DEFAULT 1
#else
#define AS5600_INVERT_DIR_DEFAULT 0
#endif

/* 接线参数（组装点读取 Kconfig 宏与 bsp 总线句柄后注入） */
typedef struct {
    i2c_master_bus_handle_t bus;
    uint16_t                dev_addr;
    uint32_t                scl_speed_hz;
    bool                    invert_dir;
} as5600_config_t;

/**
 * @brief 注入接线参数并启动 1 kHz 采样任务。幂等。
 *
 * @return ESP_OK；cfg 为 NULL 或 bus 为 NULL 返回 ESP_ERR_INVALID_ARG；
 *         其余错误码与迁移前一致（I2C / 任务创建错误）。
 */
esp_err_t as5600_init(const as5600_config_t *cfg);
```

- [ ] **Step 3: as5600.c 改造 init**

删除 `#include "bsp.h"`；删除文件内 `AS5600_DIR_INVERTED` 折叠块（`#ifdef CONFIG_AS5600_INVERT_DIR` … `#endif`）；文件头注释的「业务壳」表述改为「as5600 设备组件：1 kHz 采样任务 + 原点偏移 + $2 订阅；接线经 as5600_config_t 注入」。init 开头替换为：

```c
esp_err_t as5600_init(const as5600_config_t *cfg)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (cfg == NULL || cfg->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 读层（驱动核心）：地址/时钟/方向镜像全由 config 注入 */
    as5600_hw_config_t hw = {
        .bus          = cfg->bus,
        .dev_addr     = cfg->dev_addr,
        .scl_speed_hz = cfg->scl_speed_hz,
        .invert_dir   = cfg->invert_dir,
    };
    ESP_RETURN_ON_ERROR(as5600_hw_init(&hw),
                        TAG, "as5600_hw_init failed");
```

方向日志与首读日志中的 `AS5600_DIR_INVERTED` / `CONFIG_BSP_AS5600_I2C_ADDR` 分别替换为 `cfg->invert_dir` / `cfg->dev_addr`（日志字符串文本不动，只换变量）。函数体其余部分（tracker 复位、$2 订阅、首读、任务创建）不变。

- [ ] **Step 4: as5600/CMakeLists 去 bsp**

```cmake
idf_component_register(SRCS "as5600.c" "as5600_tracker.c" "as5600_hw.c"
                    INCLUDE_DIRS "include"
                    REQUIRES settings freertos esp_driver_i2c)
```

- [ ] **Step 5: app_main 与两处测试组装点切换**

`main/app_main.c`：`#include "as5600.h"` 改为 `#include "cartpos_as5600.h"`；`ESP_ERROR_CHECK(as5600_init());` 替换为：

```c
        cartpos_as5600_config_t cpos = {
            .bus          = bsp_get_i2c_bus(),
            .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
            .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
            .invert_dir   = AS5600_INVERT_DIR_DEFAULT != 0,
        };
        ESP_ERROR_CHECK(cartpos_as5600_init(&cpos));
```

`test/main/test_app_main.c` 同样替换（该文件已 include bsp.h）。

`main/CMakeLists.txt` REQUIRES 中 `as5600` 改为 `cartpos`；`test/main/CMakeLists.txt` 同：

```cmake
idf_component_register(SRCS "test_app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bsp settings cartpos mt6835 tmc2209 cartproto step_engine cartfsm telemetry
                    PRIV_REQUIRES unity)
```

- [ ] **Step 6: test_as5600.c B13 用例改传 config**

```c
TEST_CASE("B13 I2C 总线在位 + as5600_init() OK", "[as5600]")
{
    as5600_config_t cfg = {
        .bus          = bsp_get_i2c_bus(),
        .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
        .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
        .invert_dir   = AS5600_INVERT_DIR_DEFAULT != 0,
    };
    TEST_ASSERT_NOT_NULL(bsp_get_i2c_bus());
    TEST_ASSERT_EQUAL(ESP_OK, as5600_init(&cfg));
}
```

- [ ] **Step 7: 构建验证**

```bash
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-C test build"
```

预期：两工程构建成功（Kconfig choice 取默认，sdkconfig 无需改动）。

- [ ] **Step 8: Commit**

```bash
git add components/cartpos components/as5600 main test/main
git commit -m "cartpos 域组件落地：契约头+AS5600 实现（Kconfig choice），as5600 接线改 config 注入，组装点统一"
```

### Task 2: poleang 域组件 + telemetry 切换（摆角标定移入域实现）

**Files:**
- Create: `components/poleang/include/poleang.h`
- Create: `components/poleang/include/poleang_mt6835.h`
- Create: `components/poleang/poleang_mt6835.c`
- Create: `components/poleang/Kconfig`
- Create: `components/poleang/CMakeLists.txt`
- Modify: `components/mt6835/include/mt6835.h`
- Modify: `components/mt6835/mt6835.c`
- Modify: `components/mt6835/CMakeLists.txt`
- Modify: `components/telemetry/telemetry.c`
- Modify: `components/telemetry/include/telemetry.h`
- Modify: `components/telemetry/CMakeLists.txt`
- Modify: `main/app_main.c`
- Modify: `main/CMakeLists.txt`
- Modify: `test/main/test_app_main.c`
- Modify: `test/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `mt6835_init` 现行为（幂等、任务创建）；`mt6835_raw_to_urad` / `mt6835_last_raw` / `mt6835_vth_mrad_s` / `mt6835_is_ok`；`fw_params()` 的 `pole_zero_urad` / `pole_inv`
- Produces: `poleang_theta_urad` / `poleang_vth_mrad_s` / `poleang_is_ok`（Task 5 消费）；`poleang_mt6835_init(const poleang_mt6835_config_t *)`；`mt6835_init(const mt6835_config_t *)`（签名变更）；`cartpos_*`（Task 1 已就绪，telemetry 本任务开始消费）

- [ ] **Step 1: 新建 poleang 域组件四文件**

`components/poleang/include/poleang.h`：

```c
/*
 * poleang — 摆角域契约（已标定摆角）
 *
 * 链接期绑定：实现源文件在同组件内（当前实现 poleang_mt6835.c，
 * 经 Kconfig choice 选择）。零位/符号标定（POLE_ZERO_URAD / POLE_INV）
 * 在实现内应用——消费方读到的是标定后角度。
 * 语义：docs/superpowers/specs/2026-10-14-device-decoupling-design.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 已标定摆角（±π 环绕 µrad）；首样本前为 0。 */
int32_t poleang_theta_urad(void);

/** 已标定角速度（mrad/s，符号随 POLE_INV 翻转）；窗口未满为 0。 */
int32_t poleang_vth_mrad_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool poleang_is_ok(void);

#ifdef __cplusplus
}
#endif
```

`components/poleang/include/poleang_mt6835.h`：

```c
/*
 * poleang_mt6835 — poleang 域的 MT6835 实现（init 与接线 config）
 */
#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t host;         /* 已初始化的 SPI 主机（组装点从 bsp 获取） */
    int               clock_speed_hz;
    int               spics_io_num;
} poleang_mt6835_config_t;

/** 幂等。内部完成 mt6835 设备初始化（含 1 kHz 采样任务）。 */
esp_err_t poleang_mt6835_init(const poleang_mt6835_config_t *cfg);

#ifdef __cplusplus
}
#endif
```

`components/poleang/poleang_mt6835.c`：

```c
/*
 * poleang_mt6835 — poleang 域接口的 MT6835 实现（链接期绑定）
 *
 * 标定（POLE_ZERO_URAD / POLE_INV，来自 cartparams 注册表）在读取时
 * 应用：theta 经 mt6835_raw_to_urad 纯函数换算，vth 符号随 POLE_INV
 * 翻转。参数写入瞬间 theta 与 vth 可能取到不同快照（两次独立读取
 * 注册表），200 Hz 下一帧自愈——与迁移前单快照的差异仅出现在参数
 * 写入的那一帧，接受。
 */
#include "poleang.h"
#include "poleang_mt6835.h"
#include "mt6835.h"
#include "mt6835_urad.h"
#include "params.h"

esp_err_t poleang_mt6835_init(const poleang_mt6835_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mt6835_config_t dev = {
        .host          = cfg->host,
        .clock_speed_hz = cfg->clock_speed_hz,
        .spics_io_num  = cfg->spics_io_num,
    };
    return mt6835_init(&dev);
}

int32_t poleang_theta_urad(void)
{
    const fw_params_t *p = fw_params();
    return mt6835_raw_to_urad(mt6835_last_raw(),
                              (int32_t)p->pole_zero_urad,
                              p->pole_inv != 0);
}

int32_t poleang_vth_mrad_s(void)
{
    const fw_params_t *p = fw_params();
    int32_t v = mt6835_vth_mrad_s();
    return (p->pole_inv != 0) ? -v : v;
}

bool poleang_is_ok(void)
{
    return mt6835_is_ok();
}
```

`components/poleang/Kconfig`：

```kconfig
menu "poleang - 摆角域"

choice POLEANG_IMPL
    prompt "摆角编码器实现"
    default POLEANG_IMPL_MT6835

config POLEANG_IMPL_MT6835
    bool "MT6835（SPI 磁编码器，21-bit）"

endchoice

endmenu
```

`components/poleang/CMakeLists.txt`：

```cmake
# poleang — 摆角域组件：契约头 + 实现源文件（Kconfig choice 选择）
set(poleang_srcs "")
set(poleang_reqs "")

if(CONFIG_POLEANG_IMPL_MT6835)
    list(APPEND poleang_srcs "poleang_mt6835.c")
    list(APPEND poleang_reqs mt6835 cartparams)
endif()

idf_component_register(SRCS ${poleang_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES ${poleang_reqs})
```

- [ ] **Step 2: mt6835 接线注入**

`components/mt6835/include/mt6835.h`：include 区加 `#include "driver/spi_master.h"`；`esp_err_t mt6835_init(void);` 替换为：

```c
/* 接线参数（组装点读取 Kconfig 宏与 bsp 主机号后注入） */
typedef struct {
    spi_host_device_t host;
    int               clock_speed_hz;
    int               spics_io_num;
} mt6835_config_t;

/** 幂等。接线经 config 注入并启动 1 kHz 采样任务。 */
esp_err_t mt6835_init(const mt6835_config_t *cfg);
```

`components/mt6835/mt6835.c`：删除 `#include "bsp.h"`；init 开头替换为：

```c
esp_err_t mt6835_init(const mt6835_config_t *cfg)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (cfg == NULL || cfg->host < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 帧层（驱动核心）：时钟/CS 由 config 注入 */
    mt6835_hw_config_t hw = {
        .host          = cfg->host,
        .clock_speed_hz = cfg->clock_speed_hz,
        .spics_io_num  = cfg->spics_io_num,
    };
    ESP_RETURN_ON_ERROR(mt6835_hw_init(&hw), TAG, "mt6835_hw_init 失败");
```

文件头注释「业务壳」表述改为「mt6835 设备组件：1 kHz 采样任务 + 多圈位置/速度估计；接线经 mt6835_config_t 注入」。`components/mt6835/CMakeLists.txt`：

```cmake
idf_component_register(SRCS "mt6835.c" "mt6835_urad.c" "mt6835_hw.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_spi freertos)
```

- [ ] **Step 3: telemetry 切换到 cartpos / poleang**

`components/telemetry/telemetry.c`：include 区 `as5600.h` / `mt6835.h` / `params.h` 替换为 `cartpos.h` / `poleang.h`；任务循环体替换为：

```c
    for (;;) {
        int32_t theta = poleang_theta_urad();
        int32_t vth   = poleang_vth_mrad_s();

        int64_t x_um = cartpos_position_um();
        int32_t vx = (int32_t)lround(cartpos_velocity_mm_s());

        int n = telemetry_format_state(buf, sizeof(buf),
                                       (int32_t)x_um, theta, vx, vth);
        if (n > 0) {
            buf[n] = '\n';
            cartproto_send_raw(buf, n + 1);
        } else {
            ESP_LOGE(TAG, "组帧失败（n=%d）——帧被丢弃", n);
        }
```

`components/telemetry/include/telemetry.h` 头注释「字段来源」更新为：x_um/vx ← cartpos；theta/vth ← poleang（已标定）。`components/telemetry/CMakeLists.txt`：

```cmake
idf_component_register(
    SRCS "telemetry.c" "telemetry_format.c"
    INCLUDE_DIRS "include"
    REQUIRES cartpos poleang cartproto freertos
)
```

- [ ] **Step 4: 组装点切换**

`main/app_main.c`：`#include "mt6835.h"` 改为 `#include "poleang_mt6835.h"`；`ESP_ERROR_CHECK(mt6835_init());` 替换为：

```c
        poleang_mt6835_config_t pol = {
            .host          = bsp_get_spi_host(),
            .clock_speed_hz = CONFIG_BSP_MT6835_SPI_HZ,
            .spics_io_num  = CONFIG_BSP_MT6835_CS_GPIO,
        };
        ESP_ERROR_CHECK(poleang_mt6835_init(&pol));
```

`main/CMakeLists.txt` REQUIRES 中 `mt6835` 改为 `poleang`；`test/main/test_app_main.c` 与 `test/main/CMakeLists.txt` 同步（REQUIRES：bsp settings cartpos poleang tmc2209 cartproto step_engine cartfsm telemetry）。

- [ ] **Step 5: 构建验证**

```bash
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-C test build"
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && gcc -std=c11 -Wall -Wextra -Ihost_tests/stub -Icomponents/as5600/include -Icomponents/telemetry/include host_tests/telemetry_format_test.c components/telemetry/telemetry_format.c -o /tmp/tft && /tmp/tft; echo exit=\$?"
```

预期：两工程构建成功；telemetry_format 宿主测试 15/15 断言通过（exit=0，组帧纯函数未动；该测试自身 include as5600.h 构造期望值，属测试侧引用，不随本任务改动）。

- [ ] **Step 6: Commit**

```bash
git add components/poleang components/mt6835 components/telemetry main test/main
git commit -m "poleang 域组件落地：摆角标定自 telemetry 移入域实现；mt6835 接线改 config 注入；telemetry 切换 cartpos/poleang"
```

### Task 3: motor_conv 换算纯函数（TDD）

**Files:**
- Create: `components/motor/include/motor_conv.h`
- Create: `components/motor/motor_conv.c`
- Create: `components/motor/CMakeLists.txt`
- Test: `host_tests/motor_conv_test.c`

**Interfaces:**
- Produces: `motor_conv_mmps_to_q16` / `motor_conv_accel_to_steps` / `motor_conv_steps_to_um` / `motor_conv_um_to_steps`（签名见 Step 1；Task 4 消费）

- [ ] **Step 1: 写失败的宿主测试**

`host_tests/motor_conv_test.c`（断言宏形态照抄 [vel_ramp_test.c](../../../host_tests/vel_ramp_test.c)）：

```c
/*
 * motor_conv 宿主测试 — 手算精确向量逐条断言
 *
 * 编译：gcc -std=c11 -Wall -Wextra -Icomponents/motor/include \
 *       host_tests/motor_conv_test.c components/motor/motor_conv.c
 * 退出码 = 失败断言数（0 = 全过）。
 */
#include <stdio.h>
#include "motor_conv.h"

static int s_fail = 0;
static int s_run = 0;

#define CHECK_EQ(expr, expect) do { \
    long long _e = (long long)(expect); \
    long long _a = (long long)(expr); \
    s_run++; \
    if (_e != _a) { \
        s_fail++; \
        printf("FAIL %s:%d  %s = %lld, 期望 %lld\n", \
               __func__, __LINE__, #expr, _a, _e); \
    } \
} while (0)

int main(void)
{
    /* steps_per_mm = 40（默认 $0=8、$1=200、$2=40） */
    CHECK_EQ(motor_conv_mmps_to_q16(0,    40.0), 0);
    CHECK_EQ(motor_conv_mmps_to_q16(1000, 40.0), 2621440000LL);
    CHECK_EQ(motor_conv_mmps_to_q16(-250, 40.0), -655360000LL);
    /* VEL_MAX 注册表上限 2000 mm/s → 80000 steps/s，超出 32 位 Q16.16 */
    CHECK_EQ(motor_conv_mmps_to_q16(2000, 40.0), 5242880000LL);

    CHECK_EQ(motor_conv_accel_to_steps(0,    40.0), 0);
    CHECK_EQ(motor_conv_accel_to_steps(4000, 40.0), 160000);

    CHECK_EQ(motor_conv_steps_to_um(0,      40.0), 0);
    CHECK_EQ(motor_conv_steps_to_um(160000, 40.0), 4000000L);
    /* 舍入边界：62.5 → 63（llround 半数远离零），负向对称 */
    CHECK_EQ(motor_conv_steps_to_um(1,  16.0), 63);
    CHECK_EQ(motor_conv_steps_to_um(-1, 16.0), -63);

    CHECK_EQ(motor_conv_um_to_steps(4000000, 40.0), 160000);
    CHECK_EQ(motor_conv_um_to_steps(25, 40.0), 1);  /* 1.0 精确 */
    CHECK_EQ(motor_conv_um_to_steps(13, 40.0), 1);  /* 0.52 → 1 */
    CHECK_EQ(motor_conv_um_to_steps(12, 40.0), 0);  /* 0.48 → 0 */
    CHECK_EQ(motor_conv_um_to_steps(1,  40.0), 0);  /* 0.04 → 0 */

    printf("%s: %d 用例, %d 失败\n", s_fail ? "FAIL" : "PASS", s_run, s_fail);
    return s_fail;
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
gcc -std=c11 -Wall -Wextra -Icomponents/motor/include host_tests/motor_conv_test.c components/motor/motor_conv.c -lm -o /tmp/mct && /tmp/mct; echo exit=$?
```

预期：编译失败 `motor_conv.h: No such file or directory`（红）。

- [ ] **Step 3: 实现纯函数**

`components/motor/include/motor_conv.h`：

```c
/*
 * motor_conv — mm 域与 steps 域换算纯函数（宿主可测）
 *
 * 公式与迁移前 cartfsm 的四个换算函数逐式一致；steps_per_mm 语义
 * 由 settings 派生访问器定义（$0 细分 × $1 每转整步数 ÷ $2 每转
 * 行程 mm）。
 * 前置条件：steps_per_mm > 0（settings 校验域保证）；不满足为未定义。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** mm/s → Q16.16 steps/s（带符号；64 位承载 VEL_MAX 上限）。 */
int64_t motor_conv_mmps_to_q16(long v_mm_s, double steps_per_mm);

/** mm/s² → steps/s²。 */
uint32_t motor_conv_accel_to_steps(long a_mm_s2, double steps_per_mm);

/** 指令位置 steps → µm（四舍五入，半数远离零）。 */
long motor_conv_steps_to_um(int32_t steps, double steps_per_mm);

/** µm → 指令位置 steps（上一函数的逆换算，四舍五入）。 */
int32_t motor_conv_um_to_steps(long um, double steps_per_mm);

#ifdef __cplusplus
}
#endif
```

`components/motor/motor_conv.c`：

```c
#include "motor_conv.h"

#include <math.h>

int64_t motor_conv_mmps_to_q16(long v_mm_s, double steps_per_mm)
{
    return (int64_t)llround((double)v_mm_s * steps_per_mm * 65536.0);
}

uint32_t motor_conv_accel_to_steps(long a_mm_s2, double steps_per_mm)
{
    return (uint32_t)llround((double)a_mm_s2 * steps_per_mm);
}

long motor_conv_steps_to_um(int32_t steps, double steps_per_mm)
{
    return (long)llround((double)steps * 1000.0 / steps_per_mm);
}

int32_t motor_conv_um_to_steps(long um, double steps_per_mm)
{
    return (int32_t)llround((double)um * steps_per_mm / 1000.0);
}
```

`components/motor/CMakeLists.txt`（Task 4 再扩展为 choice 形态）：

```cmake
# motor — 电机运动域组件：契约头 + 实现源文件（Task 4 引入 Kconfig choice）
idf_component_register(SRCS "motor_conv.c"
                    INCLUDE_DIRS "include")
```

- [ ] **Step 4: 跑测试确认通过**

```bash
gcc -std=c11 -Wall -Wextra -Icomponents/motor/include host_tests/motor_conv_test.c components/motor/motor_conv.c -lm -o /tmp/mct && /tmp/mct; echo exit=$?
```

预期：`PASS: 15 用例, 0 失败`，exit=0。

- [ ] **Step 5: 构建验证（组件编入）**

```bash
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"
```

预期：主工程构建成功（motor 组件被默认编入）。

- [ ] **Step 6: Commit**

```bash
git add components/motor host_tests/motor_conv_test.c
git commit -m "motor_conv 纯函数：mm↔steps 四条换算自 cartfsm 下沉，宿主测试 15 用例锚定公式与舍入边界"
```

### Task 4: motor 域组件（tmc2209 + step_engine 组合）+ EN 注入 + 组装切换

**Files:**
- Create: `components/motor/include/motor.h`
- Create: `components/motor/include/motor_tmc2209.h`
- Create: `components/motor/motor_tmc2209.c`
- Create: `components/motor/Kconfig`
- Modify: `components/motor/CMakeLists.txt`
- Modify: `components/tmc2209/include/tmc2209.h`
- Modify: `components/tmc2209/tmc2209.c`
- Modify: `components/tmc2209/CMakeLists.txt`
- Modify: `main/app_main.c`
- Modify: `main/CMakeLists.txt`
- Modify: `test/main/test_app_main.c`
- Modify: `test/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `tmc2209_init` / `tmc2209_enable`；`step_engine_init` 及全部 vel/position API（现签名）；`settings_steps_per_mm()`；Task 3 的四个 `motor_conv_*`
- Produces: `motor.h` 十一个函数（Task 5 消费）；`motor_tmc2209_init(const motor_tmc2209_config_t *)`；`tmc2209_config_t` 新增 `en_gpio` 字段

- [ ] **Step 1: motor 契约头与实现头**

`components/motor/include/motor.h`：

```c
/*
 * motor — 电机运动域契约（指令位置）
 *
 * 链接期绑定：实现源文件在同组件内（当前实现 motor_tmc2209.c，
 * 经 Kconfig choice 选择）。速度/位置的 Q16.16 为 step_engine 现有
 * API 的定点记法（16 位整数位 + 16 位小数位）。
 * 语义：docs/superpowers/specs/2026-10-14-device-decoupling-design.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 驱动器使能引脚（EN，低有效）使能/关断；en=true 等待 ≥1 ms 唤醒。 */
esp_err_t motor_enable(bool en);

/** 设定带符号目标速度（mm/s）并喂命令看门狗。 */
void motor_set_vel_mm_s(long v_mm_s);

/** 目标清 0 斜坡停；不喂看门狗。 */
void motor_halt(void);

/** 急停：停脉冲、清递推状态与目标、冻结指令位置。 */
void motor_stop(void);

/** true ⇔ 当前速度精确为零（不经 mm/s 整数舍入）。 */
bool motor_at_rest(void);

/** 指令位置（steps → µm 四舍五入）。 */
int64_t motor_cmd_position_um(void);

/** 指令位置重定基；前置条件：电机停止（无脉冲发出）。 */
void motor_set_position_um(long um);

/** 参数作用点：追踪加速度 / 看门狗减速 / 看门狗超时 / 方向反转。 */
void motor_set_accel_mm_s2(long a_mm_s2);
void motor_set_decel_mm_s2(long d_mm_s2);
void motor_set_wdt_ms(long t_ms);
void motor_set_dir_invert(bool invert);

#ifdef __cplusplus
}
#endif
```

`components/motor/include/motor_tmc2209.h`：

```c
/*
 * motor_tmc2209 — motor 域的 TMC2209 + GPTimer 实现（init 与接线 config）
 *
 * 组合 tmc2209（UART 配置/电流/细分/EN）与 step_engine（STEP/DIR
 * 脉冲引擎）；mm↔steps 换算经 motor_conv（即读即算）。
 */
#pragma once

#include "esp_err.h"
#include "tmc2209.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    tmc2209_config_t tmc;       /* 驱动器配置（含 en_gpio） */
    int              step_gpio; /* STEP 输出引脚 */
    int              dir_gpio;  /* DIR 输出引脚 */
} motor_tmc2209_config_t;

/** 幂等。内部完成 tmc2209_init 与 step_engine_init（次序即此）。 */
esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: motor_tmc2209.c 实现**

```c
/*
 * motor_tmc2209 — motor 域接口的 TMC2209 + GPTimer 实现（链接期绑定）
 */
#include "motor.h"
#include "motor_tmc2209.h"
#include "motor_conv.h"

#include "tmc2209.h"
#include "step_engine.h"
#include "settings.h"

/* 换算即时生效：每次调用读当前派生值（$0/$1/$2 变更后即用新值） */
static double spm(void)
{
    return (double)settings_steps_per_mm();
}

esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = tmc2209_init(&cfg->tmc);
    if (err != ESP_OK) {
        return err;
    }
    return step_engine_init(cfg->step_gpio, cfg->dir_gpio);
}

esp_err_t motor_enable(bool en)
{
    return tmc2209_enable(en);
}

void motor_set_vel_mm_s(long v_mm_s)
{
    step_engine_vel_target(motor_conv_mmps_to_q16(v_mm_s, spm()));
}

void motor_halt(void)
{
    step_engine_vel_halt();
}

void motor_stop(void)
{
    step_engine_stop();
}

bool motor_at_rest(void)
{
    /* 精确判零：不经 mm/s 整数舍入（残余速度不得提前判停） */
    return step_engine_vel_now_q16() == 0;
}

int64_t motor_cmd_position_um(void)
{
    return motor_conv_steps_to_um(step_engine_position_steps(), spm());
}

void motor_set_position_um(long um)
{
    step_engine_set_position(motor_conv_um_to_steps(um, spm()));
}

void motor_set_accel_mm_s2(long a_mm_s2)
{
    step_engine_vel_set_accel(motor_conv_accel_to_steps(a_mm_s2, spm()));
}

void motor_set_decel_mm_s2(long d_mm_s2)
{
    step_engine_vel_set_decel(motor_conv_accel_to_steps(d_mm_s2, spm()));
}

void motor_set_wdt_ms(long t_ms)
{
    step_engine_vel_set_wdt_us((uint32_t)t_ms * 1000u);
}

void motor_set_dir_invert(bool invert)
{
    step_engine_set_dir_invert(invert);
}
```

`components/motor/Kconfig`：

```kconfig
menu "motor - 电机运动域"

choice MOTOR_IMPL
    prompt "电机驱动实现"
    default MOTOR_IMPL_TMC2209

config MOTOR_IMPL_TMC2209
    bool "TMC2209（UART）+ GPTimer STEP/DIR 引擎"

endchoice

endmenu
```

`components/motor/CMakeLists.txt` 替换为：

```cmake
# motor — 电机运动域组件：契约头 + 实现源文件（Kconfig choice 选择）
set(motor_srcs "motor_conv.c")
set(motor_reqs "")

if(CONFIG_MOTOR_IMPL_TMC2209)
    list(APPEND motor_srcs "motor_tmc2209.c")
    list(APPEND motor_reqs tmc2209 step_engine settings)
endif()

idf_component_register(SRCS ${motor_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES ${motor_reqs})
```

- [ ] **Step 3: tmc2209 EN 改经 config 注入**

`components/tmc2209/include/tmc2209.h`：`tmc2209_config_t` 增加字段（置于 rx_pin 之后）：

```c
    int      en_gpio;      /* ENN（低有效）；引脚方向由 bsp_init 配置，此处只驱动电平 */
```

`components/tmc2209/tmc2209.c`：`#include "bsp.h"` 改为 `#include "driver/gpio.h"`；`tmc2209_enable` 替换为：

```c
esp_err_t tmc2209_enable(bool en)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    /* ENN 低有效；引脚方向由 bsp_init 配置，这里只驱动电平 */
    gpio_set_level(s_cfg.en_gpio, en ? 0 : 1);

    if (en) {
        /* Datasheet: wait for driver to wake up after EN low */
        vTaskDelay(pdMS_TO_TICKS(ENABLE_SETTLE_MS));
    }

    return ESP_OK;
}
```

文件头注释「EN 引脚（bsp）」表述改为「EN 引脚（en_gpio 经 config 注入）」；`s_cfg` 用途注释同步（r_sense 与 en_gpio）。`components/tmc2209/CMakeLists.txt`：

```cmake
idf_component_register(
    SRCS "tmc2209_hw.c" "tmc2209.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_gpio freertos settings
)
```

- [ ] **Step 4: 组装点切换**

`main/app_main.c`：`#include "tmc2209.h"`、`#include "step_engine.h"` 改为 `#include "motor_tmc2209.h"`；tmc config 两段（`tmc2209_config_t tmc = …`、`tmc2209_init(&tmc)`、`step_engine_init(...)`）替换为：

```c
        motor_tmc2209_config_t mot = {
            .tmc = {
                .uart_num = UART_NUM_2,   /* UART0=烧录/日志、UART1=协议、UART2=TMC */
                .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
                .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
                .en_gpio = CONFIG_BSP_EN_GPIO,
                .baud = 115200,
                .slave_addr = 0,
                .r_sense = CONFIG_TMC2209_RSENSE,
            },
            .step_gpio = CONFIG_BSP_STEP_GPIO,
            .dir_gpio = CONFIG_BSP_DIR_GPIO,
        };
        ESP_ERROR_CHECK(motor_tmc2209_init(&mot));
```

`main/CMakeLists.txt` REQUIRES 替换为最终形态：

```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp cartproto settings cartpos poleang motor telemetry cartfsm
)
```

`test/main/test_app_main.c` 同 app_main 替换；`test/main/CMakeLists.txt` REQUIRES 替换为：`bsp settings cartpos poleang motor cartproto cartfsm telemetry`（PRIV_REQUIRES unity 保留）。

- [ ] **Step 5: 构建验证**

```bash
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-C test build"
```

预期：两工程构建成功；cartfsm 仍直调 tmc2209/step_engine API（Task 5 才切换），行为不变。

- [ ] **Step 6: Commit**

```bash
git add components/motor components/tmc2209 main test/main
git commit -m "motor 域组件落地：tmc2209+step_engine 组合实现（Kconfig choice），EN 引脚改 config 注入，组装点切 motor_tmc2209_init"
```

### Task 5: cartfsm 调用面切换 + 测试钩子迁移 + 依赖收紧

**Files:**
- Modify: `components/cartfsm/cartfsm.c`
- Modify: `components/cartfsm/include/cartfsm.h`
- Modify: `components/cartfsm/CMakeLists.txt`
- Modify: `main/app_main.c`（顶部启动次序注释）

**Interfaces:**
- Consumes: Task 1 `cartpos_*`、Task 2 `poleang_is_ok`、Task 4 `motor_*` 全部函数（签名见各任务）
- Produces: cartfsm 公开 API 不变（`cartfsm_start/mode/reason/calib` 与测试钩子签名不变，test_cartfsm_flow 用例不改）

- [ ] **Step 1: include 与换算胶水删除**

`components/cartfsm/cartfsm.c`：include 区 `settings.h` / `tmc2209.h` / `step_engine.h` / `as5600.h` / `mt6835.h` 替换为 `cartpos.h` / `poleang.h` / `motor.h`；`#include <math.h>` 随换算函数一并删除（llround 仅用于被删的四个函数）；删除 `mmps_to_q16` / `accel_to_steps` / `cmd_steps_to_um` / `um_to_cmd_steps` 四个静态函数（连同「换算」节注释）。

- [ ] **Step 2: 参数作用点与钳制路径切换**

```c
static void params_apply_engine(void)
{
    const fw_params_t *p = fw_params();
    motor_set_accel_mm_s2(p->acc_max);
    motor_set_decel_mm_s2(p->wdt_dec);
    motor_set_wdt_ms(p->wdt_ms);
    motor_set_dir_invert(p->x_inv != 0);
}
```

`clamp_and_commit_velocity`：`cm_soft_limit_clamp((long)as5600_position_um(), …)` 改 `(long)cartpos_position_um()`；`step_engine_vel_target(mmps_to_q16(eff))` 改 `motor_set_vel_mm_s(eff)`（s_lock 内提交 s_last_cmd_v、锁外下发的结构不变）。`supervise_soft_limit` 同两处替换（`as5600_position_um` → `cartpos_position_um`；`step_engine_vel_halt()` → `motor_halt()`；条件清零逻辑不变）。`params_apply_one` 的四个 case 改调对应 `motor_set_*`；`FP_VEL_MAX` case 结构不变。

- [ ] **Step 3: 模式转换与命令执行切换**

`enter_fault`：`step_engine_vel_set_accel(accel_to_steps(p->wdt_dec))` 改 `motor_set_accel_mm_s2(p->wdt_dec)`；`step_engine_vel_halt()` 改 `motor_halt()`。`exec_mark_left` / `exec_mark_right`：`as5600_position_um()` 改 `cartpos_position_um()`；`as5600_set_origin((float)l / 2.0f / 1000.0f)` 改 `cartpos_set_origin_mm((float)l / 2.0f / 1000.0f)`；`step_engine_set_position(um_to_cmd_steps(l / 2))` 改 `motor_set_position_um(l / 2)`。`exec_enable`：`step_engine_set_position(um_to_cmd_steps((long)as5600_position_um()))` 改 `motor_set_position_um((long)cartpos_position_um())`；`tmc2209_enable(true)` 改 `motor_enable(true)`。`exec_reset`：`step_engine_stop()` 改 `motor_stop()`；`tmc2209_enable(false)` 改 `motor_enable(false)`。

- [ ] **Step 4: 监督任务与测试钩子迁移**

`fsm_task` 内（含 `CONFIG_CARTFSM_TEST_HOOKS` 两段）：

```c
#ifdef CONFIG_CARTFSM_TEST_HOOKS
        bool a5600_ok = (s_test_enc_fail == 1) ? false : cartpos_is_ok();
        bool m6835_ok = (s_test_enc_fail == 2) ? false : poleang_is_ok();
#else
        bool a5600_ok = cartpos_is_ok();
        bool m6835_ok = poleang_is_ok();
#endif
```

丢步判定拍：`long e_um = (long)cartpos_position_um();`（测试偏置注入点随之迁移）；`long c_um = (long)motor_cmd_position_um();`。FAULT 整定段：`step_engine_vel_halt()` 改 `motor_halt()`；`if (step_engine_vel_now_q16() == 0)` 改 `if (motor_at_rest())`；`tmc2209_enable(false)` 改 `motor_enable(false)`。`cartfsm_start`：`tmc2209_enable(false)` 改 `motor_enable(false)`，其忽略返回值的前提注释改为「motor_tmc2209_init 已先由 app_main 完成」。本地变量名 `a5600_ok` / `m6835_ok` 保留（与 CR_AS5600 / CR_MT6835 原因码对应）。

- [ ] **Step 5: 头注释与 CMake 收紧**

`components/cartfsm/include/cartfsm.h` 头注释「模式机外壳（纯核心 cartfsm_core + 硬件作用点）」改为「模式机：监督任务与命令执行（纯核心 cartfsm_core + 域接口作用点）」。`components/cartfsm/CMakeLists.txt`：

```cmake
idf_component_register(
    SRCS "cartfsm.c" "cartfsm_core.c"
    INCLUDE_DIRS "include"
    REQUIRES cartpos poleang motor cartparams cartproto freertos
)
```

`main/app_main.c` 顶部启动次序注释同步（`as5600_init → mt6835_init → tmc2209_init → step_engine_init` 改为三个域实现 init）。全仓清点旧称谓残留：

```bash
grep -rn "业务壳\|外壳" components main --include="*.c" --include="*.h"
```

预期：无输出（Task 1/2/4 已随改随更，此处兜底）。

- [ ] **Step 6: 构建与依赖验证**

```bash
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-C test build"
```

预期：两工程构建成功——CMake REQUIRES 已表达依赖规则，业务组件 include 到的器件头若未在依赖闭包内会直接构建失败（依测试规则，以构建结果为准，不做纯文本判断）。测试钩子签名未变，test_cartfsm_flow.c 无需修改即编入。

- [ ] **Step 7: Commit**

```bash
git add components/cartfsm main/app_main.c
git commit -m "cartfsm 切换域接口调用面：删 mm↔steps 换算胶水，测试钩子劫持点迁至 cartpos/poleang，CMake 依赖收紧至域组件"
```

### Task 6: 全量验证 + 文档更新 + 上板清单

**Files:**
- Modify: `README.md`（工程结构节）
- Modify: 本计划附录 B（验证记录回填）

- [ ] **Step 1: host 测试全量**

以下命令已于 2026-10-14 在本机验证（WSL gcc 13.3.0，前六条全绿；第七条自 Task 3 起存在）：

```bash
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && \
gcc -std=c11 -Wall -Wextra -Icomponents/cartfsm/include -Icomponents/cartparams/include host_tests/cartfsm_core_test.c components/cartfsm/cartfsm_core.c components/cartparams/params.c -o /tmp/t1 && /tmp/t1; echo t1=\$?; \
gcc -std=c11 -Wall -Wextra -Ihost_tests/stub -Icomponents/cartproto/include host_tests/cartproto_parse_test.c components/cartproto/cartproto_parse.c -o /tmp/t2 && /tmp/t2; echo t2=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/mt6835/include host_tests/mt6835_crc_test.c components/mt6835/mt6835_urad.c -o /tmp/t3 && /tmp/t3; echo t3=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/mt6835/include host_tests/mt6835_urad_test.c components/mt6835/mt6835_urad.c -o /tmp/t4 && /tmp/t4; echo t4=\$?; \
gcc -std=c11 -Wall -Wextra -Ihost_tests/stub -Icomponents/as5600/include -Icomponents/telemetry/include host_tests/telemetry_format_test.c components/telemetry/telemetry_format.c -o /tmp/t5 && /tmp/t5; echo t5=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/step_engine/include host_tests/vel_ramp_test.c components/step_engine/vel_ramp.c -o /tmp/t6 && /tmp/t6; echo t6=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/motor/include host_tests/motor_conv_test.c components/motor/motor_conv.c -lm -o /tmp/t7 && /tmp/t7; echo t7=\$?"
```

预期：七个全部退出码 0。

- [ ] **Step 2: 两工程最终构建**

```bash
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-B build build"
powershell -ExecutionPolicy Bypass -File idf_env.ps1 -IdfAction "-C test build"
```

- [ ] **Step 3: README 工程结构更新**

「工程结构」树中 `as5600/` 行前插入三行，并调整器件行描述：

```
  cartpos/      小车位置域：契约 + AS5600 实现（Kconfig choice）
  poleang/      摆角域：契约 + MT6835 实现（含 POLE_ZERO/POLE_INV 标定）
  motor/        电机运动域：契约 + TMC2209+GPTimer 实现 + mm↔steps
                换算纯函数（motor_conv）
```

`as5600/`、`mt6835/`、`tmc2209/` 三行描述补「（器件驱动，经域组件组合）」；任务/CPU 布局段不变（域组件不新建任务）。

- [ ] **Step 4: 附录 B 回填 + Commit**

回填验证记录（命令、日期、退出码），标注上板项待执行；提交：

```bash
git add README.md docs/superpowers/plans/2026-10-14-device-decoupling.md
git commit -m "解耦收尾：README 工程结构补三域组件，计划验证记录回填（上板项标注待执行）"
```

---

## 附录 A：上板回归清单（需实机，人工执行）

按 [testing.md](../../../.agents/rules/testing.md) 要求记录日期、环境与范围；此前驱动核心拆分（2026-10-13）遗留的真机回归一并补记：

1. **编码器在线**：as5600 / mt6835 自测 B 组（[as5600] 与 [mt6835] 标签）全绿；首读日志正常。
2. **STATE 流字节核对**：端侧联调（tools/cartpole_host.py）抓取 STATE 行，与迁移前记录逐字段比对（x/theta/vx/vth 数值域一致）；标定参数（POLE_ZERO_URAD/POLE_INV）设置后 theta 数值与迁移前同参数一致。
3. **MODE reason 兼容**：拔 AS5600 / 拔 MT6835 各触发一次 FAULT，确认 MODE 行 reason 为 "AS5600" / "MT6835"；失败期间 STATE 流持续输出最后成功值（不清零、不中断）。
4. **FAULT 停机次序**：READY 下运动中注入编码器失联（flowtest），确认斜坡停 → 速度精确为零后才关 EN → 推 MODE FAULT；RESET 恢复。
5. **flowtest 全量**：test 工程 `[flowtest]` 标签用例全绿（标定、使能、软限位、失联注入——覆盖迁移不变量 2 的重定基时序与不变量 5 的注入语义；钩子已迁至域接口，用例不改）。
6. **bringup 段**：[bringup] 硬件寄存器核对（EN 引脚经 config 注入后首次真机验证）。
7. **手动段**：[manual] 手转轴 / 摆杆观察（多圈计数与方向镜像）。

## 附录 B：验证记录回填

环境：Windows 11 + ESP-IDF v6.1（`C:\esp\v6.1\esp-idf`）+ Ninja；宿主测试 WSL gcc 13.3.0。构建与宿主命令见 Global Constraints 与 Task 6 Step 1。

| 任务 | 日期 | 环境 | 命令与结果 | 上板项 |
|---|---|---|---|---|
| Task 1 | 2026-10-14 | 同上 | 主工程 `idf.py -B build build` → Project build complete；测试工程 `idf.py -C test build` → Project build complete | 附录 A 1 待上板 |
| Task 2 | 2026-10-14 | 同上 | 双工程 build → Project build complete；telemetry_format 宿主 15/15（exit=0，含 i2c_master 桩） | 附录 A 2 待上板 |
| Task 3 | 2026-10-14 | 同上 | motor_conv 宿主 RED（头文件缺失）→ GREEN PASS 15/15（exit=0）；主工程 build → Project build complete | — |
| Task 4 | 2026-10-14 | 同上 | 双工程 build → Project build complete | 附录 A 6 待上板 |
| Task 5 | 2026-10-14 | 同上 | 双工程 build → Project build complete；cartfsm_core+params 宿主 87/87（exit=0） | 附录 A 4/5 待上板 |
| Task 6 | 2026-10-14 | 同上 | host 七测全绿：87/64/2/19/15/48/15（exit=0 × 7）；双工程 build → Project build complete × 2 | 附录 A 全部待上板 |
