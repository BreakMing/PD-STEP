# 三器件驱动核心拆分实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 tmc2209 / as5600 / mt6835 三个组件各自的硬件访问核心拆成独立源文件（驱动核心层），参数全部由 init config 注入，与 settings / bsp / 任务创建解耦。

**Architecture:** 每个组件内部拆为两层文件（沿用 cartfsm_core / mt6835_urad 的同组件拆文件先例，不新建组件）：`<name>_hw.c` + 组件根目录私有头 `<name>_hw.h` 承载驱动核心（UART 帧/寄存器原语、I2C 读层、SPI 帧层）；原 `<name>.c` 退化为业务壳（任务/滤波/标定/settings 订阅/寄存器策略），通过 hw 接缝调用核心。三个组件的 public 头文件（`include/<name>.h`）与全部对外符号不变。

**Tech Stack:** ESP-IDF v6.1（CMake 组件），FreeRTOS，driver/uart + i2c_master + spi_master。

**Spec:** 用户 2026-10-13 指令（本计划"全局约束"即其逐条重述）。分析基础：会话记忆 pd-step-driver-coupling（2026-10-13 耦合评估，结论"复用触发时抽 _hw.c"）。

## Global Constraints

已确认的行为契约与项目规则（硬性，来源：用户指令 + 组件既有契约）：

1. 现有 public API 不变：`components/tmc2209/include/tmc2209.h`、`components/as5600/include/as5600.h`、`components/mt6835/include/mt6835.h` 的声明不修改，外部调用方（main、cartfsm、telemetry、各 test/）零改动。
2. 驱动核心层特征：构造参数全部由 init config 结构体注入（bus handle、slave addr、RSENSE 归属 shell、时钟/引脚等）；`*_hw.c` 不 include `settings.h`；不调用 `bsp_*`；不创建任务。
3. EN 引脚等板级操作上移给业务壳调用：`tmc2209_enable` 留在 shell，继续调 `bsp_tmc_en_set`；hw 层不触碰任何 GPIO。
4. 业务壳（采样任务/速度滤波/标定与原点/settings 订阅/寄存器配置策略）留在原组件文件。
5. AS5600 只读红线（OTP 寄存器不可写）随读层迁移到 `as5600_hw`，拆分不引入任何总线写操作。
6. 行为等价：拆分是代码搬移 + 接缝化，不改变任何对外可观察行为（返回值、日志语义、失败路径、上电时序）。
7. 遵守 .agents/rules/ 下实施、设计与计划、测试、写作规则。

建议/预估（未被规格固定，实施者可按实际代码结构调整，调整时更新本计划）：

- 文件命名 `<name>_hw.c` / `<name>_hw.h`，私有头放组件根目录（不进 `include/`，不扩 public API）。
- 提交拆分为：计划文档 1 提交 + 每组件 1 提交 + 验证回填 1 提交。

## Review Focus

规格隐含但没有任何任务的新测试直接覆盖的失效模式（重构等价性风险点）；每条已钉在所属任务的验证步骤里：

1. **tmc2209 未初始化守卫回归**：`tmc2209_read_reg/write_reg` 在 init 前必须仍返回 false（守卫随实现移入 hw.c，检查对象从 shell 状态变为 hw 状态）→ Task 1 Step 6 审计。
2. **tmc2209 init 失败路径等价**：mutex 已建、`uart_driver_install/param_config/set_pin` 失败时的返回码与可重入语义（旧代码重复 init 会重建 mutex，保持不修）→ Task 1 Step 6 审计。
3. **as5600 首读失败不阻断**：init 首读失败仅 LOGW 并返回 ESP_OK（采样任务继续重试）→ Task 2 Step 6 审计。
4. **mt6835 失败分诊信息不丢**：`last_fail` 分类与 6 字节帧原文在迁移后仍能被任务/初始化日志正确打出（经快照 getter 传递）→ Task 3 Step 6 审计。
5. **INVERT_DIR 镜像仍在读层生效**：`CONFIG_AS5600_INVERT_DIR` 改为运行时 config 布尔注入后，镜像（4095−raw）仍发生在读路径、tracker 仍只见 0..4095 → Task 2 Step 6 审计。

横切门禁（Task 4）：主工程与测试工程构建通过、host 纯逻辑测试全绿、hw 三文件 include 审计（实际 `#include` 清单，非文本搜索）。

---

### Task 1: tmc2209 — 拆出 UART 帧与寄存器读写驱动核心

**Files:**
- Create: `components/tmc2209/tmc2209_hw.h`（组件私有头）
- Create: `components/tmc2209/tmc2209_hw.c`
- Modify: `components/tmc2209/tmc2209.c`
- Modify: `components/tmc2209/CMakeLists.txt:2`（SRCS 追加 `"tmc2209_hw.c"`）

**Interfaces:**
- Consumes: 现有 public `tmc2209_config_t`（tmc2209.h:25-32，字段 uart_num/tx_pin/rx_pin/baud/slave_addr/r_sense）。
- Produces（hw 私有接缝，仅组件内使用）:
  - `typedef struct { int uart_num; int tx_pin; int rx_pin; int baud; uint8_t slave_addr; } tmc2209_hw_config_t;`
  - `esp_err_t tmc2209_hw_init(const tmc2209_hw_config_t *cfg);`
  - hw.c 同时承载 public 符号 `tmc2209_read_reg` / `tmc2209_write_reg` / `tmc2209_crc8` 的实现（声明仍在 include/tmc2209.h，不变）。

分层边界：寄存器**地址与位域语义**（REG_*、CHOPCONF_*、GCONF_*、IHOLD_IRUN_*、DRV_STATUS_*、PWMCONF_*）全部留在 shell——hw 只按地址做通用读写，不含任何寄存器语义。

- [x] **Step 1: 新建 tmc2209_hw.h（全文）**

```c
/*
 * tmc2209_hw — UART 帧与寄存器读写原语（驱动核心层，组件私有）
 *
 * 仅同组件业务壳 tmc2209.c 包含本头文件；不进入 include/。
 * 层内约束：参数全部由 tmc2209_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务；不触碰 GPIO（EN 等板级操作
 * 由业务壳负责）。寄存器地址与位域语义由业务壳组装，本层只按
 * 地址提供通用 32-bit 寄存器读写。
 *
 * public API（tmc2209.h 声明）中的 tmc2209_read_reg / tmc2209_write_reg /
 * tmc2209_crc8 实现于本层。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      uart_num;     /* UART port number, e.g. UART_NUM_2 */
    int      tx_pin;       /* TX GPIO (through 1kΩ to PDN_UART) */
    int      rx_pin;       /* RX GPIO (direct from PDN_UART) */
    int      baud;         /* Baud rate */
    uint8_t  slave_addr;   /* Slave address */
} tmc2209_hw_config_t;

/**
 * @brief 安装 UART 驱动并建立读写原语。幂等性由业务壳保证
 *        （与旧 tmc2209_init 相同：失败直接返回错误码）。
 */
esp_err_t tmc2209_hw_init(const tmc2209_hw_config_t *cfg);

#ifdef __cplusplus
}
#endif
```

- [x] **Step 2: 新建 tmc2209_hw.c**

搬移清单（全部自 `tmc2209.c` 原文逐字搬移，仅注明归属）：

| 内容 | 旧位置 | 处理 |
|---|---|---|
| 文件头注释（帧格式 + 回声处理说明） | tmc2209.c:1-14 | 随层搬移 |
| `TMC_SYNC_BYTE/TMC_WRITE_FLAG/TMC_READ_FLAG`、`READ_RESPONSE_TIMEOUT_MS` | :36-38, :127 | 原文搬移 |
| `s_uart_mutex`、hw 侧 config 拷贝与 initialized 状态 | :134-137 改造 | hw 持有 `static tmc2209_hw_config_t s_cfg; static bool s_initialized; static SemaphoreHandle_t s_uart_mutex;` |
| `tmc2209_crc8`（public 纯函数，帧构造依赖） | :143-158 | 原文搬移 |
| `build_write_dgram` / `build_read_dgram` / `parse_response_dgram` | :213-249 | 原文搬移 |
| `uart_write_reg_locked` / `uart_read_reg_locked` | :255-320 | 原文搬移 |
| `tmc2209_read_reg` / `tmc2209_write_reg`（public，mutex + 守卫） | :326-351 | 原文搬移（守卫对象即 hw 的 s_initialized，语义不变） |

新增 `tmc2209_hw_init`（承担旧 tmc2209.c:493-541 的 UART 建立段，逐行等价）：

```c
esp_err_t tmc2209_hw_init(const tmc2209_hw_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_initialized) return ESP_OK;

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* Create UART mutex */
    s_uart_mutex = xSemaphoreCreateMutex();
    if (s_uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Install UART driver */
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* RX/TX buffers must be STRICTLY greater than the UART FIFO length
     * (128 on ESP32-S3) — uart_driver_install(128, ...) fails with
     * "uart rx buffer length error".  256 gives headroom for the
     * two-wire echo (8-byte TX echoes back on RX). */
    esp_err_t err = uart_driver_install(cfg->uart_num, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(cfg->uart_num, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d: TX=%d RX=%d @ %d baud 8N1",
             cfg->uart_num, cfg->tx_pin, cfg->rx_pin, cfg->baud);

    s_initialized = true;
    return ESP_OK;
}
```

hw.c 的 include 白名单（Step 6 审计对象）：`tmc2209_hw.h`、`tmc2209.h`、`<string.h>`、`esp_log.h`、`driver/uart.h`、`freertos/FreeRTOS.h`、`freertos/semphr.h`。不含 settings.h / bsp.h。

- [x] **Step 3: 瘦身 tmc2209.c 为业务壳**

- 删除已搬移段（见 Step 2 清单）；保留全部寄存器地址/位域宏、`tmc2209_ma_to_cs`、`tmc2209_microstep_to_mres`、`calc_vsense`、功能层（set_current/set_microstep/enable/read_status）、`settings_change_cb`、init 的寄存器配置策略段。
- shell 状态收敛为：`static tmc2209_config_t s_cfg;`（仅 r_sense 被 shell 使用）、`static bool s_initialized;`、`static bool s_vsense;`。
- `tmc2209_init` 开头改为调用 hw（其余寄存器配置段逐行保留，s_initialized 置位位置保持在寄存器写之前、与旧代码一致）：

```c
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_initialized) return ESP_OK;

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    tmc2209_hw_config_t hw = {
        .uart_num   = cfg->uart_num,
        .tx_pin     = cfg->tx_pin,
        .rx_pin     = cfg->rx_pin,
        .baud       = cfg->baud,
        .slave_addr = cfg->slave_addr,
    };
    esp_err_t err = tmc2209_hw_init(&hw);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;

    /* ---- Apply default register configuration ---- */
    /*（原 :547-616 逐行保留：GCONF / CHOPCONF / IHOLD_IRUN /
     * TPOWERDOWN / TPWMTHRS / PWMCONF 写入、settings 读取、订阅） */
```

- include 清理：删 `driver/uart.h`、`freertos/semphr.h`、`esp_check.h`（shell 不再使用）；新增 `#include "tmc2209_hw.h"`。保留 `string.h`（memcpy）、`math.h`、`esp_log.h`、`freertos/FreeRTOS.h`（enable 的 vTaskDelay）、`bsp.h`、`settings.h`。
- 文件头注释更新为业务壳职责说明（寄存器策略/settings 订阅；帧与 UART 细节指向 tmc2209_hw.c）。

- [x] **Step 4: CMake 追加源文件**

`components/tmc2209/CMakeLists.txt`：`SRCS "tmc2209.c" "tmc2209_hw.c"`。REQUIRES 不变（driver/bsp/settings 仍为组件整体依赖；hw 隔离由 include 白名单保证）。

- [x] **Step 5: 构建主工程**

Run: MCP `build_project`（或 `idf.py -B build build`）
Expected: 构建通过，无未定义符号（read_reg/write_reg/crc8 由 hw.c 提供）。

- [x] **Step 6: 等价性审计（Review Focus 1/2）**

- 守卫：`tmc2209_read_reg/write_reg` 的 `if (!s_initialized) return false;` 现判 hw 状态；tmc2209_init 未调用前 hw 未初始化 → 行为不变。
- 失败路径：hw_init 内 mutex 建立成功而 uart 安装失败时返回原错误码、hw 保持未初始化（可重入重建 mutex，与旧代码一致，不修）。
- `tmc2209_init` 中 `s_initialized = true` 仍在默认寄存器写入之前（旧 :543）→ 功能层守卫时序不变。
- include 审计：列出 tmc2209_hw.c 全部 `#include`，确认无 settings.h/bsp.h。

- [x] **Step 7: Commit**

```bash
git add components/tmc2209 docs/superpowers/plans/2026-10-13-driver-core-split.md
git commit -m "tmc2209：拆出 UART 帧与寄存器读写驱动核心 tmc2209_hw（参数全由 config 注入，不碰 settings/bsp/任务），public API 不变"
```

---

### Task 2: as5600 — 拆出 I2C 读层 as5600_hw

**Files:**
- Create: `components/as5600/as5600_hw.h`
- Create: `components/as5600/as5600_hw.c`
- Modify: `components/as5600/as5600.c`
- Modify: `components/as5600/CMakeLists.txt:1`（SRCS 追加 `"as5600_hw.c"`）
- Modify: `components/as5600/Kconfig:21`（help 文案 `as5600_read_raw()` → `as5600_hw_read_raw()`，句子其余不变）

**Interfaces:**
- Consumes: `bsp_get_i2c_bus()`（bsp.h:55）、`CONFIG_BSP_AS5600_I2C_ADDR`、`CONFIG_BSP_I2C_FREQ_HZ`、`CONFIG_AS5600_INVERT_DIR`（Kconfig）。
- Produces:
  - `typedef struct { i2c_master_bus_handle_t bus; uint16_t dev_addr; uint32_t scl_speed_hz; bool invert_dir; } as5600_hw_config_t;`
  - `esp_err_t as5600_hw_init(const as5600_hw_config_t *cfg);`（add_device，含 `I2C_ADDR_BIT_LEN_7`）
  - `bool as5600_hw_read_raw(uint16_t *raw);`（2 字节突发读 12-bit 原始角；失败时 `*raw` 不变）

- [x] **Step 1: 新建 as5600_hw.h（全文）**

```c
/*
 * as5600_hw — AS5600 I2C 读层（驱动核心层，组件私有）
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   ZPOS / MPOS / CONF / MANG 是一次性可编程（OTP）：写错即永久
 *   损坏芯片。本层不包含任何寄存器写：唯一总线操作是
 *   as5600_hw_read_raw()，使用 i2c_master_transmit_receive()
 *   （寄存器地址出 + 数据入，单事务 = 读）。不得在此添加
 *   i2c_master_transmit() 或任何写辅助。
 *
 * 层内约束：参数全部由 as5600_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务。方向镜像（invert_dir，
 * 磁铁安装标定）发生在本读层，业务壳与纯逻辑 tracker 只见
 * 0..4095 的已定向读数。
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
    i2c_master_bus_handle_t bus;     /* 已初始化的 I2C 总线（业务壳从 bsp 获取） */
    uint16_t                dev_addr;     /* 7-bit 从机地址 */
    uint32_t                scl_speed_hz; /* 总线时钟 */
    bool                    invert_dir;   /* true = 镜像 raw 角（安装方向标定） */
} as5600_hw_config_t;

/**
 * @brief 在 config 给出的总线上添加 AS5600 设备。错误码与
 *        i2c_master_bus_add_device 一致。
 */
esp_err_t as5600_hw_init(const as5600_hw_config_t *cfg);

/**
 * @brief 读 12-bit 原始角（RAW ANGLE 0x0C/0x0D 单次 2 字节突发）。
 * @return true 成功（*raw ∈ 0..4095，已按 invert_dir 镜像）；
 *         false 事务失败（*raw 不变）。
 */
bool as5600_hw_read_raw(uint16_t *raw);

#ifdef __cplusplus
}
#endif
```

- [x] **Step 2: 新建 as5600_hw.c**

搬移/改写清单（源 `as5600.c`）：

| 内容 | 旧位置 | 处理 |
|---|---|---|
| 只读红线 + 寄存器图注释 | :1-14 相关节选、:46-50 | 随层搬移（STATUS/AGC/MAGNITUDE 地址注释一并保留） |
| `AS5600_I2C_TIMEOUT_MS` | :53 | 原文搬移 |
| `as5600_read_raw` | :97-124 | 改名 `as5600_hw_read_raw`，事务体逐字保留；`#if CONFIG_AS5600_INVERT_DIR` 编译期镜像改为 `if (s_cfg.invert_dir)` 运行时镜像（同一表达式 `4095 - raw`） |
| 设备句柄状态 | :86 | hw 持有 `static i2c_master_dev_handle_t s_dev;` + `static as5600_hw_config_t s_cfg;` |

新增 `as5600_hw_init`（承担旧 as5600.c:250-256 的 add_device 段）：

```c
esp_err_t as5600_hw_init(const as5600_hw_config_t *cfg)
{
    if (cfg == NULL || cfg->bus == NULL) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->dev_addr,
        .scl_speed_hz    = cfg->scl_speed_hz,
    };
    return i2c_master_bus_add_device(cfg->bus, &dev_cfg, &s_dev);
}
```

hw.c include 白名单：`as5600_hw.h`、`as5600.h`（`AS5600_COUNTS_PER_REV`）、`esp_log.h`（预留，若最终未用则删）。不含 settings.h / bsp.h。

- [x] **Step 3: 瘦身 as5600.c 为业务壳**

- 删除已搬移段；保留：状态结构与 s_lock、publish、refresh_scale、settings 回调、采样任务、init 其余段、遥测 getters、set_origin。
- `as5600_init` 中 bus 获取与 config 填充 + hw_init + 首读（`as5600_hw_read_raw`）：

```c
    i2c_master_bus_handle_t bus = bsp_get_i2c_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized (bsp_get_i2c_bus() == NULL)");
        return ESP_ERR_INVALID_STATE;
    }

#ifdef CONFIG_AS5600_INVERT_DIR
#define AS5600_DIR_INVERTED 1
#else
#define AS5600_DIR_INVERTED 0
#endif
    as5600_hw_config_t hw = {
        .bus          = bus,
        .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
        .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
        .invert_dir   = AS5600_DIR_INVERTED,
    };
    ESP_RETURN_ON_ERROR(as5600_hw_init(&hw),
                        TAG, "as5600_hw_init failed");
```

（`AS5600_DIR_INVERTED` 宏置于函数外文件顶部，同时供 :269 方向日志使用，替换其直接引用 `CONFIG_AS5600_INVERT_DIR` 的三目条件，输出文案不变。）

- 任务循环与首读处 `as5600_read_raw(&raw)` → `as5600_hw_read_raw(&raw)`。
- include 清理：删 `driver/i2c_master.h`（句柄类型经 as5600_hw.h 引入）；新增 `#include "as5600_hw.h"`。其余保留（bsp.h、settings.h、freertos、esp_task_wdt、esp_log、esp_check、math.h——math.h 若 shell 已无使用则删，以实际审计为准）。
- 文件头注释：数据路径/CPU1/2 字节突发等任务侧说明保留；只读红线改为一句指向 as5600_hw.c。

- [x] **Step 4: CMake + Kconfig**

- CMakeLists：`SRCS "as5600.c" "as5600_tracker.c" "as5600_hw.c"`。
- Kconfig AS5600_INVERT_DIR help：`applied in as5600_read_raw() (driver layer)` → `applied in as5600_hw_read_raw() (driver core layer)`。

- [x] **Step 5: 构建主工程**

Run: MCP `build_project`
Expected: 通过。

- [x] **Step 6: 等价性审计（Review Focus 3/5）**

- 首读失败路径：init 首读失败仍走 LOGW 分支并继续（返回 ESP_OK），任务创建与 s_initialized 置位时序不变。
- 镜像：`invert_dir=true` 时读层输出仍为 `(4095 - raw)`，与 `#if` 版本对同一 Kconfig 取值逐位一致；invert=false 时不镜像。
- include 审计：as5600_hw.c 无 settings.h / bsp.h / freertos 任务 API。

- [x] **Step 7: Commit**

```bash
git add components/as5600
git commit -m "as5600：拆出 I2C 读层驱动核心 as5600_hw（只读红线随迁，地址/时钟/方向镜像全由 config 注入），public API 不变"
```

---

### Task 3: mt6835 — 拆出 SPI 帧层 mt6835_hw

**Files:**
- Create: `components/mt6835/mt6835_hw.h`
- Create: `components/mt6835/mt6835_hw.c`
- Modify: `components/mt6835/mt6835.c`
- Modify: `components/mt6835/CMakeLists.txt:2`（SRCS 追加 `"mt6835_hw.c"`）

**Interfaces:**
- Consumes: `bsp_get_spi_host()`（bsp.h:62）、`CONFIG_BSP_MT6835_SPI_HZ`、`CONFIG_BSP_MT6835_CS_GPIO`、`mt6835_crc8`（mt6835_urad.h:52）。
- Produces:
  - `#define MT6835_HW_FRAME_BYTES 6`
  - `typedef enum { MT6835_HW_FAIL_SPI_ERR, MT6835_HW_FAIL_ALL_ZERO, MT6835_HW_FAIL_CRC } mt6835_hw_fail_t;`
  - `typedef struct { spi_host_device_t host; int clock_speed_hz; int spics_io_num; } mt6835_hw_config_t;`
  - `esp_err_t mt6835_hw_init(const mt6835_hw_config_t *cfg);`（add_device，MODE3）
  - `bool mt6835_hw_read_raw(uint32_t *raw21, uint8_t *status);`
  - `void mt6835_hw_last_failure(mt6835_hw_fail_t *kind, uint8_t frame[MT6835_HW_FRAME_BYTES]);`（最近一次失败快照；kind 可为 NULL，frame 可为 NULL，各自跳过）

- [x] **Step 1: 新建 mt6835_hw.h（全文）**

```c
/*
 * mt6835_hw — SPI 角度帧层（驱动核心层，组件私有）
 *
 * 48-bit 单次事务（MODE3、MSB 先行；帧格式抄录与 CRC8 见
 * mt6835_urad.h 与 docs/reviews/T4-brief.md）：
 *   TX: [0]=OP_ANGLE(0b1010)<<4, [1]=ANGLE1 地址 0x03, [2..5]=0
 *   RX: [2]=ANGLE[20:13] [3]=ANGLE[12:5]
 *       [4]=ANGLE[4:0]<<3 | STATUS[2:0]  [5]=器件 CRC8
 * 帧校验：全零帧（MISO 悬空）与 CRC 不符均判失败，并留存帧原文
 * 供业务壳分诊日志。
 *
 * 层内约束：参数全部由 mt6835_hw_init 的 config 注入；不 include
 * settings.h；不调用 bsp；不创建任务。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MT6835_HW_FRAME_BYTES 6

/* 失败分类（分诊字符串由业务壳维护） */
typedef enum {
    MT6835_HW_FAIL_SPI_ERR = 0,  /* SPI 事务错误 */
    MT6835_HW_FAIL_ALL_ZERO,     /* 全零帧（MISO 悬空/器件未应答） */
    MT6835_HW_FAIL_CRC,          /* CRC 不符 */
} mt6835_hw_fail_t;

typedef struct {
    spi_host_device_t host;        /* 已初始化的 SPI 总线（业务壳从 bsp 获取） */
    int               clock_speed_hz;
    int               spics_io_num;
} mt6835_hw_config_t;

/** 在 config 给出的总线上添加 MT6835 设备（MODE3）。 */
esp_err_t mt6835_hw_init(const mt6835_hw_config_t *cfg);

/**
 * @brief 读一次 21-bit 原始角与 STATUS[2:0]（帧校验含全零/CRC）。
 * @return false = 事务或帧校验失败（*raw21 / *status 不变）；
 *         失败详情随后用 mt6835_hw_last_failure() 取快照。
 */
bool mt6835_hw_read_raw(uint32_t *raw21, uint8_t *status);

/** 最近一次失败的分类与 6 字节帧原文（无失败时返回上次值）。 */
void mt6835_hw_last_failure(mt6835_hw_fail_t *kind,
                            uint8_t frame[MT6835_HW_FRAME_BYTES]);

#ifdef __cplusplus
}
#endif
```

- [x] **Step 2: 新建 mt6835_hw.c**

搬移/改写清单（源 `mt6835.c`）：

| 内容 | 旧位置 | 处理 |
|---|---|---|
| 帧常量 `OP_ANGLE/REG_ANGLE1/FRAME_BYTES` | :32-34 | 搬移（FRAME_BYTES 用 hw.h 宏，本地宏删除） |
| 事务头注释（T4-brief 抄录） | :4-13 节选 | 随层搬移 |
| `mt6835_read_raw` | :86-134 | 改名 `mt6835_hw_read_raw`，事务/解码/校验体逐字保留 |
| 失败分诊状态 `s_last_fail/s_last_frame` | :67-69 | hw 持有；枚举改用 `mt6835_hw_fail_t` |
| 新增 `mt6835_hw_last_failure` | — | 快照拷贝（见接口） |

新增 `mt6835_hw_init`（承担旧 mt6835.c:241-248）：

```c
esp_err_t mt6835_hw_init(const mt6835_hw_config_t *cfg)
{
    if (cfg == NULL || cfg->host < 0) return ESP_ERR_INVALID_ARG;

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->clock_speed_hz,
        .mode = 3,                          /* CPOL=1 CPHA=2（MODE3） */
        .spics_io_num = cfg->spics_io_num,
        .queue_size = 1,
    };
    return spi_bus_add_device(cfg->host, &dev_cfg, &s_dev);
}
```

`mt6835_hw_read_raw` 失败分支中 `s_last_fail = FAIL_SPI_ERR;` 等三处枚举名替换为 `MT6835_HW_FAIL_*`，其余逐字保留。

hw.c include 白名单：`mt6835_hw.h`、`mt6835_urad.h`（crc8）、`driver/spi_master.h`。不含 settings.h / bsp.h / esp_log（读路径本无日志）。

- [x] **Step 3: 瘦身 mt6835.c 为业务壳**

- 删除已搬移段；保留：状态结构/publish/速度窗、采样任务、init 其余段、遥测 getters。
- `fail_kind_t` 本地枚举删除；`fail_str` 改为接收 `mt6835_hw_fail_t`（switch 三臂文案不变）；`s_last_fail/s_last_frame` 删除，改经 `mt6835_hw_last_failure` 取快照：

```c
/* 任务失败分支（原 :204-214）等价改写 */
fail_streak++;
if (fail_streak == MT6835_FAIL_LIMIT) {
    s_ok = false;
    mt6835_hw_fail_t kind;
    uint8_t frame[MT6835_HW_FRAME_BYTES];
    mt6835_hw_last_failure(&kind, frame);
    ESP_LOGE(TAG, "SPI 读连续失败 %d 次——编码器标记 NOT OK（%s）",
             fail_streak, fail_str(kind));
    ESP_LOGE(TAG, "末帧 rx = %02x %02x %02x %02x %02x %02x"
             "（tx = a0 03 00 00 00 00；rx[2..4]=角度/status，"
             "rx[5]=器件 CRC）",
             frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
}
```

（init 首读失败处的两行日志 :260-264 同法改写。）

- `mt6835_init` 总线获取 + config 填充：

```c
    spi_host_device_t host = bsp_get_spi_host();
    if (host < 0) {
        ESP_LOGE(TAG, "SPI 总线未初始化（bsp_get_spi_host() < 0）");
        return ESP_ERR_INVALID_STATE;
    }

    mt6835_hw_config_t hw = {
        .host          = host,
        .clock_speed_hz = CONFIG_BSP_MT6835_SPI_HZ,
        .spics_io_num  = CONFIG_BSP_MT6835_CS_GPIO,
    };
    ESP_RETURN_ON_ERROR(mt6835_hw_init(&hw), TAG, "mt6835_hw_init 失败");
```

- `mt6835_read_raw` 两处调用点（任务 :189、init 首读 :255）→ `mt6835_hw_read_raw`。
- include 清理：删 `driver/spi_master.h`；新增 `#include "mt6835_hw.h"`。`string.h`（memset s_state）、freertos、esp_log、esp_check、esp_task_wdt、bsp.h 保留。文件头注释补一行"SPI 帧层见 mt6835_hw.c"。

- [x] **Step 4: CMake 追加源文件**

`SRCS "mt6835.c" "mt6835_urad.c" "mt6835_hw.c"`。

- [x] **Step 5: 构建主工程**

Run: MCP `build_project`
Expected: 通过。

- [x] **Step 6: 等价性审计（Review Focus 4）**

- 分诊快照：失败后 task/init 日志打出的 kind 与 6 字节帧原文与旧实现同源（同一 s_last_frame 拷贝）；成功读不清快照（旧代码同样保留上次失败帧）。
- include 审计：mt6835_hw.c 无 settings.h / bsp.h / freertos。

- [x] **Step 7: Commit**

```bash
git add components/mt6835
git commit -m "mt6835：拆出 SPI 帧层驱动核心 mt6835_hw（host/时钟/CS 由 config 注入，失败分诊快照接口），public API 不变"
```

---

### Task 4: 全量验证与回填

**Files:**
- Modify: `docs/superpowers/plans/2026-10-13-driver-core-split.md`（勾选 + 验证记录）
- Modify: 记忆文件 `pd-step-driver-coupling.md`（拆分完成，结论更新）

**Interfaces:** 无代码接口。

- [x] **Step 1: 主工程构建**

Run: MCP `build_project`
Expected: 通过。

- [x] **Step 2: 测试工程构建**

Run: `powershell -Command ". C:\esp\v6.1\esp-idf\export.ps1; idf.py -C test build"`（或仓库 idf_env.ps1 提供的等价入口）
Expected: 通过——所有 TEST_COMPONENTS（含三个组件 test/）编入链接，证明 public 符号闭合、测试代码零改动适配。

- [x] **Step 3: host 纯逻辑测试回归**

Run（六个宿主测试逐一，wsl gcc，命令模板见 README.md:93-96）:
`wsl gcc -std=c11 -Wall -Wextra -Werror -Ihost_tests/stub -Icomponents/mt6835/include host_tests/mt6835_crc_test.c components/mt6835/mt6835_urad.c -o /tmp/t && /tmp/t`（urad/crc/cartfsm_core/cartproto_parse/telemetry_format/vel_ramp 同法）
Expected: 全部退出码 0。

- [x] **Step 4: include 审计汇总（横切门禁）**

列出三个 `*_hw.c` 的全部 `#include` 并核对白名单（esp 总线/日志头 + 自家 hw.h + public 头 + mt6835_urad.h）；确认无 settings.h / bsp.h / freertos 任务与队列头（tmc2209_hw 的 FreeRTOS/semphr 属 UART 互斥原语，允许）。

- [x] **Step 5: 未验证项清单（如实汇报，不冒充已验）**

- 真机行为回归（[bringup] 寄存器核对、as5600 B 组硬件读、mt6835 静止稳定性）需上板运行测试固件，本会话未执行；构建通过仅为编译级证据。
- `CONFIG_AS5600_INVERT_DIR=n` 分支的编译（旧代码该分支本就不可编译，非本次引入）。

- [x] **Step 6: 回填与提交**

勾选本计划各步骤、写入验证记录（日期、环境、命令、结果）；更新记忆 pd-step-driver-coupling（拆分已完成、新接缝名称）。

```bash
git add docs/superpowers/plans/2026-10-13-driver-core-split.md
git commit -m "驱动核心拆分：验证记录回填（主/测试工程构建、host 回归、include 审计）"
```

---

## 执行方式说明（会话内已定）

本会话为自主执行模式（用户不实时应答），writing-plans 的"等待用户评审后执行"与自主模式冲突，按系统级指令优先处理：计划先行落档（可事后评审、git 可回退），由本会话按 executing-plans 方式逐任务执行；任务间发现与计划冲突的代码事实时，按实施规则停止该任务并汇报，不静默改语义。

---

## 验证记录（2026-10-13 执行会话回填）

环境：Windows 10 + ESP-IDF v6.1（IDF 于 C:\esp\v6.1\esp-idf，工具与 Python 用 C:\Espressif 的 EIM 安装，激活脚本 Microsoft.v6.1.PowerShell_profile.ps1）。

- 主工程构建：`idf.py -B build build` → Project build complete，退出码 0（Task 1/2/3 各验证一次）。
- 测试工程构建：`idf.py -C test build` → 生成 test\build\cartpole-test.bin，退出码 0，全部 TEST_COMPONENTS 编入，测试代码零改动适配。注意：不能写 `-C test -B build`——`-B` 相对当前目录解析会指向根工程 build 目录而被 IDF 拒绝。
- 宿主纯逻辑测试（`wsl gcc -std=c11 -Wall -Wextra -Werror`，退出码全 0）：cartfsm_core 79/79（须同时编译 params.c）、cartproto_parse 57/57、mt6835_crc 2/2、mt6835_urad 19/19、telemetry_format 15/15（需额外 -I as5600/mt6835/settings 的 include）、vel_ramp 48/48。
- include 审计（三个 _hw.c 的全部 #include）：
  - tmc2209_hw.c：自家 hw.h、string.h、public tmc2209.h、esp_log.h、driver/uart.h、freertos/FreeRTOS.h、freertos/semphr.h（互斥原语，允许集）
  - as5600_hw.c：as5600_hw.h、as5600.h
  - mt6835_hw.c：mt6835_hw.h、mt6835_urad.h
  - 三者均无 settings.h、无 bsp.h；构建通过为编译级依赖证据。
- 本会话未执行（上板验证项，不冒充已验）：[bringup] 寄存器核对、[spin] 实转、as5600 B 组硬件读、mt6835 静止稳定性与拔线分诊。真机行为回归需烧录测试固件后按 README 标签运行。
- 已知既有问题（非本次引入）：CONFIG_AS5600_INVERT_DIR=n 时旧代码本就无法编译（方向日志三目直接引用未定义宏）；本次的 AS5600_DIR_INVERTED 折算不受影响。

执行期裁定（详细过程见最终交接说明）：
1. 构建入口由计划写的 export.ps1 改为 EIM 激活脚本——export.ps1 会因 Git Bash 自动注入的 MSYSTEM 被 IDF 拒绝，且 EIM Python 与现有 build 缓存匹配。
2. tmc2209/CMakeLists 的 SRCS 由协作者先行加入 tmc2209_hw.c，与目标一致，保留其书写顺序。
3. as5600_hw_init 额外存储整个 config（读取时用运行时 invert_dir），是"镜像发生在读层"的必要实现，镜像表达式与原 #if 版本逐位一致。
