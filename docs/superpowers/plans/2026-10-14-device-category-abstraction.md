# 器件类别抽象与解释层上收实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 magnetic_encoder / motor 两个器件类别组件（句柄式、原生单位 counts / steps），把 cartpos / poleang 重定位为解释组件、新增 cartaxis 直线轴解释组件，业务数据反向依赖清零。

**Architecture:** 类别契约只含器件本征语义；解释层（业务侧）持有句柄做单位换算与标定；组装点创建实例、分发句柄并中转纯驱动器参数（`$3/$4`）。现有设备组件的采样任务/失败计数原样搬入类别实现，估计器语义不变。

**Tech Stack:** ESP-IDF v6.1（C、CMake 组件、Kconfig、Unity 测试工程）、gcc 宿主纯逻辑测试。

**Spec:** [2026-10-14-device-category-abstraction.md](../specs/2026-10-14-device-category-abstraction.md)（计划从规格推导；执行者实施任何任务前先读规格全文）

## Global Constraints

- 句柄式运行期绑定；类别契约头零组件依赖；每实现单实例（本台位一台编码器 ×2、电机 ×1）。
- CMake 求值陷阱（上轮实测）：类别组件的 REQUIRES / PRIV_REQUIRES 一律**无条件字面量声明**，仅 SRCS 随 Kconfig 宏条件编入。
- Kconfig：`MAGNETIC_ENCODER_AS5600` / `MAGNETIC_ENCODER_MT6835` 为两个**独立 bool**（默认 y，不是 choice）；`MOTOR_IMPL_TMC2209` choice 保留。
- 迁移不变量 1–6（规格「迁移不变量」节）：FAULT 停机次序、重定基时序、锁序、协议数值域兼容（x_um ≤1 µm、vx ≤1 mm/s、vth ≤1 mrad/s、指令位置同 x_um；行格式与 reason 逐字节）、测试注入语义、解释层单一权威。
- 术语全称（[writing.md](../../../.agents/rules/writing.md)）；被删除的 API 不得在注释/文档残留引用。
- 每个任务收尾：主工程与测试工程构建通过；涉及纯逻辑源的任务复跑对应 host 测试。
- 构建命令：`powershell -NoProfile -ExecutionPolicy Bypass -Command "Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue; Remove-Item Env:SHELL -ErrorAction SilentlyContinue; & 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'; idf.py -B build build"`（测试工程把末参数换 `idf.py -C test build`）。宿主测试：`wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && gcc -std=c11 -Wall -Wextra -I<目录> <测试.c> <源.c> -lm -o /tmp/t && /tmp/t"`，退出码 = 失败断言数。

## 规格补全与偏差记录（1–3 计划期发现向用户报告过；4–5 实施期发现）

1. 规格写 `step_engine_set_dir_invert`「无调用者」——实际 [test_step_engine.c:291](../../../components/step_engine/test/test_step_engine.c) 的用例 7 在调用（历史 Critical「陈旧 DIR 电平」的回归锚）。处置：连同该用例一并删除。锚定的问题在符号上收后**结构性消失**——cartaxis 的 X_INV 是命令取反，无静止期标志翻转，DIR 电平总在运动启动时重驱。
2. motor 契约速度为整数 steps/s：对非整数 steps_per_mm 的台位，命令目标比旧的 Q16.16 单次舍入多一层 ≤0.5 steps/s 量化（远小于 VEL 命令 1 mm/s = 40 steps/s 的粒度）。默认台位 spm=40 为整数，两条路径逐位一致；数值域兼容结论不受影响。
3. tmc2209 去 settings 后的上电初值（细分/电流）来源规格未写：**经 config 注入**——`tmc2209_config_t` 增加 `init_microstep / init_run_ma / init_hold_ma` 三字段，组装点从 settings 读取填入（与规格「接线经 config 注入」原则一致）。
4. **Task 2 实施期发现（已修复）**：计划 Task 1/Task 2 的实现代码让每个实现各自定义同名契约四函数并忽略句柄参数——单实现可链接，第二个实现编入即重复符号、句柄无法区分两台编码器，与规格「两实例并存、句柄分发」意图不符。修复：契约头定义操作表（`struct magnetic_encoder_s` 四函数指针）+ 静态内联分发函数；各实现把操作表作为实例结构首成员（init 返回其地址），四个契约函数降为实现内静态操作表函数。消费方（cartpos/poleang/组装点/测试）调用面不变；规格「绑定方式」段已同步。
5. 断言数勘误（附录 B 回填口径）：t5 telemetry_format 由 15 → **10**（被删 counts_to_um 段实为 5 条断言）；t4 mt6835_urad 由 19 → **16**（被删 counts_to_mrad_s 段实为 3 条断言）。

## Review Focus

1. **X_INV 取反对称**：set_vel / cmd_position_um / set_position 三点必须同取反，漏一处即 E−C 系统性偏差 → 丢步误报。锚：Task 4 代码三处并列审查 + 附录 A 第 7 项。
2. **theta 常量算术逐位一致**：poleang 的 2π=6283185、u64 中间量、±π wrap 必须与 mt6835_raw_to_urad 相同，否则违反数值域不变量。锚：Task 2 的 on-target 对拍用例（poleang 与 mt6835_urad 同输入同值）。
3. **$0 写者状态机**：静止即时应用 / 运动中挂起后由入口驱动应用 / 写失败放弃并告警，三条路径与 DISABLED（必静止→回调内应用）交叉。锚：Task 4 决策表实现逐行对照 + 附录 A 第 6 项。
4. **速度 62.5 counts/s 量子**：B16 静止速度阈值换算（5 mm/s ≡ 512 counts/s @40 mm/4096）、故障恢复瞬间的窗口行为不变。锚：Task 1 B16 改写 + 附录 A 第 2 项。
5. **依赖规则构建期闭合**：类别实现不得 REQUIRES settings/cartparams/bsp；解释层不得依赖器件组件。锚：Task 5 的 CMake 清单 grep 验证 + 双工程构建。

---

### Task 1: magnetic_encoder 类别组件（契约 + AS5600 实现）+ as5600 瘦身 + cartpos 重定位

**Files:**
- Create: `components/magnetic_encoder/include/magnetic_encoder.h`
- Create: `components/magnetic_encoder/include/magnetic_encoder_as5600.h`
- Create: `components/magnetic_encoder/magnetic_encoder_as5600.c`
- Create: `components/magnetic_encoder/Kconfig`
- Create: `components/magnetic_encoder/CMakeLists.txt`
- Create: `components/magnetic_encoder/test/test_magnetic_encoder.c`
- Create: `components/magnetic_encoder/test/CMakeLists.txt`
- Modify: `components/as5600/include/as5600.h`（瘦身为 tracker + 红线文档）
- Delete: `components/as5600/as5600.c`
- Modify: `components/as5600/CMakeLists.txt`
- Modify: `components/as5600/test/test_as5600.c`（只留 tracker A 组）
- Delete: `components/cartpos/cartpos_as5600.c`、`components/cartpos/include/cartpos_as5600.h`、`components/cartpos/Kconfig`
- Create: `components/cartpos/cartpos.c`
- Modify: `components/cartpos/include/cartpos.h`、`components/cartpos/CMakeLists.txt`
- Modify: `main/app_main.c`、`main/CMakeLists.txt`、`test/main/test_app_main.c`、`test/main/CMakeLists.txt`、`test/CMakeLists.txt`
- Modify: `host_tests/telemetry_format_test.c`

**Interfaces:**
- Consumes: `as5600_hw_*`、`as5600_tracker_*`（as5600 组件保留）、`bsp_get_i2c_bus()`、settings `$2`
- Produces: `magnetic_encoder_handle_t` 与契约四函数；`magnetic_encoder_as5600_init(cfg, &h)`；cartpos 四函数签名不变 + `cartpos_init(h)`

- [x] **Step 1: 新建 magnetic_encoder 契约与 AS5600 实现**

`components/magnetic_encoder/include/magnetic_encoder.h`：

```c
/*
 * magnetic_encoder — 绝对式磁编码器类别契约（句柄式多实例）
 *
 * 原生单位原则：契约只含 counts 与每转计数，不出现机械单位
 * （mm、µrad）与已标定角度——换算归解释层（cartpos / poleang）。
 * 命名与 count / accumulator 词汇参考 espp/magnetic_encoder。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct magnetic_encoder_s *magnetic_encoder_handle_t;

/** 每转原始计数（分辨率）：AS5600 为 4096，MT6835 为 2097152。 */
int32_t magnetic_encoder_counts_per_rev(magnetic_encoder_handle_t enc);

/**
 * @brief 多圈累计位置（accumulator：原始 counts，带符号，原生方向，
 *        自 init 累计）；首样本前为 0。
 *
 * int64 承载：连续单方向旋转不溢出。混叠上界：unwrap 以
 * ±每转计数/2 为阈值，单个采样周期内转过超过半转即混叠——本值
 * 与速度估计都不可信（AS5600 @1 kHz 上界约 122 rev/s，MT6835
 * 约 403 rev/s，远高于本台位物理速度）。
 */
int64_t magnetic_encoder_position_counts(magnetic_encoder_handle_t enc);

/** 角速度估计（counts/s，16 窗均值）。量子 62.5 counts/s；窗口未满为 0。 */
int32_t magnetic_encoder_velocity_counts_per_s(magnetic_encoder_handle_t enc);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool magnetic_encoder_is_ok(magnetic_encoder_handle_t enc);

#ifdef __cplusplus
}
#endif
```

`components/magnetic_encoder/include/magnetic_encoder_as5600.h`：

```c
/*
 * magnetic_encoder_as5600 — magnetic_encoder 类别的 AS5600 实现
 * （init 与接线 config + 器件自测面）
 *
 * 接线参数（总线、地址、时钟、方向镜像）由组装点读取 Kconfig 宏
 * 与 bsp 总线句柄后经本 config 注入。方向镜像是磁铁安装属性，
 * 在读层（as5600_hw）应用于 12-bit 原始角，不出现在类别契约。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;     /* 已初始化的 I2C 总线（组装点从 bsp 获取） */
    uint16_t                dev_addr;     /* 7-bit 从机地址 */
    uint32_t                scl_speed_hz; /* 总线时钟 */
    bool                    invert_dir;   /* 方向镜像（安装标定） */
} magnetic_encoder_as5600_config_t;

/**
 * @brief 注入接线参数并启动 1 kHz 采样任务。幂等。每实现单实例
 *        （本台位一台），句柄式留多实例演进。
 *
 * @return ESP_OK；cfg 或 handle_out 为 NULL、bus 为 NULL 返回
 *         ESP_ERR_INVALID_ARG；其余错误码与迁移前一致
 *         （I2C / 任务创建错误）。
 */
esp_err_t magnetic_encoder_as5600_init(const magnetic_encoder_as5600_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out);

/** 最新 12-bit 原始角 0..4095（器件自测：静止噪声 / 链路健康）。 */
uint16_t magnetic_encoder_as5600_last_raw(magnetic_encoder_handle_t enc);

/** 方向镜像的 Kconfig 默认值（折叠 CONFIG_AS5600_INVERT_DIR；组装点使用）。 */
bool magnetic_encoder_as5600_invert_dir_default(void);

#ifdef __cplusplus
}
#endif
```

`components/magnetic_encoder/magnetic_encoder_as5600.c`：

```c
/*
 * magnetic_encoder_as5600 — magnetic_encoder 契约的 AS5600 实现
 *
 * 吸收自原 as5600.c 设备组件：1 kHz 采样任务（CPU1、优先级 3，
 * I2C 阻塞事务不扰 CPU0 的 STEP ISR）、连续失败计数（≥10 次
 * NOT OK）、16 样本速度窗。只读红线与寄存器图见 as5600_hw.c；
 * 多圈 tracker 为纯逻辑（as5600_tracker.c，宿主可测）。
 *
 * 发布语义：契约值在 s_lock 内原子发布；速度 = 16 ms 窗位置差
 * × 62.5（量子 62.5 counts/s）。2 字节突发读避免跨内部角度更新
 * 的拼接（见 as5600_hw）。
 */
#include "magnetic_encoder_as5600.h"
#include "as5600.h"
#include "as5600_hw.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

static const char *TAG = "me_as5600";

#define ME_AS5600_SAMPLE_PERIOD_MS  1      /* 1 kHz */
#define ME_AS5600_FAIL_LIMIT        10
#define ME_AS5600_TASK_PRIO         3
#define ME_AS5600_TASK_CORE         1
#define ME_AS5600_TASK_STACK        3072

/* ---- 速度：16 ms 窗（17 个 1 ms 位置样本） ---- */
#define ME_AS5600_SPEED_WINDOW     16
#define ME_AS5600_HIST_LEN         (ME_AS5600_SPEED_WINDOW + 1)

/* ---- 发布状态（s_lock 保护） ---- */
typedef struct magnetic_encoder_s {
    int64_t  pos_counts;             /* 多圈累计（accumulator） */
    int32_t  hist[ME_AS5600_HIST_LEN];
    int      hist_head;
    int      hist_fill;
    int32_t  vel_counts_per_s;
    uint16_t last_raw;
} me_state_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static me_state_t s_state;
static as5600_tracker_t s_tracker;
static volatile bool s_ok = true;        /* false = I2C failing */
static bool s_initialized = false;

/* Store one new sample and refresh the velocity estimate.
 * Called from the sampling task only. */
static void publish(int32_t pos_counts, uint16_t raw)
{
    portENTER_CRITICAL(&s_lock);

    s_state.pos_counts = pos_counts;
    s_state.last_raw   = raw;

    s_state.hist_head = (s_state.hist_head + 1) % ME_AS5600_HIST_LEN;
    s_state.hist[s_state.hist_head] = pos_counts;
    if (s_state.hist_fill < ME_AS5600_HIST_LEN) {
        s_state.hist_fill++;
    }

    if (s_state.hist_fill == ME_AS5600_HIST_LEN) {
        int     oldest = (s_state.hist_head + 1) % ME_AS5600_HIST_LEN;
        int32_t delta  = pos_counts - s_state.hist[oldest];
        /* counts over 16 ms → counts/s（量子 62.5） */
        s_state.vel_counts_per_s =
            (int32_t)llround((double)delta * (1000.0 / ME_AS5600_SPEED_WINDOW));
    } else {
        s_state.vel_counts_per_s = 0;   /* window not filled yet */
    }

    portEXIT_CRITICAL(&s_lock);
}

static void me_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int        fail_streak = 0;

    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(TAG, "task WDT subscribe failed — sampling task not monitored");
    }

    while (1) {
        uint16_t raw = 0;

        if (as5600_hw_read_raw(&raw)) {
            if (fail_streak >= ME_AS5600_FAIL_LIMIT) {
                ESP_LOGI(TAG, "I2C recovered after %d consecutive failures",
                         fail_streak);
            }
            fail_streak = 0;
            s_ok = true;

            int32_t pos = as5600_tracker_update(&s_tracker, raw);
            publish(pos, raw);
        } else {
            fail_streak++;
            if (fail_streak == ME_AS5600_FAIL_LIMIT) {
                s_ok = false;
                ESP_LOGE(TAG, "I2C read failed %d times in a row — encoder marked NOT OK",
                         fail_streak);
            }
        }

        if (wdt_ok) {
            esp_task_wdt_reset();
        }

        /* Fixed 1 kHz cadence; never busy-wait (watchdog). */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ME_AS5600_SAMPLE_PERIOD_MS));
    }
}

/* ---- 契约函数 ---- */

int32_t magnetic_encoder_counts_per_rev(magnetic_encoder_handle_t enc)
{
    (void)enc;
    return AS5600_COUNTS_PER_REV;
}

int64_t magnetic_encoder_position_counts(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    int64_t p = s_state.pos_counts;
    portEXIT_CRITICAL(&s_lock);
    return p;
}

int32_t magnetic_encoder_velocity_counts_per_s(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    int32_t v = s_state.vel_counts_per_s;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

bool magnetic_encoder_is_ok(magnetic_encoder_handle_t enc)
{
    (void)enc;
    return s_ok;
}

uint16_t magnetic_encoder_as5600_last_raw(magnetic_encoder_handle_t enc)
{
    (void)enc;
    portENTER_CRITICAL(&s_lock);
    uint16_t r = s_state.last_raw;
    portEXIT_CRITICAL(&s_lock);
    return r;
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
    if (s_initialized) {
        *handle_out = (magnetic_encoder_handle_t)&s_state;
        return ESP_OK;
    }
    if (cfg == NULL || handle_out == NULL || cfg->bus == NULL) {
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

    as5600_tracker_reset(&s_tracker);

    ESP_LOGI(TAG, "counting direction: %s",
             cfg->invert_dir
                 ? "INVERTED (magnet mounted opposite; +X move counts up)"
                 : "normal (not inverted)");

    /* First read: seeds the tracker (position 0 here) and proves the
     * bus works before the task starts hammering it. */
    uint16_t raw = 0;
    if (as5600_hw_read_raw(&raw)) {
        int32_t pos = as5600_tracker_update(&s_tracker, raw);
        publish(pos, raw);
        ESP_LOGI(TAG, "AS5600 online at 0x%02x — first raw angle=%u (seeded, pos=0)",
                 cfg->dev_addr, raw);
    } else {
        ESP_LOGW(TAG, "first RAW ANGLE read at 0x%02x failed — task will retry",
                 cfg->dev_addr);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(me_task, "me_as5600",
                                            ME_AS5600_TASK_STACK, NULL,
                                            ME_AS5600_TASK_PRIO, NULL,
                                            ME_AS5600_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create sampling task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "sampling task up: 1 kHz, prio %d, CPU%d, %d counts/rev",
             ME_AS5600_TASK_PRIO, ME_AS5600_TASK_CORE, AS5600_COUNTS_PER_REV);
    *handle_out = (magnetic_encoder_handle_t)&s_state;
    return ESP_OK;
}
```

`components/magnetic_encoder/Kconfig`：

```kconfig
menu "magnetic_encoder - 磁编码器类别"

config MAGNETIC_ENCODER_AS5600
    bool "编入 AS5600 实现（I2C，12-bit，小车编码器）"
    default y

config MAGNETIC_ENCODER_MT6835
    bool "编入 MT6835 实现（SPI，21-bit，摆角编码器）"
    default y

endmenu
```

`components/magnetic_encoder/CMakeLists.txt`：

```cmake
# magnetic_encoder — 磁编码器类别组件：契约 + 实现源文件
# 依赖无条件字面量声明（各实现所需之并集）；仅源文件随 Kconfig 宏编入。
set(me_srcs "")

if(CONFIG_MAGNETIC_ENCODER_AS5600)
    list(APPEND me_srcs "magnetic_encoder_as5600.c")
endif()

idf_component_register(SRCS ${me_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES freertos esp_driver_i2c esp_driver_spi
                    PRIV_REQUIRES as5600 mt6835)
```

- [x] **Step 2: as5600 组件瘦身**

`components/as5600/include/as5600.h` 改为（保留红线文档与 tracker 全节、`AS5600_COUNTS_PER_REV`；删除 config/init、遥测 getter、`as5600_counts_to_um`、`AS5600_INVERT_DIR_DEFAULT` 折叠块——分别移入 magnetic_encoder 实现与 cartpos）：

```c
/*
 * AS5600 magnetic encoder — 12-bit angle + multi-turn tracker（驱动核心层）
 *
 * !! READ-ONLY DEVICE — RED LINE !!
 *   The AS5600 ZPOS / MPOS / CONF / MANG registers are one-time
 *   programmable (OTP): a wrong write destroys the chip permanently.
 *   Register-level red-line details live in as5600_hw.c.
 *   Do not add write helpers.
 *
 * 硬件事实：7-bit 地址 0x36、RAW ANGLE 0x0C/0D、12 bit → 4096
 * counts/rev；2GT 20T × 2 mm = 40 mm/rev（settings $2）。
 * 方向镜像（安装标定）经 magnetic_encoder_as5600_config_t 注入，
 * 在读层应用于原始角。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Counts per mechanical revolution (12-bit raw angle). */
#define AS5600_COUNTS_PER_REV   4096

/* ================================================================
 * Multi-turn tracker — pure logic, hardware independent (testable)
 * ================================================================ */

typedef struct {
    int32_t  pos_counts;   /* continuous position (counts, multi-turn, signed) */
    uint16_t last_raw;     /* previous raw angle 0..4095 */
    bool     seeded;       /* first sample received? */
} as5600_tracker_t;

/**
 * @brief Drop history — the next sample becomes position 0.
 */
void as5600_tracker_reset(as5600_tracker_t *t);

/**
 * @brief Feed one raw angle, return the updated continuous position.
 *
 * First sample only seeds the tracker (position 0, no displacement).
 * Wraparound correction:
 *   diff = raw - last_raw
 *   diff >  +2048 → diff -= 4096   (backwards across the 0/4095 edge)
 *   diff <  -2047 → diff += 4096   (forwards across the 0/4095 edge)
 * The asymmetric pair of thresholds covers all 4096 diff values with
 * no gap and no overlap.
 *
 * @param t     Tracker state.
 * @param raw   Raw angle 0..4095 (higher bits are masked off).
 * @return Continuous position in counts.
 */
int32_t as5600_tracker_update(as5600_tracker_t *t, uint16_t raw);

#ifdef __cplusplus
}
#endif
```

删除 `components/as5600/as5600.c`（`git rm`）。`components/as5600/CMakeLists.txt`：

```cmake
idf_component_register(SRCS "as5600_tracker.c" "as5600_hw.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c)
```

- [x] **Step 3: cartpos 重定位为解释组件**

删除 `components/cartpos/cartpos_as5600.c`、`components/cartpos/include/cartpos_as5600.h`、`components/cartpos/Kconfig`（`git rm`）。`components/cartpos/include/cartpos.h` 头注释改为解释组件表述并加 init（四函数签名不变）：

```c
/*
 * cartpos — 小车位置解释组件（编码器实测位置）
 *
 * 持有 magnetic_encoder 句柄，把原始 counts 解释为 mm 域小车位置：
 * 比例随 $2（units_per_rev）订阅刷新，原点偏移存 µm 整数。
 * x_um = round(counts × $2 × 1000 ÷ counts_per_rev) − 原点偏移。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 绑定编码器实例并订阅 $2（比例随变更刷新）。 */
void cartpos_init(magnetic_encoder_handle_t enc);

/** 连续位置（多圈、带符号、含回零原点偏移），µm 整数；首样本前为 0。 */
int64_t cartpos_position_um(void);

/** 16 ms 窗平滑速度 mm/s；窗口未满为 0。 */
float cartpos_velocity_mm_s(void);

/** 连续 ≥10 次读取失败为 false；一次成功即恢复 true。 */
bool cartpos_is_ok(void);

/** 声明当前位置为 new_pos_mm（纯偏移建立，不产生运动）。 */
void cartpos_set_origin_mm(float new_pos_mm);

#ifdef __cplusplus
}
#endif
```

`components/cartpos/cartpos.c`：

```c
/*
 * cartpos — 小车位置解释实现
 *
 * 机械比例（$2 → µm/count）的唯一权威在本层：位置与速度同源
 * 同值。$2 非法（≤0）回退 40 mm/rev 并告警（沿袭原 as5600 行为）。
 */
#include "cartpos.h"

#include <math.h>

#include "esp_log.h"
#include "settings.h"

static const char *TAG = "cartpos";

#define CARTPOS_DEFAULT_UPR  40.0f  /* fallback if $2 is unusable */

static magnetic_encoder_handle_t s_enc;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static long   s_offset_um = 0;    /* 原点偏移（µm 整数） */
static double s_um_per_count = 0.0;

static void refresh_scale(void)
{
    float upr = settings_units_per_rev();
    if (!(upr > 0.0f)) {
        ESP_LOGW(TAG, "invalid $2 units_per_rev=%.3f — falling back to %.1f mm/rev",
                 (double)upr, (double)CARTPOS_DEFAULT_UPR);
        upr = CARTPOS_DEFAULT_UPR;
    }

    int32_t cpr = magnetic_encoder_counts_per_rev(s_enc);
    double  umc = (double)upr * 1000.0 / (double)cpr;

    portENTER_CRITICAL(&s_lock);
    s_um_per_count = umc;
    portEXIT_CRITICAL(&s_lock);
}

/* Settings notification — only $2 changes the counts → mm scale. */
static void on_settings_change(int idx, float new_value)
{
    (void)new_value;
    if (idx == SET_MM_PER_REV) {
        refresh_scale();
        ESP_LOGI(TAG, "$2 changed — scale refreshed (%.6f µm/count)",
                 (double)settings_units_per_rev() * 1000.0
                     / (double)magnetic_encoder_counts_per_rev(s_enc));
    }
}

void cartpos_init(magnetic_encoder_handle_t enc)
{
    s_enc = enc;
    refresh_scale();
    if (!settings_subscribe(on_settings_change)) {
        ESP_LOGW(TAG, "settings 订阅失败（槽满）——$2 变更不会生效");
    }
}

/* 无偏移原始位置（µm） */
static long raw_um(int64_t counts)
{
    double umc;
    portENTER_CRITICAL(&s_lock);
    umc = s_um_per_count;
    portEXIT_CRITICAL(&s_lock);
    return (long)llround((double)counts * umc);
}

int64_t cartpos_position_um(void)
{
    portENTER_CRITICAL(&s_lock);
    long offset = s_offset_um;
    portEXIT_CRITICAL(&s_lock);
    return raw_um(magnetic_encoder_position_counts(s_enc)) - offset;
}

float cartpos_velocity_mm_s(void)
{
    double umc;
    portENTER_CRITICAL(&s_lock);
    umc = s_um_per_count;
    portEXIT_CRITICAL(&s_lock);
    return (float)((double)magnetic_encoder_velocity_counts_per_s(s_enc)
                   * umc / 1000.0);
}

bool cartpos_is_ok(void)
{
    return magnetic_encoder_is_ok(s_enc);
}

void cartpos_set_origin_mm(float new_pos_mm)
{
    long raw = raw_um(magnetic_encoder_position_counts(s_enc));
    long offset = raw - (long)llround((double)new_pos_mm * 1000.0);

    portENTER_CRITICAL(&s_lock);
    s_offset_um = offset;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "原点已重置：%.3f mm（原始 %.3f mm，偏移 %ld µm）",
             (double)new_pos_mm, (double)raw / 1000.0, offset);
}
```

`components/cartpos/CMakeLists.txt`：

```cmake
# cartpos — 小车位置解释组件：encoder counts → x_um（比例/原点归本层）
idf_component_register(SRCS "cartpos.c"
                    INCLUDE_DIRS "include"
                    REQUIRES magnetic_encoder settings)
```

- [x] **Step 4: 组装点与测试面切换**

`main/app_main.c`：include 区 `cartpos_as5600.h` 改为 `magnetic_encoder_as5600.h`；cartpos 组装块替换为：

```c
        magnetic_encoder_handle_t enc_cart;
        magnetic_encoder_as5600_config_t ecfg = {
            .bus          = bsp_get_i2c_bus(),
            .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
            .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
            .invert_dir   = magnetic_encoder_as5600_invert_dir_default(),
        };
        ESP_ERROR_CHECK(magnetic_encoder_as5600_init(&ecfg, &enc_cart));
        cartpos_init(enc_cart);
```

`test/main/test_app_main.c` 同样替换。`main/CMakeLists.txt` 与 `test/main/CMakeLists.txt` 的 REQUIRES：`cartpos` 前加 `magnetic_encoder`（motor/poleang 仍为旧形态，后续任务处理）。`test/CMakeLists.txt` 的 TEST_COMPONENTS 增加 `magnetic_encoder`：

```cmake
set(TEST_COMPONENTS "as5600 cartfsm cartproto magnetic_encoder mt6835 settings step_engine telemetry tmc2209"
    CACHE STRING "参与测试的组件（其 test/ 子目录被编入）")
```

`components/magnetic_encoder/test/CMakeLists.txt`：

```cmake
# magnetic_encoder 测试组件（test/ 子目录约定：仅测试工程的 TEST_COMPONENTS 构建发现）
idf_component_register(SRC_DIRS "."
                    INCLUDE_DIRS "."
                    REQUIRES magnetic_encoder cartpos poleang bsp settings mt6835
                    PRIV_REQUIRES unity freertos log)
```

`components/magnetic_encoder/test/test_magnetic_encoder.c`（承接原 test_as5600.c B 组与 manual 段，按新分层改写；B17/B18 转测 cartpos 解释层）：

```c
/*
 * magnetic_encoder 硬件自测（官方 test/ 子目录约定，仅测试工程编入）
 *
 * [as5600] 段承接原 test_as5600.c B13–B18 与 manual A9：
 *  - B13/B14/B15 走类别实现面（init / last_raw / is_ok）；
 *  - B16 静止速度改为 counts/s 口径（5 mm/s ≡ 512 counts/s @40 mm/4096）；
 *  - B17/B18 转测 cartpos 解释层（比例一致性、原点重置）。
 * 实例由 test_app_main 创建（幂等 init 返回既有句柄）。
 */

#include <math.h>

#include "magnetic_encoder.h"
#include "magnetic_encoder_as5600.h"
#include "cartpos.h"

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp.h"
#include "settings.h"

static const char *TAG = "me_test";

static magnetic_encoder_handle_t enc(void)
{
    magnetic_encoder_handle_t h = NULL;
    magnetic_encoder_as5600_config_t cfg = {
        .bus          = bsp_get_i2c_bus(),
        .dev_addr     = CONFIG_BSP_AS5600_I2C_ADDR,
        .scl_speed_hz = CONFIG_BSP_I2C_FREQ_HZ,
        .invert_dir   = magnetic_encoder_as5600_invert_dir_default(),
    };
    TEST_ASSERT_EQUAL(ESP_OK, magnetic_encoder_as5600_init(&cfg, &h));
    return h;
}

TEST_CASE("B13 I2C 总线在位 + magnetic_encoder_as5600_init OK", "[as5600]")
{
    TEST_ASSERT_NOT_NULL(bsp_get_i2c_bus());
    TEST_ASSERT_NOT_NULL(enc());
}

/* 只读（红线见 as5600_hw.c） */
TEST_CASE("B14 20 次实时读全部 12 bit 内且链路健康", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    for (int i = 0; i < 20; i++) {
        uint16_t raw = magnetic_encoder_as5600_last_raw(h);
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(4095, raw);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    TEST_ASSERT_TRUE_MESSAGE(magnetic_encoder_is_ok(h),
                             "is_ok 假——I2C 读失败");
    ESP_LOGI(TAG, "  last raw=%u, counts=%lld",
             magnetic_encoder_as5600_last_raw(h),
             (long long)magnetic_encoder_position_counts(h));
}

/* 静止噪声：20 样本峰峰值 ≤2 counts（验收判据见
 * docs/reviews/T8-acceptance.md），保持轴静止。峰峰值按相对首样本
 * 的「展开差」度量（与 tracker 同一 ±2048 规则）。 */
TEST_CASE("B15 静止噪声峰峰值 ≤2 counts", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    int32_t  ref     = (int32_t)magnetic_encoder_as5600_last_raw(h);
    uint16_t raw_min = 0xFFFF;
    uint16_t raw_max = 0;
    int32_t  d_min   = 0;
    int32_t  d_max   = 0;

    for (int i = 0; i < 20; i++) {
        uint16_t raw = magnetic_encoder_as5600_last_raw(h);
        if (raw < raw_min) raw_min = raw;
        if (raw > raw_max) raw_max = raw;

        int32_t d = (int32_t)raw - ref;
        if (d > 2048) {
            d -= 4096;
        } else if (d < -2047) {
            d += 4096;
        }
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    int32_t pp = d_max - d_min;
    ESP_LOGI(TAG, "  噪声：raw %u..%u，展开 p-p=%ld counts",
             raw_min, raw_max, (long)pp);

    if ((uint16_t)(raw_max - raw_min) > 2 && pp <= 2) {
        ESP_LOGW(TAG, "  注：raw 极值跨 0/4095 边界——轴停在回绕点上，"
                      "展开噪声仍 ≤2");
    }
    TEST_ASSERT_TRUE_MESSAGE(pp <= 2, "静止噪声超 2 counts（是否碰了轴？）");
}

/* 静止速度：协议口径 σ≤5 mm/s @40 mm/4096 ≡ 512 counts/s */
TEST_CASE("B16 静止速度 |v| ≤512 counts/s（16 ms 窗）", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    vTaskDelay(pdMS_TO_TICKS(50));   /* 等速度窗填充 */

    int32_t v = magnetic_encoder_velocity_counts_per_s(h);
    ESP_LOGI(TAG, "  静止速度 = %d counts/s", v);
    TEST_ASSERT_TRUE_MESSAGE(v <= 512 && v >= -512, "静止速度超 512 counts/s");
}

/* 位置/标度一致性（cartpos 解释层）：x_um = counts × $2 / 4096（未设原点时） */
TEST_CASE("B17 x_um == counts × $2 ÷ 4096（cartpos 层）", "[as5600]")
{
    magnetic_encoder_handle_t h = enc();
    cartpos_set_origin_mm(0.0f);   /* 清偏移，令 x = 原始位置 */

    long counts = (long)magnetic_encoder_position_counts(h);
    float upr   = settings_units_per_rev();
    float expect = (float)counts * upr / 4096.0f;   /* AS5600 每转计数 */
    float actual = (float)(cartpos_position_um() / 1000.0);

    ESP_LOGI(TAG, "  counts=%ld → 期望 %.3f mm，实测 %.3f mm",
             counts, (double)expect, (double)actual);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expect, actual);
}

/* 原点重置（cartpos 层）：上报位置跟随 set_origin */
TEST_CASE("B18 原点重置（设 100 / 回 0，cartpos 层）", "[as5600]")
{
    cartpos_set_origin_mm(100.0f);
    float at100 = (float)(cartpos_position_um() / 1000.0);
    ESP_LOGI(TAG, "  set_origin(100) 后 x=%.3f mm", (double)at100);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, at100);

    cartpos_set_origin_mm(0.0f);
    float at0 = (float)(cartpos_position_um() / 1000.0);
    ESP_LOGI(TAG, "  set_origin(0) 后   x=%.3f mm", (double)at0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, at0);
}

/* ---- 人工引导段（= 上板清单 A9）---- */
TEST_CASE("人工：手转 3 圈观察 x_um 变化", "[as5600][manual]")
{
    const int   secs = 15;
    const float upr  = settings_units_per_rev();

    ESP_LOGI(TAG, "请将电机轴整转 3 圈后回到出发点，现在开始（采样 %d s）", secs);
    ESP_LOGI(TAG, "(电机轴经同步带连着小车——转轴即移动小车)");

    float start = (float)(cartpos_position_um() / 1000.0);

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "  t=%2ds  x=%.3f mm  (%lld counts)",
                 sec, (double)(cartpos_position_um() / 1000.0),
                 (long long)magnetic_encoder_position_counts(enc()));
    }

    float delta = (float)(cartpos_position_um() / 1000.0) - start;
    ESP_LOGI(TAG, "结果：delta = %+.3f mm（3 圈应 ~±%.1f；1 圈 = %.1f mm）",
             (double)delta, (double)(3.0f * upr), (double)upr);
    ESP_LOGI(TAG, "A9 记录：x_um 随转轴变化 %s——记下转动方向与符号关系，"
                  "留待符号约定标定交叉核对", delta >= 0 ? "为 +" : "为 −");

    if (fabsf(delta) < 1.0f) {
        ESP_LOGI(TAG, "(未检测到转动——没转属正常)");
    }
}
```

注：B17 的分母用 `4096` 字面量并注明来源（AS5600 每转计数），避免引已瘦身的 as5600.h。

`components/as5600/test/test_as5600.c`：删除 `test_upr` 辅助函数、B13–B18 全部用例与 manual 段（移入上文新文件），只留 tracker A 组与文件头说明；`components/as5600/test/CMakeLists.txt` 的 REQUIRES 改为 `REQUIRES as5600`（bsp/settings 不再使用）。

`host_tests/telemetry_format_test.c`：删除 `#include "as5600.h"`、`test_counts_to_um` 函数及其在 main 中的调用（counts→µm 换算随 as5600_counts_to_um 移除，验收移至 cartpos 上板数值域比对）；文件头注释补一行说明。

- [x] **Step 5: 构建与 host 验证**

```bash
powershell -NoProfile -ExecutionPolicy Bypass -Command "Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue; Remove-Item Env:SHELL -ErrorAction SilentlyContinue; & 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'; idf.py -B build build; Write-Host MAIN=\$LASTEXITCODE; idf.py -C test build; Write-Host TEST=\$LASTEXITCODE"
```

预期：MAIN=0、TEST=0（poleang/motor 仍为旧形态，编译不受影响；cartfsm/telemetry 经 cartpos 新实现，签名未变）。host 复跑（t1 cartfsm_core、t2 parse、t5 telemetry_format——后者断言数由 15 变 11）：

```bash
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && gcc -std=c11 -Wall -Wextra -Icomponents/cartfsm/include -Icomponents/cartparams/include host_tests/cartfsm_core_test.c components/cartfsm/cartfsm_core.c components/cartparams/params.c -o /tmp/t1 && /tmp/t1; echo t1=\$?; gcc -std=c11 -Wall -Wextra -Ihost_tests/stub -Icomponents/as5600/include -Icomponents/telemetry/include host_tests/telemetry_format_test.c components/telemetry/telemetry_format.c -o /tmp/t5 && /tmp/t5; echo t5=\$?"
```

预期：t1=0（87/87）、t5=0（11/11）。

- [x] **Step 6: Commit**

```bash
git add components/magnetic_encoder components/as5600 components/cartpos main test host_tests/telemetry_format_test.c
git commit -m "magnetic_encoder 类别组件落地（契约+AS5600 实现，句柄式）：as5600 瘦身为驱动核心，cartpos 重定位为解释组件（\$2 比例/原点上收），组装点与 B 组自测切换"
```

### Task 2: magnetic_encoder_mt6835 实现 + mt6835 瘦身 + poleang 重定位

**Files:**
- Create: `components/magnetic_encoder/include/magnetic_encoder_mt6835.h`
- Create: `components/magnetic_encoder/magnetic_encoder_mt6835.c`
- Modify: `components/magnetic_encoder/CMakeLists.txt`（SRCS 增 MT6835）
- Delete: `components/mt6835/mt6835.c`、`components/mt6835/include/mt6835.h`
- Modify: `components/mt6835/mt6835_urad.c` / `include/mt6835_urad.h`（删 `mt6835_counts_to_mrad_s`）
- Modify: `components/mt6835/CMakeLists.txt`
- Delete: `components/poleang/poleang_mt6835.c`、`components/poleang/include/poleang_mt6835.h`、`components/poleang/Kconfig`
- Create: `components/poleang/poleang.c`
- Modify: `components/poleang/include/poleang.h`、`components/poleang/CMakeLists.txt`
- Modify: `components/mt6835/test/test_mt6835.c`、`components/magnetic_encoder/test/test_magnetic_encoder.c`、`host_tests/mt6835_urad_test.c`
- Modify: `main/app_main.c`、`test/main/test_app_main.c`

**Interfaces:**
- Consumes: `mt6835_hw_*`、`mt6835_unwrap_diff`、`bsp_get_spi_host()`
- Produces: `magnetic_encoder_mt6835_init(cfg, &h)`、`magnetic_encoder_mt6835_last_raw(h)`；poleang 三函数签名不变 + `poleang_init(h)`

- [x] **Step 1: MT6835 实现与 mt6835 瘦身**

`components/magnetic_encoder/include/magnetic_encoder_mt6835.h`：

```c
/*
 * magnetic_encoder_mt6835 — magnetic_encoder 类别的 MT6835 实现
 * （init 与接线 config + 器件自测面）
 *
 * 接线参数（SPI 主机、时钟、CS）由组装点读取 Kconfig 宏与 bsp
 * 主机号后经本 config 注入。STATUS 位诊断与失败分诊快照见本头。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "magnetic_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t host;         /* 已初始化的 SPI 主机（组装点从 bsp 获取） */
    int               clock_speed_hz;
    int               spics_io_num;
} magnetic_encoder_mt6835_config_t;

/**
 * @brief 注入接线参数并启动 1 kHz 采样任务。幂等。每实现单实例。
 *
 * @return ESP_OK；cfg 或 handle_out 为 NULL、host 为负返回
 *         ESP_ERR_INVALID_ARG；其余错误码与迁移前一致
 *         （SPI 设备挂载 / 任务创建错误）。
 */
esp_err_t magnetic_encoder_mt6835_init(const magnetic_encoder_mt6835_config_t *cfg,
                                       magnetic_encoder_handle_t *handle_out);

/** 最新原始 21-bit 读数（器件自测）。 */
uint32_t magnetic_encoder_mt6835_last_raw(magnetic_encoder_handle_t enc);

/** 最新 STATUS[2:0]（bit0 过速 bit1 磁弱 bit2 欠压；器件自测）。 */
uint8_t magnetic_encoder_mt6835_last_status(magnetic_encoder_handle_t enc);

#ifdef __cplusplus
}
#endif
```

`components/magnetic_encoder/magnetic_encoder_mt6835.c`：整体搬移现 [mt6835.c](../../../components/mt6835/mt6835.c) 的任务/失败计数/首读/STATUS 告警结构，差异点：

- 状态结构改契约发布形态：`int64_t pos_counts`、`int32_t hist[17]`、`int32_t vel_counts_per_s`、`uint32_t raw21`、`uint8_t status`（s_lock 保护）。
- `publish`：`pos_counts += mt6835_unwrap_diff(raw, s_state.raw21)` 不变；速度改为 `llround(delta × 62.5)`（与 AS5600 实现同一式）；`theta_urad`/`vth_mrad_s` 发布删除（归 poleang）。
- 契约函数：`counts_per_rev` 返回 `MT6835_COUNTS_PER_REV`；position/velocity/is_ok 同 AS5600 实现形态；另加 `magnetic_encoder_mt6835_last_raw` / `last_status`（锁内读）。
- init：收 config → `mt6835_hw_init`（时钟/CS 注入）→ 首读种子 → 建任务 → `*handle_out = &s_state`；日志原文保留。

`components/magnetic_encoder/CMakeLists.txt` 的 SRCS 块增：

```cmake
if(CONFIG_MAGNETIC_ENCODER_MT6835)
    list(APPEND me_srcs "magnetic_encoder_mt6835.c")
endif()
```

删除 `components/mt6835/mt6835.c` 与 `components/mt6835/include/mt6835.h`（`git rm`；config/init/读函数分别移入实现头与实现源）。`components/mt6835/mt6835_urad.c` / `mt6835_urad.h`：删除 `mt6835_counts_to_mrad_s`（速度归 counts/s 契约后无调用者）。`components/mt6835/CMakeLists.txt`：

```cmake
idf_component_register(
    SRCS "mt6835_urad.c" "mt6835_hw.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi
)
```

- [x] **Step 2: poleang 重定位**

删除 `components/poleang/poleang_mt6835.c`、`include/poleang_mt6835.h`、`Kconfig`（`git rm`）。`poleang.h` 头注释改解释组件表述并加 `void poleang_init(magnetic_encoder_handle_t enc);` + include `magnetic_encoder.h`（三函数签名不变）。`components/poleang/poleang.c`：

```c
/*
 * poleang — 摆角解释实现：encoder 角量 → 已标定摆角
 *
 * 零位与符号（POLE_ZERO_URAD / POLE_INV，来自 cartparams 注册表）
 * 在读取时应用。µrad 换算的整数算术与原 mt6835_raw_to_urad 逐位
 * 一致（u64 中间量、π = 3141592 / 2π = 6283185、±π 环绕、wrap 后
 * 符号翻转）；单圈角 = 多圈 counts 对每转计数取模（与原 last_raw
 * 同值，pos ≡ raw (mod cpr)）。参数写入瞬间 theta 与 vth 可能取到
 * 不同快照，200 Hz 下一帧自愈——与迁移前一致，接受。
 */
#include "poleang.h"

#include <math.h>

#include "magnetic_encoder.h"
#include "params.h"

/* floor(π × 1e6) / floor(2π × 1e6)，与 mt6835_urad 同约定 */
#define POLEANG_PI_URAD   3141592
#define POLEANG_2PI_URAD  6283185

static magnetic_encoder_handle_t s_enc;

void poleang_init(magnetic_encoder_handle_t enc)
{
    s_enc = enc;
}

int32_t poleang_theta_urad(void)
{
    const fw_params_t *p = fw_params();
    int32_t cpr = magnetic_encoder_counts_per_rev(s_enc);
    int64_t counts = magnetic_encoder_position_counts(s_enc);
    uint32_t raw_mod = (uint32_t)(((counts % cpr) + cpr) % cpr);

    int64_t a = (int64_t)((uint64_t)raw_mod * (uint64_t)POLEANG_2PI_URAD
                          / (uint64_t)cpr);
    a -= p->pole_zero_urad;

    while (a > POLEANG_PI_URAD) {
        a -= POLEANG_2PI_URAD;
    }
    while (a < -POLEANG_PI_URAD) {
        a += POLEANG_2PI_URAD;
    }

    return (p->pole_inv != 0) ? (int32_t)(-a) : (int32_t)a;
}

int32_t poleang_vth_mrad_s(void)
{
    const fw_params_t *p = fw_params();
    int32_t cpr = magnetic_encoder_counts_per_rev(s_enc);
    int32_t cps = magnetic_encoder_velocity_counts_per_s(s_enc);
    int32_t v = (int32_t)llround((double)cps * (double)POLEANG_2PI_URAD
                                 / (double)cpr / 1000.0);
    return (p->pole_inv != 0) ? -v : v;
}

bool poleang_is_ok(void)
{
    return magnetic_encoder_is_ok(s_enc);
}
```

`components/poleang/CMakeLists.txt`：

```cmake
# poleang — 摆角解释组件：encoder 角量 → 已标定 θ（零位/符号归本层）
idf_component_register(SRCS "poleang.c"
                    INCLUDE_DIRS "include"
                    REQUIRES magnetic_encoder cartparams)
```

- [x] **Step 3: 组装与测试面切换**

`main/app_main.c` / `test/main/test_app_main.c`：include `poleang_mt6835.h` 改 `magnetic_encoder_mt6835.h`；poleang 组装块替换为：

```c
        magnetic_encoder_handle_t enc_pole;
        magnetic_encoder_mt6835_config_t pcfg = {
            .host          = bsp_get_spi_host(),
            .clock_speed_hz = CONFIG_BSP_MT6835_SPI_HZ,
            .spics_io_num  = CONFIG_BSP_MT6835_CS_GPIO,
        };
        ESP_ERROR_CHECK(magnetic_encoder_mt6835_init(&pcfg, &enc_pole));
        poleang_init(enc_pole);
```

`components/mt6835/test/test_mt6835.c`：只留「纯函数向量」用例（删除其中 `mt6835_counts_to_mrad_s` 一行断言），其余用例移入 `components/magnetic_encoder/test/test_magnetic_encoder.c` 追加段：

```c
/* ================= [mt6835] 段（承接原 test_mt6835.c 硬件用例） ================= */

/* 静止稳定性：等 |v| < 50 mrad/s（≡ 16700 counts/s）持续 300 ms，
 * 再测 1 s 内 raw21 抖动 ≤2 count。 */
TEST_CASE("静止稳定性：1 s 内 raw21 抖动 ≤2 count", "[mt6835]")
{
    magnetic_encoder_handle_t h = mt6835_enc();
    bool calm = false;
    for (int t = 0; t < 10000 && !calm; t += 10) {
        int32_t v = magnetic_encoder_velocity_counts_per_s(h);
        if (v > -16700 && v < 16700) {
            calm = true;
            for (int k = 0; k < 30; k++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                v = magnetic_encoder_velocity_counts_per_s(h);
                if (v <= -16700 || v >= 16700) {
                    calm = false;
                    break;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (!calm) {
        ESP_LOGW(TAG, "10 s 内未稳定（%d counts/s）——扶稳摆杆后重试",
                 magnetic_encoder_velocity_counts_per_s(h));
    }

    uint32_t min_r = 0xFFFFFFFFu, max_r = 0;
    for (int i = 0; i < 1000; i++) {
        uint32_t r = magnetic_encoder_mt6835_last_raw(h);
        if (r < min_r) min_r = r;
        if (r > max_r) max_r = r;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(magnetic_encoder_is_ok(h));
    ESP_LOGI(TAG, "静止 raw21 范围 [%u, %u]（抖动 %u）",
             min_r, max_r, max_r - min_r);
    TEST_ASSERT_TRUE_MESSAGE(max_r - min_r <= 2, "静止抖动应 ≤ 2 count");
}

/* theta 合法域与速度有限性（2 s 连续读，poleang 解释层）+
 * 算术对拍：zero=0/inv=0 下 poleang 与 mt6835_urad 纯函数同值
 * （Review Focus 2 的锚——常量算术逐位一致）。 */
TEST_CASE("theta 合法域 ±π 与解释层对拍", "[mt6835]")
{
    fw_params_set("POLE_ZERO_URAD", 0);
    fw_params_set("POLE_INV", 0);
    magnetic_encoder_handle_t h = mt6835_enc();

    for (int i = 0; i < 2000; i++) {
        int32_t th = poleang_theta_urad();
        int32_t vth = poleang_vth_mrad_s();
        TEST_ASSERT_TRUE_MESSAGE(th >= -3141593 && th <= 3141593,
                                 "theta 应在 ±π 内");
        TEST_ASSERT_TRUE_MESSAGE(vth > -1000000 && vth < 1000000,
                                 "vth 应为有限值");
        if (i % 500 == 0) {
            TEST_ASSERT_EQUAL_INT(
                mt6835_raw_to_urad(magnetic_encoder_mt6835_last_raw(h), 0, false),
                th);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(magnetic_encoder_is_ok(h));
}

/* ---- A10：手转摆杆一整圈（圈数守恒判定逻辑与原用例一致） ---- */
TEST_CASE("人工 A10：手转摆杆一圈圈数守恒", "[mt6835][manual]")
{
    magnetic_encoder_handle_t h = mt6835_enc();
    const int secs = 15;
    const int32_t REV = 2097152;

    ESP_LOGI(TAG, "请将摆杆缓慢转一整圈后回到出发点，现在开始（采样 %d s）", secs);

    int32_t start = (int32_t)magnetic_encoder_position_counts(h);
    int32_t lo = start, hi = start;

    for (int sec = 1; sec <= secs; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int32_t p = (int32_t)magnetic_encoder_position_counts(h);
        if (p < lo) lo = p;
        if (p > hi) hi = p;
        ESP_LOGI(TAG, "  t=%2ds  pos=%+d counts（theta=%d µrad）",
                 sec, p, poleang_theta_urad());
    }

    int32_t net = (int32_t)magnetic_encoder_position_counts(h) - start;
    int32_t excursion = hi - lo;
    int32_t k = (net >= 0) ? (net + REV / 2) / REV : (net - REV / 2) / REV;
    int32_t resid = net - k * REV;
    double resid_deg = (double)resid * 360.0 / (double)REV;

    ESP_LOGI(TAG, "结果：行程 %d counts（%.2f 圈），净变化 %d counts"
                  "（%d 圈 + 残余 %+d counts ≈ %+.2f°）",
             excursion, (double)excursion / (double)REV, net, k, resid, resid_deg);

    if (excursion < REV / 10) {
        ESP_LOGI(TAG, "A10 判定：未检测到整圈转动（没转属正常）");
    } else if (excursion < REV - REV / 10) {
        ESP_LOGW(TAG, "A10 判定：行程不足——请转满一圈再试");
    } else if (resid > REV / 120 || resid < -REV / 120) {
        ESP_LOGW(TAG, "A10 判定：未回到出发点角度（残余 %+.2f° 超过 ±3°）",
                 resid_deg);
    } else {
        ESP_LOGI(TAG, "A10 判定：圈数守恒 OK（检测到 %d 圈）", k);
    }
}
```

`test_magnetic_encoder.c` 顶部补：`magnetic_encoder_mt6835.h`、`poleang.h`、`params.h` include 与 `mt6835_enc()` 辅助（形态同 `enc()`，收 `bsp_get_spi_host()` 等）；文件头注释补 [mt6835] 段说明。`host_tests/mt6835_urad_test.c`：删除 `mt6835_counts_to_mrad_s` 相关断言行。

- [x] **Step 4: 构建与 host 验证**

构建命令同 Task 1 Step 5，预期 MAIN=0、TEST=0（telemetry 经 poleang 新实现，签名未变）。host 复跑：

```bash
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && gcc -std=c11 -Wall -Wextra -Icomponents/mt6835/include host_tests/mt6835_crc_test.c components/mt6835/mt6835_urad.c -o /tmp/t3 && /tmp/t3; echo t3=\$?; gcc -std=c11 -Wall -Wextra -Icomponents/mt6835/include host_tests/mt6835_urad_test.c components/mt6835/mt6835_urad.c -o /tmp/t4 && /tmp/t4; echo t4=\$?"
```

预期：t3=0（2/2）、t4=0（18/18，减 counts_to_mrad_s 一条）。

- [x] **Step 5: Commit**

```bash
git add components/magnetic_encoder components/mt6835 components/poleang main test host_tests/mt6835_urad_test.c
git commit -m "magnetic_encoder_mt6835 实现落地：mt6835 瘦身为驱动核心（删 counts_to_mrad_s），poleang 重定位为解释组件（零位/符号上收，算术与 mt6835_urad 逐位一致）"
```

### Task 3: cartaxis 换算纯函数（TDD）

**Files:**
- Create: `components/cartaxis/include/cartaxis_conv.h`
- Create: `components/cartaxis/cartaxis_conv.c`
- Create: `components/cartaxis/CMakeLists.txt`
- Test: `host_tests/cartaxis_conv_test.c`

**Interfaces:**
- Produces: `cartaxis_mmps_to_steps` / `cartaxis_accel_to_steps` / `cartaxis_steps_to_um` / `cartaxis_um_to_steps`（Task 4 消费）

- [x] **Step 1: 写失败的宿主测试**

`host_tests/cartaxis_conv_test.c`（断言宏形态照抄 [motor_conv_test.c](../../../host_tests/motor_conv_test.c)）：

```c
/*
 * cartaxis_conv 宿主测试 — 手算精确向量逐条断言
 *
 * 编译：gcc -std=c11 -Wall -Wextra -Icomponents/cartaxis/include \
 *       host_tests/cartaxis_conv_test.c components/cartaxis/cartaxis_conv.c -lm
 * 退出码 = 失败断言数（0 = 全过）。
 */
#include <stdio.h>
#include "cartaxis_conv.h"

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
    CHECK_EQ(cartaxis_mmps_to_steps(0,    40.0), 0);
    CHECK_EQ(cartaxis_mmps_to_steps(1000, 40.0), 40000);
    CHECK_EQ(cartaxis_mmps_to_steps(-250, 40.0), -10000);
    /* VEL_MAX 注册表上限 2000 mm/s → 80000 steps/s */
    CHECK_EQ(cartaxis_mmps_to_steps(2000, 40.0), 80000);
    /* lround 半数远离零 */
    CHECK_EQ(cartaxis_mmps_to_steps(1, 0.4), 0);
    CHECK_EQ(cartaxis_mmps_to_steps(1, 0.5), 1);
    CHECK_EQ(cartaxis_mmps_to_steps(-1, 0.5), -1);

    CHECK_EQ(cartaxis_accel_to_steps(0,    40.0), 0);
    CHECK_EQ(cartaxis_accel_to_steps(4000, 40.0), 160000);

    CHECK_EQ(cartaxis_steps_to_um(0,      40.0), 0);
    CHECK_EQ(cartaxis_steps_to_um(160000, 40.0), 4000000);
    /* 舍入边界：62.5 → 63（llround 半数远离零），负向对称 */
    CHECK_EQ(cartaxis_steps_to_um(1,  16.0), 63);
    CHECK_EQ(cartaxis_steps_to_um(-1, 16.0), -63);

    CHECK_EQ(cartaxis_um_to_steps(4000000, 40.0), 160000);
    CHECK_EQ(cartaxis_um_to_steps(25, 40.0), 1);  /* 1.0 精确 */
    CHECK_EQ(cartaxis_um_to_steps(13, 40.0), 1);  /* 0.52 → 1 */
    CHECK_EQ(cartaxis_um_to_steps(12, 40.0), 0);  /* 0.48 → 0 */
    CHECK_EQ(cartaxis_um_to_steps(1,  40.0), 0);  /* 0.04 → 0 */

    printf("%s: %d 用例, %d 失败\n", s_fail ? "FAIL" : "PASS", s_run, s_fail);
    return s_fail;
}
```

- [x] **Step 2: 跑测试确认失败**

```bash
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && gcc -std=c11 -Wall -Wextra -Icomponents/cartaxis/include host_tests/cartaxis_conv_test.c components/cartaxis/cartaxis_conv.c -lm -o /tmp/tc && /tmp/tc; echo exit=\$?"
```

预期：编译失败 `cartaxis_conv.h: No such file or directory`（红）。

- [x] **Step 3: 实现纯函数**

`components/cartaxis/include/cartaxis_conv.h`：

```c
/*
 * cartaxis_conv — mm 域与 steps 域换算纯函数（宿主可测）
 *
 * steps_per_mm 语义由 settings 派生访问器定义（$0 细分 × $1 每转
 * 整步数 ÷ $2 每转行程 mm）。舍入统一 llround（半数远离零）。
 * 前置条件：steps_per_mm > 0（settings 校验域保证）；不满足为未定义。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** mm/s → steps/s（带符号）。 */
long cartaxis_mmps_to_steps(long v_mm_s, double steps_per_mm);

/** mm/s² → steps/s²。 */
uint32_t cartaxis_accel_to_steps(long a_mm_s2, double steps_per_mm);

/** 指令位置 steps → µm。 */
long cartaxis_steps_to_um(int32_t steps, double steps_per_mm);

/** µm → 指令位置 steps（上一函数的逆换算）。 */
int32_t cartaxis_um_to_steps(long um, double steps_per_mm);

#ifdef __cplusplus
}
#endif
```

`components/cartaxis/cartaxis_conv.c`：

```c
#include "cartaxis_conv.h"

#include <math.h>

long cartaxis_mmps_to_steps(long v_mm_s, double steps_per_mm)
{
    return llround((double)v_mm_s * steps_per_mm);
}

uint32_t cartaxis_accel_to_steps(long a_mm_s2, double steps_per_mm)
{
    return (uint32_t)llround((double)a_mm_s2 * steps_per_mm);
}

long cartaxis_steps_to_um(int32_t steps, double steps_per_mm)
{
    return (long)llround((double)steps * 1000.0 / steps_per_mm);
}

int32_t cartaxis_um_to_steps(long um, double steps_per_mm)
{
    return (int32_t)llround((double)um * steps_per_mm / 1000.0);
}
```

`components/cartaxis/CMakeLists.txt`：

```cmake
# cartaxis — 小车直线轴解释组件：mm↔steps 换算（Task 4 接线电机句柄）
idf_component_register(SRCS "cartaxis_conv.c"
                    INCLUDE_DIRS "include")
```

- [x] **Step 4: 跑测试确认通过**

同 Step 2 命令。预期：`PASS: 17 用例, 0 失败`，exit=0。

- [x] **Step 5: 构建验证 + Commit**

主工程构建（同 Task 1 Step 5 命令，预期 MAIN=0；cartaxis 未被引用，编入即验证）。提交：

```bash
git add components/cartaxis host_tests/cartaxis_conv_test.c
git commit -m "cartaxis 换算纯函数：mm↔steps 四条换算（lround 语义）自 motor_conv 迁移语义，宿主测试 17 用例锚定公式与舍入边界"
```

### Task 4: motor steps 域化 + cartaxis 接线 + cartfsm/flowtest 切换 + tmc2209/step_engine 清理

**Files:**
- Rewrite: `components/motor/include/motor.h`（steps 契约，句柄式）
- Rewrite: `components/motor/include/motor_tmc2209.h`、`components/motor/motor_tmc2209.c`
- Delete: `components/motor/motor_conv.c`、`components/motor/include/motor_conv.h`、`host_tests/motor_conv_test.c`
- Modify: `components/motor/CMakeLists.txt`
- Modify: `components/tmc2209/include/tmc2209.h`、`components/tmc2209/tmc2209.c`、`components/tmc2209/CMakeLists.txt`
- Modify: `components/step_engine/include/step_engine.h`、`components/step_engine/step_engine.c`、`components/step_engine/test/test_step_engine.c`
- Create: `components/cartaxis/include/cartaxis.h`、`components/cartaxis/cartaxis.c`
- Modify: `components/cartaxis/CMakeLists.txt`
- Modify: `components/cartfsm/cartfsm.c`、`components/cartfsm/CMakeLists.txt`、`components/cartfsm/test/test_cartfsm_flow.c`
- Modify: `main/app_main.c`、`main/CMakeLists.txt`、`test/main/test_app_main.c`、`test/main/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 3 四条 `cartaxis_conv_*`；`tmc2209_*`、`step_engine_*` 现签名
- Produces: `motor_handle_t` 与 motor 契约十二函数；`motor_tmc2209_init(cfg, &h)`；cartaxis 十一函数（业务面）；`tmc2209_config_t` 增三个 init 字段

- [x] **Step 1: motor 契约与实现 steps 域化**

`components/motor/include/motor.h` 整体替换：

```c
/*
 * motor — 步进电机类别契约（steps 域，句柄式）
 *
 * steps 指当前细分档的一个微步——微步档位变更即单位含义变更
 * （motor_set_microstep）。契约无方向旋钮：应用方向约定归解释层
 * （cartaxis 经指令与读数取反）。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motor_s *motor_handle_t;

/** 驱动器使能引脚（EN，低有效）使能/关断；en=true 等待 ≥1 ms 唤醒。 */
esp_err_t motor_enable(motor_handle_t m, bool en);

/** 设定带符号目标速度（steps/s）并喂命令看门狗；引擎内部定点递推
 *  平滑逼近，换向经零速。 */
void motor_set_vel(motor_handle_t m, long steps_per_s);

/** 目标清 0 斜坡停；不喂看门狗。 */
void motor_halt(motor_handle_t m);

/** 急停：停脉冲、清递推状态与目标、冻结指令位置。 */
void motor_stop(motor_handle_t m);

/** true ⇔ 当前速度精确为零（引擎内部定点域判零，不经整数舍入）。 */
bool motor_at_rest(motor_handle_t m);

/** 指令位置（微步，自上电累计）。 */
int32_t motor_position_steps(motor_handle_t m);

/** 指令位置重定基；前置条件：电机静止（无脉冲发出），违反为未定义。 */
void motor_set_position(motor_handle_t m, int32_t steps);

/** 追踪加速度 / 减速度档（steps/s²）。 */
void motor_set_accel(motor_handle_t m, uint32_t steps_per_s2);
void motor_set_decel(motor_handle_t m, uint32_t steps_per_s2);

/** 命令新鲜度看门狗：超过 ms 未收到有效速度命令，按减速档斜坡停。 */
void motor_set_wdt_ms(motor_handle_t m, uint32_t ms);

/** 细分档位（2^n，1–256）；前置条件：电机静止；变更后 steps 单位按
 *  新档解释，指令位置计数不重置——调用方负责重定基。 */
esp_err_t motor_set_microstep(motor_handle_t m, int microstep);

/** 运行 / 保持电流（mA）。 */
esp_err_t motor_set_current(motor_handle_t m, int run_ma, int hold_ma);

#ifdef __cplusplus
}
#endif
```

`components/motor/include/motor_tmc2209.h`：init 返回句柄，config 增初值三字段：

```c
/*
 * motor_tmc2209 — motor 类别的 TMC2209 + GPTimer 实现
 * （init 与接线 config）
 *
 * 组合 tmc2209（UART 配置/电流/细分/EN）与 step_engine（STEP/DIR
 * 脉冲引擎）；类别层无 mm 换算（归 cartaxis 解释组件）。
 */
#pragma once

#include "esp_err.h"
#include "tmc2209.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    tmc2209_config_t tmc;       /* 驱动器配置（含 en_gpio 与上电初值） */
    int              step_gpio; /* STEP 输出引脚 */
    int              dir_gpio;  /* DIR 输出引脚 */
} motor_tmc2209_config_t;

/**
 * @brief 内部完成 tmc2209_init 与 step_engine_init（次序即此）。幂等。
 *
 * @return ESP_OK；cfg 或 handle_out 为 NULL 返回 ESP_ERR_INVALID_ARG；
 *         tmc2209_init / step_engine_init 失败时返回其错误码。
 */
esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg,
                             motor_handle_t *handle_out);

#ifdef __cplusplus
}
#endif
```

`components/motor/motor_tmc2209.c` 整体替换（换算与 settings 依赖全部移除）：

```c
/*
 * motor_tmc2209 — motor 契约的 TMC2209 + GPTimer 实现（单实例）
 *
 * steps 域直通 step_engine（速度目标以 steps/s×65536 的定点值下发，
 * 整数 steps/s 恰为定点值的 65536 倍，无损）；细分与电流经 tmc2209
 * 功能层（含 VSENSE 分档）。
 */
#include "motor.h"
#include "motor_tmc2209.h"

#include "tmc2209.h"
#include "step_engine.h"

static struct motor_s {
    int unused;   /* 单实例句柄载体 */
} s_inst;

esp_err_t motor_tmc2209_init(const motor_tmc2209_config_t *cfg,
                             motor_handle_t *handle_out)
{
    if (cfg == NULL || handle_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = tmc2209_init(&cfg->tmc);
    if (err != ESP_OK) {
        return err;
    }
    err = step_engine_init(cfg->step_gpio, cfg->dir_gpio);
    if (err != ESP_OK) {
        return err;
    }
    *handle_out = &s_inst;
    return ESP_OK;
}

esp_err_t motor_enable(motor_handle_t m, bool en)
{
    (void)m;
    return tmc2209_enable(en);
}

void motor_set_vel(motor_handle_t m, long steps_per_s)
{
    (void)m;
    step_engine_vel_target((int64_t)steps_per_s * 65536);
}

void motor_halt(motor_handle_t m)
{
    (void)m;
    step_engine_vel_halt();
}

void motor_stop(motor_handle_t m)
{
    (void)m;
    step_engine_stop();
}

bool motor_at_rest(motor_handle_t m)
{
    /* 精确判零：引擎内部定点域（残余速度不得提前判停） */
    (void)m;
    return step_engine_vel_now_q16() == 0;
}

int32_t motor_position_steps(motor_handle_t m)
{
    (void)m;
    return step_engine_position_steps();
}

void motor_set_position(motor_handle_t m, int32_t steps)
{
    (void)m;
    step_engine_set_position(steps);
}

void motor_set_accel(motor_handle_t m, uint32_t steps_per_s2)
{
    (void)m;
    step_engine_vel_set_accel(steps_per_s2);
}

void motor_set_decel(motor_handle_t m, uint32_t steps_per_s2)
{
    (void)m;
    step_engine_vel_set_decel(steps_per_s2);
}

void motor_set_wdt_ms(motor_handle_t m, uint32_t ms)
{
    (void)m;
    step_engine_vel_set_wdt_us(ms * 1000u);
}

esp_err_t motor_set_microstep(motor_handle_t m, int microstep)
{
    (void)m;
    return tmc2209_set_microstep(microstep);
}

esp_err_t motor_set_current(motor_handle_t m, int run_ma, int hold_ma)
{
    (void)m;
    return tmc2209_set_current(run_ma, hold_ma);
}
```

删除 `motor_conv.c` / `motor_conv.h` / `host_tests/motor_conv_test.c`（`git rm`；语义已由 Task 3 cartaxis_conv 承接）。`components/motor/CMakeLists.txt`：

```cmake
# motor — 步进电机类别组件：契约 + 实现源文件（Kconfig choice 选择）
set(motor_srcs "")

if(CONFIG_MOTOR_IMPL_TMC2209)
    list(APPEND motor_srcs "motor_tmc2209.c")
endif()

idf_component_register(SRCS ${motor_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES tmc2209
                    PRIV_REQUIRES step_engine)
```

- [x] **Step 2: tmc2209 去 settings（初值经 config 注入）**

`components/tmc2209/include/tmc2209.h`：`tmc2209_config_t` 增三个字段（置于 r_sense 之后）：

```c
    int      init_microstep;  /* 上电细分档（组装点从 settings 读取注入） */
    int      init_run_ma;     /* 上电运行电流 mA */
    int      init_hold_ma;    /* 上电保持电流 mA */
```

`@brief` 注释中「Also subscribes to settings changes for $0/$3/$4.」删除，改为「Initial microstep / currents come from the config (the assembly point fills them from settings).」。

`components/tmc2209/tmc2209.c`：删除 `#include "settings.h"`、`settings_change_cb` 函数与 init 末尾的订阅块；`apply_default_registers` 与 init 日志中 `settings_microstep()` / `settings_run_current_ma()` / `settings_hold_current_ma()` 分别替换为 `s_cfg.init_microstep` / `s_cfg.init_run_ma` / `s_cfg.init_hold_ma`。`components/tmc2209/CMakeLists.txt` 的 REQUIRES 去掉 `settings`。

- [x] **Step 3: step_engine 删除方向反转机制**

[step_engine.c](../../../components/step_engine/step_engine.c)：删除 `s_dir_invert` 声明（:59）与 `step_engine_set_dir_invert`（:503-511）；三处 XOR 退化——:130 `bool physical = logical_dir ^ s_dir_invert;` → `bool physical = logical_dir;`，:246 `if (s_dir ^ s_dir_invert)` → `if (s_dir)`，vel_now 快照处删除 `invert` 局部量、`return (dir ^ invert) ? … : …` → `return dir ? (int64_t)v : -(int64_t)v;`（以 grep invert 清零为准）。[step_engine.h](../../../components/step_engine/include/step_engine.h)：删除 `step_engine_set_dir_invert` 声明及其注释块；文件头与 vel/position 注释中「物理域」措辞删除（域即引擎逻辑域）。[test_step_engine.c](../../../components/step_engine/test/test_step_engine.c)：删除用例 7「方向反转：DIR 电平即时翻转与计数反向」整段（其锚定的「陈旧 DIR 电平」问题在符号上收后结构性消失，见计划头部偏差记录 1）。

- [x] **Step 4: cartaxis 接线组件**

`components/cartaxis/include/cartaxis.h`：

```c
/*
 * cartaxis — 小车直线轴解释组件（mm 域命令面 ↔ motor steps 域）
 *
 * 换算即读即算 settings 派生 steps_per_mm（$0×$1÷$2）；X_INV 经
 * 指令与读数统一取反（cartfsm 的调用面签名与旧 motor.h 一致）；
 * $0 细分档由本组件单一写者持有（决策表见规格「cartaxis」节）。
 * 语义：docs/superpowers/specs/2026-10-14-device-category-abstraction.md。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 绑定电机实例并订阅 $0/$1/$2（$0 变更按静止/运动两路处理）。 */
void cartaxis_init(motor_handle_t m);

/** 以下签名与语义同旧 motor.h 业务面（消费方 cartfsm / flowtest 只改名）。 */
esp_err_t cartaxis_enable(bool en);
void cartaxis_set_vel_mm_s(long v_mm_s);
void cartaxis_halt(void);
void cartaxis_stop(void);
bool cartaxis_at_rest(void);
int64_t cartaxis_cmd_position_um(void);
void cartaxis_set_position_um(long um);   /* 前置：电机静止 */
void cartaxis_set_accel_mm_s2(long a_mm_s2);
void cartaxis_set_decel_mm_s2(long d_mm_s2);
void cartaxis_set_wdt_ms(long t_ms);

#ifdef __cplusplus
}
#endif
```

`components/cartaxis/cartaxis.c`：

```c
/*
 * cartaxis — 小车直线轴解释实现
 *
 * $0 单一写者：订阅 $0/$1/$2——$1/$2 只影响换算系数（即读即算，
 * 即时生效）；$0 变更时静止则回调内立即「推细分→按新单位重定基
 * （µm 值不变）」，运动中先斜坡停并挂起，此后任意入口在静止时应用
 * （cartfsm 监督任务 1 kHz 调用读数，滞后 ≤ 数拍；DISABLED 必静止，
 * 回调内即完成）。细分写失败放弃本次并告警（多为链路故障，bringup
 * 诊断后重设 $0 恢复）。
 */
#include "cartaxis.h"
#include "cartaxis_conv.h"

#include "esp_log.h"

#include "settings.h"
#include "params.h"

static const char *TAG = "cartaxis";

static motor_handle_t s_m;
static volatile bool s_pending_microstep = false;

static double spm(void)
{
    return (double)settings_steps_per_mm();
}

static long sgn(void)
{
    return (fw_params()->x_inv != 0) ? -1L : 1L;
}

/* 无挂起检查的原始读数（apply_microstep_now 内部使用，防递归） */
static long raw_cmd_um(void)
{
    return sgn() * cartaxis_steps_to_um(motor_position_steps(s_m), spm());
}

static void apply_microstep_now(void)
{
    s_pending_microstep = false;
    long um = raw_cmd_um();

    esp_err_t err = motor_set_microstep(s_m, settings_microstep());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "细分写入失败（%d）——保持旧档，链路恢复后重设 $0", err);
        return;
    }
    cartaxis_set_position_um(um);   /* 静止下按新单位重定基，µm 值不变 */
}

static void maybe_apply_pending(void)
{
    if (s_pending_microstep && motor_at_rest(s_m)) {
        apply_microstep_now();
    }
}

static void on_settings_change(int idx, float new_value)
{
    (void)new_value;
    if (idx != SET_MICROSTEP) {
        return;   /* $1/$2 影响换算系数，即读即算即时生效 */
    }
    if (motor_at_rest(s_m)) {
        apply_microstep_now();
    } else {
        motor_halt(s_m);
        s_pending_microstep = true;
    }
}

void cartaxis_init(motor_handle_t m)
{
    s_m = m;
    if (!settings_subscribe(on_settings_change)) {
        ESP_LOGW(TAG, "settings 订阅失败（槽满）——$0 变更不会生效");
    }
}

esp_err_t cartaxis_enable(bool en)
{
    return motor_enable(s_m, en);
}

void cartaxis_set_vel_mm_s(long v_mm_s)
{
    maybe_apply_pending();
    motor_set_vel(s_m, sgn() * cartaxis_mmps_to_steps(v_mm_s, spm()));
}

void cartaxis_halt(void)
{
    maybe_apply_pending();
    motor_halt(s_m);
}

void cartaxis_stop(void)
{
    motor_stop(s_m);
}

bool cartaxis_at_rest(void)
{
    maybe_apply_pending();
    return motor_at_rest(s_m);
}

int64_t cartaxis_cmd_position_um(void)
{
    maybe_apply_pending();
    return raw_cmd_um();
}

void cartaxis_set_position_um(long um)
{
    motor_set_position(s_m,
                       (int32_t)(sgn() * cartaxis_um_to_steps(um, spm())));
}

void cartaxis_set_accel_mm_s2(long a_mm_s2)
{
    motor_set_accel(s_m, cartaxis_accel_to_steps(a_mm_s2, spm()));
}

void cartaxis_set_decel_mm_s2(long d_mm_s2)
{
    motor_set_decel(s_m, cartaxis_accel_to_steps(d_mm_s2, spm()));
}

void cartaxis_set_wdt_ms(long t_ms)
{
    motor_set_wdt_ms(s_m, (uint32_t)t_ms);
}
```

`components/cartaxis/CMakeLists.txt` 替换为：

```cmake
# cartaxis — 小车直线轴解释组件：mm↔steps 换算 + $0 单一写者 + X_INV 取反
idf_component_register(SRCS "cartaxis_conv.c" "cartaxis.c"
                    INCLUDE_DIRS "include"
                    REQUIRES motor settings cartparams)
```

- [x] **Step 5: cartfsm 与 flowtest 调用面切换**

[cartfsm.c](../../../components/cartfsm/cartfsm.c)：include `motor.h` → `cartaxis.h`；全部 `motor_` 前缀改 `cartaxis_`（params_apply_engine、clamp_and_commit_velocity、supervise_soft_limit、params_apply_one、enter_fault、exec_mark_right、exec_enable、exec_reset、fsm_task settling 段、cartfsm_start）；**删除** params_apply_engine 的 `motor_set_dir_invert(...)` 行与 params_apply_one 的 `FP_X_INV` case（X_INV 由 cartaxis 读时取反，两处不再有作用点）；文件头注释「motor / cartproto」措辞改「cartaxis / cartproto」。`components/cartfsm/CMakeLists.txt`：REQUIRES 中 `motor` 改 `cartaxis`。

[test_cartfsm_flow.c](../../../components/cartfsm/test/test_cartfsm_flow.c)：include `motor.h` → `cartaxis.h`；:234 `motor_at_rest()` → `cartaxis_at_rest()`；:246 `motor_set_vel_mm_s(200)` → `cartaxis_set_vel_mm_s(200)`。

- [x] **Step 6: 组装点切换与 $3/$4 中转**

`main/app_main.c`：include 区改（`motor_tmc2209.h` 保留、增 `cartaxis.h` 与 `motor.h`——中转电流用）；motor 组装块替换为：

```c
        motor_handle_t mot;
        motor_tmc2209_config_t mcfg = {
            .tmc = {
                .uart_num = UART_NUM_2,   /* 端口分配：UART0=烧录/日志、UART1=协议、UART2=TMC */
                .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
                .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
                .en_gpio = CONFIG_BSP_EN_GPIO,
                .baud = 115200,
                .slave_addr = 0,
                .r_sense = CONFIG_TMC2209_RSENSE,
                .init_microstep = settings_microstep(),
                .init_run_ma = settings_run_current_ma(),
                .init_hold_ma = settings_hold_current_ma(),
            },
            .step_gpio = CONFIG_BSP_STEP_GPIO,
            .dir_gpio = CONFIG_BSP_DIR_GPIO,
        };
        ESP_ERROR_CHECK(motor_tmc2209_init(&mcfg, &mot));
        cartaxis_init(mot);
        /* $3/$4 中转：纯驱动器参数由组装点推送（返回值接住并告警，
         * 消灭旧订阅回调吞返回值的脱钩窗口） */
        s_mot = mot;
        if (!settings_subscribe(on_driver_settings)) {
            ESP_LOGW("app_main", "settings 订阅失败（槽满）——$3/$4 变更不会生效");
        }
```

文件顶部（app_main 之前）加：

```c
static motor_handle_t s_mot;   /* $3/$4 中转用（app_main 单次启动） */

static void on_driver_settings(int idx, float new_value)
{
    (void)new_value;
    if (idx == SET_RUN_MA || idx == SET_HOLD_MA) {
        esp_err_t err = motor_set_current(s_mot, settings_run_current_ma(),
                                          settings_hold_current_ma());
        if (err != ESP_OK) {
            ESP_LOGE("app_main", "电流写入失败（%d）——保持旧值", err);
        }
    }
}
```

启动次序注释同步（cartpos/poleang/cartaxis 解释组件 + 组装点订阅）。`main/CMakeLists.txt` REQUIRES：`bsp cartproto settings magnetic_encoder cartpos poleang motor cartaxis telemetry cartfsm`。`test/main/test_app_main.c` 同样替换（含 s_mot 与中转）；`test/main/CMakeLists.txt` REQUIRES：`bsp settings magnetic_encoder cartpos poleang motor cartaxis cartproto cartfsm telemetry`。

- [x] **Step 7: 构建与 host 验证**

构建命令同 Task 1 Step 5，预期 MAIN=0、TEST=0。host 复跑 cartfsm_core（t1=0，87/87）与 vel_ramp（t6=0，48/48）：

```bash
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && gcc -std=c11 -Wall -Wextra -Icomponents/cartfsm/include -Icomponents/cartparams/include host_tests/cartfsm_core_test.c components/cartfsm/cartfsm_core.c components/cartparams/params.c -o /tmp/t1 && /tmp/t1; echo t1=\$?; gcc -std=c11 -Wall -Wextra -Icomponents/step_engine/include host_tests/vel_ramp_test.c components/step_engine/vel_ramp.c -o /tmp/t6 && /tmp/t6; echo t6=\$?"
```

- [x] **Step 8: Commit**

```bash
git add components/motor components/tmc2209 components/step_engine components/cartaxis components/cartfsm main test host_tests
git commit -m "motor 类别契约切 steps 域（句柄式）：cartaxis 解释组件接线（mm 面、\$0 单一写者、X_INV 取反），tmc2209 去 settings（初值经 config 注入、\$3/\$4 组装点中转），step_engine 删方向反转机制，cartfsm/flowtest 调用面切换"
```

### Task 5: 全量验证 + 文档更新 + 上板清单

**Files:**
- Modify: `README.md`（工程结构节）
- Modify: 本计划附录 B（验证记录回填）

- [x] **Step 1: host 测试全量**

```bash
wsl -e bash -c "cd /mnt/c/Users/84580/workspace/cld/PD-STEP && \
gcc -std=c11 -Wall -Wextra -Icomponents/cartfsm/include -Icomponents/cartparams/include host_tests/cartfsm_core_test.c components/cartfsm/cartfsm_core.c components/cartparams/params.c -o /tmp/t1 && /tmp/t1; echo t1=\$?; \
gcc -std=c11 -Wall -Wextra -Ihost_tests/stub -Icomponents/cartproto/include host_tests/cartproto_parse_test.c components/cartproto/cartproto_parse.c -o /tmp/t2 && /tmp/t2; echo t2=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/mt6835/include host_tests/mt6835_crc_test.c components/mt6835/mt6835_urad.c -o /tmp/t3 && /tmp/t3; echo t3=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/mt6835/include host_tests/mt6835_urad_test.c components/mt6835/mt6835_urad.c -o /tmp/t4 && /tmp/t4; echo t4=\$?; \
gcc -std=c11 -Wall -Wextra -Ihost_tests/stub -Icomponents/telemetry/include host_tests/telemetry_format_test.c components/telemetry/telemetry_format.c -o /tmp/t5 && /tmp/t5; echo t5=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/step_engine/include host_tests/vel_ramp_test.c components/step_engine/vel_ramp.c -o /tmp/t6 && /tmp/t6; echo t6=\$?; \
gcc -std=c11 -Wall -Wextra -Icomponents/cartaxis/include host_tests/cartaxis_conv_test.c components/cartaxis/cartaxis_conv.c -lm -o /tmp/t7 && /tmp/t7; echo t7=\$?"
```

预期：七个全部退出码 0（t4 为 18/18、t5 为 11/11、t7 为 17/17，其余不变）。

- [x] **Step 2: 依赖规则构建期验证（Review Focus 5）**

```bash
grep -n "REQUIRES" components/magnetic_encoder/CMakeLists.txt components/motor/CMakeLists.txt components/as5600/CMakeLists.txt components/mt6835/CMakeLists.txt components/tmc2209/CMakeLists.txt components/step_engine/CMakeLists.txt components/cartpos/CMakeLists.txt components/poleang/CMakeLists.txt components/cartaxis/CMakeLists.txt components/cartfsm/CMakeLists.txt components/telemetry/CMakeLists.txt main/CMakeLists.txt
```

逐条对照规格依赖规则 1–6：类别实现与器件组件的 REQUIRES 中不得出现 `settings`、`cartparams`、`bsp`；cartfsm 不得出现 `motor`/`magnetic_encoder`/器件；解释层不得出现器件组件。

- [x] **Step 3: 双工程最终构建**

构建命令同 Task 1 Step 5，预期 MAIN=0、TEST=0。

- [x] **Step 4: README 工程结构更新 + 附录回填 + Commit**

「工程结构」树更新：`magnetic_encoder/`（磁编码器类别：契约 + AS5600/MT6835 实现，句柄式）、`cartaxis/`（直线轴解释：mm↔steps、$0 单一写者）两行新增；`cartpos/`、`poleang/`、`motor/` 行描述改为解释组件 / steps 域类别组件表述；as5600/mt6835 行注明「（驱动核心，经类别组件组合）」；任务/CPU 布局段核对（任务集合不变，任务名 me_as5600/me_mt6835）。回填附录 B 后提交：

```bash
git add README.md docs/superpowers/plans/2026-10-14-device-category-abstraction.md
git commit -m "类别抽象收尾：README 工程结构更新（magnetic_encoder/cartaxis/解释组件），计划验证记录回填（上板项标注待执行）"
```

---

## 附录 A：上板回归清单（需实机，人工执行）

1. **编码器在线**：`[as5600]` 与 `[mt6835]` 标签 B 组全绿（新口径：B16 为 counts/s、B17/B18 在 cartpos 层、含解释层对拍用例）；首读日志正常。
2. **STATE 数值域比对**：端侧联调（tools/cartpole_host.py）抓 STATE 行，行格式与 reason 逐字节一致；四个数值字段与迁移前记录差 ≤1 量化级（x_um ±1 µm、vx ±1 mm/s、vth ±1 mrad/s）；标定参数（POLE_ZERO_URAD/POLE_INV、$2）设置后数值与迁移前同参数一致。
3. **MODE reason 兼容**：拔 AS5600 / 拔 MT6835 各触发一次 FAULT，reason 为 "AS5600" / "MT6835"；失败期间 STATE 流保持最后成功值。
4. **FAULT 停机次序**：READY 下运动中注入编码器失联（flowtest），确认斜坡停 → 速度精确为零后关 EN → 推 MODE FAULT；RESET 恢复。
5. **flowtest 全量**：`[flowtest]` 用例全绿（3c 运动态注入语义不变）。
6. **$0 变更两条路径（新增）**：DISABLED 下 `$0` 改变后立即生效（x_um/cmd_position_um 读数不变）；READY 运动中 `$0` 改变 → 斜坡停 → 静止后自动应用，cmd_position_um 的 µm 值在变更前后一致（≤1 步重定基误差）。
7. **X_INV 符号验证（若台位启用）**：VEL 200 → x_um 增大；|E−C| 无系统性偏差。
8. **bringup / manual 段**：寄存器核对（含 init_microstep/电流经 config 注入后的上电值）；A9/A10 手动段。

## 附录 B：验证记录回填

环境：Windows 11 + ESP-IDF v6.1（`C:\esp\v6.1\esp-idf`）+ Ninja；宿主测试 WSL gcc 13.3.0。构建与宿主命令见 Global Constraints 与各任务 Step。

| 任务 | 日期 | 环境 | 命令与结果 | 上板项 |
|---|---|---|---|---|
| Task 1 | 2026-10-14（d5b6853） | 同上 | 双工程构建 MAIN=0 TEST=0；host t1=0（87/87）、t5=0（10/10——被删 counts_to_um 段实为 5 条断言，见偏差记录 5） | 附录 A 1/2 待上板 |
| Task 2 | 2026-10-14（0af7b84） | 同上 | 双工程构建 MAIN=0 TEST=0；host t3=0（2/2）、t4=0（16/16——被删 counts_to_mrad_s 段实为 3 条断言，见偏差记录 5）；契约改操作表分发见偏差记录 4 | 附录 A 1/2/3 待上板 |
| Task 3 | 2026-10-14（f8f5e23） | 同上 | TDD 红灯（cartaxis_conv.h 不存在编译失败）→ 绿灯 t7=0（18/18——计划写 17 系计数笔误）；主工程 MAIN=0 | — |
| Task 4 | 2026-10-14（5521dd6） | 同上 | 双工程构建 MAIN=0 TEST=0；host t1=0（87/87）、t6=0（48/48） | 附录 A 4/5/6/7/8 待上板 |
| Task 5 | 2026-10-14 | 同上 | host 七测全量：t1=0（87/87）、t2=0（64/64）、t3=0（2/2）、t4=0（16/16）、t5=0（10/10）、t6=0（48/48）、t7=0（18/18）；依赖规则 grep 逐条核对通过（规则 1–6）；双工程构建沿用 Task 4 结果（此后仅文档改动） | 附录 A 全部待上板 |
