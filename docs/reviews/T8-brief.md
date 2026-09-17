# T8 任务包 · AS5600 编码器驱动 + 多圈计圈

> 分发对象：执行 Agent · 工作区：**主 worktree `C:\Users\SM\ESP_workspace\PD-step-test`（master）**，直接在工作区改动，**禁止一切 git 操作**（沙箱禁写 .git/objects；提交由用户执行）
> 上游事实源：PLAN.md（§3.1 数据流、§3.2 任务划分、§4.5 状态帧、D6/D7/D10、T8 节）、components/bsp/include/bsp.h、components/bsp/CMakeLists.txt、components/settings/include/settings.h、main/app_main.c

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 实现 `components/as5600/`（骨架已存在，API 见 §4，算法见 §5）
3. app_main 接线（§6）+ Kconfig/CMake（§7）
4. 自测（§8，≥20 例）+ 双配置构建验证（§9）
5. 最终答复：改动文件清单、计圈算法推导（含跨零修正与噪声分析）、自测预期输出、构建证据、给用户的上板操作说明（§10）

**红线（越界即验收不通过）**：
- 🚨 **绝对禁止任何 I2C 写操作**——AS5600 的 ZPOS/MPOS/CONF/MANG 等寄存器写入会烧写**一次性 OTP**，烧错永久损坏芯片。本组件**只允许读操作**（`i2c_master_transmit_receive` 读寄存器），**代码中出现任何 `i2c_master_transmit` 写调用或寄存器写辅助函数即验收不通过**
- 禁止改动：`components/bsp/`、`components/protocol/`、`components/gcode_parser/`、`components/settings/`、`components/tmc2209/`、`components/step_engine/`、`components/planner/`、`main/app_m3_test.c`、`main/app_joint_test.c`（全部已验收闭环）
- 只准改动：`components/as5600/` 下全部文件、`main/app_main.c`（§6 指定锚点）、`components/bsp/Kconfig` **不动**
- 禁止 git 操作（含 add/commit/stash/checkout）
- 主工作区有一批 **CRLF 行尾噪音 M**（`git status` 可见，`git diff --numstat` 为空）：**忽略，勿 add、勿还原**
- 等待必须用 `vTaskDelay` / `vTaskDelayUntil`（看门狗），**禁止忙等**
- 只准使用 §1 列出的公开 API，禁止 include 其他组件的内部头

## 1. 可用 API（已核实存在，验收按此检查）

```c
/* bsp（T2）—— I2C 总线由 bsp 创建，本组件只取 handle 并添加设备 */
i2c_master_bus_handle_t bsp_get_i2c_bus(void);   /* NULL 表示总线未初始化 */

/* settings（T11）—— counts→mm 换算因子 */
float settings_units_per_rev(void);   /* $2，默认 40.0 mm/圈 */
bool  settings_subscribe(settings_change_cb_t cb);   /* 最多 4 槽，已用 3（tmc2209/planner/app $10） */

/* main/app_main.c 现状（T7 后） */
static void cb_on_set_origin(void *ctx, float new_pos_mm) { planner_set_origin(new_pos_mm); }
```

**IDF v6 新 I2C master API**（与 `components/bsp/bsp.c` 同一套，组件名 `esp_driver_i2c`）：

```c
i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
i2c_master_transmit_receive(dev, &reg, 1, rx_buf, rx_len, timeout_ms);   /* 只读 */
```

## 2. 硬件/平台事实

| 项 | 值 |
|---|---|
| 器件 | AS5600 磁编码器，7-bit 地址 **0x36**（`CONFIG_BSP_AS5600_I2C_ADDR`；BSP 启动扫描已实测"AS5600 detected at 0x36"） |
| 总线 | I2C_NUM_0，SDA=GPIO8 / SCL=GPIO9，**400kHz**，板上有外部上拉（T2/T0 定案） |
| 关键寄存器（**只读**） | `0x0C`=RAW ANGLE[11:8]（低 4 位有效）、`0x0D`=RAW ANGLE[7:0]；`0x0B`=STATUS（bit5 MD / bit4 ML / bit3 MH 磁铁检测）；`0x1A`=AGC；`0x1C/0x1D`=MAGNITUDE |
| 分辨率 | 4096 counts/圈（12 bit） |
| 传动 | 2GT 20 齿 × 2mm = **40 mm/圈**（D7，`settings_units_per_rev()`）→ **1 count = 40/4096 = 9.7656 µm** |
| 采样 | **1kHz**（PLAN §3.1/§3.2）→ 单次 I2C burst 读 2 字节 @400kHz 约 100–200µs，占用 <20% CPU |
| 任务 | `as5600_task`，优先级 **3**，**固定 CPU1**（避开 CPU0 上的 step ISR 与 comm_task；PLAN §3.2 只规定优先级，核绑定理由见 §5.4） |

## 3. 任务边界

本任务交付"传感层"：I2C 读角度 → 多圈计圈 → 连续位置（counts / mm）+ 速度估计 + G92 偏移接口。

**不做**：失步判断、闭环补偿（D6 明确排除，二期候选）；状态帧组包（T9）；`?` 响应（T9）。

## 4. 组件 API（定稿，验收按此检查）

```c
/* components/as5600/include/as5600.h */

/* ============ 纯逻辑计圈（硬件无关，可单测） ============ */

typedef struct {
    int32_t  pos_counts;   /* 连续位置（counts，含多圈，可负） */
    uint16_t last_raw;     /* 上次原始角度 0..4095 */
    bool     seeded;       /* 是否已有首个样本 */
} as5600_tracker_t;

/* 归零（丢弃历史，以首个 raw 为起点 0） */
void    as5600_tracker_reset(as5600_tracker_t *t);

/* 喂入一个原始角度，返回更新后的连续位置（counts）。
 * 首个样本仅播种（pos_counts=0），不产生位移。 */
int32_t as5600_tracker_update(as5600_tracker_t *t, uint16_t raw);

/* ============ 驱动 + 任务 ============ */

/* 添加 I2C 设备（用 bsp_get_i2c_bus()）并创建 1kHz 采样任务。
 * 幂等；bus 为 NULL 时返回 ESP_ERR_INVALID_STATE。 */
esp_err_t as5600_init(void);

/* ============ 遥测（T9 消费） ============ */

float    as5600_position_mm(void);       /* 连续位置（mm），受 G92 偏移影响 */
int32_t  as5600_position_counts(void);   /* 连续位置（counts） */
float    as5600_velocity_mm_s(void);     /* 平滑速度（mm/s），静止≈0 */
uint16_t as5600_last_raw(void);          /* 最近原始角度（调试/自测用） */
bool     as5600_is_ok(void);             /* false = I2C 读连续失败 */

/* ============ G92 偏移（D10：与指令位置同步平移） ============ */

/* 把"当前编码器位置"设为 new_pos_mm（不产生运动）。
 * 实现：offset = raw_position_mm − new_pos_mm；上报位置 = raw_position_mm − offset。 */
void     as5600_set_origin(float new_pos_mm);

/* ============ 自测（仅 CONFIG_AS5600_SELF_TEST=y） ============ */
#ifdef CONFIG_AS5600_SELF_TEST
int  as5600_self_test_run(void);   /* 末行输出 "SELF-TEST DONE: <n>/<total> PASS" */
#endif
```

## 5. 实现规格

### 5.1 角度读取（每 1ms 一次）

```
reg = 0x0C; i2c_master_transmit_receive(dev, &reg, 1, buf, 2, 50ms)
raw = ((buf[0] & 0x0F) << 8) | buf[1]        /* 0..4095 */
```
- **必须 burst 连续读 2 字节**（两次独立读会在角度更新瞬间产生拼接错误值）
- 读失败：计数 +1，连续失败 ≥10 次置 `as5600_is_ok()=false`（恢复后清零），**不阻塞任务**
- 采样用 `vTaskDelayUntil(&last, pdMS_TO_TICKS(1))` 保证 1kHz 稳定周期

### 5.2 多圈计圈（核心算法，PLAN T8 节指定）

```
if (!seeded) { last_raw = raw; seeded = true; pos_counts = 0; return 0; }

diff = (int32_t)raw − (int32_t)last_raw;
if      (diff >  2048) diff -= 4096;   /* 反向跨 0/4095 边界 */
else if (diff < -2047) diff += 4096;   /* 正向跨 0/4095 边界 */

pos_counts += diff;
last_raw = raw;
```
- 阈值不对称（`>2048` / `<-2047`）覆盖全部 4096 种 diff，无遗漏、无重叠
- **跨零修正后，边界抖动天然退化为 ±1 count**：raw 4095→0 得 diff=+1（正向跨零 1 count），0→4095 得 diff=−1。故无需额外死区即可满足"静止噪声 ≤2 counts"判据（§8 有专项用例验证）
- 极限转速核算：单次采样可**可靠分辨**的最大步进为 ±2048 counts（半圈；超过会被跨零修正误判方向）。1kHz 采样 → 上限 2048 counts/ms = 2,048,000 counts/s = **500 圈/s = 30000 rpm**。PLAN 要求"对应 >4500 rev/min 仍有余量"→ 4500 rpm = 75 圈/s = **307 counts/ms**，占上限的 **15%**（余量 6.7 倍）✓

### 5.3 单位换算与 G92 偏移

```
mm_per_count = units_per_rev / 4096.0f          /* @40mm/圈 → 0.0097656 mm/count */
raw_mm       = pos_counts * mm_per_count
position_mm  = raw_mm − offset_mm               /* offset 默认 0（上电位置 = 0，D10） */
as5600_set_origin(v): offset_mm = raw_mm − v
```
- `units_per_rev` **运行期读取 `settings_units_per_rev()` 并订阅 $2 变更刷新缓存**（与 planner 的 k 缓存同模式；注意 settings 订阅槽仅剩 1 个，若 subscribe 返回 false 必须 W 级日志告警）
- 上电即 `offset_mm = 0` → 位置从 0 开始（D10"上电默认当前位置为 0"）

### 5.4 速度估计（1kHz 采样 + 平滑）

```
inst_speed(counts/ms) = pos_counts − pos_counts_prev      /* 1ms 差分 */
```
- 窗口平均：**最近 N=16 个 1ms 差分**（16ms 窗口）→ `velocity_mm_s = (pos_counts − pos_counts_16ms_ago) × mm_per_count × 1000 / 16`
- 原始差分噪声 ±1 count = ±9.77 mm/s 瞬时；16 点平均后约 **±2.4 mm/s** ✓（T9 仅上报量级，精度足够）
- 用环形缓冲保存最近 17 个位置样本即可算出（内存 17×4B）

### 5.5 任务与线程安全

- `as5600_task`：优先级 3、栈 ≥3072、**固定 CPU1**（`xTaskCreatePinnedToCore`）。理由：I2C 读是阻塞调用（100–200µs），放 CPU1 不会延迟 CPU0 上的 STEP 脉冲 ISR（T6 已验证 ISR 对时序敏感）
- 位置/速度发布：`int32_t`/`float` 的对齐读写不可分割性不保证 float，**位置用 counts（int32）作真值并加 `portMUX` 或 seqlock 保护**，mm 值由其换算得出（T9 会做原子快照，本任务保证"读到的 counts 是完整的一次更新"即可）
- `as5600_set_origin` 从 comm 任务上下文调用（G92 回调），需与采样任务临界区互斥

## 6. app_main 接线（只动两处锚点）

1. `#include "as5600.h"`
2. `bsp_init()` 成功分支内、**自测段之前**（即"日志镜像 install"之后）——保证自测运行时 1kHz 采样任务已在跑：
   ```c
   esp_err_t as_err = as5600_init();
   if (as_err != ESP_OK) {
       ESP_LOGE(TAG, "AS5600 init failed: %s", esp_err_to_name(as_err));
   }
   ```
3. **G92 扩展（D10 关键）**——`cb_on_set_origin` 增加一行，使 G92 同时平移 C 与 E：
   ```c
   static void cb_on_set_origin(void *ctx, float new_pos_mm)
   {
       (void)ctx;
       planner_set_origin(new_pos_mm);
       as5600_set_origin(new_pos_mm);   /* D10: 编码器上报位置同步平移 */
   }
   ```
4. 日志镜像 `#if` 条件追加 `|| defined(CONFIG_AS5600_SELF_TEST)`（**两处**：钩子定义块、install 调用块——自测需在 12V 墙充供电下经 UART1 看日志）
5. 自测调用（`step_engine`/`planner` 自测之后、`tmc2209_init` 之前）：
   ```c
   #ifdef CONFIG_AS5600_SELF_TEST
       as5600_self_test_run();
   #endif
   ```

## 7. Kconfig / CMake

**`components/as5600/Kconfig`**（新建）：

```kconfig
menu "AS5600 Encoder"

config AS5600_SELF_TEST
    bool "Run AS5600 encoder self-test on boot"
    default n
    help
        Runs the AS5600 self-test during startup: pure-logic
        multi-turn tracking cases plus live I2C read / noise checks.
        Ends with a guided manual check (turn the shaft 3 turns).
        Final line: "SELF-TEST DONE: <n>/<total> PASS".

endmenu
```

**`components/as5600/CMakeLists.txt`**（骨架 `SRCS ""` + `REQUIRES driver` → 实装）：

```cmake
idf_component_register(SRCS "as5600.c" "as5600_tracker.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bsp settings freertos esp_driver_i2c)

if(CONFIG_AS5600_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "as5600_self_test.c")
endif()
```

> I2C 组件名参照 `components/bsp/CMakeLists.txt`（`esp_driver_i2c`）——**不要**写 `REQUIRES driver`（IDF v6 已拆分）。

**`main/CMakeLists.txt`**：`REQUIRES` 追加 `as5600`（只加不改）。

## 8. 自测（`CONFIG_AS5600_SELF_TEST=y`，上板自跑，≥20 例）

前置：自测开头调 `as5600_init()`（幂等，task 开始 1kHz 采样）。用例组：

**A 纯逻辑组（`as5600_tracker_*`，不依赖硬件，12 例）**——直接构造 raw 序列：

| # | 输入序列 | 期望 |
|---|---|---|
| 1 | 首样本 100 | 播种，位置 0 |
| 2 | 100 → 100 | 0（静止） |
| 3 | 100 → 200 → 300 | +200 counts |
| 4 | 4090 → 10 | **+16**（正向跨零：diff −4080→+16） |
| 5 | 10 → 4090 | **−16**（反向跨零：diff +4080→−16） |
| 6 | 0 → 4095 → 0 → 4095（边界抖动 ×8） | 净漂移 **≤1 count**（跨零修正后每步 ∓1） |
| 7 | 模拟正向 3 圈（步进 64 counts ×192 步） | **+12288 counts = 120.0 mm**（PLAN 验收判据的等价纯逻辑验证） |
| 8 | 模拟反向 3 圈 | **−12288 counts** |
| 9 | diff 恰为 +2048（同圈半圈） | +2048（**不**触发跨零） |
| 10 | diff 恰为 −2047（同圈） | −2047（**不**触发跨零） |
| 11 | 每步 +500 counts ×20（快速正转） | +10000（无丢圈） |
| 12 | 正转 2 圈后反转 2 圈 | 回到 0（净零，方向对称） |

**B 硬件组（实机 I2C，6 例）**：

| # | 项 | 判据 |
|---|---|---|
| 13 | `bsp_get_i2c_bus()` 非 NULL 且 `as5600_init()` 返回 OK | 通信建立 |
| 14 | 连续读 20 次 raw | 全部落在 **0..4095** |
| 15 | 静止噪声 | 20 次采样 raw 极差 **≤2 counts**（判据来源：PLAN T8 节） |
| 16 | 静止速度 | `as5600_velocity_mm_s()` 绝对值 **≤5 mm/s** |
| 17 | 位置/换算一致性 | `as5600_position_mm()` ≈ `pos_counts × units_per_rev / 4096` |
| 18 | G92 偏移 | `as5600_set_origin(100.0f)` 后 `as5600_position_mm()` == 100.0（±0.01）；再 `as5600_set_origin(0)` 复原 |

**C 人工核对段（引导式，不计入 PASS 总数，log 输出供人工判读）**：

```
============ MANUAL CHECK ============
Turn the motor shaft 3 FULL TURNS by hand (either direction), now.
Sampling for 15 s — position will be logged every second.
```
每秒 log 一次 `position_mm`；15s 后输出：
```
MANUAL RESULT: delta = <Δ> mm  (expect ~±120.0 for 3 turns; 1 turn = 40.0 mm)
```
- Δ 由"段末位置 − 段首位置"得出，**不做自动 PASS/FAIL**（人工按 PLAN 判据核对 ±0.05mm）
- 若 15s 内 Δ≈0 则附加提示 `(no rotation detected — normal if you did not turn the shaft)`

输出行：`PASS <case>` 逐例 + 末行 `SELF-TEST DONE: 18/18 PASS`（A+B 共 18 例；人工段单独输出）。总时长 ≤30s（含 15s 人工段）。

## 9. 构建验证（三配置，全部零错误零警告）

```powershell
powershell -File idf_env.ps1 build                                     # ① 默认
# sdkconfig 追加 CONFIG_AS5600_SELF_TEST=y 构建                         # ② AS5600 自测
# 再追加 CONFIG_PLANNER_SELF_TEST=y 构建                                # ③ 验证与既有自测共存
```

## 10. 上板操作（写给用户，你只写不执行）

1. PC USB 烧录：`powershell -File idf_env.ps1 -IdfAction "flash" -Port COM15`
2. CH343P 飞线（USB 插 PC）：`RX ← GPIO41`、`GND ↔ GND`；VOFA+ 115200 raw text
3. **换 PD 墙充 12V 上电**（本自测只读 I2C，电机**不转**；12V 非必需，但保持与既有流程一致、便于后续 M4 联测）
4. 观察日志：A/B 组逐条 PASS → 人工段横幅出现时**手动匀速转动电机轴 3 整圈** → 核对 `MANUAL RESULT` 的 Δ ≈ ±120.0 mm

## 11. 验收判据（规划 Agent 按此验收）

| # | 项 | 判据 |
|---|---|---|
| 1 | 🚨 **只读红线** | 全组件**零 I2C 写调用**（代码审查 `i2c_master_transmit` 出现即不合格）；寄存器只读 0x0C/0x0D/0x0B 等 |
| 2 | 红线 | 仅 `components/as5600/` + `main/app_main.c` 锚点 + `main/CMakeLists.txt` 改动；其余组件零改动；无 git 操作 |
| 3 | 计圈算法 | 与 §5.2 规则一致（阈值 `>2048` / `<-2047`）；跨零修正与边界抖动行为符合 §8 用例 |
| 4 | API | 与 §4 签名逐一一致 |
| 5 | 自测 | A+B **18/18 PASS**；人工段 Δ 与 3 圈理论值吻合（用户确认） |
| 6 | 构建 | 三配置零警告零错误 |
| 7 | 集成 | app_main 接线含 G92 双向平移（D10）；日志镜像门控已加 |

---

## 附 A：与 PLAN 的偏差预登记（验收时入档）

1. 「宿主机单测」→ **上板自测**（v1.8 定案沿用：本机无主机编译器；计圈纯逻辑以 Kconfig 门控编入固件在目标机执行）
2. 新增 `as5600_tracker_t` 纯逻辑结构与 `as5600_self_test_run()` 钩子（PLAN 未列，为可测性——计圈算法与 I2C 解耦）
3. 速度窗口 N=16 与"平滑"实现细节（PLAN 只要求"速度估计"，未规定算法）
4. `as5600_task` 核绑定 CPU1（PLAN §3.2 只规定优先级 3；核绑定理由见 §5.5）
