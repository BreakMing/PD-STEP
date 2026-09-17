# T10 任务包 · 状态机与立即停（sysctl）

> 分发对象：执行 Agent · 工作区：**主 worktree `C:\Users\SM\ESP_workspace\PD-step-test`（master）**，直接在工作区改动，**禁止一切 git 操作**（沙箱禁写 .git/objects；提交由用户执行）
> 上游事实源：PLAN.md（§3.2 任务划分、§4.4 立即停语义、§4.5 状态帧、D6/D11、T10 节 + v1.16 遗留项）、components/protocol/include/protocol.h、components/step_engine/include/step_engine.h、components/planner/include/planner.h、components/monitor/include/monitor.h、components/tmc2209/include/tmc2209.h、components/settings/include/settings.h、main/app_main.c

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 新建 `components/sysctl/`（API 见 §4，规格见 §5）
3. planner / monitor / app_main 三处既有文件的小改动（§5.3–§5.5，**本任务许可范围内**）
4. 自测（§8）+ 双配置构建验证（§9）
5. 最终答复：改动文件清单、状态迁移表、`!` 后的位置基准推导（含 `$10` 两种取值）、自测预期输出、构建证据、给用户的上板操作说明（§10）

**红线（越界即验收不通过）**：
- **本任务许可改动的既有文件仅 6 处**（其余一律不动）：
  1. `main/app_main.c`（§6 锚点）
  2. `main/CMakeLists.txt`（REQUIRES 追加 sysctl）
  3. `components/planner/{planner.c, include/planner.h}` —— **仅新增 `planner_rebase_to_executed()`**，其余一字不动
  4. `components/monitor/{monitor.c, include/monitor.h, CMakeLists.txt}` —— **仅替换 STATE 来源 + 加看门狗订阅**
  5. `components/as5600/as5600.c` —— **仅加看门狗订阅与喂狗**（§5.6），其余一字不动
  6. `sdkconfig.defaults` —— **仅追加 `CONFIG_ESP_TASK_WDT_PANIC=y`**（§5.7，已定案）
- 禁止改动：`components/bsp/`、`components/protocol/`、`components/gcode_parser/`、`components/settings/`、`components/tmc2209/`、`components/step_engine/`、`components/as5600/`（除上述第 5 项）、`main/app_m3_test.c`、`main/app_joint_test.c`
- **protocol.c/h 零改动**——`!` 仍由 protocol 的 `stop_cb` 分发，只是回调内容改为调 sysctl（而不是直接调 step_engine）
- **D6 边界**：sysctl 只做状态迁移与停止动作，**不得**加入失步判断、限位拦截、故障决策
- **D11**：仅立即停，**不得**实现暂停/恢复/续跑
- 禁止 git 操作（含 add/commit/stash/checkout）
- 主工作区有一批 **CRLF 行尾噪音 M**：**忽略，勿 add、勿还原**
- 等待必须用 `vTaskDelay` / `vTaskDelayUntil`（看门狗），**禁止忙等**
- 只准使用 §1 列出的公开 API，禁止 include 其他组件的内部头

## 1. 可用 API（已核实存在，验收按此检查）

```c
/* protocol（T3）—— 停止事件分发 */
void protocol_register_stop_cb(protocol_stop_cb_t cb);   /* cb: void (*)(void) */

/* step_engine（T6）—— 立即停 + 位置真值 */
void      step_engine_stop(void);              /* 清队列、停脉冲、位置冻结在已执行值 */
int32_t   step_engine_position_steps(void);    /* 已执行步数（物理方向，含 $10 反相） */
bool      step_engine_busy(void);              /* 队列非空或脉冲在途 */

/* planner（T7）—— 位置记账（本任务新增一个 API，见 §4） */
float   planner_position_mm(void);
int32_t planner_position_steps(void);
void    planner_set_origin(float new_pos_mm);

/* monitor（T9）—— 状态帧（本任务替换其 STATE 来源，见 §5.5） */
esp_err_t monitor_init(void);

/* tmc2209（T5） */
esp_err_t tmc2209_enable(bool en);

/* settings（T11） */
bool  settings_invert_dir(void);     /* $10 */
float settings_steps_per_mm(void);   /* k */
```

## 2. 上游事实（实现前逐条核对）

| # | 事实 | 来源 |
|---|---|---|
| 1 | **§4.4 原文**：`!` =「清空段队列、停脉冲输出；**指令位置计数停在已执行步数**；无恢复命令（D11）」 | PLAN §4.4 |
| 2 | **`!` 的当前实现是"直连"**：`app_main` 的 `cb_on_stop()` **直接**调 `step_engine_stop()`（T6 接的）。T10 要把它改道经 sysctl | app_main.c |
| 3 | **⚠ 关键缺陷：`!` 后 planner 的位置基准会错位**。planner 的 `pos_steps` 在 `enqueue()` 成功时**一次性跳到目标值**（T7 定案："target of the last accepted command"）。`!` 清队列后它**不知道移动被取消**，于是：`G1 X1000` 走到 300mm 被 `!` 停在 300，但 planner 仍记着 1000 → **下一条 `G1 X50` 会算出 −950mm 的 delta**，轴反向狂奔 ✗。这正是 §4.4 要求"指令位置计数停在已执行步数"的原因，也是本任务验收第 2 条的必解项 | PLAN §4.4 + T7 定案 |
| 4 | **`$10`（invert_dir）会翻转物理方向**：`step_engine_position_steps()` 是**物理方向**的已执行步数（符号 = `dir ^ invert`）；而 planner 的逻辑位置在 `dir` 为正时递增。所以两者关系是：`invert=0` 时相等、`invert=1` 时**符号相反**。位置 rebase 必须处理这一点 | T6 定案⑤ + T7 |
| 5 | **T9 的 STATE 目前是推导值**：`monitor.c` 里 `step_engine_busy() ? "RUN" : "IDLE"`。T9 验收已入档"STATE 来源暂为 busy，T10 交付状态机后替换" | PLAN v1.16 T9 节偏差入档⑧ |
| 6 | **sysctl 没有独立任务**：PLAN §3.2 的任务表里没有 sysctl_task。状态必须**按需计算**（无任务、无轮询开销） | PLAN §3.2 |
| 7 | **看门狗现状**：`CONFIG_ESP_TASK_WDT_INIT=y`、超时 30s、`CHECK_IDLE_TASK_CPU0/1=y`、**`TASK_WDT_PANIC` 未设置**（超时只告警不重启）。当前**只有 comm_task（protocol.c）与 placeholder_task（app_main.c）订阅并喂狗**；`as5600_task` 与 `monitor_task` **未订阅**（未订阅 = 不被监控 = 无保护） | 本次核查 |
| 8 | 状态帧的 STATE 取值必须是 **`IDLE` / `RUN`**（全大写） | PLAN §4.5（T9 已修正） |
| 9 | 状态机的取值域是 **`BOOT → IDLE ↔ RUN`**，**无 HOLD / ALARM**（v1.1 已删除） | PLAN T10 节 |

## 3. 任务边界

本任务交付"系统状态与停止控制"：状态机 + `!` 端到端贯通 + 喂狗点覆盖。

**不做**：失步判断、故障停机、限位拦截（D6）；暂停/恢复（D11）；状态帧组包（T9 已完成）。

## 4. 组件 API（定稿，验收按此检查）

```c
/* components/sysctl/include/sysctl.h */

typedef enum {
    SYSCTL_BOOT = 0,   /* 初始化完成前 */
    SYSCTL_IDLE,       /* 待机（队列空且无脉冲） */
    SYSCTL_RUN,        /* 运动中 */
} sysctl_state_t;

/**
 * @brief 初始化状态机：BOOT → IDLE。
 * 幂等。须在 protocol_start / step_engine_init / planner_init 之后调用。
 */
esp_err_t sysctl_init(void);

/**
 * @brief 立即停（`!` 的端到端处理）。
 *
 * 顺序固定（顺序即正确性）：
 *   1. step_engine_stop()              停脉冲、清队列、位置冻结
 *   2. planner_rebase_to_executed()    把逻辑位置回退到实际停止处
 *   3. 状态置 IDLE
 *
 * 从 comm_task 上下文调用（protocol 的 stop_cb）。
 */
void sysctl_stop(void);

/**
 * @brief 当前状态。BOOT 之后按需由引擎忙闲推导并缓存。
 */
sysctl_state_t sysctl_get_state(void);

/**
 * @brief 状态的字符串形式，恒为 "BOOT" / "IDLE" / "RUN"（供状态帧）。
 */
const char *sysctl_state_str(void);

/* ---- 自测（仅 CONFIG_SYSCTL_SELF_TEST=y） ---- */
#ifdef CONFIG_SYSCTL_SELF_TEST
int sysctl_self_test_run(void);
#endif
```

**planner 新增（唯一许可的 planner 改动）**：

```c
/* components/planner/include/planner.h */

/**
 * @brief 把指令位置回退到"实际已执行"的位置（PLAN §4.4）。
 *
 * 在立即停（`!`）清空段队列之后调用：引擎位置已冻结，本函数据它重算
 * 逻辑位置，使下一条 G 命令从轴实际停下的地方算起，而不是从被取消的
 * 目标算起。同时刷新 G92/G91 的基准（解析器侧的 G91 基准由 T7 的
 * 位置提交机制跟随，无需额外处理）。
 */
void planner_rebase_to_executed(void);
```

## 5. 实现规格

### 5.1 状态机（无任务，按需推导）

```
BOOT  ──sysctl_init()──▶  IDLE  ⇄  RUN
                                  (step_engine_busy() 驱动)
        任意状态 ──sysctl_stop()──▶ IDLE
```

- `sysctl_init()` 把状态从 `BOOT` 置为 `IDLE`
- `sysctl_get_state()` 内部刷新：`BOOT` 保持不变；否则 `s_state = step_engine_busy() ? RUN : IDLE`
- **不加任务、不加轮询**（事实 #6）
- 状态变量用 `volatile` 或轻量 `portMUX`（被 comm_task 写、被 monitor_task 读）

### 5.2 `!` 端到端链路（PLAN T10 节指定）

```
上位机发 '!'
  → protocol（实时字符，行中间即可触发）
    → s_stop_cb()  ← app_main 注册的回调
      → sysctl_stop():
          ① step_engine_stop()              脉冲立即停（T6 已验证 <1ms）、队列清空、位置冻结
          ② planner_rebase_to_executed()    逻辑位置回退到实际停止处
          ③ s_state = SYSCTL_IDLE
```

- **顺序不可调换**：必须先停引擎（位置才定下来），再 rebase（读冻结后的位置），最后置状态
- `protocol.c` **零改动**（只是回调实现换了内容）

### 5.3 `planner_rebase_to_executed()` 的位置推导（本任务最需要想清楚的一步）

前提事实：planner 的 `pos_steps` 是**逻辑位置**（`dir=true` 时递增）；`step_engine_position_steps()` 是**物理已执行步数**（`DIR` 引脚高时递增，`DIR = dir ^ $10`）。因此：

```
logical_delta = physical_delta          （$10 = 0）
logical_delta = −physical_delta         （$10 = 1）
```

**实现建议**——在 planner 内记录"队列起点快照"，`!` 时据此回退：

```c
/* enqueue() 内：在调用 step_engine_enqueue() 之前，若队列当时为空则记录快照
 * （enqueue 成功后引擎会立刻 busy，所以必须在 enqueue 前判断） */
bool was_busy = step_engine_busy();
if (!was_busy) {
    s_queue_start_eng = step_engine_position_steps();   /* 入队前的物理位置 */
    s_queue_start_pos = s_pos_steps;                    /* 入队前的逻辑位置 */
}

/* rebase */
void planner_rebase_to_executed(void)
{
    int32_t delta = step_engine_position_steps() - s_queue_start_eng;
    if (settings_invert_dir()) {
        delta = -delta;
    }
    s_pos_steps = s_queue_start_pos + delta;
}
```

要求与自查：
- `$10 = 0` / `$10 = 1` **两种取值都要在报告里代入验证**
- `!` 后 `planner_position_mm()` 必须等于轴实际停下的逻辑位置（§8 用例 8/9 会断言）
- 若实现者认为有更简洁且同样正确的推导（例如直接维护 G92 基准映射），**允许替换方案**，但必须在报告中给出完整推导并说明为何覆盖所有情形（含多段队列、`$10` 两种取值、G92 之后立即 `!`）

### 5.4 其余 planner 行为保持不变

- `enqueue()` 仍在成功时提交目标位置（**运动结束后的位置语义不变**）
- 只有 `!` 触发 rebase
- **不得**改动 enqueue 的 delta 计算方式、软限位、钳制、求解逻辑

### 5.5 monitor 的 STATE 来源替换

```c
/* monitor.c —— build_status_frame() 内 */
- const char *state = step_engine_busy() ? "RUN" : "IDLE";
+ const char *state = sysctl_state_str();
```

- `monitor` 的 `CMakeLists.txt` `REQUIRES` 追加 `sysctl`
- `monitor_self_test.c` 中依赖 `step_engine_busy()` 推导 STATE 的断言（用例 16 的 `ASSERT_STR_EQ("RUN", state)`）**应继续成立**——因为 sysctl 的 RUN 就是由 busy 推导的；若因时序出现 flake，可改为先调用一次 `sysctl_get_state()` 刷新再断言
- 除此之外 monitor 逻辑一字不动

### 5.6 看门狗覆盖（PLAN T10 节"喂狗点覆盖所有任务"）

| 任务 | 现状 | 本任务动作 |
|---|---|---|
| comm_task | ✓ 已订阅+喂狗（protocol.c） | 不动 |
| placeholder_task | ✓ 已订阅+喂狗（app_main.c） | 不动 |
| **as5600_task** | ✗ 未订阅 | **加订阅 + 循环内喂狗**（在 `components/as5600/as5600.c` 内；这是本任务对 as5600 的**唯一**许可改动——仅增订阅与 `esp_task_wdt_reset()`） |
| **monitor_task** | ✗ 未订阅 | **加订阅 + 循环内喂狗**（`components/monitor/monitor.c`） |

- 两任务的周期分别是 1ms 与 10ms，远小于 30s 超时 ✓
- 订阅失败（返回非 ESP_OK）时打 **W 级日志**并继续运行（不 abort）
- 若任务退出路径存在，须 `esp_task_wdt_delete()`

> ⚠ **对 §5.6 的例外说明**：as5600 出现在"禁止改动"名单中，但**本任务明确授权**只能新增看门狗订阅与喂狗调用；除此之外 as5600 一字不动。

### 5.7 启用 `CONFIG_ESP_TASK_WDT_PANIC`（已定案，必做）

**决策已定（2026-09-17）：启用。** 权衡如下：

| 选项 | 后果 | 结论 |
|---|---|---|
| 保持 `n` | 任务卡死时只打印告警，系统继续带病运行；**卡死的系统无法响应 `!`**，电机可能持续转动 ✗ | 不采纳 |
| **启用 `y`** | 超时触发 panic → 重启；重启后 EN 高（TMC 失能）+ 队列空 → **电机停下来** ✓ | **采纳** |

**动作**：在 `sdkconfig.defaults` 中追加

```
CONFIG_ESP_TASK_WDT_PANIC=y
```

该文件在版本控制内，会随提交生效；构建时 confgen 会将其落到 `sdkconfig`。

**已知影响（必须在完成报告中复述）**：panic 重启后**位置基准归零**——`planner` / `as5600` / `monitor` 的 G92 参考全部回到上电态，**上位机必须重新 `G92` 建零**。这是"失控时优先停下来"与"位置连续性"之间的取舍，已明确接受。

## 6. app_main 接线（只动指定锚点）

1. `#include "sysctl.h"`
2. `cb_on_stop()` 改为走 sysctl：
   ```c
   static void cb_on_stop(void)
   {
       sysctl_stop();   /* T10: 停引擎 + 位置 rebase + 回 IDLE */
   }
   ```
3. 新增 `sysctl_init()` 调用，位置在 **`monitor_init()` 之前**（monitor 要读 sysctl 状态）：
   ```c
   esp_err_t sc_err = sysctl_init();
   if (sc_err != ESP_OK) {
       ESP_LOGE(TAG, "sysctl init failed: %s", esp_err_to_name(sc_err));
   }
   ```
4. 自测调用（与 monitor 自测同处"晚自测"位置）：
   ```c
   #ifdef CONFIG_SYSCTL_SELF_TEST
       sysctl_self_test_run();
   #endif
   ```
5. 日志镜像 `#if` 条件追加 `|| defined(CONFIG_SYSCTL_SELF_TEST)`（**两处**）
6. `placeholder_task` **保留不动**（其喂狗已就位；是否移除不在本任务范围）

## 7. Kconfig / CMake

**`components/sysctl/Kconfig`**（新建）：

```kconfig
menu "System Control"

config SYSCTL_SELF_TEST
    bool "Run sysctl state-machine / immediate-stop self-test on boot"
    default n
    help
        Runs the state-machine and '!' immediate-stop self-test.
        Final line: "SELF-TEST DONE: <n>/<total> PASS".

endmenu
```

**`components/sysctl/CMakeLists.txt`**（骨架 `SRCS ""` → 实装）：

```cmake
idf_component_register(SRCS "sysctl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES step_engine planner settings freertos)

if(CONFIG_SYSCTL_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "sysctl_self_test.c")
endif()
```

**`main/CMakeLists.txt`**：`REQUIRES` 追加 `sysctl`（只加不改）。

## 8. 自测（`CONFIG_SYSCTL_SELF_TEST=y`，上板自跑，≥14 例）

前置：`sysctl_init()`。位置断言用 `planner_position_mm()` 与 `step_engine_position_steps()` 交叉比对。**测完务必 `step_engine_stop()` + `sysctl_stop()` 收尾**，不残留脉冲。

**A 状态机组（6 例）**：

| # | 操作 | 期望 |
|---|---|---|
| 1 | `sysctl_init()` 幂等，返回 OK | ✓ |
| 2 | 初始化后 `sysctl_get_state()` | `SYSCTL_IDLE`（非 BOOT） |
| 3 | `sysctl_state_str()` | 恒为 `"IDLE"`/`"RUN"`/`"BOOT"` 之一（全大写）；静止时为 `"IDLE"` |
| 4 | 入队一段 → `sysctl_get_state()` | `SYSCTL_RUN`；`state_str()` == `"RUN"` |
| 5 | 等段跑完 → `sysctl_get_state()` | 回 `SYSCTL_IDLE` |
| 6 | `sysctl_stop()` 后立即查状态 | `SYSCTL_IDLE`，且 `!step_engine_busy()` |

**B 立即停与位置基准组（8 例）——本任务核心**：

| # | 操作 | 期望 |
|---|---|---|
| 7 | 入队长段（如 20000 步 @2000 steps/s ≈ 10s），跑 ~800ms 后 `sysctl_stop()` | `!busy()`；位置冻结（再等 200ms 读不变） |
| 8 | 承接 7：`planner_position_mm()` 与 `step_engine_position_steps()/k` | **两者相等（±0.001）**——即"指令位置停在已执行步数"（§4.4） |
| 9 | 承接 7：该位置**显著小于**被取消的目标（如目标 250mm、实际停在 ~20mm） | ✓ 证明 rebase 生效（若等于目标则说明 rebase 没做） |
| 10 | **承接 9 发新命令**：`planner_enqueue(实际位置 + 20mm, ...)` | 正常入队并执行；最终位置 == 实际位置 + 20mm（**这是 T10 验收第 2 条**：`!` 后新命令不受影响） |
| 11 | `!` 后 `G91` 相对移动基准正确 | 相对目标解析基于 rebase 后的位置（若可测；否则在报告中说明由 B10 间接覆盖） |
| 12 | 连续两次 `sysctl_stop()`（幂等） | 不崩溃、状态仍 `IDLE`、位置不变 |
| 13 | `!` 后继续入队 **多段**（如 3 段各 2000 步） | 全部正常执行、位置累计正确（验证 rebase 后队列可复用） |
| 14 | 长段执行中 `sysctl_stop()`，随后立即 `sysctl_get_state()` 与 `state_str()` | 均为 `IDLE`（状态与引擎一致，PLAN T10 节"状态与帧内 STATE 一致性"） |

**C 看门狗组（并入上述用例，不单列）**：
- 用例 4/5 期间覆盖 as5600_task 与 monitor_task 的正常运行；喂狗订阅失败会在日志留 W（人工核对）
- 不做"故意卡死触发 WDT"的破坏性用例（会 panic 重启），改为**代码审查 + 启动日志核对**

输出行：`PASS <case>` 逐例 + 末行 `SELF-TEST DONE: <n>/<total> PASS`。总时长 ≤30s。

## 9. 构建验证（两配置，全部零 error 零 warning）

```powershell
powershell -File idf_env.ps1 build                                    # ① 默认（自测关）
# sdkconfig 追加 CONFIG_SYSCTL_SELF_TEST=y 构建                        # ② 自测开
```

> 若遇到 `esp_lcd_panel_rgb.c` 之类的 **IDF 内置组件编译器 ICE**（Segmentation fault），直接重试即可——T9 验收期出现过一次，属偶发环境问题。

## 10. 上板操作（写给用户，你只写不执行）

1. PC USB 烧录：`powershell -File idf_env.ps1 -IdfAction "flash" -Port COM15`
2. CH343P 三线：`RX ← GPIO41`、`TX → GPIO42`、`GND ↔ GND`
3. **PD 墙充 12V 上电**（本测试电机要真转）
4. **`!` 端到端验证序列**：
   ```
   M17
   G92 X0
   $13=1000            ← 临时放大软限位，留出长行程
   G1 X1000 F3000 A60  ← 长行程（约 20 s）
   ...运动中（观察到 RUN 帧持续刷出）...
   !                   ← 立即停
   ?                   ← 应回 <IDLE|C:<实际停位>|E:<接近同一值>|V:0.000|T:OK>
   G1 X50 F1200        ← 关键：应从"实际停位"继续走（而不是反向狂奔）
   ?
   $13=600             ← 用完恢复默认
   ```
5. **判读标准**：

| 观察 | 期望 |
|---|---|
| 发 `!` 后 | 脉冲**立即**停止；不再有 RUN 帧；`?` 回 `IDLE` |
| `C` 冻结值 | = 实际停下的位置（**不是** 1000） |
| `C` 与 `E` | 差值仍在 0.1mm 级（M4 已验证） |
| 随后 `G1 X50` | **从实际停位走到 50.000** ✓ — 这是本任务验收第 2 条 |
| 若 `G1 X50` 反向狂奔 | ✗ rebase 未生效，回退检查 §5.3 |

## 11. 验收判据（规划 Agent 按此验收）

| # | 项 | 判据 |
|---|---|---|
| 1 | 红线 | 仅 §0 列出的 4 类文件改动（sysctl 新组件 + planner 一个 API + monitor STATE/看门狗 + app_main 锚点 + CMake）；**protocol.c/h 零改动**；无 git 操作 |
| 2 | D6/D11 | 无失步判断/限位拦截/故障决策；无暂停恢复 |
| 3 | 状态机 | `BOOT → IDLE ↔ RUN`，取值只有三态；`sysctl_state_str()` 全大写；无独立任务 |
| 4 | **`!` 端到端** | 长行程中发 `!` → 脉冲立即停（<1ms）、队列清空、**C 冻结在已执行位置**、STATE 回 `IDLE`；**后续新 G 命令从实际停位正常执行**（T10 验收两条硬判据） |
| 5 | 位置推导 | `$10 = 0/1` 两种取值下的 rebase 推导完整且经自测用例 8/9 断言 |
| 6 | 看门狗 | as5600_task 与 monitor_task 已订阅并喂狗；订阅失败有 W 日志；**`sdkconfig.defaults` 已加 `CONFIG_ESP_TASK_WDT_PANIC=y`** 且构建后 `sdkconfig` 中该项为 y |
| 7 | 自测 | **14/14 PASS** |
| 8 | 构建 | 两配置零警告零错误 |
| 9 | 上板 | §10 全部判读项通过（用户确认） |

---

## 附 A：与 PLAN 的偏差预登记（验收时入档）

1. **新增 `planner_rebase_to_executed()`**（PLAN T10 节未列）：§4.4"指令位置计数停在已执行步数"在 T7 的"入队即提交目标"语义下无法自动满足，必须显式回退——否则 `!` 后下一条命令会按被取消的目标算 delta（事实 #3）
2. **monitor 的 STATE 来源替换为 sysctl**（T9 验收已预登记为移交项）
3. **as5600_task 增加看门狗订阅**（本任务授权的最小改动，其余一字不动）
4. **as5600 / monitor 的喂狗**：因两任务均为固定周期且周期 ≪ 超时，采用"循环内 `esp_task_wdt_reset()`"而非"仅订阅"
5. **sysctl 无独立任务**（PLAN §3.2 任务表无 sysctl_task），状态按需推导并缓存
6. **`CONFIG_ESP_TASK_WDT_PANIC` 由 `n` 改为 `y`**（2026-09-17 定案，安全性权衡见 §5.7）。已知影响：panic 重启后位置基准归零，上位机须重新 `G92` 建零；此项**改变了既有系统的失效行为**，是 T10 引入的唯一全局性策略变更
7. `placeholder_task` 保留（移除不在本任务范围）
8. 自测用"长段 + `sysctl_stop()`"覆盖 `!` 链路，不做真实 UART1 的 `!` 字符注入（后者留上板验证）
