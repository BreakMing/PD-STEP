# T9 任务包 · 遥测组包（monitor_task）

> 分发对象：执行 Agent · 工作区：**主 worktree `C:\Users\SM\ESP_workspace\PD-step-test`（master）**，直接在工作区改动，**禁止一切 git 操作**（沙箱禁写 .git/objects；提交由用户执行）
> 上游事实源：PLAN.md（§3.1 数据流、§3.2 任务划分、§4.5 状态帧格式、D6/D12、T9 节 + v1.15 遗留项）、components/protocol/include/protocol.h、components/as5600/include/as5600.h、components/planner/include/planner.h、components/step_engine/include/step_engine.h、components/tmc2209/include/tmc2209.h、components/settings/include/settings.h、main/app_main.c

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 实现 `components/monitor/`（骨架已存在，API 见 §4，规格见 §5）
3. app_main 接线（§6）+ Kconfig/CMake（§7）
4. 自测（§8）+ 双配置构建验证（§9）
5. 最终答复：改动文件清单、旗标映射表设计、自测预期输出、构建证据、给用户的上板操作说明（§10）

**红线（越界即验收不通过）**：
- 禁止改动：`components/bsp/`、`components/protocol/`、`components/gcode_parser/`、`components/settings/`、`components/tmc2209/`、`components/step_engine/`、`components/planner/`、`components/as5600/`（全部已验收闭环）、`main/app_m3_test.c`、`main/app_joint_test.c`
- 只准改动：`components/monitor/` 下全部文件、`main/app_main.c`（§6 指定锚点）、`main/CMakeLists.txt`（只加不改）
- **协议层零改动**——状态帧通过 `protocol_send_raw()` 发送、通过 `protocol_register_status_cb()` 注册，protocol.c/h 一个字都不动（T3 占位逻辑保留，仅被覆盖）
- **D6 红线：纯上报，零判断**——不得包含失步判断、故障停机、限位拦截等任何决策逻辑
- 禁止 git 操作（含 add/commit/stash/checkout）
- 主工作区有一批 **CRLF 行尾噪音 M**（`git status` 可见，`git diff --numstat` 为空）：**忽略，勿 add、勿还原**
- 等待必须用 `vTaskDelay` / `vTaskDelayUntil`（看门狗），**禁止忙等**
- 只准使用 §1 列出的公开 API，禁止 include 其他组件的内部头

## 1. 可用 API（已核实存在，验收按此检查）

```c
/* protocol（T3）—— 发送与注册 */
void protocol_send_raw(const char *data, int len);          /* TX 互斥保护，不追加 \r\n */
void protocol_register_status_cb(protocol_status_cb_t cb);  /* cb: void (*)(void) */

/* planner（T7）—— 指令位置（comm 任务单写者，32 位对齐读原子） */
int32_t planner_position_steps(void);
float   planner_position_mm(void);

/* as5600（T8）—— 实测位置与速度（portMUX 保护，跨任务/跨核安全） */
float   as5600_position_mm(void);
float   as5600_velocity_mm_s(void);
bool    as5600_is_ok(void);

/* step_engine（T6）—— 运动状态与指令速度 */
bool     step_engine_busy(void);            /* 队列非空或脉冲在途 */
uint32_t step_engine_velocity_steps(void);  /* 指令瞬时速度 steps/s */

/* tmc2209（T5）—— 驱动状态 */
bool tmc2209_read_status(tmc2209_status_t *st);   /* 解码 DRV_STATUS */
/* tmc2209_status_t 字段：raw, stst, ola, olb, s2ga, s2gb, otpw, ot, cs_actual */

/* settings（T11）—— ⚠ 订阅槽已满，只能用读取接口 */
int settings_report_hz(void);    /* $9，默认 10，合法域 [1, 50] */
```

## 2. 上游事实（实现前逐条核对）

| # | 事实 | 来源 |
|---|---|---|
| 1 | **settings 订阅槽已满 4/4**（tmc2209 / planner / app_main 的 $10 / as5600）→ **T9 不得调用 `settings_subscribe()`**，须在每次组帧时直接调 `settings_report_hz()` | PLAN v1.15 T8 节偏差入档⑤ |
| 2 | **状态帧格式**：`<STATE\|C:<mm>\|E:<mm>\|V:<mm/min>\|T:<旗标>>` + `\r\n`；**STATE ∈ `IDLE` / `RUN`（全大写）**；位置 3 位小数 | PLAN §4.5 |
| 3 | **T3 占位帧用的是 `<Idle\|…>`（混合大小写）——与 §4.5 不符**。已核查：代码中除 `protocol.c/h` 的注释外**无任何地方依赖 `Idle` 拼写**（自测、脚本均未断言）。T9 一律输出 **`IDLE`/`RUN`** | 本次核查 |
| 4 | **状态机是 T10 的活**（`sysctl`，BOOT→IDLE↔RUN）。T9 阶段 STATE 用 `step_engine_busy()` 推导：true → `RUN`，false → `IDLE`。T10 接管后替换来源 | PLAN §3.2 / T10 节 |
| 5 | **D12**：运动中按 report_hz 周期发；**静止时不主动发**，`?` 随时可查 | PLAN D12 / §4.5 |
| 6 | **`?` 回调在 comm_task 上下文执行**（protocol.c 中 `s_status_cb()` 由 RX 字节循环调用）→ 组帧函数必须**短小**（判据 <10ms），TMC 读（~1ms UART 事务）可接受 | protocol.c |
| 7 | `?` 是**实时字符**：行中间到达也立即处理，`status_cb` 内直接发送即可（TX 互斥保护） | protocol.h |
| 8 | `tmc2209_read_status()` 通过 UART2 与 TMC 通信（与协议 UART1 物理隔离），内部有互斥锁 | tmc2209.h |
| 9 | 任务划分：`monitor_task` **优先级 2**（低于 comm_task 5、planner 4、as5600_task 3） | PLAN §3.2 |
| 10 | as5600 位置已含 G92 偏移（D10 双向平移已实现）；planner 位置同理 → **C 与 E 天然可比** | T8 验收 §5 |

## 3. 任务边界

本任务交付"上报层"：把已有的位置/速度/状态数据**格式化成状态帧发出去**。

**不做**：失步判断、故障决策、停机逻辑（D6 明确排除）；状态机（T10）；`!` 处理（T6/T10）。

## 4. 组件 API（定稿，验收按此检查）

```c
/* components/monitor/include/monitor.h */

/* ---- 纯函数（硬件无关，自测目标） ---- */

/**
 * @brief 把 DRV_STATUS 解码结果映射为旗标字符串。
 *
 * 单一最严重旗标（优先级从高到低），输出如 "OK" / "OT" / "S2GA" / "OLB" / "OTPW"。
 * 读取失败（st == NULL）时输出 "ERR"。
 *
 * @param st   DRV_STATUS 解码结果；NULL 表示读取失败
 * @param buf  输出缓冲
 * @param len  缓冲长度（至少 8 字节）
 */
void monitor_tmc_flag(const tmc2209_status_t *st, char *buf, size_t len);

/**
 * @brief 组一帧状态文本（不含 \r\n）。
 *
 * 格式：<STATE|C:x.xxx|E:x.xxx|V:x.xxx|T:FLAG>
 *
 * @param buf       输出缓冲
 * @param len       缓冲长度（至少 64 字节）
 * @param state     "IDLE" 或 "RUN"
 * @param c_mm      指令位置 mm
 * @param e_mm      编码器位置 mm
 * @param v_mm_min  速度 mm/min
 * @param flag      旗标串（monitor_tmc_flag 的输出）
 * @return 写入的字节数（不含结尾 NUL）
 */
int monitor_format_frame(char *buf, size_t len, const char *state,
                         float c_mm, float e_mm, float v_mm_min,
                         const char *flag);

/* ---- 运行时 ---- */

/**
 * @brief 初始化：注册 status_cb + 创建 monitor_task。幂等。
 *
 * 必须在 protocol_start()、step_engine_init()、planner_init()、
 * as5600_init() 之后调用（本组件读它们的状态）。
 */
esp_err_t monitor_init(void);

/**
 * @brief 立即组一帧并发送（`?` 查询路径，也被 monitor_task 复用）。
 */
void monitor_send_status(void);

/**
 * @brief 已发送的周期帧计数（自测/调试用）。
 */
uint32_t monitor_frames_sent(void);

/* ---- 自测（仅 CONFIG_MONITOR_SELF_TEST=y） ---- */
#ifdef CONFIG_MONITOR_SELF_TEST
int monitor_self_test_run(void);   /* 末行 "SELF-TEST DONE: <n>/<total> PASS" */
#endif
```

## 5. 实现规格

### 5.1 状态帧格式（严格按 PLAN §4.5）

```
<IDLE|C:0.000|E:0.000|V:0.000|T:OK>\r\n
<RUN|C:120.000|E:119.971|V:3000.000|T:OK>\r\n
```

- `STATE`：**全大写** `IDLE` / `RUN`（见 §2 事实 3）
- `C`/`E`：`%.3f`（3 位小数，PLAN 规定）
- `V`：`%.3f`（统一 3 位，便于上位机统一解析）
- `T`：单一旗标串
- 结尾 `\r\n`（§4.2 定案）

### 5.2 TMC 旗标映射（优先级从高到低）

| 优先级 | 条件 | 输出 |
|---|---|---|
| 1 | `st == NULL`（UART 读失败） | `ERR` |
| 2 | `ot`（过温关断） | `OT` |
| 3 | `s2ga` | `S2GA` |
| 4 | `s2gb` | `S2GB` |
| 5 | `ola` | `OLA` |
| 6 | `olb` | `OLB` |
| 7 | `otpw`（过温预警） | `OTPW` |
| 8 | 以上均无 | `OK` |

- **单一旗标**（取最严重），避免帧过长；多故障并存时其余旗标不上报（PLAN §4.5 举例即为单旗标）
- `stst`（standstill）是正常运行态、非故障，**不进旗标**；`cs_actual` 也不进帧（帧内无对应字段）

### 5.3 C / E / V 数据来源

| 字段 | 来源 | 说明 |
|---|---|---|
| `C` | `planner_position_mm()` | 指令位置（逻辑账） |
| `E` | `as5600_position_mm()` | 编码器实测（已含 G92 偏移，与 C 可比） |
| `V` | `as5600_velocity_mm_s() × 60.0f` | **实测速度**转 mm/min（PLAN §3.1 数据流指定 monitor 从 as5600 取位置与速度） |
| `T` | `tmc2209_read_status()` → `monitor_tmc_flag()` | 每帧实时读取 |

- 三个位置/速度读取各自自带同步保护（planner 32 位原子读、as5600 portMUX），**无需再套一层锁**
- `V` **带符号**（反向运动为负），与 E/C 的方向语义一致；不取绝对值（D6：原样上报）
- `as5600_is_ok()` 为 false 时（I2C 长期失败），E 仍按 `as5600_position_mm()` 输出最后值（**不判断、不改写**，符合 D6）；如需标识可另议

### 5.4 monitor_task 与周期上报

```
monitor_task（优先级 2，栈 ≥3072，CPU0 或 CPU1 均可——无强实时要求）
每 10 ms tick 一次（vTaskDelayUntil）：
    if (step_engine_busy()) 累积 tick；
        累积到 100 / report_hz 个 tick（= 1000/report_hz ms）→ monitor_send_status() + 清零
    else 清零累积（静止不发，D12）
```

- `report_hz` 合法域 `[1, 50]`（settings 校验保证），故周期 ∈ `[20, 1000] ms`
- 10ms tick → `report_hz=50` 时每 2 tick 发一帧 ✓；`report_hz=10` 时每 10 tick ✓
- `report_hz` **每帧实时读取**（不得缓存、不得订阅，见 §2 事实 1）
- 静止→运动的切换由 busy 边沿自然处理，无需状态机
- **`monitor_frames_sent()` 只在周期路径自增**（单一写者 = monitor_task），`?` 按需帧不计入——这样既免去原子保护，也让用例 16 的计数语义精确
- 组帧缓冲用**栈上局部数组**（≥64 字节），`monitor_send_status()` 被 comm_task 与 monitor_task 并发调用时无共享状态；`protocol_send_raw()` 自带 TX 互斥 ✓

### 5.5 `?` 查询路径

`monitor_init()` 里 `protocol_register_status_cb(monitor_send_status_wrapper)`，其中 wrapper 直接调 `monitor_send_status()`。

- 执行上下文为 **comm_task**（§2 事实 6），函数必须短小：格式化 + 一次 TMC 读（~1ms）+ 一次 `protocol_send_raw()`
- **不阻塞**：不得在 status_cb 里等待、延时或获取会与 RX 循环互锁的锁
- 静止时 `?` 响应 <10ms（判据）

## 6. app_main 接线（只动指定锚点）

1. `#include "monitor.h"`
2. 在**所有依赖初始化完成之后**、`xTaskCreate(placeholder_task...)` 之前（即现有 `#ifdef CONFIG_APP_M3_TEST` 块附近）：
   ```c
   /* ---- T9 telemetry (needs planner + as5600 + step_engine + tmc2209) ---- */
   esp_err_t mon_err = monitor_init();
   if (mon_err != ESP_OK) {
       ESP_LOGE(TAG, "Monitor init failed: %s", esp_err_to_name(mon_err));
   }
   ```
   > 位置必须在 `tmc2209_init()` 与 `planner_init()` 之后——否则首帧会读不到 TMC 或拿到错误的位置基准。
3. 日志镜像 `#if` 条件追加 `|| defined(CONFIG_MONITOR_SELF_TEST)`（**两处**：钩子定义块、install 调用块）
4. 自测调用——**⚠ 必须放在 §6 第 2 点的 `monitor_init()` 之后（即 `#ifdef CONFIG_APP_M3_TEST` 块附近的"晚自测"位置），不能放进常规自测段**（`bsp_init()` 之后那一段）。原因：自测要读 planner / tmc2209 / step_engine 的状态，而这三个组件在常规自测段**尚未初始化**（`planner_init`/`tmc2209_init` 都在 `protocol_start()` 之后）。as5600 例外（已在常规自测段之前初始化）。
   ```c
   #ifdef CONFIG_MONITOR_SELF_TEST
       monitor_self_test_run();
   #endif
   ```

## 7. Kconfig / CMake

**`components/monitor/Kconfig`**（新建）：

```kconfig
menu "Monitor / Telemetry"

config MONITOR_SELF_TEST
    bool "Run monitor telemetry self-test on boot"
    default n
    help
        Runs the telemetry self-test: TMC flag-mapping and frame
        formatting cases (pure logic) plus live '?'-frame and
        periodic-reporting checks.
        Final line: "SELF-TEST DONE: <n>/<total> PASS".

endmenu
```

**`components/monitor/CMakeLists.txt`**（骨架 `SRCS ""` → 实装）：

```cmake
idf_component_register(SRCS "monitor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES protocol planner step_engine as5600 tmc2209 settings freertos)

if(CONFIG_MONITOR_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "monitor_self_test.c")
endif()
```

**`main/CMakeLists.txt`**：`REQUIRES` 追加 `monitor`（只加不改）。

## 8. 自测（`CONFIG_MONITOR_SELF_TEST=y`，上板自跑，≥16 例）

前置：`monitor_init()`（幂等）。位置/速度断言一律读对应组件 API 做交叉比对，不自造期望值。

**A 纯函数组（旗标映射 + 帧格式，10 例）**：

| # | 输入 | 期望 |
|---|---|---|
| 1 | `st == NULL` | `"ERR"` |
| 2 | 全 flag=0 | `"OK"` |
| 3 | `ot=1` | `"OT"` |
| 4 | `s2ga=1` | `"S2GA"` |
| 5 | `s2gb=1` | `"S2GB"` |
| 6 | `ola=1` | `"OLA"` |
| 7 | `olb=1` | `"OLB"` |
| 8 | `otpw=1` | `"OTPW"` |
| 9 | `otpw=1` + `s2ga=1` + `ot=1`（多故障） | `"OT"`（取最严重） |
| 10 | 帧格式：`("IDLE", 0, 0, 0, "OK")` | 逐字符严格等于 `<IDLE\|C:0.000\|E:0.000\|V:0.000\|T:OK>`，返回长度 == strlen |

**B 集成组（实机，6 例）**：

| # | 项 | 判据 |
|---|---|---|
| 11 | `monitor_init()` 幂等且返回 OK | ✓ |
| 12 | 静止时 `monitor_send_status()` 不崩溃、帧以 `<IDLE\|` 开头 | ✓ |
| 13 | 帧内 C 与 `planner_position_mm()` 一致（±0.001） | ✓ |
| 14 | 帧内 E 与 `as5600_position_mm()` 一致（±0.001） | ✓ |
| 15 | T 字段与 `tmc2209_read_status()` 当前值映射一致（正常硬件应为 `OK`） | ✓ |
| 16 | **周期上报**：入队一段较长运动（`step_engine_enqueue`，**不使能 TMC，轴不转**），等 ~1s，读 `monitor_frames_sent()` 增量 ≈ `settings_report_hz()`（±2） | ✓ |

> 用例 16 说明：自测阶段不使能 TMC（12V 未必接通、也不该让电机转），但 `step_engine_busy()` 会因脉冲队列为真 → 状态推导为 `RUN` → 周期帧照发，正好验证上报链路。段参数用低速度大步数（如 5000 步 @ 2000 steps/s ≈ 2.5s），确保 1s 采样窗口内持续 busy。测完 `step_engine_stop()` 收尾。

输出行：`PASS <case>` 逐例 + 末行 `SELF-TEST DONE: <n>/<total> PASS`。总时长 ≤10s。

## 9. 构建验证（两配置，全部零错误零警告）

```powershell
powershell -File idf_env.ps1 build                                  # ① 默认（自测关）
# sdkconfig 追加 CONFIG_MONITOR_SELF_TEST=y 构建                      # ② 自测开
```

- ① 的 ELF **不含** `MONITOR`/`SELF-TEST DONE` 自测串，但**必须含** `IDLE` 与状态帧格式串（组件主体始终编入）
- ② 的 ELF 含自测串

## 10. 上板操作（写给用户，你只写不执行）

1. PC USB 烧录：`powershell -File idf_env.ps1 -IdfAction "flash" -Port COM15`
2. CH343P 飞线（USB 插 PC）：`RX ← GPIO41`、`GND ↔ GND`；若要发 `?` 查询还需 `TX → GPIO42`
3. 12V 供电（电机运动需要；仅查询静态帧可不接）
4. 验证序列：
   ```
   ?                      ← 静止（IDLE）帧
   M17
   G92 X0
   G1 X120 F1200          ← 运动中应看到 ~10Hz 的 RUN 帧连续刷出
   ?                      ← 运动结束后 IDLE 帧
   G92 X0                 ← 日志给出编码器 raw 值
   ```

预期：
- 静止：偶尔一条 `<IDLE|C:0.000|E:0.000|V:0.000|T:OK>`
- 运动中：每 100ms 一条 `<RUN|C:x.xxx|E:x.xxx|V:xxxx.xxx|T:OK>`，C 递增、E 紧随（**这是 M4「C/E 一致」的首次实时可观测**）
- 结束后：位置回到 IDLE 帧

## 11. 验收判据（规划 Agent 按此验收）

| # | 项 | 判据 |
|---|---|---|
| 1 | 红线 | 仅 `components/monitor/` + `main/app_main.c` 锚点 + `main/CMakeLists.txt` 改动；**protocol.c/h 零改动**；无 git 操作 |
| 2 | D6 | 全组件**零决策逻辑**（无失步判断、无停机、无限位拦截） |
| 3 | 帧格式 | 严格 `§5.1`（全大写 STATE、`%.3f`、`\r\n`）；与 PLAN §4.5 一致 |
| 4 | 订阅约束 | **未调用 `settings_subscribe()`**；`report_hz` 每帧实时读取 |
| 5 | 自测 | A+B **16/16 PASS** |
| 6 | 构建 | 两配置零警告；① 无自测串、有主体串 |
| 7 | 上板 | 静止 `?` 正确；运动中周期帧频率 ≈ `$9`；C/E 数值与实际一致；`?` 响应 <10ms |
| 8 | 里程 | 与 T8 合并达成 **M4「位置可信」**（C/E 可实时对比 + 端到端运动验证） |

---

## 附 A：与 PLAN 的偏差预登记（验收时入档）

1. **STATE 拼写修正**：T3 占位帧为 `<Idle|…>`（混合大小写），本任务按 PLAN §4.5 改为 **`IDLE`/`RUN`**（全大写）。已核查代码层无依赖，但**上位机/脚本若按 `Idle` 解析需同步**（T12 脚本注意）
2. **STATE 来源临时为 `step_engine_busy()`**（真实状态机属 T10），T10 接管后应替换为 sysctl 状态
3. **旗标映射采用"单一最严重"策略**（PLAN 只给示例 `OK`/`OT`/`S2GA`，未规定多故障并存行为）；新增 `ERR`（TMC 读失败）与 `OTPW`、`OLA`/`OLB` 输出
4. **V 字段取自 as5600 实测速度**（PLAN §4.5 未指定来源；§3.1 数据流图指向 as5600，且与 E 同为实测量、自洽）
5. **`report_hz` 不订阅而是每帧读取**（settings 订阅槽已满 4/4）
6. 新增 `monitor_format_frame()` / `monitor_tmc_flag()` 纯函数与 `monitor_frames_sent()` 计数钩子（PLAN 未列，为可测性）
7. 「人为制造 TMC 过温/断相」的验收项 **改为纯函数映射表全覆盖**（物理上不可行且危险），实机仅验证 `T=OK` 正常路径
