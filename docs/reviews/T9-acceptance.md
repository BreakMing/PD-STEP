# T9 验收报告 · 遥测组包（monitor_task）

> 日期：2026-09-17 · 分支：master · 提交：待用户提交 · 验收人：规划 Agent
> 结论：**验收通过（代码级 + 两配置构建 + 上板自测 16/16 + 运动中的 C/E 一致性验证）**。状态帧格式、旗标映射、周期上报、`?` 即时查询全部上板验证。上板过程暴露 **1 处 P1 设计语义偏差**（C 报的是"目标位置"而非"执行位置"，导致运动中 C/E 对比无失步信息），以方案 A 修正后复测：**运动中 C−E 恒定在 0.07~0.12mm 无累积、终点差值 1 count**。**与 T8 合并达成 M4 里程碑「位置可信」，并同时结掉 T8 遗留的 3 圈判据。**

## 0. 上板过程总览

| 轮次 | 结果 | 暴露问题 | 定级 | 处置 |
|---|---|---|---|---|
| 代码级验收 | 通过（零 P0） | — | — | — |
| 上板①（自测 + 静止帧） | **16/16 PASS**；`<IDLE\|C:0.000\|E:0.010\|V:36.621\|T:OK>` | — | — | — |
| 上板②（`G1 X120` 运动） | 发现 **C 恒定在 120.000**，E 从 0 追赶 | **C 语义偏差**（目标位置 vs 执行位置） | **P1（设计语义）** | 见 §5 |
| 上板③（修正后复测） | C 随 E 同步增长；C−E 恒定；终点 1 count | — | — | **闭环** |

**复盘价值**：该问题不是 bug，而是**规格理解偏差**——PLAN T9 节要求"指令位置取原子快照（避免读到 ISR 更新中的值）"，明确指向 `step_engine_position_steps()`（脉冲 ISR 累加）；而首版实现用了 planner 的 `pos_steps`（comm_task 在入队成功时一次性跳到目标值）。两者都叫"指令位置"，但**只有前者能让 C−E 在运动全程等于实时失步量**。该偏差在静止帧与自测用例中**完全看不出来**（静止时两者相等），只有真实运动才能暴露——又一个"唯上板可见"的实例。

## 1. 红线检查（✓ 通过）

| 约束 | 结果 |
|---|---|
| **protocol.c/h 零改动** | ✓ `git diff --numstat -- components/protocol` 为空；状态帧经 `protocol_send_raw()` 发送、经 `protocol_register_status_cb()` 注册，T3 占位逻辑仅被覆盖 |
| **D6 纯上报零判断** | ✓ 全组件无失步判断 / 故障停机 / 限位拦截 |
| **未订阅 settings** | ✓ 零 `settings_subscribe()`；`$9` 每帧实时读取（订阅槽已满 4/4） |
| 改动面 | ✓ `components/monitor/`（骨架 + 4 新文件）+ `main/app_main.c`(+16/-2) + `main/CMakeLists.txt`(+1/-1)；其余组件零改动 |

## 2. 代码精读（核实清单）

| # | 项 | 结论 |
|---|---|---|
| 1 | 帧格式 | ✓ `<%s\|C:%.3f\|E:%.3f\|V:%.3f\|T:%s>` + `\r\n`，严格符合 PLAN §4.5 |
| 2 | STATE 大写 | ✓ `RUN`/`IDLE` 全大写（**修正了 T3 占位的 `Idle`**；已核查代码层无依赖） |
| 3 | 旗标映射 | ✓ 单一最严重：ERR > OT > S2GA > S2GB > OLA > OLB > OTPW > OK；`stst`/`cs_actual` 不进帧 |
| 4 | 周期上报 | ✓ 10ms tick + `1000/(hz×10)` 计算；hz=10 → 10 tick = 100ms |
| 5 | 静止不发 | ✓ busy 为假时清计数并 continue（D12） |
| 6 | `?` 路径 | ✓ 注册 `monitor_send_status`；函数短小（格式化 + 一次 TMC 读 ~1ms + 一次 TX） |
| 7 | 线程安全 | ✓ 组帧用栈上缓冲；`s_frames_sent` 单一写者（仅周期路径）；ref 用 portMUX 保护 |
| 8 | V 来源 | ✓ `as5600_velocity_mm_s() × 60`，带符号 |
| 9 | T 来源 | ✓ 每帧实时 `tmc2209_read_status()` |
| 10 | 任务参数 | ✓ 优先级 2、栈 3072、CPU1（与 as5600 同核，CPU0 留给脉冲 ISR） |
| 11 | 接线位置 | ✓ `monitor_init()` 在 tmc2209/planner/as5600/step_engine 全部初始化之后；**自测放在"晚自测"位置**（常规自测段时 planner/tmc2209 尚未初始化） |
| 12 | 幂等 | ✓ `s_initialized` 守卫 |

**自测实现亮点**（超出任务包要求）：
- 用例 13/14 用 `within_readings()` 区间容忍——as5600 是 1kHz 独立采样，帧构建与比较读取之间可能差一个 count，硬比较会偶发假失败
- 用例 15 对 TMC UART 读**重试 3 次**，避免单次通信抖动误判
- 用例 16 先断言 `!step_engine_busy()` 确保环境干净，收尾 `step_engine_stop()` 不留残余脉冲

## 3. 构建验证（验收人实测）

| 配置 | bin | ELF 自测串 | 编译 |
|---|---|---|---|
| ① MONITOR=n | 0x47380 | **无** ✓（主体串在） | 零 error 零 warning |
| ② MONITOR=y | 0x482b0 | 有 ✓ | 零 error 零 warning |
| ②′ 修正后重建 | 0x48580 | 有 ✓ | 零 error 零 warning |

> 期间遇到一次 **`esp_lcd_panel_rgb.c` 编译器内部错误（ICE, Segmentation fault）**——IDF 内置组件的偶发编译崩溃（并发内存压力），与本任务代码无关；直接重试即通过。

## 4. 上板实测

### 4.1 自测 16/16 PASS

```
PASS test_flag_null ... test_flag_priority   (9 例：旗标映射全覆盖 + 优先级)
PASS test_frame_format                        (逐字符 + 负数格式)
PASS test_init_idempotent / test_idle_frame / test_frame_c / test_frame_e / test_frame_t
  static frame: <IDLE|C:0.000|E:0.010|V:36.621|T:OK>
  C frame=0.000, engine steps=[0,0] → [0.000,0.000] mm
  E frame=0.010 as5600=[0.010,0.010]
  T=OK (matches live DRV_STATUS)
PASS test_periodic_reporting
  periodic: 10 frames in 1 s, $9=10 Hz
SELF-TEST DONE: 16/16 PASS
```

### 4.2 运动中的 C/E 一致性（M4 核心判据）

修正后 `G1 X120 F1200` 全程采样：

| C (mm) | E (mm) | **C − E** |
|---|---|---|
| 1.587 | 1.484 | +0.103 |
| 11.587 | 11.504 | +0.083 |
| 41.588 | 41.484 | +0.104 |
| 61.588 | 61.465 | +0.123 |
| 81.588 | 81.494 | +0.094 |
| 101.588 | 101.465 | +0.123 |
| 119.588 | 119.521 | +0.067 |

- **差值锁在 +0.07 ~ +0.12mm，全程无累积趋势** → **无丢步**（若丢步，C 增而 E 不跟，差值会单调放大）
- 0.1mm 为**运动中的系统性滞后**：20mm/s 下约 5ms 的电磁+机械响应延迟，叠加联轴器/同步带的弹性变形
- V 全程 1171.9 / 1208.5 / 1245.1 mm/min（均值 ≈1208，目标 F1200=1200）→ 速度准确，波动来自 16ms 窗口的量化（±1 count ≈ ±36.6 mm/min）

### 4.3 终点到位（M4 到位判据）

```
?  →  <IDLE|C:120.000|E:120.010|V:0.000|T:OK>
```

| 指标 | 值 |
|---|---|
| C（指令，执行位置） | 120.000 mm |
| E（编码器实测） | **120.010 mm** |
| **C − E** | **−0.010 mm = 1 count** |
| 相对 120mm 行程的偏差 | **0.008%** |

运动中的 0.1mm 滞后在静止后收敛到 1 count——**弹性回弹的实证**，也印证 §4.2 对 0.1mm 成因的判断。

## 5. 验收期修复：C 语义修正（P1）

### 5.1 现象

首版上板时 `G1 X120` 全程帧内 `C:120.000` **恒定不变**，E 从 1.24 缓慢增长到 119.28。C−E 在运动中等于"剩余距离"（可达百余 mm），**不含任何失步信息**。

### 5.2 根因

数据源选错：

| 计数 | 更新者 | 更新方式 | 语义 |
|---|---|---|---|
| planner `pos_steps`（首版用的） | comm_task | **入队成功即一次性跳到目标值** | 目标位置 |
| `step_engine_position_steps()`（应为） | **脉冲 ISR** | 每发一个脉冲 +1 | 执行位置 |

PLAN T9 节原文"指令位置取**原子快照**（关中断或 seqlock，**避免读到 ISR 更新中的值**）"——明确指向后者。而 D6 要求 AS5600 纯遥测、**由上位机判断失步**，上位机要能判断，C 必须是与 E 可比的**实时位置**。

该偏差在静止帧与自测中不可见（静止时两值相等），仅真实运动可暴露。

### 5.3 修复（方案 A，4 文件）

```c
/* monitor.c —— C 改为执行位置 + G92 参考基准 */
float k = settings_steps_per_mm();
int32_t exec_steps = step_engine_position_steps();
float c_mm = (float)(exec_steps - s_ref_steps) / k + s_ref_mm;
```
- 新增 `monitor_note_origin(new_pos_mm)`：记录 G92 时刻的引擎位置与设定坐标（`portMUX` 保护）
- `monitor_init()` 内以 0mm 对齐（D10 上电位置为 0）
- `app_main` 的 G92 回调增加一行调用 → **与 planner / as5600 三方同步平移，满足 D10"C 与 E 同时平移"**
- **未触碰 planner**（守住红线）

自测同步加固：
- 用例 13 改为验证"帧内 C == 引擎位置 ÷ k"
- **用例 16 新增断言"运动中 C 必须增长"** —— 防回归护栏，若将来有人改回目标位置，该用例直接 FAIL

### 5.4 验证（✓）

§4.2 与 §4.3 的实测即为验证：C 从 1.587 平滑增长至 120.000，C−E 全程恒定且终点收敛至 1 count。

## 6. M4 里程碑达成

| M4 判据 | 结果 |
|---|---|
| 编码器全程校验运动到位（C/E 一致） | ✅ 全程 C−E 恒定无累积；终点差 1 count（0.008%） |
| **T8 遗留：手动/电机转 3 圈 = 120mm ±0.05mm** | ✅ 本次以电机带动 3 圈验证：**120.010mm，偏差 0.010mm**，远优于判据 |
| T8 遗留：3 圈整段判据待 `?` 一键验证 | ✅ 本任务交付后以 `?` 直接读取完成 |

**M4「位置可信」达成。** 数据链路 `planner → step_engine → TMC2209 → 电机 → AS5600 → monitor → 上位机` 全线贯通，且 C/E 可在运动中实时对拍。

## 7. 遗留与观察项

1. **静止时 V 有 ±36.6 mm/min 抖动**（`0.000` 与 `±36.621` 交替）——编码器 ±1 count 量化噪声经 16ms 窗口的等效值。按 D6"原样上报"正确，但上位机可能觉得静止速度在跳；若要改善可在 T12 评估加显示死区（属显示层决策，不改固件语义）
2. **`ST2GA`/`ST2GB` 等 StallGuard 信息未进帧**——PLAN §4.5 未要求，属二期
3. **`cs_actual` 未上报**——同上；若上位机需要判断实际电流档位可扩展帧格式（需 PLAN 决策）
4. **STATE 来源仍为 `step_engine_busy()`**——T10 交付状态机后应替换为 sysctl 状态

## 8. 提交（沙箱限制，用户本地执行）

```powershell
git add components/monitor main/app_main.c main/CMakeLists.txt
git commit -m 'T9: telemetry - status frame packing (IDLE/RUN, C/E/V/T) with TMC flag map, 10 Hz periodic reporting while moving, on-demand ? query; C is the EXECUTED position (engine ISR counter + G92 reference) so C-E is the live loss-of-step amount; acceptance fix: C source corrected from planner target to engine executed position; 16-case on-board self-test PASS'

git add docs/reviews/T9-brief.md docs/reviews/T9-acceptance.md PLAN.md
git commit -m 'docs: T9 brief + acceptance (16/16 PASS, C-E constant within 0.1mm in motion, 1-count at rest) + PLAN v1.16 - T9 closed, M4 achieved'
```

> 注：`bsp/`、`settings/`、`docs/reviews/T4*`、`T11*`、`idf_env.ps1`、`tools/` 为 CRLF 噪音，**不纳入**提交。
