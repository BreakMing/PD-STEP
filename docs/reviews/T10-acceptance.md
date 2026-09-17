# T10 验收报告 · 状态机与立即停（sysctl）

> 日期：2026-09-17 · 分支：master · 提交：待用户提交 · 验收人：规划 Agent
> 结论：**验收通过（代码级 + 两配置构建 + 上板自测 14/14 + `!` 端到端两条硬判据全过）**。状态机 `BOOT→IDLE↔RUN`、`!` 端到端贯通（含位置 rebase）、看门狗覆盖全部就位。**上板实测：`!` 打断后 C 冻结在实际停位 301.600mm（非被取消目标 1000mm），随后 `G1 X50` 平滑执行至 50.000mm 精确命中。** 位置 rebase 由执行模型设计得比任务包建议更完善（见 §5）。

## 0. 结果总览

| 阶段 | 结果 |
|---|---|
| 代码级验收 | 通过（零 P0；红线精确） |
| 构建验证 | 两配置零 error 零 warning |
| 上板自测 | **14/14 PASS**（12.7 s） |
| 上板 `!` 端到端 | **两条硬判据全过** |

## 1. 红线检查（✓ 通过）

| 约束 | 结果 |
|---|---|
| **protocol.c/h 零改动** | ✓ `git diff --numstat -- components/protocol` 为空 |
| 授权改动面（6 处） | ✓ 精确命中，无越界 |
| ├ `main/app_main.c` | +22/−7（锚点） |
| ├ `main/CMakeLists.txt` | +1/−1 |
| ├ planner | +79（仅新增 `planner_rebase_to_executed()` 及其锚点维护） |
| ├ monitor | +16/−2（STATE 来源 + 看门狗） |
| ├ as5600.c | +13（仅看门狗订阅与喂狗） |
| └ `sdkconfig.defaults` | +6（仅 `CONFIG_ESP_TASK_WDT_PANIC=y` + 注释） |
| **D6** | ✓ 全组件无失步判断 / 故障决策 / 限位拦截 |
| **D11** | ✓ 仅立即停，无暂停 / 恢复 / 续跑 |

其余工作区 `M` 文件为 CRLF 噪音（`numstat` 为空），不纳入提交。

## 2. 代码精读（核实清单）

| # | 项 | 结论 |
|---|---|---|
| 1 | 状态机取值域 | ✓ `BOOT`/`IDLE`/`RUN` 三态，无 HOLD/ALARM |
| 2 | 无独立任务 | ✓ `sysctl_get_state()` 按需由 `step_engine_busy()` 推导并缓存；`sysctl_init()` 置 IDLE |
| 3 | **`!` 顺序** | ✓ `① step_engine_stop() → ② planner_rebase_to_executed() → ③ state=IDLE`（顺序即正确性，注释中亦明确） |
| 4 | 状态字符串 | ✓ `sysctl_state_str()` 恒返回全大写三值之一 |
| 5 | monitor STATE 来源 | ✓ 改为 `sysctl_state_str()`；`REQUIRES` 追加 sysctl |
| 6 | 看门狗 | ✓ as5600_task / monitor_task 均订阅 + 逐周期喂狗；订阅失败仅 W 日志不 abort |
| 7 | `TASK_WDT_PANIC` | ✓ `sdkconfig.defaults` 已加 `CONFIG_ESP_TASK_WDT_PANIC=y`，并把"重启后位置基准归零、上位机须重新 G92"写进注释 |
| 8 | app_main 接线 | ✓ `sysctl_init()` 在 `monitor_init()` **之前**；`cb_on_stop()` 改调 `sysctl_stop()`；日志镜像两处 `#if` 已加 |
| 9 | 幂等 | ✓ `sysctl_init()` 幂等；`sysctl_stop()` 可重复调用（用例 12 验证） |
| 10 | 位置 rebase 推导 | ✓ 见 §5（含 `$10` 两种取值） |

## 3. 构建验证（验收人实测）

| 配置 | bin | ELF 自测串 | 编译 |
|---|---|---|---|
| ① SYSCTL=n | 0x489b0 | 无（主体串在） | 零 error 零 warning |
| ② SYSCTL=y（上板态） | 0x47e40 | 有 | 零 error 零 warning |

## 4. 上板实测

### 4.1 自测 14/14 PASS（12.7 s）

```
sysctl: state machine up: BOOT -> IDLE (derived, no task)
preconditions: $7=3000 $8=2000 $12=0 $13=600 $10=0 k=80.000
frame aligned: engine 0 steps (0.000 mm) declared as 0.000 mm
State machine:            PASS ×6
Immediate stop ('!') + position reference:   PASS ×8
SELF-TEST DONE: 14/14 PASS
```

**核心用例实测值**：

| 用例 | 日志摘录 | 结论 |
|---|---|---|
| 7 停止冻结 | `stopped at engine 2637 steps (32.963 mm), target was 275.000 mm` | ✓ 冻结且位置仍在 200ms 后不变 |
| **8 位置对账（§4.4）** | `commanded 32.963 mm vs executed 32.963 mm (engine 2637 steps)` | ✓ **逐位一致**——"指令位置停在已执行步数" |
| 9 rebase 生效 | `cancelled target 275.000 mm, frozen at 32.963 mm` | ✓ 未停在目标 |
| **10 `!` 后新命令** | `after '!': 32.963 mm -> 52.963 mm (1600 steps, expected 1600)` | ✓ **精确 1600 步**（T10 硬判据） |
| 11 G91 基准 | PASS | ✓ 相对基准 = rebase 后位置 |
| 12 重复 stop | PASS（位置与状态均不变） | ✓ 幂等 |
| 13 多段复用 | `3 segments after '!': engine 6000 steps (expected 6000)` | ✓ 队列可复用 |
| 14 状态一致性 | PASS | ✓ state 与 `busy()` 相符 |

> 自测期间 TMC 未使能（轴不动，`E` 恒 0.000），仅驱动脉冲队列——与任务包设计一致。周期帧同时持续刷出，亦是对 T9 遥测的回归验证。

### 4.2 `!` 端到端（真机、真电机）

```
[19:29:23.818] <RUN|C:299.062|E:298.936|V:3002.930|T:OK>       ← 停止前最后帧
[19:29:23.864] !
[19:29:23.877] planner: rebase to executed: 24128 steps (301.6000 mm)
[19:29:23.883] sysctl: '!' immediate stop: pulses stopped, queue cleared, command position rebased to executed
```

- `!` 之后**再无任何 RUN 帧** → 脉冲立即停 ✓
- **C 冻结在 301.600mm**，而非被取消的目标 1000mm ✓

```
?  →  <IDLE|C:301.600|E:301.680|V:0.000|T:OK>
```

- STATE 回 `IDLE` ✓；C = 实际停位 ✓；C−E = 0.08mm ✓

**关键判据——随后的新命令**：

```
G1 X50 F1200
→ C: 301.325 → 300.438 → … → 50.388    （平滑下降，无反向狂奔）

?  →  <IDLE|C:50.000|E:50.010|V:0.000|T:OK>
```

- **C = 50.000 精确命中** ✓ — `!` 打断后位置基准完全正确，系统可继续工作

**顺带确认**：
- 加速段 V 从 293 → 3076 mm/min 平滑爬升（约 0.83 s，符合 `A60`），巡航稳定 ~3000 ✓
- 全程 **C−E 差值 0.02~0.18mm 且无累积趋势**，终点 1 count — M4 成果在 T10 下依然成立 ✓

## 5. 设计亮点：执行模型改进了任务包的 rebase 方案

任务包建议的方案是"在 `enqueue` 且队列为空时记录队列起点快照"。执行模型改为维护一个 **执行↔指令参考对** `(s_ref_eng, s_ref_pos)`，并在 **三个时机**锚定：

| 锚定时机 | 任务包建议 | 执行模型实现 |
|---|---|---|
| `planner_init()`（上电原点，D10） | ✓ | ✓（并额外处理"init 跑在引擎已被驱动之后"的情形） |
| `enqueue` 且队列为空 | ✓ | ✓（且在 enqueue **之前**采样，注释说明原因） |
| **`planner_set_origin()`（G92）** | ✗ **遗漏** | ✓ **补上** |

**遗漏这一条会造成真实错误**：

```
轴停在 40mm → G92 X0（声明当前位置为 0）→ 立即发 !
  任务包方案：用旧锚点(0,0) rebase → 位置回退到 40mm   ✗ 错（应为 0）
  执行模型：  G92 时已重锚(3200 steps, 0) → rebase 得 0mm  ✓ 对
```

代码注释中明确写出了该理由：*"Without this, a `!` arriving while the axis is standing still would rebase the commanded position back to the pre-G92 frame."* —— 说明是**推导后**的设计，而非巧合。

**该方案的价值**：把 `!` 的 rebase 化简为"冻结计数器经参考对映射"，同时天然保住 G92 偏移。任务包已尽到"给出可选方案并允许替换"的说明义务（§5.3），执行模型的替换方案**覆盖了原方案的全部情形并修复了边界缺陷**，予以采纳。

## 6. 自测实现的额外亮点

- `align_frame()`：自测开始先用 G92 声明当前坐标，使用例 8 的"commanded == executed"断言**字面成立**（且对"引擎位置落在软限位窗口外"的情形做了兜底）
- `expected_cmd_mm()`：**独立推导**期望值（自身处理 `$10` 符号），与 planner 的 rebase 实现不同源 —— 避免同义反复
- 用例 10/11/13 均按 `settings_invert_dir()` 调整期望符号，为 `$10=1` 场景预留了正确性

## 7. M5 进展与遗留

| M5 内容 | 状态 |
|---|---|
| T10 状态机 + 立即停 | ✅ 本任务闭环 |
| T12 验收脚本 | ⏳ 待做（M5 最后一个任务） |

**遗留与观察项**：

1. **`placeholder_task` 仍在**（T1 遗留的占位任务，带喂狗）—— 移除不在本任务范围；T12 阶段可评估是否清理
2. **panic 重启后位置基准归零** —— 已定案接受（"失控时优先停下来"），需上位机重新 `G92`。T12 脚本若遇到意外重启应能识别
3. **STATE 来源已统一为 sysctl** —— T9 的移交项关闭
4. **`is_ok` / `cs_actual` 等未进帧**（T9 遗留，不变）

## 8. 提交（沙箱限制，用户本地执行）

```powershell
git add components/sysctl components/as5600/as5600.c components/planner components/monitor main/app_main.c main/CMakeLists.txt sdkconfig.defaults
git commit -m 'T10: sysctl state machine (BOOT/IDLE/RUN, no task) + ! end-to-end stop (protocol -> sysctl -> engine stop + position rebase + IDLE); planner_rebase_to_executed() keeps the commanded position at the executed steps after ! (PLAN 4.4, $10-aware, anchored at init/enqueue-from-idle/G92); monitor STATE now from sysctl; task-WDT subscribe+kick for as5600/monitor; CONFIG_ESP_TASK_WDT_PANIC=y; 14-case on-board self-test PASS'

git add docs/reviews/T10-brief.md docs/reviews/T10-acceptance.md PLAN.md
git commit -m 'docs: T10 brief + acceptance (14/14 PASS, ! stops with C frozen at executed position 301.600mm, next move to 50.000mm exact) + PLAN v1.17 - T10 closed'
```

> 注：`bsp/`、`settings/`、`docs/reviews/T4*`、`T11*`、`idf_env.ps1`、`tools/` 为 CRLF 噪音，**不纳入**提交。
