# T7 验收报告 · 运动规划器（planner）

> 日期：2026-09-16 · 分支：master（主工作区直接开发）· 提交：待用户提交 · 验收人：规划 Agent
> 结论：**验收通过（代码级 + 上板自测 gcode 33/33 + planner 21/21）**。代码级验收阶段零 P0，接口/算法/接线/红线逐条对齐任务包；上板首轮 gcode 32/33 暴露 1 处**用例缺陷**（test 32 `s_move_return` 与 `rec_clear()` 顺序颠倒，非解析器缺陷），一行修复后复测 33/33。planner 核心（Q16.16 钳制链、梯形/三角求解、软限位、阻塞背压 + `!` 竞态、$0 重派生 k）一次上板全过。**任务闭环。**

## 0. 上板问题总览

| 轮次 | 结果 | 暴露问题 | 定级 | 根因 | 修复 |
|---|---|---|---|---|---|
| 1 | gcode 32/33 + planner 21/21 | `test_rejected_move_keeps_relative_base` expected 2 got 0 | 用例缺陷 | C `rec_clear()` 会重置 `s_move_return=0`，用例先 `s_move_return=2` 再 `rec_clear()`，顺序颠倒把 2 清回 0 | 交换两行（`rec_clear(); s_move_return=2;`） |
| 2 | gcode 33/33 + planner 21/21 | — | — | — | **闭环** |

**根因本质**：C 测试桩与 Python 参考模型对 `clear()` 的语义不一致——C `rec_clear()` 清 records **并重置** `s_move_return/s_set_param_return/s_restore_return`，Python `clear()` 只清 events **不动** `move_return`。执行模型写 C 用例 32 时按 Python 语义写了 `s_move_return=2; rec_clear();`。同组 test 31/33 均为正确顺序 `rec_clear(); s_move_return=2;`，故仅 test 32 挂。生产代码（gcode_parser.c）零缺陷。

---

## 1. 红线检查（✓ 通过）

| 文件 | 状态 | 归属 |
|---|---|---|
| components/planner/{planner.c, planner_self_test.c, Kconfig, include/planner.h, CMakeLists.txt} | 新增（CMakeLists.txt 为骨架占位→实装） | 本组件 ✓ |
| components/gcode_parser/{gcode_parser.c, gcode_parser_self_test.c, include/gcode_parser.h, test/test_gcode_parser_verify.py} | 修改 | §5 允许锚点 ✓ |
| main/{app_main.c, CMakeLists.txt} | 修改 | §6/§7 允许锚点 ✓ |

- `protocol/`、`step_engine/`、`settings/`、`tmc2209/`、`bsp/`、`main/app_joint_test.c` **零内容改动** ✓（settings/bsp 在工作区仅换行符差异，`git diff --numstat` 为空，不纳入提交）
- planner 组件 `REQUIRES` 仅 step_engine/settings/freertos，无 tmc2209/protocol/bsp 依赖 ✓
- 无浮点入 ISR（planner 全任务上下文）✓

## 2. app_main 接线（✓ 与任务包 §6 一致）

1. `#include "planner.h"` ✓
2. `step_engine_init` 成功后、parser 创建前：`planner_init()`（幂等）✓
3. `cb_on_move → planner_enqueue`（void→int，返回 status 供 parser 回 `error:2`）✓
4. `cb_on_set_origin → planner_set_origin`；`on_set_absolute/relative` 保持 no-op（parser 内部已换算 G90/G91）✓
5. **$10 孤儿接口补线**：`step_engine_set_dir_invert(settings_invert_dir())` 播种 + `settings_subscribe(cb_on_settings_change)` 跟踪运行时变更 ✓
6. 自测块位于 step_engine 自测之后、tmc2209_init 之前；日志镜像 `#if` 追加 `CONFIG_PLANNER_SELF_TEST` ✓

## 3. 代码精读（核心清单）

| # | 项 | 结论 |
|---|---|---|
| 1 | 管线顺序 | ✓ 软限位 → $7/$8 mm 域钳制 → mm→steps(lroundf) → 求解 → 阻塞入队 → 位置提交 |
| 2 | Q16.16 钳制链 | ✓ v/a 上限钳 `65535.0f`（`float_to_q16` = `(uint32)(v*65536+0.5f)`，65536 会回绕）；下限 `1e-3` 防 `v_q16=0` ISR 除零 / `a_q16=0` 永不加速 |
| 3 | 梯形/三角判别 | ✓ `d=v²/(2a)`；`2d≤N` 梯形（accel=decel=floor(d)）、`2d>N` 三角（odd N 差 1 步给减速）、N=1 兜底 v_cruise>0 |
| 4 | 三段和不变量 | ✓ `accel+cruise+decel==steps` 求解内自检（T6 P0#3 教训）；求解失败防御返回 error:2 |
| 5 | 阻塞背压 + `!` 竞态 | ✓ enqueue 失败→查 `busy()` 假→判 '!' 清队列（丢段、不提交位置、回 ok）；否则 vTaskDelay(1ms) 重试；论证注释与任务包 §4.3 一致 |
| 6 | 位置提交时机 | ✓ delta==0 也提交（G92 式重同步）；`planner_set_origin` 不查软限位（只约束移动目标） |
| 7 | 单位换算 | ✓ `k = settings_steps_per_mm()`，订阅 $0/$1/$2 变更 refresh；$10 反相在引擎输出层，planner 只管逻辑方向 |
| 8 | parser 位置提交缺陷修复 | ✓ `parse_g0_g1` 回调先行、`ret==2→error:2`、接受后才 `current_pos_mm=target`；模态 F/A 在回调前更新（GRBL 语义） |

## 4. 用例缺陷修复（test 32 顺序颠倒）

**缺陷**（gcode_parser_self_test.c test 32）：

```c
s_move_return = 2;      /* 先设 2 */
rec_clear();            /* rec_clear() 又把它清回 0 */
gcode_parser_feed(p, "G1 X50");   /* 桩返回 0 → ok(0)，断言 got 0 */
```

**修复**：交换两行。上板复测 test 32 PASS（33/33）。

## 5. 构建验证（✓ 验收人实测）

| 配置 | 结果 |
|---|---|
| 默认（CONFIG_PLANNER_SELF_TEST=y，sdkconfig 已就绪） | `Project build complete` 零告警零错误，bin 0x46c50 |
| 双自测（追加 CONFIG_GCODE_PARSER_SELF_TEST=y） | 零告警零错误，bin 0x4acd0 |
| Python 参考模型 | **33/33 PASS**（`test_gcode_parser_verify.py`） |

## 6. 上板自测（2026-09-16，COM15，PC USB 供电 + UART0，TMC 失能）

```
gcode_selftest:   SELF-TEST DONE: 33/33 PASS
planner_selftest: SELF-TEST DONE: 21/21 PASS
```

关键上板证据（planner 21 例）：
- 梯形/三角/临界/钳制求解 9 例全过（含 `test_solve_a_clamp`：$8×80=160000 → 65535，**P3 端到端兑现**）
- 步数精确：+1234 / −567 / 连续 +1234−567+891=1558，引擎位置增量逐 bit 精确
- 亚步取整：0.02mm→2 步、0.001mm→0 步（lroundf）
- F 钳制：9000→$7 3000→4000 steps/s；A 钳制：9000→$8 2000→160000→65535
- 软限位：601mm→error:2 且无动作；600/0 边界含入
- $0 重派生 k：$0=8→k=40、恢复 16→k=80（订阅生效）
- 队列背压：17×64=1088 步，第 17 段阻塞等待不丢步

## 7. 提交（沙箱限制，用户本地执行）

代码提交（master，排除换行符噪声文件）：

```powershell
git add components/planner components/gcode_parser/gcode_parser.c components/gcode_parser/gcode_parser_self_test.c components/gcode_parser/include/gcode_parser.h components/gcode_parser/test/test_gcode_parser_verify.py main/app_main.c main/CMakeLists.txt
git commit -m "T7: motion planner - mm->steps + soft-limit + trapezoid/triangle solver, Q16.16 accel clamp (P3: \$8*80=160000->65535), blocking enqueue with '!' race handling; gcode_parser on_move void->int + commit-position-after-accept (soft-limit fix); wire \$10 dir-invert; 21-case planner self-test + 33-case gcode self-test"
```

文档提交：

```powershell
git add docs/reviews/T7-acceptance.md PLAN.md
git commit -m "docs: T7 acceptance report + PLAN v1.13 - planner 21/21 + gcode 33/33 on-board; one test-harness order bug fixed"
```

> 注：settings/、bsp/、docs/reviews/T4*、T11*、idf_env.ps1、tools/t3_stress_test.py 为换行符差异，**不纳入**提交。
