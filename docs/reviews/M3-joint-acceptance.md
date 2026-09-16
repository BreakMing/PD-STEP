# M3 验收报告 · 联合测试（真 G-code 驱动真电机）

> 日期：2026-09-16 · 分支：master · 提交：待用户提交 · 验收人：规划 Agent
> 结论：**验收通过（代码级 + 双配置构建 + 上板一次全过 10/10）**。真 G-code 文本经 `parser → planner → step_engine → TMC2209 → 电机` 全链路驱动，步数逐 bit 精确（+3200 / +400 / +2），软限位 `error:2`、`G92`、亚步取整、相对模式全部符合。**PLAN §8 M3 判据"一条 G1 精确步数执行"达成 → 里程碑 ✅。**

## 0. 结果总览

| 阶段 | 结果 | 备注 |
|---|---|---|
| 代码级验收 | 通过 | 红线 4 处改动精确命中；10 条用例与任务包逐条对齐；无 P0/P1 |
| 双配置构建 | 通过 | on 含 `M3 TEST`、off 无残留；均零警告 |
| 上板首轮 | **10/10 PASS** | 本任务首轮即全过（对比 T5/T6/T7 的多轮修复） |

**首轮全过原因**：M3 验证的链路两端（T6 engine、T7 planner）均已各自闭环验收，本任务增量仅为「G-code 文本入口」+「真电机」两项，风险面窄。

## 1. 红线检查（✓ 通过）

`git diff --numstat` 实质改动仅 3 文件，全部纯新增（无删除）：

| 文件 | 改动 | 归属 |
|---|---|---|
| main/app_m3_test.c | 新增 391 行 | 本任务 ✓ |
| main/Kconfig | +10（新增 APP_M3_TEST，只加不改） | §2 允许锚点 ✓ |
| main/CMakeLists.txt | +4（条件源文件，只加不改） | §2 允许锚点 ✓ |
| main/app_main.c | +13（前向声明、镜像 `#if`×2、调用点） | §2 允许锚点 ✓ |

- `components/` **零实质改动** ✓
- `main/app_joint_test.c` **零改动**（`git diff` 为空）✓
- 工作区其余 `M` 文件（bsp/settings/T4*/T11*/idf_env/tools）为 CRLF 换行符噪音，`numstat` 为空，**不纳入提交** ✓

## 2. 代码精读（核实清单）

| # | 项 | 结论 |
|---|---|---|
| 1 | 入口与位置一致性 | ✓ `app_m3_test_run(app_main 的 s_parser)`，复用真实 parser 实例，坐标基准后续可用 |
| 2 | 不走 protocol line handler | ✓ 直接 `gcode_parser_feed()`，UART1 无逐行 `ok` 污染 |
| 3 | 位置真值来源 | ✓ 全部用 `step_engine_position_steps()`（`?` 状态帧仍为 T3 占位，未用于断言） |
| 4 | 派生值运行期读取 | ✓ `k`/`soft_min`/`soft_max`/模态默认全部从 settings 读取，无硬编码 |
| 5 | 软限位前置检查 | ✓ `soft_min<=0 && soft_max>=40` 不满足则明确报错并 FAIL 退出，电机不使能 |
| 6 | 用例 10 条 | ✓ 与任务包 §3.2 逐条一致；期望步数用 `lroundf(目标mm × k)` 派生 |
| 7 | 步数断言 | ✓ **零容差**精确相等；偏移相对起手 base |
| 8 | 无运动用例断言 | ✓ 四重：engine 不忙 + engine/planner/parser 三处位置均不变（T7 位置提交修复的端到端兑现） |
| 9 | C9 越界值 | ✓ 运行期 `settings_soft_max() + 1.0f` 拼入 G-code 文本，不硬编码 601 |
| 10 | 失能终态 | ✓ 覆盖全部 return 路径（NULL parser / 软限位 / enable 失败 / 正常收尾） |
| 11 | 无忙等 | ✓ 轮询全用 `vTaskDelay`（10ms 周期 + 15s 超时） |
| 12 | DRV 判据 | ✓ `stst=1 且无 ot/s2ga/s2gb`；`ola/olb` 报告但按"12V 未接通属预期"不判定 |

## 3. 双配置构建验证（✓ 验收人实测）

| 配置 | bin 大小 | `M3 TEST` 字符串 | 编译 |
|---|---|---|---|
| on（M3=y，两个自测=n） | 0x45220 | **存在** ✓ | 零 error 零 warning |
| off（M3=n，临时 sdkconfig 副本） | 0x43750 | **不存在** ✓ | 零 error 零 warning |

- 差值 6864 字节 ≈ `app_m3_test.c` 代码量，合理
- on 配置额外确认 `JOINT TEST`（T5T6）与 `SELF-TEST DONE`（自测）字符串**均正确排除**，证明三个开关全部生效
- 产物时间戳（20:52:44 obj）> 源码（20:14:33），确认真实重编译
- 项目 `sdkconfig` 保持上板就绪态（M3=y、两自测=not set）

## 4. 上板实测（2026-09-16 21:02，COM 经 CH343P / UART1 镜像）

```
m3_test:   RESULTS  (10/10 cases PASS)
m3_test:     DRV:  PASS
m3_test:     HOME: PASS (returned to start position)
m3_test:   M3 TEST: PASS
```

### 4.1 逐例数据

| 用例 | G-code | 期望步数 | 实测 offset | 实测时长 | 理论时长 | 结论 |
|---|---|---|---|---|---|---|
| A1 | `G1 X40 F1200` | +3200 | **+3200** | 2040 ms | 2045 ms | ✓ 误差 0.2% |
| A2 | `G1 X0 F1200` | 0 | **0** | 2040 ms | 2045 ms | ✓ |
| B3 | `G91` | 无运动 | — | — | — | ✓ |
| B4 | `G1 X5 F1200`(rel) | +400 | **+400** | 290 ms | 290 ms | ✓ |
| B5 | `G1 X-5 F1200`(rel) | 0 | **0** | 290 ms | 290 ms | ✓ |
| B6 | `G90` | 无运动 | — | — | — | ✓ |
| C7 | `G1 X0.02 F1200` | +2 | **+2** | 20 ms | — | ✓ lroundf(1.6)=2 |
| C8 | `G1 X0 F1200` | 0 | **0** | 20 ms | — | ✓ |
| C9 | `G1 X601.000 F1200` | error:2 | 位置不变 | — | — | ✓ `planner: soft limit: target 601.0000 outside [0.0000, 600.0000]` |
| C10 | `G92 X0` | 无运动 | — | — | — | ✓ |

### 4.2 关键证据

- **步数精确性**：A1 +3200（40mm×80）、B4 +400、C7 +2 —— 全部与理论值**逐 bit 相等**（零容差断言通过）
- **时长吻合**：A1 实测 2040ms vs 理论 2045ms（加速 32 步 0.04s + 巡航 3136 步 1.96s + 减速 0.04s）；B4 实测 290ms vs 理论 290ms —— 梯形速度曲线在真电机上兑现
- **12V 接通确证**：`DRV_STATUS: stst=1 ola=0 olb=0 s2ga=0 s2gb=0 otpw=0 ot=0 cs_actual=25`
  - `ola/olb=0`（开载标志未置）→ 线圈有电流 → **12V 电机电源已接通**；若未接通这里会置 1
  - `cs_actual=25` 与 T5 初始化的 800mA 档位（CS=25）一致
  - `stst=1` → 静止检测在 300ms 等待后正确断言（T5T6 教训的有效性再次确认）
- **软限位端到端**：C9 被 planner 拒绝并回 `error:2`，且 engine **零脉冲**、parser 与 planner 位置**均未漂移**（T7 位置提交修复的实机兑现）
- **位置归零**：`final position: engine=0 (rel base 0), planner=0 (rel base 0)` —— 全程净位移为零，坐标基准干净
- **总耗时**：987ms → 6272ms ≈ **5.3s**（预估 ~6s）
- **电机物理表现**（用户目视确认）：40mm 正转 1 圈 → 反向回起点 → 5mm 小幅往返 → 2 步微动 → 静止

## 5. 遗留与风险

1. **`?` 状态帧仍为 T3 占位**（固定 `<Idle|C:0.00|…>`）——本次测试刻意绕开，真实遥测由 T9 交付
2. `W (939) uart: GPIO 41 is not usable, maybe used by others` 为 T3 已定性的记账式无害告警
3. 测试期间禁止向 UART1 发命令（main_task 与 comm_task 对 parser 存在竞争）——已在任务包与日志横幅中双重提示

## 6. 提交（沙箱限制，用户本地执行）

```powershell
git add main/app_m3_test.c main/Kconfig main/CMakeLists.txt main/app_main.c
git commit -m 'M3: joint test - real G-code text through parser->planner->engine->motor; 10 cases (absolute/relative/sub-step/soft-limit error:2/G92), exact zero-tolerance step assertions, soft-limit precondition, terminal disable; 10/10 on-board PASS'

git add docs/reviews/M3-joint-brief.md docs/reviews/M3-joint-acceptance.md PLAN.md
git commit -m 'docs: M3 joint test brief + acceptance (10/10 PASS on-board, exact steps +3200/+400/+2, DRV clean, HOME pass) + PLAN v1.14 - M3 milestone achieved'
```

> 注：`components/*`、`docs/reviews/T4*`、`T11*`、`idf_env.ps1`、`tools/` 为 CRLF 噪音，**不纳入**提交。
