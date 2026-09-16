# 任务包 · M3 联合测试（真 G-code 驱动真电机）

> 分发对象：执行 Agent · 工作区：**主 worktree `C:\Users\SM\ESP_workspace\PD-step-test`（master）**，直接在工作区改动，**禁止一切 git 操作**（沙箱禁写 .git/objects；提交由用户执行）
> 上游事实源：PLAN.md（§8 M3 判据）、docs/reviews/T7-acceptance.md、components/planner/include/planner.h、components/gcode_parser/include/gcode_parser.h、components/step_engine/include/step_engine.h、components/tmc2209/include/tmc2209.h、components/settings/include/settings.h、main/app_main.c、main/app_joint_test.c（结构参照）

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 实现 `main/app_m3_test.c`（规格见 §3）
3. Kconfig / CMake / app_main 锚点改动（见 §2）
4. 双配置构建验证（§4）
5. 最终答复：改动文件清单、用例数值推导（按实际 settings 值代入）、预期日志全文模板、构建证据、给用户的上板操作说明（§5 模板基础上按实际填）

**红线（越界即验收不通过）**：
- **只准改动 4 处**：新建 `main/app_m3_test.c`、改 `main/Kconfig`（追加一个开关）、改 `main/CMakeLists.txt`（条件源文件，只加不改）、改 `main/app_main.c`（两处锚点，见 §2）
- **禁止改动 `components/` 下任何文件**——planner / gcode_parser / step_engine / tmc2209 / settings / protocol / bsp 全部已验收闭环
- **禁止改动 `main/app_joint_test.c`**（T5T6 联合测试已归档，保持原样）
- 禁止 git 操作（含 add/commit/stash/checkout）
- 主工作区有一批 **CRLF 行尾噪音 M**（`git status` 可见，`git diff --numstat` 为空）：**忽略，勿 add、勿还原**
- 等待必须用 `vTaskDelay` 轮询（看门狗），**禁止忙等**
- 只准使用 §1 列出的公开 API，禁止 include 组件内部头

## 1. 可用 API（已核实存在，验收按此检查）

```c
/* gcode_parser（T4/T7）—— 本测试以真实 G-code 文本行作为入口 */
gcode_parser_t *gcode_parser_create(float init_feed_mm_min, float init_accel_mm_s2,
                                    const parser_output_iface_t *iface);
parser_status_t gcode_parser_feed(gcode_parser_t *p, const char *line);  /* 不含 \r\n */
/* 返回：PARSER_OK=0 / PARSER_ERR_RANGE=2（软限位）/ PARSER_ERR_BAD_PARAM=4 / ... */

/* planner（T7）—— on_move 消费方 */
planner_status_t planner_enqueue(float target_mm, float feed_mm_min, float accel_mm_s2);
void      planner_set_origin(float new_pos_mm);
int32_t   planner_position_steps(void);
float     planner_position_mm(void);

/* step_engine（T6）—— 位置验证的唯一真值来源 */
esp_err_t step_engine_init(int step_gpio, int dir_gpio);   /* app_main 已调，勿重复 */
void      step_engine_stop(void);
int32_t   step_engine_position_steps(void);
uint32_t  step_engine_velocity_steps(void);
bool      step_engine_busy(void);

/* tmc2209（T5） */
esp_err_t tmc2209_enable(bool en);            /* true=EN低使能（内含 ≥1ms 唤醒等待） */
bool      tmc2209_read_status(tmc2209_status_t *st);   /* stst/ola/olb/s2ga/s2gb/otpw/ot/cs_actual */

/* settings（T11） */
float settings_steps_per_mm(void);   /* k，默认 80 */
float settings_soft_min(void);       /* $12，默认 0.0 */
float settings_soft_max(void);       /* $13，默认 600.0 */
```

## 2. 改动面（4 处，锚点精确）

1. **`main/Kconfig`**：在既有 `APP_JOINT_TEST` 之后追加（**只加不改**，不动既有块）：

```kconfig
config APP_M3_TEST
    bool "M3 joint test: real G-code lines via parser->planner->engine (real motor)"
    default n
    help
        Drives the full G-code chain with real command text: parser ->
        planner -> step_engine -> TMC2209 -> motor.  Verifies exact step
        counts for absolute/relative/sub-step moves, soft-limit rejection
        (error:2), and G92.  The motor physically moves — 12V required.
        Runs on boot after the G-code parser is registered.
```

2. **新建 `main/app_m3_test.c`**：入口 `void app_m3_test_run(gcode_parser_t *parser);`，规格见 §3。文件头注明 "motor physically moves; 12V supply required; runs after parser registration"。

3. **`main/CMakeLists.txt`**：追加条件编译（**只加不改**既有 joint test 块）：

```cmake
if(CONFIG_APP_M3_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "app_m3_test.c")
endif()
```

4. **`main/app_main.c` 两处锚点**（其余一律不动）：
   - 日志镜像 `#if` 条件追加 `|| defined(CONFIG_APP_M3_TEST)`（**两处**：钩子定义块、install 调用块）
   - 前向声明（与既有 `app_joint_test_run` 声明并列）：
     ```c
     #ifdef CONFIG_APP_M3_TEST
     void app_m3_test_run(gcode_parser_t *parser);
     #endif
     ```
   - 调用点：既有 `CONFIG_APP_JOINT_TEST` 块**之后**、`xTaskCreate(placeholder_task...)` 之前：
     ```c
     /* ---- M3 joint test (real G-code driving real motor, gated by Kconfig) ---- */
     #ifdef CONFIG_APP_M3_TEST
         if (s_parser != NULL) {
             app_m3_test_run(s_parser);
         }
     #endif
     ```

> **为何传入 app_main 的 parser**：使用真实注册的 parser 实例，其内部 `current_pos_mm` 与 planner 的 `pos_steps` 同源推进，测试后用户继续手工发命令时坐标基准一致（自建第二个 parser 会造成双份位置轨迹，禁止）。

## 3. 测试规格

### 3.0 关键事实（实现前必读，写错即返工）

1. **`?` 状态帧不可用于位置验证**：当前 `protocol.c` 的 `placeholder_status_cb` 固定回 `<Idle|C:0.00|E:0.00|V:0.00|T:OK>`（T9 遥测未实现）。**所有位置断言一律用 `step_engine_position_steps()`**。
2. **不要经过 `parser_line_handler`**：M3 直接调 `gcode_parser_feed(parser, "…")`，避免每行往 UART1 回 `ok` 污染日志流。判定用返回值。
3. **软限位可能被 NVS 改过**：越界用例的目标必须**动态**取 `settings_soft_max() + 1.0f`，不要硬编码 601（并在日志打印实际值）。
4. **位置断言用增量**：起手记录 `base = step_engine_position_steps()`，全程断言相对 base 的偏移（不假设 0）。
5. **k 与派生值运行期读取**：`k = settings_steps_per_mm()`，所有期望步数由 k 推导；用例数值在日志中打印，便于人工核对。
6. **模态 A 初值来自 NVS**：用例未写 `A` 时用 parser 的模态初值（`settings_default_accel()`）——本测试**不覆盖** settings，验证的是真实链路。

### 3.1 起手（顺序固定）

1. log 醒目警告横幅：`MOTOR WILL MOVE — ensure 12V supply and free travel`
2. log 派生参数：`k`、`soft_min`、`soft_max`、`default_feed/default_accel`
3. **软限位前置检查（必做——否则全部移动用例可能被拒）**：本测试的移动目标为 0 / 0.02 / 5 / 40 mm，必须落在 `[soft_min, soft_max]` 内。判据：`soft_min <= 0.0f` 且 `soft_max >= 40.0f`；不满足则 log ERROR 提示
   `soft limits [<min>, <max>] do not cover test travel (0..40mm) — run M502 or set $12=0 $13=600, then re-run`，
   打印 `M3 TEST: FAIL (soft limit range)` 并 return（**不留使能态**）
4. `base = step_engine_position_steps()`
5. `tmc2209_enable(true)` → 失败则 log ERROR、打印 `M3 TEST: FAIL (tmc2209_enable)` 并 return（**不留电机使能态**）
6. `vTaskDelay(10ms)` 后发首条命令

### 3.2 用例（四段，逐条 PASS/FAIL log）

| 段 | # | G-code 行 | 期望 status | 本段移动（步） | 段末位置（步，相对 base） | 说明 |
|---|---|---|---|---|---|---|
| A 绝对+真电机 | 1 | `G1 X40 F1200` | OK(0) | +3200 | **3200** | 40mm×k，真电机转 1 圈@16 微步 |
| | 2 | `G1 X0 F1200` | OK(0) | −3200 | **0** | 绝对回起点（有实际运动，非空操作） |
| B 相对模式 | 3 | `G91` | OK(0) | 0 | 0 | 模式切换，无运动 |
| | 4 | `G1 X5 F1200` | OK(0) | +400 | **400** | 相对换算（0 + 5mm） |
| | 5 | `G1 X-5 F1200` | OK(0) | −400 | **0** | 相对回零 |
| | 6 | `G90` | OK(0) | 0 | 0 | 恢复绝对 |
| C 亚步与软限位 | 7 | `G1 X0.02 F1200` | OK(0) | +2 | **2** | 1.6 步 lroundf → 2 |
| | 8 | `G1 X0 F1200` | OK(0) | −2 | **0** | 绝对回零 |
| | 9 | `G1 X<soft_max+1> F1200`（默认 601） | **RANGE(2)** | 0 | **0** | 软限位拒动，engine 零动作 |
| | 10 | `G92 X0` | OK(0) | 0 | 0 | 建零，无运动 |

> 表中步数为 **@k=80** 参考值；实现须由运行期读取的 `settings_steps_per_mm()` 推导（日志打印实际 k 供人工核对）。#9 的越界目标同理须运行期取 `settings_soft_max() + 1.0f`（"默认 601"仅作说明）。

**逐条断言**：
- `gcode_parser_feed()` 返回值 == 期望 status（#9 特别重要：`PARSER_ERR_RANGE`=2）
- 每条移动后轮询 `step_engine_busy()`（10ms 周期 `vTaskDelay`，超时 15s → FAIL）至 idle，再读 `step_engine_position_steps()`，断言其**相对起手 base 的偏移 == 表中"段末位置"**（逐条核对，不做范围容差——步数必须精确相等）
- 每段结束 log：`PASS <case>` / `FAIL <case> (expected …, got …)`
- #9 额外断言：拒动前后 `gcode_parser_get_position()` 与 `planner_position_steps()` 均不变、`step_engine_busy()` 为 false（**T7 位置提交修复的端到端验证**——软限位拒动既不得污染坐标，也不得产生任何脉冲）

### 3.3 收尾（顺序固定，任何提前 return 路径也必须执行失能）

1. `tmc2209_read_status()`：log `stst/ola/olb/s2ga/s2gb/otpw/ot/cs_actual`；判据 `stst=1` 且无 `ot/s2ga/s2gb`（`ola/olb` 在保持电流下应为 0——**前提：12V 已接通**；若未接通 12V 则线圈无电流，`ola/olb` 可能置 1，属预期，应在日志中说明而非误判为故障）。**读前 `vTaskDelay(300ms)`**（T5T6 教训：TMC2209 standstill 检测窗口 ~84ms，立即读 stst=0 假象）
2. `tmc2209_enable(false)` —— **失能为终态**
3. 最终位置归零确认：log `final position = planner_position_steps()`（相对 base 应为 0）
4. 汇总行：`M3 TEST: PASS` / `M3 TEST: FAIL (<原因>)`，逐条列出各用例结果

### 3.4 通用实现要求

- 总时长控制在 **30s 内**：用例 #1/#2 各约 2s（40mm @F1200：加减速各 32 步 ×0.04s + 巡航 3136 步 1.96s），#4/#5 各约 0.3s，其余为毫秒级
- 电机使能放最前、失能放最后（含所有 FAIL 路径）
- 所有 FAIL 路径打 ERROR log + 汇总行，**不 abort 不死循环**
- 不使用 `planner_selftest_last_seg()`（该钩子受 `CONFIG_PLANNER_SELF_TEST` 门控，本测试不依赖它）
- 不覆盖任何 settings 值（本测试验证的是真实链路，参数全部运行期读取）

## 4. 构建验证（两配置，证据必附）

```powershell
cd C:\Users\SM\ESP_workspace\PD-step-test
powershell -File idf_env.ps1 -IdfAction "-B C:\Users\SM\AppData\Local\Temp\build-m3-off build"
```

1. **off 构建**（先做，新符号默认 n）：`strings` 查 ELF **不含** `M3 TEST` 字样；零 error 零 warning
2. 翻开关（项目根 `sdkconfig`）：
   - 追加 `CONFIG_APP_M3_TEST=y`
   - **同时关闭** `CONFIG_PLANNER_SELF_TEST` 与 `CONFIG_GCODE_PARSER_SELF_TEST`（改为 `# CONFIG_… is not set`）——避免开机先跑 54 例自测拖延时间、污染日志
   - **顺序要求**：必须在 off 构建**之后**再改 sdkconfig。off 构建会让 confgen 识别新符号 `CONFIG_APP_M3_TEST`；若先手改再构建，追加的符号可能被重新生成时丢弃。改完确认 `idf.py` 重生成后三个符号状态正确（M3=y、两个自测=not set）
3. **on 构建**（第二个全新 Temp 目录）：ELF 含 `M3 TEST`；零 error 零 warning
4. sdkconfig 保持上板就绪态（M3=y、两个自测=n），在答复中注明

## 5. 上板操作（写给用户，你只写不执行）

供电与接线（沿 T5 验收方案 4）：
1. PC USB 烧录：`powershell -File idf_env.ps1 -IdfAction "-B <on构建目录> -p COM15 flash"`（烧完退出 monitor）
2. CH343P 飞线模块（**其 USB 插 PC 取电**，与板子只需两根线）：RX ← GPIO41、GND ↔ GND；VOFA+ 115200 raw text 接收
3. **换 PD 墙充 12V 上电**（此时板子单口 USB-C 无数据链路，日志全靠 UART1 镜像通道）
4. 上电自跑，全程无需按键；**测试期间请勿向 UART1 发送 G-code 命令**（测试在 main_task 内直接调 parser，与 comm_task 的 parser 调用存在竞争）

预期证据：
- **日志（UART1 镜像）**：起手参数、逐条 `PASS`、DRV_STATUS、`M3 TEST: PASS`
- **电机肉眼可见**：40mm 正向转 1 圈（@16 微步）→ 反向回起点 → 5mm 小幅往返 → 2 步微动 → 静止
- 测试结束后固件进入正常运行态，用户可继续手工验证真实使用方式：
  `M17` → `G1 X10 F600` → `?` → `G92 X0` → `M18`

## 6. 验收判据（规划 Agent 按此验收）

| # | 项 | 判据 |
|---|---|---|
| 1 | 红线 | 仅 §2 四处改动；`components/` 与 `app_joint_test.c` 零改动；无 git 操作 |
| 2 | 用例 | 10 条用例与 §3.2 期望一致；位置断言全部基于 `step_engine_position_steps()` 增量 |
| 3 | 安全 | 失能终态（含 FAIL 路径）；无忙等；软限位越界值动态取用 + 起手覆盖检查存在 |
| 4 | 构建 | 双配置零警告；off 无 `M3 TEST` 字符串残留 |
| 5 | 上板 | 10/10 用例 PASS、DRV_STATUS 干净、汇总 `M3 TEST: PASS`、电机物理转动与描述一致（用户确认） |
| 6 | 里程 | PLAN §8 M3 判据"一条 G1 精确步数执行"达成 → 里程碑可标 ✅ |
