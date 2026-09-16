# 任务包 · T5×T6 联合测试（首次引擎驱动真电机）

> 分发对象：执行 Agent（Claude Code）· 工作区：**主 worktree `C:\Users\SM\ESP_workspace\PD-step-test`（master，合并后 6fc93a0）**，直接在工作区改动，**禁止一切 git 操作**（沙箱禁写 .git/objects；提交由用户执行）
> 上游事实源：docs/reviews/T5-acceptance.md、docs/reviews/T6-acceptance.md（§8 P3 Q16.16 上限教训）、components/tmc2209/include/tmc2209.h、components/step_engine/include/step_engine.h、components/settings/include/settings.h、main/app_main.c（已合并版）

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 实现联合测试（改动面见 §2，测试规格见 §3）
3. 双配置构建验证（§4）
4. 最终答复：改动文件清单、段参数推导（按实际 settings 值代入）、预期日志全文模板、构建证据、给用户的上板操作说明（§5 模板基础上按实际填）

**红线（越界即验收不通过）**：
- **只准改动 4 个文件**：新建 `main/Kconfig`、新建 `main/app_joint_test.c`、改 `main/CMakeLists.txt`、改 `main/app_main.c`（两处锚点，见 §2）
- **禁止改动 `components/` 下任何文件**——六个组件全部已验收闭环
- 禁止 git 操作（含 add/commit/stash/checkout）
- 构建必须 `-B` 指向 Temp 全新目录（项目 build/ 目录不在沙箱可写白名单）
- 主工作区有一批 **CRLF 行尾噪音 M**（git status 可见，内容零变化）：**忽略，勿 add、勿还原**
- 等待必须用 `vTaskDelay` 轮询（看门狗），禁止忙等
- 只准使用 §1 列出的公开 API，禁止 include 组件内部头

## 1. 可用 API（已核实存在，验收按此检查）

```c
/* step_engine（T6） */
esp_err_t step_engine_init(int step_gpio, int dir_gpio);   /* app_main 已调，勿重复 */
bool      step_engine_enqueue(const step_seg_t *seg);      /* 段校验：accel_steps+cruise_steps+decel_steps 必须严格 == steps */
void      step_engine_stop(void);
int32_t   step_engine_position_steps(void);
uint32_t  step_engine_velocity_steps(void);
bool      step_engine_busy(void);

/* tmc2209（T5） */
esp_err_t tmc2209_enable(bool en);            /* true=EN低使能（内含≥1ms唤醒等待） */
bool      tmc2209_read_status(tmc2209_status_t *st);   /* DRV_STATUS 解码：stst/ola/olb/s2ga/s2gb/otpw/ot/cs_actual */

/* settings（T11） */
int   settings_microstep(void);      /* $0，默认 16 */
int   settings_full_steps(void);     /* $1，默认 200 */
```

## 2. 改动面（4 文件，锚点精确）

1. **新建 `main/Kconfig`**：

```kconfig
menu "Application"
config APP_JOINT_TEST
    bool "T5+T6 joint motion test (real motor, runs on boot)"
    default n
endmenu
```

2. **新建 `main/app_joint_test.c`**：入口 `void app_joint_test_run(void)`，规格见 §3。文件头注明"motor physically moves; 12V supply required; runs after all subsystems up"。
3. **`main/CMakeLists.txt`**：仿 step_engine 组件的条件编译模式：

```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp protocol gcode_parser settings step_engine tmc2209
)
if(CONFIG_APP_JOINT_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "app_joint_test.c")
endif()
```

4. **`main/app_main.c` 两处锚点**（其余一律不动）：
   - 日志镜像 `#if` 条件追加 `|| defined(CONFIG_APP_JOINT_TEST)`（两处：钩子定义、install 调用）
   - 调用点：parser 注册成功分支之后、`xTaskCreate(placeholder_task...)` 之前：

```c
#ifdef CONFIG_APP_JOINT_TEST
    app_joint_test_run();
#endif
```

## 3. 测试规格

### 3.0 前置派生（任务上下文算好，禁在 ISR 算——本测试无 ISR 代码）

```
spr = settings_full_steps() × settings_microstep()     /* 默认 200×16 = 3200 steps/圈 */
安全检查：若派生 v_cruise > 40000 steps/s（微步 256 边缘），log ERROR 并中止（50kHz 性能门）
```

### 3.1 Phase A — 正向梯形（2 圈）

| 参数 | 公式 | @spr=3200 参考值 |
|---|---|---|
| dir | true | — |
| steps | 2×spr | 6400 |
| v_cruise | spr（1 圈/s） | 3200 st/s |
| accel | spr（1 圈/s²） | 3200 st/s² |
| accel/cruise/decel_steps | spr/2, spr, spr/2 | 1600/3200/1600 |
| 预期时长 | 3.0 s | 3.0 s |

判据：enqueue true；`step_engine_position_steps() == 2×spr` **精确相等**；实测时长 3.0s ±5%。巡航中段（启动后 ~1.5s）采样一次 `step_engine_velocity_steps()`，记录值应 ≈ v_cruise ±5%。

### 3.2 Phase B — 反向回原点

同 A 但 `dir=false`。判据：位置回到 **0**（精确）；时长同 A。A 结束到 B 开始间隔 ≥100ms。

### 3.3 Phase C — '!' 立即停（交互项，验证 T6 停止链路上板）

| 参数 | 公式 | @spr=3200 参考值 |
|---|---|---|
| dir | true | — |
| v_cruise | spr/4（0.25 圈/s） | 800 st/s |
| accel | spr/8（0.125 圈/s²） | 400 st/s² |
| accel/cruise/decel_steps | 200/800/... 按 v²/(2a) 与 15s 巡航推导 | 800/12000/800，共 13600 |
| 预期窗口 | ~19 s（若不按 '!'） | 19 s |

- 启动前 log 醒目横幅：`PHASE C: send '!' NOW via UART1 to test immediate stop`
- 每 1s log 一次：position + velocity + busy
- **用户按 '!'** → protocol 实时字符 → stop_cb → `step_engine_stop()` → 脉冲即刻停
- 判定：busy=false 且 `position < 13600`（提前停）；若走完 19s position==13600，log "no '!' received (segment completed)"——不算 FAIL，但记录
- 结束后 log 停止位置与从 Phase C 启动到停止的实测时长

### 3.4 收尾（必做，顺序固定）

1. `tmc2209_read_status()`：log stst/ola/olb/s2ga/s2gb/otpw/ot/cs_actual；判据 stst=1、无 ot/短路/开路（ola/olb 在静止保持电流下应为 0）
2. `tmc2209_enable(false)` —— **电机失能为终态，任何提前 return 路径也必须先失能**
3. 汇总行：`JOINT TEST: PASS` / `FAIL (<原因>)`，逐项 A/B/C 结果

### 3.5 通用实现要求

- 每段 enqueue 后轮询 `step_engine_busy()`（10ms 周期 vTaskDelay），A/B 超时 5s、C 超时 25s → FAIL
- 电机使能放最前（`tmc2209_enable(true)` 后 10ms 再发首段）
- 所有 FAIL 路径打 ERROR log + 汇总行，不 abort 不死循环
- 三段构造务必自查 **accel_steps+cruise_steps+decel_steps == steps**（T6 验收 P0#3 教训：此处笔误编译零告警）
- 电流用 tmc2209_init 已应用的 settings 默认值（$3/$4），**测试代码不得覆盖**——本测试就是验证 settings→init→运动的真实链路

## 4. 构建验证（两配置，证据必附）

```powershell
cd C:\Users\SM\ESP_workspace\PD-step-test
# 方式一（T5/T6 已验证的 wrapper）：
powershell -File idf_env.ps1 -IdfAction "-B C:\Users\SM\AppData\Local\Temp\build-joint-off build"
# 方式二（若 wrapper 报 python 环境不一致）：直调
& 'C:\Espressif\tools\python_env\idf6.0_py3.11_env\Scripts\python.exe' 'C:\Users\SM\.espressif\v6.0.2\esp-idf\tools\idf.py' -B C:\Users\SM\AppData\Local\Temp\build-joint-off build
```

1. **off 构建**（先做，此时 sdkconfig 尚无新符号，默认 n）：`strings` 查 ELF **不含** `JOINT TEST` 字样；零 error 零 warning
2. 翻开关：项目根 `sdkconfig` 追加 `CONFIG_APP_JOINT_TEST=y`（该文件未跟踪、不入库；T5/T6 同款做法）
3. **on 构建**（第二个全新 Temp 目录）：ELF 含 `JOINT TEST`；零 error 零 warning
4. sdkconfig 保持 =y（上板就绪态），在答复中注明

## 5. 上板操作（写给用户，你只写不执行）

供电与接线（沿 T5 验收方案 4）：PC USB 烧录（`idf.py -B <on构建目录> -p COM15 flash`，烧完退出）→ CH343P 飞线模块 RX←GPIO41、GND↔GND → VOFA+ 115200 raw text → 换 PD 墙充 12V 上电自跑。
用户唯一交互：**Phase C 横幅出现后经 VOFA+ 发裸字符 `!`**（raw 模式直接敲，无需回车——T3 验收已验证实时字符语义）。A/B 阶段请勿按键。
预期证据（VOFA+ / UART1）：A/B 位置与时长数字、C 停止位置、收尾 DRV_STATUS、`JOINT TEST: PASS`。电机肉眼可见：加速→匀速→减速→停→反向同轮廓→低速长段中按 `!` 即刻停。

## 6. 验收判据（规划 Agent 按此验收）

| # | 项 | 判据 |
|---|---|---|
| 1 | 红线 | 仅 §2 四文件改动；组件零改动；无 git 操作 |
| 2 | 段构造 | 三段之和==steps 逐段核对；参数与 §3 公式一致 |
| 3 | 安全 | 失能终态；无忙等；无 API 越界 |
| 4 | 构建 | 双配置零警告；off 无字符串残留 |
| 5 | 上板 | A/B 位置精确、时长 ±5%、C '!' 即停、DRV_STATUS 干净、汇总 PASS |
| 6 | 电机 | 物理转动轮廓与 Phase 描述一致（用户肉眼确认） |
