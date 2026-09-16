# T5×T6 联合测试 · 验收报告（代码级）

> 日期：2026-09-16 · 工作区：master 主 worktree（合并后 6fc93a0 之上，工作区改动待提交）· 验收人：规划 Agent
> 结论：**代码级验收通过（附验收修复 P1×1 + 简报偏差补齐×2）**。红线、段参数、安全设计全部核验；构建双配置零警告（off 13:32 无字符串残留 / on 13:47 含修复）。**上板执行待用户**（Phase C 需交互发 `!`）。

## 1. 红线（✓ 通过）

- 真实内容改动仅 4 文件：新建 `main/Kconfig`（`APP_JOINT_TEST`，default n，含 help）、新建 `main/app_joint_test.c`（443 行）、`main/CMakeLists.txt`（条件编译 `target_sources`）、`main/app_main.c`（3 锚点：前置声明、镜像 `#if` ×2 扩展、调用点）
- `components/` 零改动；无 git 操作；构建全走 Temp
- 主工作区其余 M 均为已知 CRLF 行尾噪音（`autocrlf=false diff` 内容为空）

## 2. 段参数核对（✓，@spr = 200×16 = 3200）

| Phase | steps | accel/cruise/decel | v_cruise | accel | 预期 |
|---|---|---|---|---|---|
| A 正向 | 6400 | 1600/3200/1600 | 3200 | 3200 | 3.0s，位置 == 6400 精确 |
| B 反向 | 6400 | 同 A | 3200 | 3200 | 位置 == 0 精确 |
| C 低速长段 | 13600 | 800/12000/800 | 800 | 400 | 19s 窗口，`!` 即停 |

三段之和自查函数 `seg_check_sum` 在每个 enqueue 前调用（T6 P0#3 教训已内置）；C 的 accel_steps=800 与 v²/(2a)=800²/(2×400) 手工推导一致。

## 3. 验收修复（执行 Agent 交付后的精读发现）

### P1 · Phase C 超时判定反转（假阳性风险）

原代码：
```c
if (!step_engine_busy() && elapsed >= TIMEOUT_PHASE_C_MS) {   /* ✗ 取反了 */
```
注释写 "still busy"，条件却测**不忙**。后果：引擎卡死（busy 永真——本阶段最需要抓的故障模式）时 25s 退出循环，FAIL 分支不触发，`stopped_early=true` 被误报为 `PASS (stopped early by '!')`。修复为 `if (step_engine_busy())`。

### 偏差补齐 1 · Phase A 巡航中段速度采样（简报 §3.1 要求，执行者曾跳过）

轮询循环内在 elapsed≥1500ms 采样 `step_engine_velocity_steps()`，判 ±5%（3000–3360），超差返回 `PHASE_FAIL_VELOCITY`。轮廓恒为 1s 加速 + 1s 巡航 + 1s 减速（v=a=spr、cruise=spr），1.5s 必在巡航中段——该点与 spr 无关，通用。

### 偏差补齐 2 · teardown 的 ola/olb 判定（简报 §3.4 要求，执行者曾只记录不判定）

`drv_ok` 补上 `!st.ola && !st.olb`（静止保持电流下线圈通电应无开路；T5 bringup 实测 0/0 佐证）。

## 4. 设计核验（✓）

- Phase C 不自调 `step_engine_stop()`——`!` 经 protocol 实时字符 → stop_cb → 引擎停，**测的就是完整生产链路**（正确）
- 电机失能为强制终态：所有 FAIL/提前 return 路径最终都经 teardown 或根本未使能
- 电流用 tmc2209_init 的 settings 值，测试不覆盖（正确——测 settings→TMC→运动真实链路）
- 安全检查：v_cruise > 40000 steps/s 中止（微步 256 时 spr=51200 会被拦截）
- 等待全为 vTaskDelay 轮询，无忙等；仅用 §1 公开 API
- Phase B 在 A 失败时跳过并计入 FAIL；C 独立执行（A 失败仍测停止链）

## 5. 构建证据

| 配置 | 目录 | bin 时间 | 判别 |
|---|---|---|---|
| off | Temp\build-joint-off | 13:32:47 | ELF 不含 `JOINT TEST` ✓ |
| on（含验收修复） | Temp\build-joint-on | **13:47:47** | ELF 含 `JOINT TEST` ✓，零 error 零 warning |

sdkconfig 保持 `CONFIG_APP_JOINT_TEST=y`（上板就绪态）。

## 6. 上板第一轮实测（2026-09-16 14:08，COM15，PD 墙充 12V）：运动全过，teardown 时序缺陷修复待复测

> 插曲：13:57 一轮烧错固件（T5 时代 t5_build，62 例自测+bringup 日志流指纹即暴露），主工作区 sdkconfig 三 TMC 开关实为全关，重烧 build-joint-on 后正确。

| Phase | 实测 | 判定 |
|---|---|---|
| A 正向 | 位置 **6400 精确**、2950ms（设计 3000ms，98.3%）、巡航中段速度 3200（±5% 判定 OK） | **PASS** |
| B 反向 | 位置 **0 精确**、2960ms | **PASS** |
| C 低速长段 | 位置 **13600 精确**、**18910ms vs 设计 19s**（2s+15s+2s 分毫不差）；逐秒遥测与理论完全吻合（t=1s pos198≈200/vel400，巡航 +800 步/s，t=18s 减速 vel400）；用户肉眼确认两圈梯形加减速+恒速轮廓 | PASS（但**用户未按 `!`**，early_stop=no——交互停止链路证据缺失） |
| Teardown | stst=**0** → FAIL | **测试代码时序缺陷** |

**stst=0 根因**：DRV_STATUS 在最后脉冲后 9ms 即读，而 TMC2209 standstill 检测需 ~2^20 内部时钟周期（≈84ms @12.5MHz）无脉冲才置位——读早了，硬件无恙（ola/olb/ot/短路全 0、cs_actual=25 与运行电流 CS=25 一致）。**修复**：teardown 先等 300ms 再读（bin 14:14:50）。

**第二轮待办**：① stst 转正复测 ② **Phase C 横幅后实际按 `!`**（本轮漏按）——交互停止链路（protocol 实时字符→stop_cb→引擎停）是本测试的核心遗留证据。

## 7. 上板操作（第二轮，待用户执行）

1. **PC USB 烧录**：`idf.py -B C:\Users\SM\AppData\Local\Temp\build-joint-on -p COM15 flash`（烧完即拔）
2. **接线**：CH343P 飞线模块 RX←GPIO41、GND↔GND；VOFA+ 115200 raw text
3. **换 PD 墙充 12V**，上电自跑
4. **Phase C 横幅出现后**，经 VOFA+ 敲裸字符 `!`（无需回车）——A/B 阶段请勿按键

预期日志锚点：`mid-cruise velocity: ~3200 (OK)`、A `position=6400 elapsed≈3000ms`、B `position=0`、C `early_stop=yes` + 停止位置 < 13600、teardown `ola=0 olb=0 ot=0`、末行 `JOINT TEST: PASS`。
电机肉眼可见：加速→匀速→减速→停 3s（2 圈）→ 反向同轮廓回起点 → 低速长段中按 `!` 即刻停。

## 8. 提交（上板通过后，用户本地执行）

```powershell
cd C:\Users\SM\ESP_workspace\PD-step-test
git add main/Kconfig main/app_joint_test.c main/CMakeLists.txt main/app_main.c docs/reviews/T5T6-joint-brief.md docs/reviews/T5T6-joint-acceptance.md
git commit -m "T5T6 joint test: first engine-driven real motor (trapezoid fwd/rev + interactive '!' stop chain); acceptance fixes: Phase C stuck-busy timeout detection, mid-cruise velocity check, ola/olb assertion"
```
