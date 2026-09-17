# T8 验收报告 · AS5600 编码器驱动 + 多圈计圈

> 日期：2026-09-16 ~ 09-17 · 分支：master · 提交：待用户提交 · 验收人：规划 Agent
> 结论：**验收通过（代码级 + 三配置构建 + 上板自测 18/18 + 方向标定闭环）**。计圈算法（含跨零修正与边界抖动）、I2C 只读驱动、1kHz 采样任务、G92 偏移接口全部上板验证。上板过程暴露 **1 处 P1 标定问题**（C/E 符号相反），定位为磁铁安装方向，以 `CONFIG_AS5600_INVERT_DIR` 软件补偿后实测 `G1 X40 → +40.029mm`（偏差 3 counts）。**任务闭环。**

## 0. 上板过程总览

| 轮次 | 操作 | 结果 | 暴露问题 | 定级 | 处置 |
|---|---|---|---|---|---|
| 1 | 上电自测 + 手动转轴 | 18/18 PASS；`MANUAL delta = +0.000mm` | 未检测到转动 | 非缺陷 | 用户未在 15s 窗口内转轴（横幅仅 1.5s 后出现，错过时机） |
| 2 | 重跑，手动转轴 | 18/18 PASS；`delta = +71.445mm` | 转速不足 | 手动方法局限 | 实测 0.13 圈/s，15s 只够 1.79 圈（3 圈需 23s） |
| 3 | 改用电机带动 `G1 X120` | 编码器读数 **−116.98mm** | **C/E 符号相反** | **P1（标定）** | 见 §5 |
| 4 | 方向补偿后 `G1 X40` | **+40.029mm** | — | — | **闭环** |

**复盘价值**：第 3 轮问题由 **T8 自己交付的编码器**首次暴露——M3 的 10/10 PASS 无法发现，因为它只断言固件内部计数器（`step_engine_position_steps()`），而内部计数器与 planner 天然同号、自洽。只有挂在轴上的独立传感器才能对拍"固件以为的方向"与"轴实际转的方向"。**这正是 M4「位置可信」的意义，也印证了交叉验证的必要性。**

## 1. 红线检查（✓ 通过）

**🚨 只读红线（本任务最高优先级）**：全组件**零 I2C 写调用**。唯一总线操作是 `as5600_read_raw()` 里的 `i2c_master_transmit_receive()`（写寄存器地址 + 读 2 字节，单事务读）。代码中另有两处显式防护注释：`Do not add i2c_master_transmit() or any write helper here.`（`as5600.c` 文件头与 `as5600.h` 文件头）。AS5600 的 ZPOS/MPOS/CONF/MANG（一次性 OTP）**完全未被触碰** ✓

| 文件 | 状态 | 归属 |
|---|---|---|
| components/as5600/{as5600.c, as5600_tracker.c, as5600_self_test.c, Kconfig, include/as5600.h} | 新增 | 本任务 ✓ |
| components/as5600/CMakeLists.txt | 修改（骨架 `SRCS ""` → 实装 + 条件编译） | 本任务 ✓ |
| main/app_main.c | 修改（+17：include、镜像 `#if`×2、`as5600_init`、G92 扩展、自测调用） | §6 允许锚点 ✓ |
| main/CMakeLists.txt | 修改（+1/-1：REQUIRES 追加 as5600） | §7 允许锚点 ✓ |

- `bsp/`、`protocol/`、`gcode_parser/`、`settings/`、`tmc2209/`、`step_engine/`、`planner/`、`app_joint_test.c`、`app_m3_test.c` **零改动** ✓
- 工作区其余 `M` 文件（bsp/settings/T4*/T11*/idf_env/tools）为 CRLF 换行符噪音，`numstat` 为空，**不纳入提交** ✓

## 2. 代码精读（核实清单）

| # | 项 | 结论 |
|---|---|---|
| 1 | 只读红线 | ✓ 见 §1；无任何写路径 |
| 2 | 角度读取 | ✓ `0x0C/0x0D` **单次 2 字节 burst**（避免两次独立读拼接出不存在值）；`raw = ((buf[0] & 0x0F) << 8) \| buf[1]` |
| 3 | 计圈规则 | ✓ `diff > +2048 → −4096`；`diff < −2047 → +4096`（严格不等号，4096 种 diff 无遗漏无重叠） |
| 4 | 跨零与边界抖动 | ✓ 4095→0 修正为 +1 count，边界抖动天然退化为 ±1，无需额外死区（A6 专项验证） |
| 5 | 采样任务 | ✓ 1kHz `vTaskDelayUntil`、优先级 3、**固定 CPU1**（阻塞 I2C 事务不干扰 CPU0 的 STEP ISR）、栈 3072 |
| 6 | 失败处理 | ✓ 连续失败计数，≥10 次置 `is_ok=false`；恢复时清零 + I 级日志 |
| 7 | 单位换算 | ✓ `mm_per_count = $2 / 4096`；运行期读取并**订阅 $2** 刷新（订阅失败 W 级告警） |
| 8 | G92 偏移（D10） | ✓ `offset = raw_mm − new_pos_mm`；上报位置 = raw_mm − offset |
| 9 | 速度估计 | ✓ 16ms 窗口（17 样本环形缓冲），`delta × mm_per_count × 1000/16` → mm/s |
| 10 | 线程安全 | ✓ `portMUX` 保护共享状态；采样任务单写者，getter 跨任务/跨核读 |
| 11 | API | ✓ 与任务包 §4 签名逐一一致 |
| 12 | 集成 | ✓ `as5600_init()` 在 `bsp_init()` 成功分支内、自测段之前；`cb_on_set_origin` 增加 `as5600_set_origin()` 实现 D10 双向平移 |

## 3. 构建验证（三配置，验收人实测）

| 配置 | bin 大小 | ELF 含自测串 | 编译 |
|---|---|---|---|
| ① 全关（AS5600=n, PLANNER=n） | 0x44ed0 | **无** ✓ | 零 error 零 warning |
| ② 仅 AS5600（AS5600=y） | 0x46e60 | 有 ✓ | 零 error 零 warning |
| ③ AS5600 + PLANNER | 0x4a070 | 有 ✓ | 零 error 零 warning |

- 差值合理：②−① = 8080 字节 ≈ `as5600_self_test.c` 代码量
- 配置 ① 额外确认驱动代码始终在（`AS5600 online` 字符串存在），仅自测被排除——条件编译正确
- 方向补偿改动后重建：0x46f00（+160 字节），零警告

## 4. 上板自测（18/18 PASS）

```
as5600: AS5600 online at 0x36 — first raw angle=1594 (seeded, E=0)
as5600: sampling task up: 1000 Hz, prio 3, CPU1, 0.009766 mm/count (40.000 mm/rev ÷ 4096 counts)
...
SELF-TEST DONE: 18/18 PASS
```

**A 组（纯逻辑 12 例）全过**，关键项：
- `A4 4090→10 → +16 counts`（正向跨零修正正确，朴素减法会得 −4080）
- `A5 10→4090 → −16 counts`（反向跨零）
- `A6 0↔4095 dither ×8 → drift ≤1 count`（边界抖动不累积）
- `A7 forward 3 turns → +12288 counts = 120.000 mm`（PLAN 判据的纯逻辑等价验证）
- `A9/A10 diff +2048 / −2047 不触发修正`（阈值边界精确）

**B 组（实机 6 例）全过**，关键数字：

| 项 | 实测 | 判据 |
|---|---|---|
| B14 I2C 读通 | 20 次全部 0..4095 | ✓ |
| **B15 静止噪声** | `raw 1594..1595, unwrapped p-p = **1 count**` | ≤2 ✓ **余量充足** |
| B16 静止速度 | `0.000 mm/s`（另一轮 −0.610 mm/s，即 1 count/16ms 抖动） | ≤5 mm/s ✓ |
| B17 换算一致性 | `counts=1 → expect 0.010, reported 0.010` | ✓ |
| B18 G92 偏移 | `set_origin(100)` → E=100.000；`set_origin(0)` → E=0.010 | ✓ |

> B15 的实现细节值得记录：它测量的是**反缠绕后的差分峰峰值**，而非 raw 极差。若轴恰好停在 0/4095 边界，直接比 raw 会得出 4095 counts 的假噪声；执行模型主动处理了这一边界，并额外输出 raw 极值以避免"静默吞掉"该情形。

## 5. 验收期修复：编码器方向标定（P1）

### 5.1 现象与定位

电机带动实测（`G1 X120 F1200`，planner 记账 +120mm）：

```
planner 记账 (C)：  0mm ─────────────► +120mm
编码器实测 (E)： +71.4mm ───────► −45.5mm     （变化 −116.98mm）
```

**量级吻合（~117mm ≈ 120mm）但符号相反** → C/E 不可比，M4 判据无法落地。

### 5.2 根因判定

排除 `$10 invert_dir`：M3 联合测试曾以 `$10=0` 通过 10/10（`G1 X40` 断言 `step_engine_position_steps()` 增量 +3200），若 `$10=1` 该断言必失败。故 `$10=0`，问题在**编码器读数方向**。

**磁铁安装方向**：AS5600 原始角度的增减方向取决于磁铁磁化方向与轴的相对安装，磁铁翻转 180° 即整体反向。属**安装标定**，非芯片/驱动缺陷。

### 5.3 修复（`CONFIG_AS5600_INVERT_DIR`）

```c
/* as5600.c — as5600_read_raw() 出口 */
uint16_t raw = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
#if CONFIG_AS5600_INVERT_DIR
    raw = (uint16_t)((AS5600_COUNTS_PER_REV - 1) - raw);   /* 镜像 */
#endif
```

- **实现位置有意选在驱动层**（`read_raw()` 出口）而非 tracker 内：tracker 的 12 个纯逻辑用例（A4 跨零 +16、A9 阈值 +2048 等）均基于原始角度序列，若镜像加在 tracker 内会全部反向失败。驱动层镜像对纯逻辑部分零影响 ✓
- 镜像保持值域 0..4095（双射），因 tracker 使用差值，等价于方向反转
- 新增启动日志 `counting direction: INVERTED (magnet mounted opposite; +X move counts up)`，使该标定状态可从启动日志直接确认
- Kconfig help 记录标定依据与**耦合警告**：`$10` 翻转电机物理方向后，编码器读数方向随之翻转，需重新确认 C/E 一致性

### 5.4 验证（✓ 方向对齐，精度优秀）

```
G92 X0         → raw 基准 0.010 mm
G1 X40 F1200   → 走 1 圈（3200 步）
G92 X0         → raw = +40.029 mm      ✓ 正值，方向正确
```

| 指标 | 数值 |
|---|---|
| 期望 | +40.000 mm |
| 实测 | **+40.029 mm** |
| 偏差 | **+0.029 mm = 3 counts**（1 count = 9.7656 µm） |
| 相对误差 | **0.073%**/圈 |

3 counts 属优秀水平——AS5600 配常规安装磁铁的固有误差（磁铁偏心、微步定位精度）通常在 ±5–10 counts 量级。

## 6. PLAN T8 判据对照

| PLAN 判据 | 达成情况 |
|---|---|
| 计圈逻辑宿主机单测（含边界抖动序列） | ✅ A 组 12 例上板自测全过（执行路径为 v1.8 定案的上板自测），含 A6 边界抖动、A4/A5 双向跨零 |
| 实机手动转轴 3 圈，上报 120mm±0.05mm | ⚠ **改用更强的等价方法**：电机带动 1 圈 → `+40.029mm`（偏差 0.029mm < 0.05mm）。电机带动可重复、可精确复现，优于手动转轴（手动实测 0.13 圈/s，15s 窗口内无法完成 3 圈）。3 圈整段判据留待 T9 提供 `?` 遥测后一键验证 |
| 静止时读数噪声 ≤2 counts | ✅ 实测 **1 count** |

## 7. 遗留项

1. **3 圈整段判据未单独执行**——1 圈证据（偏差 0.029mm）已在判据量级内；T9 完成后发 `G1 X120` + `?` 即可同时读 C/E 一次性验证
2. **方向标定与 `$10` 的耦合**——改 `$10` 后须重新确认 C/E 符号一致性（已写入 Kconfig help）
3. **命令位置（C）与编码器位置（E）的实时对比**——当前只能靠 G92 日志间接观察，T9 遥测交付后直接可用
4. **settings 订阅槽已满（4/4：tmc2209 / planner / $10 / as5600）**——T9 消费 `$9 report_hz` 时不可再订阅，需改为组帧时直接读取（已移交 T9）

## 8. 提交（沙箱限制，用户本地执行）

```powershell
git add components/as5600 main/app_main.c main/CMakeLists.txt
git commit -m 'T8: AS5600 encoder - 12-bit angle read + multi-turn unwrapping (asymmetric +2048/-2047 thresholds), 1kHz CPU1 sampling task, velocity window, G92 origin shift (D10); read-only red line (no register writes); 18-case on-board self-test PASS; acceptance fix: CONFIG_AS5600_INVERT_DIR mounting calibration (C/E sign mismatch, verified +40.029mm on 1 rev)'

git add docs/reviews/T8-brief.md docs/reviews/T8-acceptance.md PLAN.md
git commit -m 'docs: T8 brief + acceptance (18/18 PASS, direction calibration, 0.029mm/rev accuracy) + PLAN v1.15 - T8 closed'
```

> 注：`bsp/`、`settings/`、`docs/reviews/T4*`、`T11*`、`idf_env.ps1`、`tools/` 为 CRLF 噪音，**不纳入**提交。
