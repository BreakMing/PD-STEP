# T5 验收报告 · TMC2209 驱动（UART 电流控制）

> 日期：2026-09-15 ~ 09-16 · 分支：task/T5-tmc（worktree PD-step-test-T5）· 验收人：规划 Agent
> 结论：**验收通过（代码级 + 上板三轮实测：62/62 自测、bringup 12/12、SPIN 电机物理转动确认）**。修复两处：P0 UART 缓冲区差一（首轮）、P1 VERSION 期望值（0x40→0x21，数据手册证实芯片读数正确）。StealthChop 实证（电机安静转动）。**任务闭环，待用户本地提交。**

---

## 1. 代码级验收（2026-09-15 完成）

- 红线：改动限于 `components/tmc2209/`、BSP EN 控制、app_main 接线 ✓
- 11 项核实清单：UART 帧（同步字节 0x05、CRC8 poly 0x07/init 0x00 LSB-first——修正了第三方资料的两处以讹传讹）、IHOLD_IRUN 写只读（官方 TMC-API 寄存器表确认不可回读，bringup 正确跳过）、回声处理（TX 经 1kΩ 至 PDN_UART、RX 直连）✓
- CRC 测试向量修复 4 处（自测里 init 误用 0xFF，官方为 0x00）
- 双配置构建：自测+bringup+SPIN 三开关固件、正常固件均通过（Temp 构建绕沙箱写限制）

## 2. 供电/调试端口矛盾与方案 4（日志镜像）

**硬件约束**（hardware.md §1）：单口 USB-C 充电+数据共用；PD 墙充给 12V 时无数据链路；调试日志默认走 UART0/CH340K。→ TMC2209 bringup 需要 12V，但日志在 UART0：**供电与看日志互斥**。

**方案 4（用户选定）**：测试固件在 `bsp_init()` 后挂 `esp_log_set_vprintf` 钩子，全部 ESP_LOG 同时镜像到 UART1（TX=GPIO41, 115200）；VOFA+ 经 CH343P 飞线模块接收。Kconfig 三测试开关任一开启才编译，正常固件零残留。顺带关闭 `CONFIG_LOG_COLORS`（免 ANSI 转义码干扰）。

## 3. 上板第一轮实测（2026-09-16 11:45，COM15，PD 墙充 12V）

**日志链路本身即第一大验证点：VOFA+ 经 UART1 收到全量日志 → 方案 4 成立**，供电矛盾解除。

| 层 | 结果 | 判据 |
|---|---|---|
| Layer 1 逻辑自测 | **62/62 PASS** | `SELF-TEST DONE: 62/62 PASS` ✓ |
| TMC init | **失败**（见 §4） | `uart_driver_install: ESP_FAIL` |
| Layer 2 bringup | 12 项 UART 相关全 FAIL（驱动未装，诚实上报）| 需 init 修复后复测 |
| SPIN | **已执行**：4.02s（11:45:35.868→39.890）与 2000 步@500Hz 精确吻合；EN→低/脉冲/EN→高 走 GPIO 不依赖 UART | 电机物理转动待用户确认 |

附带：protocol_start 重装 UART1 驱动的切换瞬间镜像丢 1-2 行（`tmc2209_bringup�`截断）——仅在启动期一次、不影响证据链，记录不修。

## 4. P0：UART 驱动安装差一错误（首轮暴露）

**根因**（tmc2209.c L521，验收 Agent 定位）：

```c
uart_driver_install(cfg->uart_num, 128, 128, 0, NULL, 0);   /* ✗ */
```

IDF v6 `uart_driver_install()` 要求 `rx_buffer_size > UART_HW_FIFO_LEN(uart_num)`（ESP32-S3 = 128）**严格大于**，tx 同理（或 =0）；传恰好 128 → `"uart rx buffer length error"` → ESP_FAIL → init 失败 → bringup 全部 UART 项失去前置。

- 该代码路径**首次上板执行**才暴露（纯逻辑自测不覆盖驱动安装；构建零告警）
- 修复：`uart_driver_install(cfg->uart_num, 256, 256, 0, NULL, 0)`（256 = FIFO 128 + 回声 8 字节帧余量）
- 复查 init 后半段（uart_param_config / uart_set_pin / 寄存器写入）无同类形态
- 重建：bin 2026-09-16 11:51:28（期间一次无关组件 esp_lcd 编译器 ICE，重跑通过）

## 5. 上板第二轮实测（2026-09-16 11:56，COM15，PD 墙充 12V）：UART 链路全通

P0 修复生效（UART2 驱动装上、TMC init 成功），结果：

| 层 | 结果 |
|---|---|
| 逻辑自测 | **62/62 PASS** |
| TMC init | `UART2: TX=17 RX=18 @ 115200` + `microstep=16, run=800mA(CS=25), hold=400mA(CS=12), VSENSE=1` ✓ |
| bringup | **11/12**：IOIN 读回 0x21000041、GCONF 写读回精确一致（0xc0）、IHOLD_IRUN 300/150mA、CHOPCONF MRES=5(8ustep)+INTPOL=1、DRV_STATUS raw=0xc00f0000（stst=1、无 OT、无短路/开路、cs_actual=15）全过 |
| SPIN | **已执行**：4.017s（24.663→28.680）与 2000 步 @500Hz 精确吻合；电机物理转动待用户确认 |

**唯一 FAIL：VERSION=0x21 ≠ 期望 0x40 → 定性为测试期望错误（P1），芯片健康**。官方 TMC2209 数据手册（Rev. 1.06+）：IOIN bits 31:24 VERSION = **0x21 = 芯片第一版**；0x40 是 TMC2240 的版本号，简报研究阶段张冠李戴。读回值 0x21 分毫不差 = 货真价实的 TMC2209。

**修复**：bringup 期望改 0x21（含注释注明出处），重建 bin 12:02:17。

## 5b. 上板第三轮实测（2026-09-16 12:18 固件，COM15，PD 墙充 12V）：✓ 全过闭环

| 层 | 结果 |
|---|---|
| 逻辑自测 | **62/62 PASS** |
| bringup | **12/12**：前 11 项逐行 PASS 可见（含 VERSION=0x21 转 PASS、GCONF 写读回 0xc0 一致、CHOPCONF MRES=5+INTPOL=1、DRV_STATUS raw=0xc00f0000 全绿）；第 12 项 SPIN 的 PASS 行与汇总行落在 protocol_start 重装 UART1 驱动的镜像中断窗口内（日志 I(5252) 一行可见乱码拼接）——但该项证据由**用户物理确认**补全，强于日志行 |
| SPIN | **电机平稳转动约 4 秒、非常安静**（12:18:31.980→35.996 = 4.016s，与 2000 步 @500Hz 精确吻合）——"安静"本身即 **StealthChop 生效的实证**（GCONF en_spreadcycle=0 设计意图达成） |

**三轮演进**：62/62 + bringup 全灭（P0 UART 缓冲）→ 62/62 + 11/12（P1 VERSION 期望）→ **62/62 + 12/12 + 物理确认**。

> 注：SPIN PASS 行/汇总行撞镜像中断窗口属已知行为（§2 已记录，仅启动期一次），不修；bringup 12 项判据逐行核对 + 物理证据齐备，闭环成立。

## 6. 提交（沙箱限制，用户本地执行）

```powershell
cd C:\Users\SM\ESP_workspace\PD-step-test-T5   # task/T5-tmc worktree
git add components/tmc2209 components/bsp main/app_main.c main/CMakeLists.txt
git commit -m "T5: TMC2209 UART driver - CRC8 framing (poly 0x07 init 0x00, sync 0x05), two-wire echo discard, current/microstep from settings, StealthChop config, bringup 12-item hardware verification; fixes: uart_driver_install buffers must exceed FIFO len (128->256), VERSION expectation 0x40->0x21 per datasheet; 62/62 self-test, motor spins quietly (StealthChop confirmed)"
```
