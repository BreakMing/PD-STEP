# T2 验收报告（BSP 引脚与外设初始化）

> 验收日期：2026-09-12
> 验收对象：分支 `task/T2-bsp`，提交 `43dbb8f T2: BSP pin config and peripheral initialization`
> 验收方式：代码审查 + 解析后 sdkconfig 逐项核对 + v6.0.2 头文件 API 核对 + 构建产物审查 + **需求方上板实测**
> 任务包：本轮对话交付的 T2 任务包（对话记录）

---

## 结论：**通过，T2 关闭，已合并 master**

代码级验收零问题（无 P1/P2），上板实测核心项全过，并顺带了结一项 T0 遗留未知。

## 代码级验收（验收方审查）

| 项 | 结果 | 证据 |
|---|---|---|
| git 工作流 | ✓ | master 切出 `task/T2-bsp`，单提交，6 文件 485 行全部在任务范围内 |
| Kconfig 引脚项 | ✓ | 13 项与 docs/hardware.md §3 逐一相符：UART1 10/11@115200、TMC 17/18、STEP=5、DIR=6、EN=21、DIAG=16、I2C 8/9@400kHz、AS5600=0x36 |
| EN 失能最早执行 | ✓ | `app_main.c:49` `bsp_early_init()` 为函数第一行，先于任何日志；EN 输出即置高（失能） |
| UART1 初始化 | ✓ | 115200 8N1、RX 缓冲 1024B、TX 512B；尾部发 "BSP UART1 ready" |
| GPIO 初始化 | ✓ | STEP/DIR 输出置低、DIAG 输入无上下拉 |
| I2C 双保险 | ✓ | 无上拉建总线 → 扫 0x08–0x77 → 零应答则删总线带内部上拉重建重扫；两轮结果均记日志；0x36 命中打印结论行 |
| v6 新 API 用法 | ✓ | `i2c_master_bus_config_t` 字段名、`i2c_master_probe` 签名与本机 v6.0.2 头文件核对一致；`glitch_ignore_cnt=7` 为官方推荐值 |
| CMake 精细化 | ✓ | bsp 依赖 `esp_driver_uart esp_driver_i2c esp_driver_gpio`（T1 P3 技术债清偿）；main 依赖 bsp |
| 构建 | ✓ | 1068/1068 零错误零警告；bin 225,696 B（较 T1 +47KB，UART/I2C 驱动，合理） |
| 范围红线 | ✓ | 未动 sdkconfig.defaults；未碰 TMC 寄存器（T5）、协议（T3）、AS5600 读数（T8） |

非阻塞备注（无需修复）：`bsp_early_init` 不查 `gpio_config` 返回值（GPIO21 无失败路径，可接受）；DIAG 悬空输入为 TMC 未配置前常态，本期不读。

## 上板实测（需求方执行，2026-09-12）

| 验收项 | 结果 |
|---|---|
| `AS5600 detected at 0x36` | ✓ 核心验收通过 |
| **首轮（无内部上拉）即扫到** | ✓ **板上有外部 I2C 上拉 → T0 遗留"上拉有无未知"正式定案**（固件内部上拉兜底路径保留作保险） |
| UART1：TX=10 RX=11 @115200 | ✓ 日志确认 |
| GPIO：EN=21 OUT HIGH（失能）等 | ✓ 日志确认 |
| 固件版本 43dbb8f | ✓ 与分支提交一致，未烧错 |

**新发现（已记入 hardware.md §3）**：板上 GPIO10（UART1 TX）接有 LED，UART 空闲态为高电平 → LED 常亮属正常现象，非故障；电气无害（LED+限流约 2–5mA），不影响后续 CH343P 收发。若该 LED 另有用途，可经 Kconfig 换 UART1 引脚，无需改代码。

## 遗留移交

- UART1 收发回环验收 → 按 PLAN 移交 T3 联测（T2 只验到初始化）
- 硬件事实变更已同步：hardware.md（上拉定案、LED 备注）、PLAN.md（T2 完成 + v1.4）
