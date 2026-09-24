# PD-STEP · ESP32-S3 单轴 G-code 运动控制器

基于 ESP32-S3 + TMC2209 + AS5600 的单轴步进运动控制器。上位机通过串口流式下发 G-code 子集，固件完成梯形加减速运动规划、精确步数控制与编码器位置遥测。协议风格兼容 GRBL 子集，便于用现成上位机软件联调。

## 硬件配置

| 模块 | 器件 | 职责 |
|---|---|---|
| 主控 | ESP32-S3-WROOM-1-N16R8 | 16MB Flash + 8MB PSRAM |
| 驱动 | TMC2209 ×1 | 步进电机驱动，单线 UART 配置 + STEP/DIR 控制 |
| 反馈 | AS5600 ×1（电机尾部直装） | 12bit 磁编码器，I2C，多圈计圈 → mm 遥测 |
| 供电 | CH224K PD 诱骗模块 | 从 USB-PD 取 12V 供电机驱动 |
| 烧录 | CH340K 板载串口 | 仅固件烧录与日志（UART0） |
| 上位机通道 | 飞线 USB 转串口 → UART1（GPIO41/42） | 协议通道，与烧录口完全隔离 |

## 功能

- **G-code 子集**：`G0`/`G1`（X 目标位置，F 速度 / A 加速度均为模态）、`G92` 建零（坐标偏移，同时作用于指令位置与编码器上报位置）
- **梯形加减速**点到点运动，位置以步数精确控制
- **实时命令**：`?` 状态查询（任意时刻、行中可打断）、`!` 立即停止（丢队列即停）
- **遥测**：运动中周期上报状态帧 `<STATE|C:x|E:x|V:x|T:flags>`（默认 10Hz），C=指令执行位置，E=编码器实测位置
- **参数持久化**：`$0`–`$13` 共 14 项参数（细分、电流、传动比、速度/加速度上限等）NVS 存储、断电保持、运行时可改；`$$`/`M503` 查看，`M502` 恢复默认
- **软限位**：默认 0–600mm，越界返回 `error:2`
- **流控**：逐行 `ok` 应答，队列满时自然背压

## 协议要点

- UART1，115200-8N1，行结尾 `\r\n`
- 错误码：`error:1` 未知命令 / `error:2` 越界 / `error:3` 行超时（100ms）/ `error:4` 参数错误 / `error:5` 队列或存储异常
- 空行与 `;` 注释行返回 `ok`；括号注释 `(...)` 在数值位合法

## 工程结构

```
components/
  bsp/          引脚与外设初始化
  protocol/     UART1 驱动、行组装、实时命令、应答
  gcode_parser/ G-code / $ 命令解析
  planner/      梯形运动规划、软限位、mm→steps 换算
  step_engine/  定时器 ISR 步脉冲发生、位置计数
  tmc2209/      驱动器寄存器配置与状态读取
  as5600/       磁编码器驱动、多圈跟踪
  monitor/      遥测状态帧组帧
  sysctl/       系统状态机（BOOT→IDLE↔RUN）、立即停
  settings/     NVS 参数表、订阅分发、自测
main/           应用入口与测试载体
tools/          上位机测试脚本（host_sim.py 端到端验收等）
docs/           硬件定案、验收记录（reviews/）、开发计划 PLAN.md
```

## 构建与烧录

开发环境：ESP-IDF v6.0.2（原生，非 Arduino）。

```powershell
idf.py build          # Debug 构建
idf.py -B build flash monitor
```

- Release 构建：`idf.py -B build_release -D SDKCONFIG=sdkconfig.release build`（裁剪后体积 -29.5%）
- 修改 Kconfig（如引脚）后需删除 `sdkconfig` 重新配置
- 烧录用 UART0；上电自检日志镜像、自测固件等由 Kconfig 开关门控，正常固件零残留

## 测试与验收

- 每组件 `test/` 目录含单元测试（编入固件 + Kconfig 开关，上板真实执行）
- `tools/host_sim.py`：纯上位机端到端验收脚本（流控、1000 段流式、失步判据、立即停、模态、G92、断电持久性、遥测）
- 开发任务 T0–T12 全部闭环，里程碑 M1–M5（通信闭环 / 电机受控 / G-code 运动 / 位置可信 / 整机交付）全部达成，验收记录见 `docs/reviews/` 与 `docs/acceptance.md`

## 文档

- [PLAN.md](PLAN.md) — 开发计划与协议规范（含全部设计决策记录）
- [docs/hardware.md](docs/hardware.md) — 硬件定案与引脚表
- [docs/acceptance.md](docs/acceptance.md) — 整机验收数据
