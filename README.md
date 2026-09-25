# cartpole 固件 · ESP32-S3 倒立摆小车控制器

cartpole 实验台的运动端固件：ESP32-S3 + TMC2209 步进驱动 +
AS5600 小车位置编码器 + MT6835 摆角编码器。上端（Hi3516CV610 或
PC 联调机）通过串口以 **protocol.md 线协议** 驱动：200 Hz 状态流、
`VEL` 速度模式跟踪、手动两点标定（`MARK LEFT`/`MARK RIGHT`/`ENABLE`）、
三态模式机与参数注册表。

**线协议唯一权威：本仓库之外的项目主仓库
`deploy/v2/protocol.md`**（命令词法、模式转换表、参数注册表、
符号约定、时序）。

## 硬件

| 模块 | 器件 | 引脚 | 职责 |
|---|---|---|---|
| 主控 | ESP32-S3-WROOM-1-N16R8 | — | 16MB Flash + 8MB PSRAM，240 MHz |
| 步进驱动 | TMC2209 | EN=21（低有效）、STEP=5、DIR=6、UART TX=17/RX=18 | 42 步进电机驱动，单线 UART 配置 |
| 小车编码器 | AS5600（电机尾部直装） | I2C SDA=8 / SCL=9 @400 kHz | 12 bit 多圈 → x_um 遥测与丢步判定 |
| 摆角编码器 | MT6835（21 bit） | SPI MISO=46、MOSI=40、SCK=39、CS=7 | ±π µrad 摆角与角速度遥测 |
| 主机通道 | UART1 | TX=41 / RX=42 @460800 8N1 | protocol.md 线协议（经 CH343P USB） |
| 供电 | CH224K PD | — | USB-PD 12V 供电机 |
| 烧录/日志 | UART0 | 板载 CH340K | 烧录与 ESP_LOG 控制台 |

机械：2GT 20T 同步带，40 mm/圈；细分 8（settings `$0`，NVS
namespace `cart`）→ 1600 steps/圈 = 40 steps/mm = 25 µm/step。

## 工程结构

```
components/
  bsp/          引脚与外设初始化（UART1/GPIO/I2C/SPI）
  cartproto/    线协议载体：行组装、命令解析（纯函数）、MODE/OK/ERR 应答
  cartfsm/      三态模式机：纯核心（转换表/标定合法性/软限位窗口/丢步）
                + 参数注册表 + 1 kHz 监督任务（标定状态/编码器失联/
                丢步/软限位/FAULT 次序）
  step_engine/  GPTimer ISR 速度模式：vel_ramp 每步递推（纯函数）+
                带符号目标 + 命令看门狗 + 换向经零速
  as5600/       小车编码器 1 kHz 采样（多圈计数 + 16 ms 速度窗）
  mt6835/       摆角编码器 1 kHz SPI 采样（帧格式抄录见
                docs/reviews/T4-brief.md）+ ±π 环绕换算（纯函数）
  telemetry/    200 Hz STATE 组帧（纯函数 + 常开任务）
  settings/     NVS 机械参数（$0–$4：细分/整步/每转行程/电流）
  tmc2209/      驱动配置（电流/细分）
main/app_main.c 启动接线（含 Kconfig 门控自测入口）
host_tests/     宿主可跑的纯逻辑测试（gcc，无 ESP 依赖）
tools/          cartpole_host.py 联调工具、自测构建片段
docs/reviews/   抄录与上板验收记录
```

任务/CPU 布局：CPU0 = cartproto 通讯（prio 5）+ STEP ISR；
CPU1 = as5600/mt6835 采样（prio 3）、cartfsm 监督（prio 4）、
telemetry 组帧（prio 2）。全部订阅任务看门狗。

## 构建

需要 ESP-IDF v6.1（`C:\esp\v6.1\esp-idf`，PowerShell）：

```powershell
. C:\esp\v6.1\esp-idf\export.ps1
idf.py -B build build                    # 正式固件
idf.py -B build flash monitor            # 烧录 + 日志
```

自测构建（Kconfig 门控，见各组件 Kconfig）：

```powershell
idf.py -B build-selftest -DSDKCONFIG=build-selftest/sdkconfig `
      -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/selftest.defaults" build
idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig `
      -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" build
```

- **selftest**：各组件向量自测（不驱动电机），上电自运行，
  每组件结束行 `SELF-TEST DONE: <n>/<total> PASS`。
- **flowtest**：在 selftest 之上加 cartfsm 注入式流程自测
  （会开 EN；有 12V 时小车会真实运动）。

宿主纯逻辑测试（无需硬件）：

```
wsl gcc -std=c11 -Wall -Wextra -Werror \
  -Ihost_tests/stub -Icomponents/<c>/include ... host_tests/<t>.c ...
```

## 联调

`tools/cartpole_host.py`（`pip install pyserial`）：

```
python tools/cartpole_host.py --port COM7 query|mark-left|mark-right|enable|vel|config|dump|state|wdt-test|console
```

上板验收按主仓库 `deploy/v2/onboard-checklist.md`（C 阶段，两点
标定规程）与 `docs/reviews/T10-acceptance.md`（端到端五用例）
执行并回填。
