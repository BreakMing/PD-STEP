# 硬件测试工程

本目录提供两种入口，均与仓库根目录的正式固件分别构建。硬件参数以
[硬件清单](../docs/hardware.md)为准。

| 构建选项 | 产物 | 交互入口 |
| --- | --- | --- |
| `CURRENT_MEASUREMENT=OFF`（默认） | `cartpole-test` | 原生 USB 的 Unity 菜单；用例见各组件 `test/`。 |
| `CURRENT_MEASUREMENT=ON` | `cartpole-current-test` | UART0 的固定空载测流指令。 |

## Unity 测试

在仓库根目录加载 ESP-IDF 环境后执行：

```bash
idf.py -C test -B "$PWD/build/acceptance-tests" -DCURRENT_MEASUREMENT=OFF build
```

进入菜单后按用例名称或序号选择。`[spin]`、`[flowtest]` 会使能电机；
实际运行需要可自由转动的电机轴及相应接线条件。

## UART0 空载电流测量

此入口适用于 PCB 已安装到电机、尚未接入滑台的阶段。它复用 `cartproto` 的 UART0
行收发载体，使用 `CURRENT` 前缀的诊断指令。测量入口仅初始化板载编码器，
不启动正式模式机、200 Hz `STATE` 任务和 Unity 菜单。

在仓库根目录构建，配置与产物保留在专用目录：

```bash
source /opt/esp/idf/export.sh
current_test_build="$PWD/build/current-measurement"
idf.py -C test -B "$current_test_build" \
  -DIDF_TARGET=esp32s3 -DSDKCONFIG="$current_test_build/sdkconfig" \
  -DCURRENT_MEASUREMENT=ON build
```

构建配置、定时边界和固定电流参数以
[current_sequence.h](main/current_sequence.h)为准。
Flash 参数由构建目录的 `flash_args` 生成。烧录前核对当前 ESP32 的 USB 标识，
通过原生 USB 写入该目录中的产物。复位后从 UART0（GPIO43 TX、GPIO44 RX，
460800 8N1，3.3 V TTL，共地）交互；USB-C 可以改接 PD 电源。

### 指令与状态

每个指令以换行结束；命令完整匹配、大写，串口接收层兼容 CRLF。

| 指令 | 行为 |
| --- | --- |
| `CURRENT INFO` | 读取阶段、EN 电平、VBUS、编码器、TMC 寄存器和供电协议状态；最后输出 `CURRENT INFO_END`。 |
| `CURRENT START` | 空闲且检查通过后，运行约 4 秒、保持约 2 秒，随后关闭使能；最终输出 `CURRENT DONE`。 |
| `CURRENT PD20` / `CURRENT PD9` | 空闲且 EN 为高、PD 握手有效时，请求 20 V / 9 V 并验证母线电压；不使能电机。 |
| `CURRENT STOP` | 优先于已排队请求处理，立即停脉冲、关闭使能并回到空闲；输出 `CURRENT STOPPED`。 |

| 阶段 | EN 电平与脉冲 | 转换条件 |
| --- | --- | --- |
| `IDLE` | EN 为高、无脉冲。 | 只在接受明确的 `CURRENT START` 后进入 `RUN`。 |
| `RUN` | EN 为低，目标为 500 steps/s；启动时按 10000 steps/s² 加速。 | 开始后约 4 秒停止脉冲，进入 `HOLD`。 |
| `HOLD` | EN 为低，无脉冲；驱动按自身静止延时切换保持电流。 | 开始后累计约 6 秒关闭使能，回到 `IDLE`。 |

运行电流设定为 300 mA，保持电流设定为 150 mA，细分为 8。
停止、自动结束和硬件检查失败均会清理待执行请求，不会自动开始下一轮。
重复启动返回 `CURRENT ERROR BUSY`，不延长原来的时间窗口。

`START` 前要求电压回读处于 8.0～21.5 V、板载编码器有效且 STATUS 为 0、
TMC 没有温度或短路告警。驱动配置还会核对 IOIN 版本、GCONF、CHOPCONF，
并使用 IFCNT 增量确认配置写帧被器件接收。电源使用当前协商档位。
运行及保持期间每约 100 ms 检查上述电压、编码器与驱动健康状态，失败即关闭使能。
步进引擎启用 100 ms 命令看门狗，测量任务在运行阶段每 25 ms 更新目标。

电脑 USB 约 5 V 时，`CURRENT START` 应返回 `CURRENT ERROR VBUS EN=1`。
这项检查用于验证供电条件拒绝分支，不能算作电机实转验收。

### PD 调档契约

切档复用 CH224Q 驱动。`PD=1` 只表示协议握手有效，不能单独证明电压档位。
时间、采样间隔与稳定次数以 [current_pd.h](main/current_pd.h) 为准：写入请求后至少等待
3 秒，再以不小于 100 ms 的间隔取得连续 5 次 PD 有效且 VBUS 位于目标 ±5% 的读数，
才输出 `CURRENT PD_DONE`；6 秒仍未满足条件则输出超时。`PD_REQUEST` 仅表示写入完成。

| 条件 | 结果 |
| --- | --- |
| 运行、保持或另一轮切档尚未结束 | 拒绝调档，报告 `BUSY`，不写电压寄存器。 |
| 电机空闲但实际 EN 不为高 | 关闭电机并报告 `ENABLE`，不写电压寄存器。 |
| PD 状态读取失败、握手无效或初始 VBUS 不在运行范围 | 报告对应错误，不写电压寄存器。 |
| 前置条件满足 | 清理排队请求、请求目标电压，进入 `PD_WAIT`；期间维持 EN 为高。 |
| 请求写入失败、验证超时或等待期间收到 STOP | 保持禁用，电压档位视为未确认；后续 START 返回 `PD_UNVERIFIED`，允许重新请求并确认 9 V 或 20 V。 |
| 连续稳定采样成功 | 回到空闲，报告目标、实测 VBUS、PD 与 EN；不会自动启动电机。 |

切档期间 `INFO` 和 `STOP` 仍可处理。`STOP` 不撤销已经发出的电压请求，
也不会自动回到 9 V。每次请求开始、完成或失败都会清理排队请求；启动和调档
必须分别发送并等待响应。`INFO` 中的 `TARGET_V` 是请求记录，实测电压以 `VBUS_MV` 为准。
USB-C 接电脑供电时，PD 握手无效会拒绝调档。实板首次升至 20 V 后，先在 EN 为高时
用仪表确认 VBUS 和 3.3 V 电源，再单独开始运动测量。

### 示波器采集

Micsig CP1003B 当前采用 100 mV/A 档位。
[探头手册](https://evision-webshop.eu/media/e1/a6/a4/1791281195/CP%20Series%20-%20User%20Manual.pdf)
的“连接”和“功能检查”章节要求
示波器输入为 1 MΩ、直流耦合；精确测量前预热至少 20 分钟，空夹口闭合锁定后消磁调零，
再夹住一根电机导线并锁定。探头范围切换后须同步示波器中的电流换算设置。

保留原始电压显示时，可先采用 20 mV/格、1 秒/格，设置上升沿单次触发，
记录运行、保持和禁用的完整过程；触发电平可从 10 mV 起调整。
若示波器以电流显示，100 mV/A 对应 10 A/V。

运行电流的 RMS 应在稳定运行阶段的完整电流周期上测量，跳过启动加速、保持和禁用段。
按 [TMC2209 手册 §9](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf)
的电流公式，使用 110 mΩ 采样电阻、VSENSE=1 和 IRUN 档位 9，标称运行有效值约 306 mA，
对应探头输出有效值约 30.6 mV、理想正弦峰值约 43.3 mV；实际值仍需仪表测量。
保持阶段是一相固定位置的电流，不能把单相静止读数直接当作 150 mA RMS。
可以记录同一固定相位降流前后的电流比例，并结合 `MSCURACT`、`MSCNT`、
`CS_ACTUAL` 和实际波形判断。

只有实测波形和读数才能用于相电流验收；驱动寄存器、时序完成和编码器转动均为独立检查。

## 宿主时序测试

在仓库根目录执行以下单元测试，不访问硬件：

```bash
gcc -std=c11 -Wall -Wextra -Werror -Itest/main \
  host_tests/current_sequence_test.c test/main/current_sequence.c \
  -o /tmp/pd-step-current-sequence-test
/tmp/pd-step-current-sequence-test
```

该测试覆盖命令接受与拒绝、阶段边界、重复启动、延迟处理和各阶段停止。
实板电机测量另行执行并保留串口与示波器证据。

PD 验证状态的单元测试：

```bash
gcc -std=c11 -Wall -Wextra -Werror -Itest/main \
  host_tests/current_pd_test.c test/main/current_pd.c \
  -o /tmp/pd-step-current-pd-test
/tmp/pd-step-current-pd-test
```

该测试验证调档准入、等待时限、电压边界、连续采样、失败后禁止启动以及重试恢复。
