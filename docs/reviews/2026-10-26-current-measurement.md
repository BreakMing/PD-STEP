# UART0 空载电流测量记录

日期：2026-10-26。PCB 已安装到电机后方，尚未接入滑台。板载 MT6835 已安装，
外摆杆编码器未接入。用户已授权修改独立测试工程、烧录测流固件和执行本组空载测量。
当前芯片 USB 标识为 `44:1B:F6:CA:59:F8`，之前的正式固件启动检查见
[重新烧录记录](2026-10-26-firmware-reflash.md)。

## 测量入口与验证范围

测量固件为 `cartpole-current-test`，UART0 经 ST-Link VCP 以 460800 8N1 连接电脑。
固定参数、指令、时序和示波器操作见[硬件测试指南](../../test/README.md)。
本记录区分串口与电机动作验证、探头捕获结果和相电流数值验收。

| 项目 | 结果 |
| --- | --- |
| 宿主时序单元测试 | 命令接受/拒绝、重复开始、阶段边界、延迟处理及各阶段中止均通过。执行文件为 `/tmp/pd-step-current-sequence-test`，命令输出记录为 `current_sequence_test: 命令、阶段边界、重复开始和各阶段中止全部通过`，退出码为 0；复现命令见测试指南。 |
| 测流固件构建 | 会话 `76816`，退出码为 0，二进制大小为 292784 字节。 |
| Unity 测试工程构建 | 会话 `9257`，退出码为 0，原有测试工程仍可构建。 |
| 测流固件烧录 | 会话 `64499`，退出码为 0；bootloader、分区表和应用三个写入散列均通过。 |
| 约 5 V 供电检查 | 会话 `32931`，退出码为 0；`CURRENT INFO`、`STOP` 正常，非法命令被拒绝；`START` 返回 `CURRENT ERROR VBUS EN=1`，没有使能电机。 |
| PD 下三轮空载序列 | 三轮均完成运行、保持和自动禁用，结束后的查询均为 `PHASE=IDLE EN=1`。 |
| 相电流功能检查 | 依据串口记录及用户口述示波器观察，约 9 V 与 20 V 下的空载运行、保持降流和结束禁用归零功能检查通过。 |
| 相电流精度 | 用户报告运行正弦电流峰值约 ±424 mA；按正弦波估算 RMS 约 300 mA，与 300 mA 设定对应的约 306 mA 标称 RMS 接近。此为峰值换算估计，不是仪表直接测得的 RMS；精度仍待直接测量或原始波形复核。 |

构建最初因 ESP-IDF 提前依赖扫描未采用测量模式的条件依赖，缺少 `esp_timer.h` 而停止；
统一声明测试入口依赖后，测流模式和 Unity 模式分别构建成功。
生产驱动和正式固件行为没有因本次测量入口发生修改。

## 两轮 PD 空载运行

用户确认 PD 电源和示波器准备好后，通过 UART0 发出 `CURRENT START`。
每次开始前均读取到有效板载编码器、STATUS 为 0、PD=1 和处于空闲禁用状态。
配置检查记录 GCONF=`000000c4`、CHOPCONF=`15030045`，
两次启动的 IFCNT 分别从 6 增至 8、从 8 增至 10。

| 观测 | 第 1 轮 | 第 2 轮 |
| --- | --- | --- |
| 执行会话与退出码 | `71274`，0 | `28172`，0 |
| 固件 VBUS 回读范围 | 8.878～8.920 V | 8.878～8.920 V |
| 固件累计运行与保持时长 | 6003 ms | 6000 ms |
| 指令步数 | 1990 | 1991 |
| 编码器累计变化 | 2615968 counts | 2609440 counts |
| 运行阶段 CS_ACTUAL | 9 | 9 |
| 保持阶段 CS_ACTUAL | 约 1 秒时为 6；结束后查询为 4。 | 约 1 秒时为 6，结束前已为 4。 |
| 结束后的使能电平 | EN=1 | EN=1 |
| 健康检查 | 没有 `CURRENT ERROR`，没有温度或短路触发。 | 没有 `CURRENT ERROR`，没有温度或短路触发。 |
| 示波器采集 | 用户反馈没有捕获。 | 用户报告约 ±400 mA、15 Hz 正弦波，随后约 −300 mA，再依次为 −280、−260、−230、−200、−160 mA；测试结束后约 0 mA。 |

保持阶段的档位变化体现了驱动的静止延时和降流过程，不能把进入 `HOLD` 的瞬间
当作保持电流已经稳定。实际电流仍以仪表波形为依据。
编码器变化证明轴发生转动；本次没有据此验收无失步精度、带载能力或滑台行程。

[第 1 轮逐行记录](evidence/2026-10-26-current-measurement/pd-run-1.json)与
[第 2 轮逐行记录](evidence/2026-10-26-current-measurement/pd-run-2.json)
包含主机收发时间、配置、编码器、驱动档位和阶段转换。
时间戳来自主机接收时刻，实际阶段累计时长由 `ELAPSED_MS` 字段提供。

## 示波器口述结果与解释

用户在 9 V 第 3 轮后报告约 ±400 mA、15 Hz 的波形；由于这是口述峰值、没有 RMS 直接测量值，
当时仅记录为约 283 mA 的正弦估算，尚未获取截图或原始波形。
用户在 20 V 第二轮后说明，约 400 mA 是正弦运行结束后的静止保持电流，持续约 650 ms，并非 RMS。运行正弦的峰值报告为 ±424 mA；后续保持阶段的电流与持续时间见 20 V 记录。

用户在 9 V 第 2 轮后报告以下观察，原文保存在
[示波器观察记录](evidence/2026-10-26-current-measurement/scope-observation.json)。
当时没有取得截图、波形文件或直接计算的运行 RMS，因此以下正弦 RMS 是估算值，不作为精度验收结果。

| 观察 | 对照依据与判断 |
| --- | --- |
| 约 ±400 mA 正弦波 | 按上下峰值为 +400/−400 mA 理解，峰峰值约 800 mA。若近似理想正弦，RMS 估计为 `400/√2≈283 mA`；这不是示波器直接测得的 RMS。 |
| 频率约 15 Hz | 当前 STEP 目标 500 steps/s、8 细分，每个电流周期对应 4 个整步，预期频率为 `500/(8×4)=15.625 Hz`，与口述近似读数一致。 |
| 停转后约 −300 mA | 停止 STEP 后驱动仍使能，绕组保持当前电角度的电流，因此单相可能停留在负值。负值本身不表示异常；具体符号还取决于被测相和探头方向。 |
| 负向电流阶梯降流 | 用户确认波形依次为约 −280、−260、−230、−200、−160 mA，均在零线下方；幅值逐级下降与固件设置的静止延时及分档降流相符。 |
| 结束后约 0 mA | 用户确认电流回到零附近，与串口完成记录及结束查询中的 EN=1 一致，支持禁用后绕组电流归零。 |

TMC2209 的 `TPOWERDOWN` 定义静止后的降流等待时间，`IHOLDDELAY` 定义各次电流档位下降的延时，
定义见 [TMC2209 手册 §5.2](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf)。
串口记录中的运行档位 9、保持档位 4 对应电流比例 `(4+1)/(9+1)=0.5`；
口述停止前后幅值比约为 `160/300≈0.53`，支持降流功能已经生效，但不能代替精度测量。

按手册 §9 公式、采样电阻 110 mΩ、VSENSE=1 和运行档位 9 计算，
标称运行 RMS 约 306 mA、理想正弦峰值约 433 mA。
口述 ±400 mA 的粗略读数接近这一量级，目前不据此调整驱动电流换算。
保持设定 150 mA 是驱动的等效正弦 RMS 标度；轴静止在某个电角度时，
单根绕组的直流电流由该相位决定，不能直接把口述 160 mA 与 150 mA 作精度比较。

## 第 3 轮 9 V 复测

会话 `28385` 退出码为 0，运行与保持累计 6001 ms、1991 steps，板载编码器累计变化
2606867 counts，结束后为 `PHASE=IDLE EN=1`，VBUS 回读约 8.920 V。
运行档位为 9；保持阶段约 1 秒时为 6，约 2 秒时为 4。
逐行证据见[第 3 轮串口记录](evidence/2026-10-26-current-measurement/pd-run-3.json)。

用户报告“测量到了峰峰值为+-424mA的电流”。按正负峰值为 ±424 mA 理解，
峰峰值约 848 mA，若稳定段近似正弦，则估算 RMS 为 `424/√2≈300 mA`，
与 300 mA 设定相符。该值来自峰值换算，不是示波器直接测得的 RMS，
也未用于宣告电流精度通过。口述与解释分别记录在
[第 3 轮示波器记录](evidence/2026-10-26-current-measurement/scope-observation-3.json)。

## 20 V 调档前的空闲基线

当前测流固件下连续执行 10 次 `CURRENT INFO`，每次均报告 `PHASE=IDLE EN=1 READY=1`、
板载编码器 `OK=1 STATUS=0`、CH224Q `PD=1`，且没有 `CURRENT ERROR`。
VBUS 回读范围为 8.887～8.920 V。该检查没有发送运动或调档命令，
结果保存在[9 V 空闲基线](evidence/2026-10-26-current-measurement/9v-idle-baseline.json)。

## PD 切档测试固件

用户授权增加 `CURRENT PD20`、`CURRENT PD9` 并烧录测试固件，要求执行前说明后续操作。
两条命令已加入独立测流入口，具体行为与失败恢复见
[PD 调档契约](../../test/README.md#pd-调档契约)。请求电压、握手状态和实际电压分别记录；
切档期间电机保持禁用。等待失败或被中止后，不允许直接启动电机，须先重新确认电压档位。

| 检查 | 结果与证据 |
| --- | --- |
| 调档状态单元测试 | 准入、至少 3 秒等待、采样最小间隔、连续有效采样、电压边界、超时、中止及 9 V 重试恢复通过，退出码为 0；[输出](evidence/2026-10-26-current-measurement/pd-control/pd-unit-output.txt)。 |
| 测流命令与时序单元测试 | 新增调档命令接受、非法档位拒绝，原有运行/保持/停止边界通过，退出码为 0；[输出](evidence/2026-10-26-current-measurement/pd-control/sequence-unit-output.txt)。 |
| 切档版测流固件构建 | 会话 `26826`，退出码为 0，应用 294976 字节；[构建输出](evidence/2026-10-26-current-measurement/pd-control/build-output.txt)及[产物散列](evidence/2026-10-26-current-measurement/pd-control/firmware.json)。 |
| 默认 Unity 模式构建 | 会话 `84567`，退出码为 0；[构建输出](evidence/2026-10-26-current-measurement/pd-control/unity-build-output.txt)。 |
| 切档版烧录 | 会话 `24964`，退出码为 0，三个写入区域的散列均通过；[烧录输出](evidence/2026-10-26-current-measurement/pd-control/flash-output.txt)。启动日志 ELF SHA256 前缀 `afc14cf8d` 与产物一致。 |
| 电脑 USB 供电拒绝检查 | 退出码为 0；约 4.960～4.977 V、`PD=0` 下，PD20/PD9 返回 `PD_HANDSHAKE`，PD28 返回 `COMMAND`，START 返回 `VBUS`；均保持 EN=1；[完整记录](evidence/2026-10-26-current-measurement/pd-control/usb-safety-check.json)。 |
| 20 V 空闲调档 | 会话 `44757`，退出码为 0；从 8.920 V 请求 20 V，3409 ms 后确认，完成后五次查询为 20.060～20.085 V；始终 EN=1，等待期间 INFO 可响应；[逐行记录](evidence/2026-10-26-current-measurement/pd-control/pd20-idle-check.json)。 |
| 外部电压确认 | 用户在 20 V 档口述实测 VBUS=20 V、逻辑电源=3.32 V；[原文记录](evidence/2026-10-26-current-measurement/pd-control/external-voltage.json)。 |
| 20 V 空载电机序列 | 两轮均完成 4 秒运行、2 秒保持与自动禁用；第 2 轮示波器读数见下文。 |
| 20 V 返回 9 V | 会话 `17375`，退出码为 0；请求后 3418 ms 完成验证，完成后五次空闲查询为 8.887～8.937 V，PD=1、EN=1；[逐行记录](evidence/2026-10-26-current-measurement/pd-control/pd9-idle-check.json)及[摘要](evidence/2026-10-26-current-measurement/pd-control/pd9-idle-summary.json)。 |
| 等待期间 STOP 恢复 | 会话 `94120`，退出码为 0；PD20 等待期间发 STOP 后，状态为 UNCERTAIN，START 返回 `PD_UNVERIFIED EN=1`；重新请求 PD9，3407 ms 后确认并完成五次 9 V 回读；[逐行记录](evidence/2026-10-26-current-measurement/pd-control/pd-interrupt-recovery.json)及[摘要](evidence/2026-10-26-current-measurement/pd-control/pd-interrupt-recovery-summary.json)。 |

烧录后的首次查询 `CURRENT INFO` 收到 `CURRENT ERROR COMMAND EN=1`，会话 `24673`
因等待 INFO_END 超时而退出，退出码为 1；[首次失败记录](evidence/2026-10-26-current-measurement/pd-control/usb-first-query.json)
已保留。随后采集启动日志并连续查询三次均成功，会话 `75810` 退出码为 0，见
[启动及串口诊断](evidence/2026-10-26-current-measurement/pd-control/uart-after-flash-diagnostic.json)。
原检查脚本再次执行通过，没有修改固件或放宽断言。首次异常的具体原因尚未确定，
不能据此宣布 UART0 完整可靠性验收通过。

用户改接 PD 后，完成 EN 为高的 20 V 调档、外部电压确认、两轮空载序列，
并完成 20 V 降回 9 V 的空闲调档。结果支持当前电源与本板在空闲时的 9 V、20 V 请求和回读功能。
等待期间 STOP 中止恢复也已实板验证：START 被锁定，重新验证 9 V 后恢复。
外部仪表型号、截图及电压波形未提供，不据此评定调档瞬态或电压测量精度。

## 20 V 空载序列

用户确认电源实测值与示波器就绪后，分别执行两轮相同的 300/150 mA、4 秒运行加 2 秒保持序列。
两轮均读取到有效板载编码器，运行档位为 9，保持阶段依次读取到 6 和 4；
没有出现 CURRENT ERROR，结束后均为 `PHASE=IDLE EN=1`。首轮因示波器时间轴设置有误没有保留可读数波形；
用户调整时间轴后重做第二轮，报告如下。

| 观测 | 第 1 轮 | 第 2 轮 |
| --- | --- | --- |
| 执行会话与退出码 | `46548`，0 | `42021`，0 |
| 查询 VBUS 范围 | 20.060～20.085 V | 20.077～20.144 V |
| 累计运行与保持时间 | 6013 ms | 6010 ms |
| 指令步数 | 1993 | 1991 |
| 编码器累计变化 | 2633121 counts | 2608412 counts |
| 示波器采集 | 用户反馈时间轴设置有误，未取得可读波形。 | 用户调整为 1 秒/格并确认就绪后重测：运行正弦峰值约 ±424 mA；正弦结束后静止电流约 400 mA，持续约 650 ms；随后约 350、320、280、240 mA 各持续 130 ms，约 200 mA 持续 830 ms，结束后回到约 0 mA。 |

[第 1 轮串口记录](evidence/2026-10-26-current-measurement/pd-control/pd20-run-1.json)、
[第 1 轮采集条件](evidence/2026-10-26-current-measurement/pd-control/scope-capture-1.json)和
[第 2 轮串口记录](evidence/2026-10-26-current-measurement/pd-control/pd20-run-2.json)
分别保留证据。第 2 轮示波器口述及解释见[20 V 示波器记录](evidence/2026-10-26-current-measurement/pd-control/scope-observation-pd20-run-2.json)。

运行设定为 300 mA。按 110 mΩ 采样电阻、VSENSE=1 和 IRUN 档位 9，标称运行 RMS 约 306 mA。
用户报告运行正弦波峰值约为 +424/−424 mA；按正弦波换算，RMS 估计约 `424/√2≈300 mA`，
与标称值相差约 −2%。这是峰值换算估计，并非仪表直接测得 RMS，也没有截图或原始波形用于复核，
因此记录为量级相符，电流精度尚未按直接 RMS 测量验收。

用户澄清约 400 mA 是正弦运行结束后的静止保持电流，约持续 650 ms，不是 RMS。
随后观察到约 350、320、280、240 mA，各持续约 130 ms，再降至约 200 mA 并持续约 830 ms，
结束禁用后回到约 0 mA。五个保持电流阶段的报告时长合计 `650+4×130+830=2000 ms`，
与固件设定的 2 秒保持窗口一致。TMC2209 的 `IHOLDDELAY=6` 与逐级降流行为相符；
这些示波器时间为用户口述近似值，不能据此推断每一档的精确边界或保持电流精度。
完整用户口述及分段记录见[20 V 示波器记录](evidence/2026-10-26-current-measurement/pd-control/scope-observation-pd20-run-2.json)。

## 当前状态与待回填

当前运行含 PD20/PD9 指令的测流测试固件，USB-C 接 PD 电源，目标状态已验证为 9 V，
UART0 保持连接，电机空闲且 EN=1。20 V 升档、外部 VBUS/3.3 V 检查、两轮运行序列、
降回 9 V 和 STOP 中止恢复检查均已完成。

第 2 轮运行正弦的峰值约为 ±424 mA，换算 RMS 约 300 mA，与约 306 mA 标称值接近；
该值是估算，不是仪表直接测得的 RMS。正弦结束后用户观测到约 400 mA 静止电流，
随后依次降至约 350、320、280、240、200 mA，最终禁用后回到约 0 mA。
约 9 V 和 20 V 下，300/150 mA 设定的运行、保持降流和禁用归零功能检查通过；
相电流精度仍待稳定运行阶段直接测得 RMS 或可追踪原始波形。

未接滑台和外摆杆，因此实际行程标定、软限位、带载性能及完整闭环验收仍留待对应硬件接入。

## 证据

- [烧录输出](evidence/2026-10-26-current-measurement/flash-output.txt)
- [USB 供电下安全检查](evidence/2026-10-26-current-measurement/usb-safety-check.json)
- [固件大小与散列](evidence/2026-10-26-current-measurement/firmware.json)
- [用户口述示波器观察](evidence/2026-10-26-current-measurement/scope-observation.json)
