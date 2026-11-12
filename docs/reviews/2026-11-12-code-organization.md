# 测试代码组织与构建检查

日期：2026-11-12。整理基于 `test-refactor` 分支的 `fc8eb4e`。
用户授权整理代码并提交 Git；本次只运行软件检查，没有烧录、调档或启动电机。

## 整理范围

空载测流入口、运行时序和 PD 验证逻辑统一放在
[test/main/current_measurement](../../test/main/current_measurement/)：
`app_main.c` 负责硬件与指令分发，`current_sequence.c/.h` 负责运行时序，
`current_pd.c/.h` 负责调档状态验证。构建入口和宿主测试命令见
[硬件测试指南](../../test/README.md)。

五个文件迁移后的内容与原提交逐字节一致，参数、指令和失败恢复逻辑均沿用原实现。
CMake 按原有构建选项选择新路径；默认 Unity 菜单仍由 `test_app_main.c` 提供。
[迁移文件及内容散列](evidence/2026-11-12-code-organization/source-moves.json)记录对应关系。

BSP 的 I2C 扫描日志只报告 ESP32-S3 内部上拉是否启用及地址应答情况，
不从扫描应答推断板上上拉来源。扫描、总线重建和重试顺序沿用原实现。
CH224Q 驱动、用例与启动代码中的注释统一引用[硬件清单](../hardware.md)，
默认请求、协议握手和实际 VBUS 分别说明。

文档增加[验证记录索引](README.md)，纳入本次软件检查和
[板载 MT6835 重新烧录记录](2026-11-12-mt6835-reflash.md)。
测流记录中的波形所属轮次和六段保持电流计数已按现有证据核对。
原始终端日志保持采集格式，未重新排版。

## 检查结果

| 检查 | 结果与证据 |
| --- | --- |
| 运行时序宿主单元测试 | 整理前后均通过，退出码均为 0；[整理前](evidence/2026-11-12-code-organization/sequence-before.txt)、[整理后](evidence/2026-11-12-code-organization/sequence-after.txt)。 |
| PD 状态宿主单元测试 | 整理前后均通过，退出码均为 0；[整理前](evidence/2026-11-12-code-organization/pd-before.txt)、[整理后](evidence/2026-11-12-code-organization/pd-after.txt)。 |
| 正式固件构建 | 退出码为 0；[构建输出](evidence/2026-11-12-code-organization/firmware-build.txt)。 |
| 测流模式构建 | `CURRENT_MEASUREMENT=ON`，退出码为 0；[构建输出](evidence/2026-11-12-code-organization/current-measurement-build.txt)。 |
| Unity 模式构建 | `CURRENT_MEASUREMENT=OFF`，退出码为 0；[构建输出](evidence/2026-11-12-code-organization/unity-build.txt)。 |
| 构建入口核对 | 根据三份真实 `compile_commands.json` 核对：正式工程不编入测试入口，测流模式只编入迁移后的三个源码文件，Unity 模式只选择原菜单入口；[结果](evidence/2026-11-12-code-organization/entry-selection.json)。 |

三项构建按顺序执行，会话为 `55416`，退出码为 0；
[构建命令与退出码](evidence/2026-11-12-code-organization/build-results.json)和
[产物大小与散列](evidence/2026-11-12-code-organization/artifacts.json)保留具体结果。
软件构建与目录检查不替代实板电机、动态编码器或 PD 电源验收。
板上仍运行整理前已经烧录的正式固件，本次新产物均未写入硬件。
