# T1 验收报告（工程骨架与构建基线）

> 验收日期：2026-09-12
> 验收对象：分支 `task/T1-skeleton`，提交 `1deb665 T1: project skeleton and build baseline`
> 验收方式：**全程只读**（文件审查 + 构建产物审查 + git 历史审查 + 本机 IDF v6.0.2 源码核对），未修改任何文件
> 依据标准：PLAN.md T1 节验收标准 + 交付给执行模型的 T1 任务包

---

## 总评

**完成度约 85%。** 工程结构、git 工作流、核心 sdkconfig 全部合格，构建产物完整。
存在 1 个必改项（P1：Release 日志开关未真正实现）和若干清理项。
**建议：修复 P1 + 清理 P2 后再合并回 master。** 修复任务包见同目录 `T1-fix-brief.md`。

---

## 一、通过项（有产物证据）

| # | 验收项 | 结论 | 证据 |
|---|---|---|---|
| 1 | `idf.py build` 零错误 | ✓ | `build/PD-step-test.bin`（178512 B）、bootloader、partition-table.bin 齐全；`ninja size` 输出正常；app 分区占用 9%（2M 中剩 91%） |
| 2 | 目录结构符合 PLAN 3.3 节 | ✓ | 10 个组件目录（as5600/bsp/gcode/monitor/planner/protocol/settings/step_engine/sysctl/tmc2209）+ main + tools + docs 全部就位；tools/host_sim.py 正确未实现（T12 范围） |
| 3 | git 工作流 | ✓ | master(T0: 8a7cc97) → `task/T1-skeleton` 单次干净提交；工作区干净；**PLAN.md 与 docs/hardware.md 零改动**（`git diff --stat` 为空）；.gitignore 正确排除 build/、sdkconfig、managed_components 等 |
| 4 | 关键配置实际生效 | ✓ | 解析后 sdkconfig 逐项验证：`IDF_TARGET=esp32s3`、CPU 240MHz、**八线 PSRAM 生效**（`SPIRAM_MODE_OCT=y`、`SPIRAM_SPEED_80M=y`、`SPIRAM_USE_MALLOC=y`）、Flash 16MB 80MHz QIO、tick 1kHz、中断看门狗 300ms 双核、任务看门狗 30s 且双核 idle 已订阅 |
| 5 | partitions.csv | ✓ | nvs 24K @0x9000 + factory 2M @0x10000，app 偏移 64K 对齐正确（避开常见对齐坑）；多出的 phy_init 4K 无害 |
| 6 | 不越界 | ✓ | app_main.c 仅横幅 + 芯片信息 + 喂狗占位任务，无任何 T2+ 功能代码 |
| 7 | Flash 模式 DIO 疑点 | ✓ 排除 | 镜像头 spi_mode=2(DIO) 但配置为 QIO——**经查 IDF v6.0.2 Kconfig 源码确认为设计行为**：ROM/一级 bootloader 不支持直接 QIO，`ESPTOOLPY_FLASHMODE` 字符串统一映射为 "dio"，由二级 bootloader 运行时切换 QIO。非错误 |

## 二、问题清单（按严重度排序）

### P1【必改】Release 日志开关未真正实现

- **位置**：顶层 `CMakeLists.txt`（第 5–10 行）
- **现象**：只设置了 `CMAKE_BUILD_TYPE=Debug` 默认值 + 注释，声称 Release 用 `-DCMAKE_BUILD_TYPE=Release -DESP_LOG_LOCAL_LEVEL=ESP_LOG_NONE`。但 ESP-IDF **不会**根据 CMAKE_BUILD_TYPE 自动关日志，`idf.py -D` 传的是 CMake 变量而非编译宏，注释描述的路径实际不生效。
- **影响**：验收标准"RELEASE 下关闭全部 ESP_LOG 输出"未满足；当前无论怎么配，日志都会编译进固件。
- **修复**：见 `T1-fix-brief.md` P1 节（`add_compile_definitions` + 生成器表达式，含验证方法）。

### P2【应改】sdkconfig.defaults 含 7 处无效/过时符号

构建日志（`build/log/idf_py_stderr_output_28896`）明确报告。**最终生效值经逐项验证全部正确**（结果侥幸正确，因为有效符号同时也在文件里），但死符号会误导后续维护，且其中 1 处实际未生效：

| 行号 | 符号 | 日志报告 | 处置 |
|---|---|---|---|
| 30 | `CONFIG_ESP32S3_SPIRAM_SUPPORT=y` | 被改名替换为 `CONFIG_SPIRAM` | 改写为 `CONFIG_SPIRAM=y` |
| 31 | `CONFIG_ESP32S3_SPIRAM_MODE_OCT=y` | 未知符号 | 删除（正确符号 `CONFIG_SPIRAM_MODE_OCT=y` 已在第 32 行存在） |
| 34–35 | `CONFIG_SPIRAM_BANKSWITCH_ENABLE=y` / `_RESERVE=8` | 未知符号 | 删除（原版 ESP32 专用选项，S3 无此功能） |
| 22 | `CONFIG_FREERTOS_HZ_1000=y` | 未知选择符 | 删除（字符串写法 `CONFIG_FREERTOS_HZ=1000` 第 27 行已生效） |
| 22 | `CONFIG_PARTITION_TABLE_CUSTOM_APP_BIN_OFFSET=0x10000` | 未知符号 | 删除（app 偏移由分区表决定，不受该项控制） |
| 72 | `CONFIG_NEWLIB_NANO_FORMAT=y` | 改名后依赖未满足，**实际未生效** | 删除（v6.0.2 默认 C 库已是 picolibc：`CONFIG_LIBC_PICOLIBC=y`，此选项属于旧 newlib 体系，不再适用） |

### P3【技术债·不阻塞】组件 CMakeLists 占位写法

- 所有组件 `SRCS ""` + `INCLUDE_DIRS "include"`，但各组件 `include/` 目录并不存在（空目录无内容）。T1 占位可接受，T2 写真代码时自然解决。
- 5 个组件 `REQUIRES driver`。本次构建通过（v6.0.2 伞组件仍在），但官方方向是按需依赖精细组件（`esp_driver_uart` / `esp_driver_gpio` / `esp_driver_i2c` / `esp_driver_gptimer`）。**T2 写真代码时顺手替换**，本次不修。

### P4【环境问题·不影响固件】gdbinit 生成失败

- 构建日志有 Python Traceback：`ESP_ROM_ELF_DIR environment variable is not defined`。
- 原因：IDF 装在 `C:/Users/SM/.espressif/v6.0.2`，但 Python 环境取自 `C:/Espressif/tools/...`，两套安装混搭导致 ROM ELF 路径变量缺失。
- 影响：编译/烧录/运行全部正常，仅 GDB 符号调试不可用。**执行模型无需处理**，由需求方择机统一环境或忽略。

### P5【待人工验证】上板项无证据

- 构建日志仅有 cmake/reconfigure/ninja/size 记录，**没有 flash 和 monitor 的痕迹**。
- "烧录打印横幅"与"看门狗空转 10 分钟不复位"两条验收**无法从产物确认**，需需求方插板人工执行。

### INFO（小瑕疵·可修可不修）

- 横幅硬编码 "PSRAM: 8MB"（`app_main.c:52`）——PSRAM 初始化失败时也会打印 8MB。建议 T2 改用 `esp_psram_get_size()` 实报。
- `placeholder_task` 中 `while(1)` 后存在不可达代码（`esp_task_wdt_delete` / `vTaskDelete`）。
- `sdkconfig.defaults` 里 `CONFIG_LOG_MAXIMUM_LEVEL_DEBUG=y` 与 P1 的 Release 编译期裁剪是两层机制，共存无冲突，保留即可。

## 三、处置建议

| 项 | 处置 | 时点 |
|---|---|---|
| P1 | 修复（任务包已备好） | 合并前 |
| P2 | 清理（任务包已备好，与 P1 同一提交） | 合并前 |
| P3 | 不修 | T2 顺手替换 REQUIRES，include 目录随真代码自然出现 |
| P4 | 不处理 | 需求方择机 |
| P5 | 人工验证 | 下次插板时 |
| INFO | 不强求 | T2 顺手 |

修复完成后：`task/T1-skeleton` 合回 master → 放行 T2 / T4 / T7 / T11 四路并行。
