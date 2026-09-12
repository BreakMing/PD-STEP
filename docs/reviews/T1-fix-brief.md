# T1-fix 任务包：修复验收 P1 + 清理 P2

> 用途：直接交给执行模型（Claude Code）在 `task/T1-skeleton` 分支上完成 T1 收尾。
> 背景：T1 已验收（见同目录 `T1-acceptance.md`），完成度 85%，本任务包处理合并前的
> 2 个待修项。**只做 P1 和 P2，其他项（P3/P4/P5/INFO）明确不做。**
> 修改完成后将结果提交在本分支，不合并 master。

---

以下为交给执行模型的提示词正文：

`````
# 任务：T1-fix · 修复验收问题（仅 2 项）并提交

## 背景
仓库是 ESP32-S3 单轴运动控制器固件。T1（工程骨架与构建基线）已在分支
task/T1-skeleton 上完成并验收，验收结论是 2 项待修：
- P1【必改】Release 编译时未关闭 ESP_LOG，CMakeLists 里只有注释没有实现
- P2【应改】sdkconfig.defaults 存在 7 处无效/过时 Kconfig 符号

先读 docs/reviews/T1-acceptance.md（验收报告，只读不改）了解全部细节。

## 范围约束
- 只修改 2 个文件：顶层 CMakeLists.txt、sdkconfig.defaults
- 不做 P3（组件 REQUIRES）、不动 P4（gdbinit）、不做 P5（上板）
- 不修改 PLAN.md、docs/hardware.md、docs/reviews/ 下任何文件
- 在 task/T1-skeleton 分支上工作，完成后一个提交

## 环境信息
- ESP-IDF 版本：v6.0.2，激活命令：<填写你的 export.ps1 路径>
- 本次任务纯构建验证，无需连接板子

## 修复项 1（P1）：实现 Release 日志编译期裁剪

当前顶层 CMakeLists.txt 只设了 CMAKE_BUILD_TYPE 默认值和注释，ESP-IDF 不会
根据 CMAKE_BUILD_TYPE 自动关日志。要求实现：

在 `project()` 之前加入（确保对所有组件生效）：

```cmake
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(ESP_LOG_LOCAL_LEVEL=ESP_LOG_NONE)
endif()
```

并更新原注释，说明正确用法：Debug 构建 `idf.py build`；Release 构建
`idf.py -DCMAKE_BUILD_TYPE=Release build`。

## 修复项 2（P2）：清理 sdkconfig.defaults 无效符号

按验收报告 P2 表格逐行处置，共 7 处：
1. `CONFIG_ESP32S3_SPIRAM_SUPPORT=y` → 改写为 `CONFIG_SPIRAM=y`
2. `CONFIG_ESP32S3_SPIRAM_MODE_OCT=y` → 删除（`CONFIG_SPIRAM_MODE_OCT=y` 已存在）
3. `CONFIG_SPIRAM_BANKSWITCH_ENABLE=y` → 删除
4. `CONFIG_SPIRAM_BANKSWITCH_RESERVE=8` → 删除
5. `CONFIG_FREERTOS_HZ_1000=y` → 删除（保留 `CONFIG_FREERTOS_HZ=1000`）
6. `CONFIG_PARTITION_TABLE_CUSTOM_APP_BIN_OFFSET=0x10000` → 删除
7. `CONFIG_NEWLIB_NANO_FORMAT=y` → 删除（v6 默认 C 库是 picolibc，此选项不适用）

注意保留所有有效的行（SPIRAM_MODE_OCT、SPIRAM_SPEED_80M、SPIRAM_USE_MALLOC、
FREERTOS_HZ=1000 等），只删改上述 7 处。

## 重要：改 defaults 后必须重新生成配置

sdkconfig.defaults 只在 sdkconfig 不存在时被读取。改完后必须：

```powershell
Remove-Item .\sdkconfig -ErrorAction SilentlyContinue
idf.py fullclean
idf.py build
```

否则构建仍用旧 sdkconfig，验证会失真。

## 验收标准（全部满足才算完成）

1. Debug 构建：`idf.py fullclean` 后 `idf.py build` 零错误。
2. Release 构建：`idf.py -DCMAKE_BUILD_TYPE=Release build` 零错误。
3. Debug 产物验证：`findstr /m "placeholder_task started" build\PD-step-test.bin`
   能命中（日志字符串在）。
4. Release 产物验证：同样命令**不**命中（日志被编译期剔除）。
5. 清理后构建日志（build\log\ 下最新 stderr 文件）中，来自本项目的
   unknown kconfig symbol 告警归零（IDF 自身的 bt/fatfs NOTE 可忽略）。
6. 解析后 sdkconfig 逐项确认仍然生效：
   CONFIG_SPIRAM=y、CONFIG_SPIRAM_MODE_OCT=y、CONFIG_SPIRAM_SPEED_80M=y、
   CONFIG_FREERTOS_HZ=1000、CONFIG_IDF_TARGET="esp32s3"。
7. Release 构建的 .bin 体积应明显小于 Debug（日志字符串与 rodata 被裁剪），
   在汇报中给出两者体积对比。
8. 只改了 CMakeLists.txt 和 sdkconfig.defaults 两个文件（git diff --stat 确认），
   提交信息清晰。

## 汇报要求
完成后报告：两种构建的 .bin 体积对比、7 条验收逐条勾选结果、
Release 产物 findstr 的实际输出、遗留 TODO。
````
