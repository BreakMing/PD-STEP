# T1-fix 验收报告（P1 Release 日志裁剪 + P2 配置清理）

> 验收日期：2026-09-12
> 验收对象：分支 `task/T1-skeleton`，提交 `2e58be5 T1-fix: P1 Release log stripping + P2 sdkconfig cleanup`
> 验收方式：代码审查 + IDF v6.0.2 源码级核对 + **本轮由验收方直接执行 Release/Debug 双构建实测**
> 前序报告：同目录 `T1-acceptance.md`（T1 初验，提出 P1/P2）、`T1-fix-brief.md`（修复任务包）

---

## 结论：**通过，T1 全部关闭，可合并 master**

上一轮遗留的 3 条"无证据"验收项（标准 2/4/7）本轮全部补齐实测证据。

## 实测证据（验收方亲自构建，2026-09-12）

### Release 构建（`idf.py -DCMAKE_BUILD_TYPE=Release build`，清除 sdkconfig 缓存后全新构建）

| 验证项 | 结果 |
|---|---|
| 构建零错误 | ✓ 1057/1057 目标完成，"Project build complete" |
| 生效配置 | ✓ sdkconfig 实测：`LOG_DEFAULT_LEVEL_NONE=y`、`LOG_DEFAULT_LEVEL=0`、`LOG_MAXIMUM_EQUALS_DEFAULT=y`、`LOG_MAXIMUM_LEVEL=0` |
| 日志字符串剔除 | ✓ `findstr` 检索 "placeholder_task started" 与横幅 "Single-Axis Motion Controller" 均**未命中** |
| 固件体积 | **125,776 字节**（0x1EB50） |

### Debug 对照构建（恢复用，同法全新构建）

| 验证项 | 结果 |
|---|---|
| 构建零错误 | ✓ |
| 日志字符串存在 | ✓ findstr 命中 |
| 固件体积 | **178,512 字节**（与 T1 初验时完全一致） |
| 配置还原 | ✓ LOG_DEFAULT_LEVEL_INFO=3、CMakeCache CMAKE_BUILD_TYPE=Debug |

### 体积对比（标准 7）

**Debug 178,512 B → Release 125,776 B，裁剪 52,736 B（约 29.5%）**。证据充分。

## 方案偏差说明（已认可）

执行方未采用任务包建议的 `add_compile_definitions(ESP_LOG_LOCAL_LEVEL=...)`，改用 **sdkconfig.release 叠加文件**方案（3 个文件改动而非 2 个）。经核对：

- IDF v6.0.2 无原生 sdkconfig.release 机制，手动 `list(APPEND SDKCONFIG_DEFAULTS)` 是正确实现
- 符号名（`LOG_DEFAULT_LEVEL_NONE` / `LOG_MAXIMUM_EQUALS_DEFAULT`）与 Kconfig choice 结构完全匹配
- 该方案避开了全局定义宏与 vfs / nvs_flash 等组件自带 `LOG_LOCAL_LEVEL` 的 -Werror 重定义冲突——**比任务包原方案更稳妥**

## P2 清理复核

sdkconfig.defaults 中 7 处无效符号全部按任务包处置；重配置日志中项目侧 unknown symbol 告警归零（仅剩 IDF 自身 bt/fatfs NOTE）。实测双构建均正常。

## 遗留事项

1. **【建议补文档】sdkconfig 缓存坑**：Debug↔Release 切换必须先 `Remove-Item .\sdkconfig`，否则 kconfgen 沿用旧值、Release 裁剪**静默失效**（本轮实测流程已遵循）。建议在 CMakeLists.txt 注释中补充此操作说明。
2. **【环境备忘·对后续任务重要】** 本机 ESP-IDF 为 VS Code 插件式安装，标准 export.ps1 不可用，构建需显式构造环境：
   - `IDF_PATH=C:\Users\SM\.espressif\v6.0.2\esp-idf`
   - `IDF_TOOLS_PATH=C:\Espressif\tools`（注意不是 C:\Espressif）
   - `IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python_env\idf6.0_py3.11_env`
   - `ESP_IDF_VERSION=6.0.2`（组件管理器必需）
   - PATH 前置：`C:\Espressif\tools\cmake\4.0.3\bin`、`C:\Espressif\tools\ninja\1.12.1`、`C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin`
   - 用 venv python 直接调 `$IDF_PATH\tools\idf.py`
3. **【T0 遗留·非 T1 范围】** P5 上板验证（烧录横幅 + 看门狗 10 分钟）仍待需求方插板执行。
4. 构建尾声 idf.py 在 IDF 仓库内做 git 版本戳时被沙箱拦截（index.lock），不影响产物，但 app 版本号字符串可能缺失——上板验证时留意横幅即可。

## 处置

`task/T1-skeleton`（1deb665 + 2e58be5）可合并回 master。合并后放行 T2 / T4 / T7 / T11 四路并行。
