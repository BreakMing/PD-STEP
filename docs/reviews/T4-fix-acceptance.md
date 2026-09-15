# T4-fix 验收报告 · G-code 解析器修复轮

> 日期：2026-09-15 · 分支：task/T4-parser（**提交由需求方本地执行**，命令见 §4.3）· 验收人：规划 Agent
> 结论：**代码级验收通过**（含 P3 残留修复轮）。修复简报 5 条硬性判据满足 4 条；唯一未完成项"单一清晰提交"系沙箱限制——执行 Agent 报告的 `.git/objects` 写拦截在验收环境**复现为完全相同的错误**（同一对象哈希 3aaefef0…，即 main/CMakeLists.txt 的 blob），确认为沙箱策略级限制而非环境异常。上板实测（30/30 自测 + 探针）仍待执行，完成后 T4 闭环。

## 1. 逐项核验

### 修复项 1：P1-2 括号注释（✓ 已修复）

| 证据 | 位置 |
|---|---|
| parse_move 数值位跳注释 | gcode_parser.c L212–216（词字母后、skip_ws 后、parse_float 前调 skip_comment） |
| parse_g92 同 | gcode_parser.c L312–315 |
| 行为选择已披露 | 注释明确：数值位只认 `(...)`；`;` 在数值位仍致 error:4（与修复前一致）——符合简报建议方向 |
| 规格对齐 | PLAN v1.8 §4.1 已定案（词间与词后数值位均可） |
| 验证 | `G1 X(hello)10`、`G1 X5 F(hi)100` → PARSER_OK（代码级推演 + Python 模型复跑 + 自测用例断言，三重一致） |

### 修复项 2：P1-1 测试真实可执行（✓ 代码级闭环，上板待执行）

- **host_test/ 伪 exe 已删除**：目录已空（git 不跟踪空目录，功能上等同删除）
- **gcode_parser_self_test.c**（30 用例移植为普通函数 + ASSERT 宏自跑）：
  - 逐用例对照 test_gcode_parser.c（Unity 版）核验：**断言无削弱**（含错误路径的 record_count=0 检查、模态保持、G91/G90 切换、范围钳制等全部保留）
  - 汇总行 `SELF-TEST DONE: %d/30 PASS` 符合简报要求的格式
  - header 声明以 `#ifdef CONFIG_GCODE_PARSER_SELF_TEST` 守卫（gcode_parser.h L185–187）
- **Kconfig**：GCODE_PARSER_SELF_TEST（bool，default n）✓
- **CMakeLists.txt**：CONFIG_GCODE_PARSER_SELF_TEST 条件编译 ✓
- **app_main.c L183–189**：协议栈启动前调用，#ifdef 守卫 ✓
- **双配置构建（验收人 2026-09-15 实测，含 P3 修复后的重建）**：
  - 默认配置（自测关）：`idf.py build` 零错误，ELF 中**不含** "SELF-TEST DONE" 字符串
  - 自测开配置：`idf.py build` 零错误；ELF 中**含** "SELF-TEST DONE" 字符串与新用例串 `G1 X (hello) 20`——自测代码（含 P3 新用例）真实编入并链接
- **Python 参考模型对齐**：_parse_move/_parse_g92 先 `_strip_comment` 再解析，实测 30/30 PASS

### 修复项 3：P2/P3 顺手项（✓）

- app_main.c 思考注释块已清理 → 现 L106–111 为 4 行简洁说明（解析成功→ok；错误→error:n；查询类先回数据、line handler 统一补 ok）
- `$$` 占位串 off-by-one 已修：app_main.c L83 `protocol_send_raw(msg, strlen(msg))`

### 红线核查（✓）

- **protocol/bsp 零实质改动**：`git diff` 显示 components/bsp/Kconfig、tools/t3_stress_test.py 与 HEAD 内容完全一致（仅行尾符幻影差异）；验收人已 `git checkout --` 还原，工作区不再出现这两个文件
- sdkconfig.defaults / sdkconfig.release 未动（sdkconfig 本身未跟踪，当前 SELF_TEST=y 为上板自测就绪状态）
- 占位回调保持占位（T5/T6/T8/T11 接入点注释完好，未越界实现）

## 2. P3 残留（带空格词后注释）——✓ 已由验收 Agent 修复

- **问题**：`G1 X (hello) 10` / `G1 X5 F (hi) 100`——Python（全局剥注释后解析）返回 OK，C（skip_comment 后未再跳空白，parse_float 遇前导空格失败）报 error:4
- **修复**（gcode_parser.c 共 2 行）：
  - parse_move L216：`s = skip_comment(s);` 后补 `s = skip_ws(s);`
  - parse_g92 L315：同上
- **测试同步扩展**（保持 30 用例不变，括号注释用例内加 3 组断言）：
  - 自测 gcode_parser_self_test.c L393–405：`G1 X (hello) 20`→ok、`G1 X5 F (hi) 200`→ok（F=200）
  - Unity test_gcode_parser.c L321–336：同上
  - Python test_gcode_parser_verify.py L577–589：同上
- **验证**：Python 30/30 PASS；双配置重建零错误；ELF 含新用例字符串（见 §1 修复项 2）
- **C/Python 分歧就此消除**：两边对 `G1 X(hello)10`、`G1 X (hello) 20`、`G1 X5 F (hi) 200` 行为一致（均 OK）

## 3. 提交（沙箱限制，需求方本地执行）

| 项 | 说明 |
|---|---|
| 执行 Agent 报告属实 | 3 次重试均失败，对象 `3aaefef0…`（main/CMakeLists.txt blob）。验收环境复现**完全相同错误**（同哈希）→ 沙箱策略级拦截，非环境异常，重试无意义 |
| 仓库状态无污染 | 无 index.lock 残留、无半写对象、9 个待提交对象全部缺失（写入从未成功过）——本地直接执行提交无风险 |
| 处置 | 需求方在**自己的终端**执行 §4.3 命令（与 T3 a6458cb 提交流程相同）；后续任务包不再要求执行 Agent 执行 git 提交 |
| 上板实测 | 自测固件 30/30 + 默认固件探针（§4.1/§4.2），板子验收时未连接 |

## 4. 上板验收步骤（需求方执行）

### 4.1 自测固件（当前 build/ 已就绪——sdkconfig 已置 SELF_TEST=y 并完成构建）

接板子后（环境配方见 PLAN §12-6），在 IDF 环境执行：

```powershell
idf.py -p COMx flash monitor
```

预期：启动横幅后打印 30 个用例结果，最终行 **`SELF-TEST DONE: 30/30 PASS`**（tag: gcode_selftest）

### 4.2 默认固件 + 探针

```powershell
# 把 sdkconfig 中 CONFIG_GCODE_PARSER_SELF_TEST=y 改回 "# CONFIG_GCODE_PARSER_SELF_TEST is not set"
idf.py -p COMx build flash monitor
```

VOFA+（UART1，GPIO41/42，115200，原始文本模式）逐条发送：

| 探针 | 预期 |
|---|---|
| `G42` | error:1 |
| `G1` | error:4 |
| `$5?` | `$5=0.00` + ok |
| `G1 X(hello)5` | ok |
| `G1 X (hello)5` | ok（P3 修复形态） |
| `G91` → `G1 X10` | ok / ok |
| `;c` | ok |

### 4.3 提交（需求方在自己的终端执行）

```powershell
# 项目根目录下（行尾幻影文件已由验收人还原，无需处理）
git add components/gcode_parser main/app_main.c main/CMakeLists.txt
git commit -m "T4: G-code parser (fix round) - value-position paren comments (incl. spaced form), on-target self-test via Kconfig"

git add PLAN.md docs/reviews/T4-acceptance.md docs/reviews/T4-fix-brief.md docs/reviews/T4-fix-acceptance.md
git commit -m "docs: T4 fix-round acceptance + PLAN v1.8"

git log --oneline -3
```

（上板证据可在 4.1/4.2 完成后补记进本报告，与 T3 流程同理）

## 5. 结论

代码级验收通过：三 P1 中两个（括号注释、测试真实可执行）修复到位且有实测构建证据，30 用例断言未削弱，P3 残留（带空格词后注释）已由验收 Agent 修复并三处测试覆盖，红线零违反。**T4 闭环还差三步：需求方本地提交（§4.3）→ 上板自测 30/30（§4.1）→ 探针验证（§4.2）。** 全部完成后合并主线进入 T5（TMC2209 驱动）。
