# T4-fix 任务包 · 复制以下全部内容给执行 Agent

````
# 任务：ESP32-S3 单轴运动控制器 · T4-fix G-code 解析器修复轮

## 背景
- T4 首验不通过（见 docs/reviews/T4-acceptance.md，先完整阅读）。
- 两处规格已由需求方定案并写入 PLAN v1.8 §4.1（先重读该节）：
  - 定案 1a：括号注释 `(...)` 在**词间与词后数值位均合法**（`G1 X(hello)10`、`G1 X5 F(hi)100` 必须解析成功）
  - 定案 2a：空行/纯注释行回 `ok`（当前实现已如此，无需改）
- 上一轮的工作未提交，全部改动当前滞留工作区（components/gcode_parser/ 未跟踪、main/ 两个文件已修改）——本任务在其基础上继续并最终提交。

## 环境
- ESP-IDF v6.0.2，构建环境按 PLAN.md 第 12 节第 6 条配方。
- 本机无主机 C 编译器、无 QEMU——C 单元测试的唯一真实执行路径是**编入固件上板跑**（本任务的核心目标）。

## 工作内容（按序）

1. **修 P1-2（括号注释）**：gcode_parser.c 的 parse_move 与 parse_g92 中，
   词字母之后、调用 parse_float 之前，先跳过可选的 `(...)` 注释（复用
   skip_comment 或等价逻辑；`;` 至行尾的规则同样在数值位生效与否保持
   与 PLAN §4.1 一致——建议数值位只认 `(...)`，`;` 仍只在词间/行首生效，
   汇报中说明你的选择）。修改后 `G1 X(hello)10` 与 `G1 X5 F(hi)100`
   必须 PARSER_OK。同步更新 Python 参考模型（如行为已一致则不动）。
2. **修 P1-1（测试真实可执行）**：
   a. 删除 `components/gcode_parser/host_test/`（伪 exe，Xtensa ELF，Windows 不可运行）
   b. 将 `test/test_gcode_parser.c` 编入固件：gcode_parser 组件加 Kconfig 开关
      `GCODE_PARSER_SELF_TEST`（bool，默认 n）。开启时 app_main 在协议栈启动前
      顺序执行全部 30 个 TEST_CASE 的逻辑（不依赖 unity runner 宏的话，
      把用例改写为普通函数 + 简单断言宏的自跑 main 也接受，保持 30 个
      用例的断言内容不变），结果以 `PASS/FAIL x/30` 汇总行打到 ESP_LOG，
      结束打一行 `SELF-TEST DONE: <n>/30 PASS`。
   c. 构建两次验证：默认配置 build 零错误；`-D GCODE_PARSER_SELF_TEST=y`
      配置（删 sdkconfig 后）build 零错误。
3. **修 P2/P3 顺手项**：
   a. 清理 app_main.c L110–151 的思考过程注释块（"Wait — no... Actually..."
      整段），保留最终的简洁说明（2-4 行：解析成功→ok；错误→error:n；
      查询类回调先发数据、line handler 统一补 ok）
   b. `$$` 占位串：`protocol_send_raw(..., 46)` 改用 `strlen()`，消除 off-by-one
4. **git 提交**：全部工作（含上一轮滞留的 T4 改动）以逻辑清晰的提交落在
   `task/T4-parser` 分支。**若 git 对象写入报 Permission denied，重试两次，
   仍失败则在汇报中明确报告（提交可由需求方代执行），不许静默放弃**。
   提交后 `git log --oneline -3` 附汇报。

## 范围红线
- 不改 protocol/bsp 组件；不动 T3 已验收行为（空行/注释→ok 保持）
- 不动 sdkconfig.defaults / sdkconfig.release（Kconfig 新项默认值即可）
- 不实现 T5/T6/T9/T11 的功能（占位回调保持占位）

## 完成定义（全部满足才算完成）
- `G1 X(hello)10` 与 `G1 X5 F(hi)100` 解析为 PARSER_OK（代码级，行号证据）
- host_test/ 目录已删除
- 自测开关两种配置均 build 零错误（附两次构建输出尾部）
- 30 个用例全部保留且断言未削弱（对照上轮验收报告 §3 的用例清单）
- 单一清晰提交（或汇报说明提交失败的原因）
- 汇报：每条修复的文件+行号证据、构建输出、遗留 TODO

## 上板验收预告（需求方执行，你不用做）
- 烧录 SELF_TEST=y 固件 → monitor 看 `SELF-TEST DONE: 30/30 PASS`
- 烧录默认固件 → 探针：`G42`→error:1；`G1`→error:4；`$5?`→`$5=0.00`+ok；
  `G1 X(hello)5`→ok；`G91`/`G1 X10`→ok；`;c`→ok
````
