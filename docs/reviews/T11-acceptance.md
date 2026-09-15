# T11 验收报告 · 设置与 NVS

> 日期：2026-09-15 · 分支：task/T11-settings（**零提交**，全部改动滞留工作区）· 验收人：规划 Agent
> 结论：**验收通过（含验收 Agent 修复轮）**——P1-1（$3 反向交叉校验）、P1-2（Python 参考模型分叉）已由验收 Agent 修复并复验（Python 30/30 + 双配置构建零错误）；P3 Unity 文件删除经需求方定案接受为合理清理。**待上板验收（§6）+ 需求方本地提交后闭环。**

## 1. 总评

T11 主体完成度高于 T4 首验轮：任务包的全部设计定案（API 签名、NVS 布局、校验规则表、启动序、胶水层分工）被准确执行，34 例 settings 自测（要求 ≥24）覆盖矩阵完整，gcode_parser 30 例保持且三个新用例（rc 传播 / M502 / 裸 $5→error:4）质量好。两处 P1 都是"边界遗漏"而非结构性缺陷：一处校验漏拦、一处工具链没跟上接口变更。

## 2. 通过项

| 项 | 结果 | 证据 |
|---|---|---|
| settings API（任务包 §1） | ✓ | settings.h：三值状态枚举、init/reload/set/get/is_valid/reset_defaults/count/name、15 个命名取值器、4 槽订阅、`settings_steps_per_mm()` 派生——逐项对照全部在位 |
| 校验规则（任务包 §3 ↔ PLAN §5） | ✓（除 $3 见 P1-1） | settings.c validate_locked() L78–150 逐参数对照 §5 表：$0 幂次 1–256、$1∈{200,400}、$2 (0,500]、$3 (0,2000]、$5≤$7、$6≤$8、$9 [1,50]、$10∈{0,1}、$11 八档波特率枚举、$12/$13 [-1000,1000] 且 $12<$13；整数参数非整数拒绝（floorf 校验 L87–91） |
| 交叉校验双向 | ✓（$7/$8/$12/$13） | case 7 拦 `s_values[5]>value`（缩 $7 破坏 $5≤$7 时拒绝）；case 8 同理；case 12/13 用 `$12<$13` 双向。**唯 $3 漏（P1-1）** |
| NVS 布局与容错（§2） | ✓ | namespace `settings`、键 `p0`–`p13`、u32 位存（memcpy L171–173）；缺键静默默认（L236–238）；坏值回退默认 + W 日志、键保留（L239–243）；nvs_flash_init 满页/新版本擦除重试（L268–276） |
| 解析器接口四处变更（§4） | ✓ | ① create(init_feed, init_accel, iface)——gcode_parser.c L554–556，`PARSER_DEFAULT_F/A` 宏删除；② on_get_param 成员删除；③ on_set_param 返回 int，rc 4→BAD_PARAM / 5→新枚举 PARSER_ERR_STORAGE / 其他非 0→BAD_PARAM（L381–385）；④ on_restore_defaults + M502 分支（L527–534），M502 后模态 F/A 不重置（注释 + 头文件双处写明） |
| `$n?` 移除落地 | ✓ | parse_dollar() `?` 分支已删（L378–390 重写）；裸 `$5`→error:4；头文件命令列表移除 `$n?` 行 |
| app_main 接线（§5） | ✓ | 启动序 bsp_early_init→settings_init→横幅→bsp_init→[自测]→protocol_start→create(settings_default_feed/accel)→注册（L155–233）；$$ 组包 14 行 `$%d=%.3f\r\n`（L74–84，snprintf 返回值 n 作长度——正确）；on_set_param 映射 0/4/5（L89–99）；line handler 增 PARSER_ERR_STORAGE→error:5（L133–135）；on_get_param 占位已删 |
| protocol 触碰（§6） | ✓ | 仅波特率来源一处：settings_uart_baud() + Kconfig 回退（protocol.c L354–358）+ 日志用实际值；CMakeLists 加 settings 依赖（REQUIRES 声明，非逻辑改动） |
| settings 自测 | ✓ | **34 例**（要求 ≥24）：默认值抽查、14 参数往返、非法矩阵（$0 7/0/512/16.5、$1=300、$2 0/501、$3 0/2001、$4>$3、$5 0/交叉、$6>$8 反向、$9 0/51、$10=2、$11=123456、$12≥$13）、未知 idx（14/-1/100）、持久性 set→reload、恢复默认、通知 set/reset、派生 80/160、坏 NVS 值回退（直写 p0=7）、count/name；结尾 reset_defaults 清场；`SELF-TEST DONE: n/total PASS` 格式 |
| gcode_parser 自测 30 例保持 | ✓ | `rg -c "static bool test_"` = 30；删 `$n?` 用例换入 test_dollar_set_param_and_return（rc 0/4/5 三态传播断言）、test_m502_restore_defaults（含模态不重置断言）、test_m503_and_bare_dollar_n（裸 $5→error:4）——断言未削弱 |
| 构建双配置 | ✓（验收人实测） | 默认配置零错误；`CONFIG_SETTINGS_SELF_TEST=y`+`CONFIG_GCODE_PARSER_SELF_TEST=y` 零错误；ELF 确认链接 settings_selftest + gcode_selftest 双自测（SELF-TEST DONE 字符串 ×2） |
| 范围红线 | ✓ | bsp 零改动（git diff master --stat 证实）；protocol 仅许可的一处；T5/T6/T7/T9 功能零实现（占位回调保持占位并注明接入点）；sdkconfig.defaults/release 未动；TMC/AS5600 代码未碰；settings 不依赖 protocol/gcode_parser/bsp（CMake REQUIRES 仅 nvs_flash） |
| PLAN.md 无越权 | ✓ | git diff master -- PLAN.md 全部为规划 Agent 的 v1.10 编辑（本会话产出），执行 Agent 未改计划书 |

## 3. P1 问题（修复后才能上板闭环）

### P1-1 $3 缺反向交叉校验（PLAN §5 规格违例）

- **PLAN §5 定稿**：交叉校验"$4≤$3 …… **两个方向都要拦**"
- **实现**：validate_locked() case 3（settings.c L106–108）只检查 (0,2000]，**未检查新 $3 ≥ 当前 $4**。$7/$8/$12/$13 四处都做了双向，唯 $3 漏
- **复现**：默认 $4=400 → 发 `$3=200` → 当前返回 SETTINGS_OK（error 不会回）→ 库内 $4=400 > $3=200，不变式破坏
- **后果**：T5 消费 $3/$4 换算 IRUN/IHOLD 时出现 IHOLD>IRUN，TMC2209 行为不符合预期
- **修复**：case 3 增一行 `if (iv < (int)s_values[4]) return false;`
- **对应测试缺失**：自测 test_crosscheck_p4_gt_p3 只测正向（$4 大于 $3 被拒）；需补反向用例（$4=400 时 $3=200 → INVALID）

### P1-2 Python 参考模型与 C 实现分叉（未同步接口变更）

- **现象**：test_gcode_parser_verify.py 仍是旧语义——`$n?` 用例（L687–691：`p.feed("$5?")==PARSER_OK`）和 `on_get_param` 回调（L412、L426）还在；`gcode_parser_create` 未加 init_feed/init_accel 参数；模态默认仍是硬编码 300/200（C 侧宏已删，初值来自 NVS）
- **危害**：参考模型测的是它自己（复跑 30/30 全绿恰因如此），对 C 实现的"参考"作用已失效——它还在为已删除的 `$n?` 语法背书。后续任何依赖该模型做行为对照的验收都会得出错误结论
- **修复**：删 test_dollar_query_param + get_param 回调接线；create 加双初值参数；`$n?` 用例替换为裸 `$5`→error:4；补 M502 / on_set_param 返回值传播（rc 4/5）用例；模型头部注释标注接口版本对应 gcode_parser.h 当前签名

### P1-3 零提交（流程违例，T4 首验重演）

分支 task/T11-settings 已切出但零提交，全部改动滞留工作区（4 修改 + 4 新增 + 1 删除）。任务包明确要求：沙箱写入失败需在汇报中报告，不许静默放弃。（T4-fix 已定性本环境 git 对象写入被沙箱策略拦截——若执行 Agent 同因，仍应显式报告；无论原因，合并前必须形成提交，可由需求方本地代执行。）

## 4. P3 问题（入档即可）

| 级 | 问题 | 说明 |
|---|---|---|
| P3 | Unity 测试文件被删（test_gcode_parser.c，461 行） | 任务包措辞是"同步更新 Unity 测试"，执行 Agent 选择了删除。客观上该文件自 T4 起就未编入构建（v1.8 定案后上板自测是唯一真实执行路径），属死文件清理，删除理由成立——但超出字面授权。**入档为已接受偏差**（与 host_test 伪 exe 删除同类），不再恢复 |
| P3 | Kconfig 自测注释宣称 "before protocol stack" | settings Kconfig help 文本说自测在协议栈前运行——与实现一致（app_main L184–190 位于 protocol_start 之前），非问题，仅记录核对结果 |

## 5. 修复轮记录（验收 Agent 执行，2026-09-15 需求方授权）

| # | 修复 | 文件:位置 | 复验 |
|---|---|---|---|
| 1 | P1-1：case 3 补反向交叉校验 `if (iv < (int)s_values[4]) return false;` | settings.c L108–109 | Python n/a；构建零错误 |
| 2 | P1-1 对应用例：test_crosscheck_p3_lt_p4（$3=200 当 $4=400 → INVALID；$3=500 → OK） | settings_self_test.c L246–254，runner L481 | settings 自测 **35 例**（34+1） |
| 3 | P1-2：Python 参考模型同步——create 加 init_feed/init_accel 参数（默认值注释为测试兜底，固件来自 NVS）；删 `$n?` 分支与 get_param 回调；`_fire` 支持返回值；`$n=v` 与 M502 实现返回值传播（rc 4/5 → error:4/5）；三用例替换（$n? → rc 传播 + M502 模态不重置 + M503/裸 $5，含 `$5?` → error:4 断言） | test_gcode_parser_verify.py | **30/30 全绿**（实测） |
| 4 | 重建自测固件（sdkconfig 双自测开启） | build/ | 构建零错误；ELF 双 SELF-TEST 字符串确认 |

P3（Unity 文件删除）按需求方定案接受，不再恢复。

## 5b. 上板首轮结果（2026-09-15，自测固件）

| 自测 | 结果 |
|---|---|
| gcode_parser | **30/30 PASS** |
| settings | **34/35**——`test_invalid_p5_default_feed` FAIL（expected 1, got 0） |

- **根因定性：测试用例缺陷，实现无 bug**。原用例 L262 先 `settings_set(7, 500)` 想把 $7 压到 500，但当时 $5=600，该 set 恰被 case 7 反向交叉校验拒绝（正是规格要求的拦截），铺垫静默失败 → $7 仍 3000 → 随后 `$5=600` 合法通过，断言落空
- **修复（验收 Agent）**：重写用例为双向覆盖——$5=0→INVALID、$7=500(当 $5=600)→INVALID（case 7 反向，原套件漏测）、合法过渡路径（$5=400→$7=500）、$5=600(当 $7=500)→INVALID；其余 9 处未检查返回值的铺垫 set 逐一核对均合法，无同类隐患
- 重建通过（构建零错误）。另：启动序上板确认正确——settings 先于横幅初始化、解析器初值 feed=600/accel=500 从 NVS 默认取到（`G-code parser registered (feed=600, accel=500)`）
- 佐证 T4 教训：C 用例必须上板真实执行，host 侧无执行路径（本例即首跑抓出）

## 5c. 上板探针轮发现 P1（2026-09-15，VOFA+ 11 探针）

探针 1–9、11 全过；**探针 10（NVS 断电持久性）失败**：`$5=1000`→ok→断电重启→`$$` 回 `$5=600.000`。

- **根因（验收 Agent 定位）**：`load_all_locked()` 单趟加载，加载到 p5 时校验交叉约束引用**尚未加载的 s_values[7]**（冷启动=零初始化）→ `1000 > 0` 判非法 → 回退默认。前向引用交叉对全部受累：$5→$7、$6→$8、$12→$13（任何已存值冷启动即被误杀）；$3→$4 因判据方向侥幸无害。板子实际打了 W 日志（UART0 控制台，VOFA+ 在 UART1 不可见）
- **自测为何没抓到**：`test_persistence_set_reload` 的 reload 复用上一轮内存值（$7=3000 残留），把前向引用缺陷掩盖——只有冷启动（全零起步）暴露
- **修复（验收 Agent）**：`load_all_locked()` 改两阶段——阶段 1 读满全部 14 槽（缺键→默认），阶段 2 对**完整集合**逐一校验，非法→默认+W；坏值回退语义不变
- **回归用例**：`test_load_forward_reference`（判别性构造：内存毒化 $7=500、NVS 直写合法集 p5=1000/p7=3000、reload——旧代码必失败回退 600，新代码正确加载 1000）；settings 自测 **35→36 例**
- 构建：自测配置零错误，ELF 确认新用例编入；ESP_ROM_ELF_DIR 已补入 idf_env.ps1（gdbinit 再生成需要）；发现 Kconfig 再生成会把 `=n` 改写为 `# ... is not set` 注释形态——sdkconfig 开关切换须按注释形态匹配或直接追加（追加优先级更高）
- **待上板**：36 例自测 + 默认固件探针 10 重跑

## 6. 上板验收最终结果（2026-09-15，P1 修复后）

**轮次 A——自测固件**（`CONFIG_SETTINGS_SELF_TEST=y` + `CONFIG_GCODE_PARSER_SELF_TEST=y`）：

| 自测 | 结果 |
|---|---|
| settings（36 例，含 `test_load_forward_reference` 回归） | **36/36 PASS** |
| gcode_parser | **30/30 PASS** |

**轮次 B——默认固件探针 10 复测**（17:20–17:21 实录）：

```
$5=1000        → ok
[断电重启]
$$             → $5=1000.000（其余 13 项默认）+ ok   ← P1 修复生效
```

结合首轮探针 1–9、11 全过 + 本轮复测，**验收判据 ①–⑤ 全部满足**。中途两次无效复测记录在案：自测固件上跑持久性探针（开机自测清参数，测试对象错误）与未重烧即复测（板上为修复前二进制）——均非缺陷复发。

## 7. 最终结论

**T11 上板验收通过。** 过程完整轨迹：代码级验收（修复 3 处）→ 上板自测 34/35（用例缺陷修复）→ 探针 1–9/11 过、探针 10 抓出 P1（前向引用加载 bug）→ 修复 + 回归用例 → 36/36 + 30/30 → 探针 10 复测通过。

**复盘要点（三条，供后续任务沿用）：**
1. **带交叉校验的批量加载必须两阶段**：先填满、再对完整集合校验——单趟加载的前向引用（$5→$7 等）在冷启动时读的是零值
2. **reload 测试掩盖冷启动缺陷**：reload 复用内存残留值，只有真断电才暴露；NVS 持久性必须上板断电验证，不可用 reload 替代
3. **W 日志在 UART0、协议探针在 UART1，双口证据不可互见**：上板排障时两路都要看（monitor + VOFA+），本轮根因定性靠的是代码路径推演，若有 UART0 日志可一步定位

**遗留移交**：无阻塞项。$n?/M503 决策已定案（§4.3），实现偏差三条见 §4。

