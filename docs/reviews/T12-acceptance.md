# T12 验收报告 · 上位机模拟脚本与端到端验收（host_sim.py）

> 日期：2026-09-17 · 分支：master · 提交：待用户提交 · 验收人：规划 Agent
> 结论：**验收通过（代码级 + 静态自检 + 上板实跑 8/8 PASS）**。PLAN T12 的 8 项用例全部实现并通过。上板过程暴露 **1 处脚本缺陷**（用例 8 的"静止自发帧"误判），定位并修复后重测通过；**固件侧零缺陷**。**与 T10 合并达成 M5 里程碑「整机交付」，`T0–T12` 全部闭环。**

## 0. 结果总览

| 阶段 | 结果 |
|---|---|
| 代码级验收 | 通过（红线精确，覆盖度完整） |
| 静态自检 | `py_compile` ✓ / CLI ✓ / 端口错误路径 ✓ / **协议解析器 14/14** ✓ |
| 上板实跑 | **8/8 PASS**（见 `docs/acceptance.md`） |
| 上板暴露问题 | 1 处**脚本**缺陷（用例 8），已修复重测通过 |

## 1. 红线检查（✓ 通过）

| 约束 | 结果 |
|---|---|
| **零固件改动** | ✓ `main/`、`components/`、`sdkconfig*` 的 `git diff --numstat` **全空** |
| 改动面 | ✓ 仅新增 `tools/host_sim.py` + `tools/README.md`（+1 行） |
| 运行产物 | ✓ `docs/acceptance.md`（脚本生成，非手写） |
| 无 git 操作 | ✓ |

## 2. 覆盖度与判据核对

PLAN T12 的 8 项用例逐条实现：

| 用例 | 实现 | 判据要点 |
|---|---|---|
| 1 ok 流控窗口 | `case1_flow_control` | 20 行不等响应 → 恰好 20 个 ok、零丢失；黑盒边界（不窥探内部"第 17 行阻塞"） |
| 2 1000 段流式执行 | `case2_3_stream` | 1000 段全 `ok`，无 `error:*`、无超时 |
| 3 **无失步判据** | 同上（每段后 `?`） | `\|C-E\| ≤ 0.1mm` **且** 无递增趋势（`last_max > 2×first_max` 即判失败）——区分真丢步与采样时序差 |
| 4 `!` 立即停 | `case4_immediate_stop` | 状态曾为 RUN；C 冻结（500ms 后差 ≤0.001）；显著小于被取消目标；恢复命令到位且 **E 位移匹配**（防反向狂奔） |
| 5 F/A 模态 | `case5_modal_fa` | 耗时对比：段1≈段2（模态沿用），段3 < 段2 的 60% |
| 6 G92 同步平移 | `case6_g92_shift` | G92 后 C 与 E **各自**到位且 `\|C-E\|` 保持 |
| 7 NVS 持久性 | `case7_nvs_phase1/2` | **两阶段 + 真实断电**（不接受软复位替代，T11 教训） |
| 8 遥测帧 | `case8_telemetry` | 频率 ±2、逐帧正则、`STATE=RUN`、`T=OK`、**静止 1s 零自发帧**（D12） |

## 3. 静态自检（验收人独立执行）

| 项 | 结果 |
|---|---|
| `python -m py_compile tools/host_sim.py` | ✓ 通过 |
| `--help` | ✓ 五个开关齐备（`--skip-power-cycle` / `--phase` / `--fast` / `--only` / `--report`） |
| 端口打不开（`COM99`） | ✓ 友好退出：打印三条排查提示，无裸异常堆栈，exit code 1 |
| **协议解析器**（验收人另写一次性测试） | ✓ **14/14 符合预期**，含两个关键防御点：ESP_LOG 行归为噪音；T3 旧占位格式（2 位小数）**不**被误认为合法帧 |

## 4. 上板实跑（8/8 PASS，详见 `docs/acceptance.md`）

| 用例 | 关键实测 |
|---|---|
| 1 | 20/20 ok，0.01 s |
| 2 | 1000/1000 段 ok，241.2 s |
| **3** | **最大 \|C−E\| = 0.039 mm**，首/末 10% 段最大均 0.039（无递增），平均 0.0139，超差 0 段 |
| 4 | 停在 311.538mm（目标 420），冻结差 0.000，恢复后 **C=50.000 精确**，E 位移 261.56 vs 期望 261.54 |
| 5 | t1=3.11 / t2=3.12（比值 1.01）/ t3=1.30（42%） |
| 6 | dC = dE = −60.000（完全同步） |
| 7 | 真实断电后 **$5 = 1500.000** 保持 ✓（已恢复 1000） |
| 8 | 20 帧/2s（10.0Hz）、STATE=RUN 20/20、T≠OK 0、**静止自发帧 0** |

**用例 3 是"无失步"的核心证据**：1000 段往返、最大偏差 0.039mm 且首末一致——比 T9/T10 的短测试更强。

## 5. 上板暴露的脚本缺陷（用例 8）与修复

### 5.1 现象

首次运行判 **FAIL**，唯一失败项：`静止 1 s 内自发帧：20 个（必须为 0）`。但同条目其他数据全部正常（频率 10.0Hz 精确、20/20 格式合法、20/20 `STATE=RUN`、0 个 `T≠OK`），且静止时 `?` 明确应答 `IDLE`。

### 5.2 定位：脚本缺陷，固件正确

- **代码依据**：`monitor_task` 中 `if (!step_engine_busy()) { ticks = 0; continue; }` —— 静止时明确不发帧（T9 验收已逐行审过）
- **量化吻合**：`wait_idle()` 每轮发一个 `?`（**产生** 1 个响应帧）却只**消费** 1 个帧；同时 `monitor_task` 以 `$9` 频率产生周期帧 → **净积压 ≈ 10 帧/秒**。该用例运动 4 s，2 s 采集窗口后剩约 2 s → 积压 ≈ **20 帧**，与观测值完全一致
- 结论：这 20 帧是**运动末尾的周期帧**，被随后的"静止监听"误计

### 5.3 修复

新增 `Board.drain_input()`：开始静止监听前排空输入缓冲（两次，中间停顿 50ms），并在注释中写明**为何不掩盖真实缺陷**——固件若真在静止时发帧，排空后仍会被计数。

同时新增 `--only LIST`（只跑指定用例，未选中项不写入报告）与报告 header 的运行参数记录，便于修复后快速重测。

### 5.4 验证

重测（`--only 8`）：`静止自发帧=0`，其余项不变 → **PASS**。

## 6. 设计亮点（值得留档）

| 项 | 价值 |
|---|---|
| 用例 7 用报告文件中的 HTML 注释传递跨阶段状态 | 两阶段脚本必须传"原 `$5`"才能恢复现场；避免额外 `.state` 文件或手工抄数 |
| `setup_axis()` 先打印"滑台须位于行程左端" | setup 会向 +300mm 移动，而脚本不知道轴在哪 —— 零成本防撞 |
| 所有运动目标在运行期依据 `$$`（软限位、k）计算 | 不硬编码，参数被改过也不会越界 |
| 协议解析器容忍并**计数**非协议行 | 防御"测试固件日志镜像混入 UART1"，且把"跳过 N 行"写进报告（本次为 0，证明通道纯净） |
| `case_failed()` 单例异常不中断全局 | 保证一次运行拿到所有用例结果 |

## 7. M5 达成与项目闭环

| M5 内容 | 状态 |
|---|---|
| T10 状态机 + 立即停 | ✅（14/14 + `!` 端到端两条硬判据） |
| **T12 验收脚本** | ✅ 本任务闭环（8/8 PASS） |

**`T0–T12` 全部闭环，五个里程碑（M1–M5）全部达成。**

## 8. 遗留与观察项

1. **用例 5 以耗时对比间接判定 F/A 模态** —— 协议无模态读回接口（`$n?` 已于 v1.10 移除），属既定偏差
2. **`placeholder_task` 仍在固件中**（T1 遗留的占位任务，带喂狗）—— 不影响功能，可留作后续清理项
3. **WDT panic 重启后位置基准归零** —— T10 定案接受的权衡，上位机须重新 `G92`
4. **本次验收使用正常固件**（所有测试开关关闭），与"整机交付态"一致；若后续要复跑组件自测，需临时开启对应 Kconfig

## 9. 提交（沙箱限制，用户本地执行）

```powershell
git add tools/host_sim.py tools/README.md docs/acceptance.md
git commit -m 'T12: host simulator + end-to-end acceptance (8 cases: flow control, 1000-segment streaming with per-segment C/E loss-of-step check, ! immediate stop, F/A modal, G92 shift, NVS power-cycle, telemetry frame rate/format/flags); acceptance fix: drain stale RX before the idle-silence check (wait_idle produced one ? response per poll while monitor emitted at $9 Hz, so the move tail was miscounted); 8/8 PASS'

git add docs/reviews/T12-brief.md docs/reviews/T12-acceptance.md PLAN.md
git commit -m 'docs: T12 brief + acceptance (8/8 PASS end-to-end) + PLAN v1.18 - T12 closed, M5 achieved, T0-T12 complete'
```

> 注：`docs/acceptance_case8.md`（用例 8 单独重测的中间产物）已被合并进 `docs/acceptance.md`，可不纳入提交或一并加入（无害）。
