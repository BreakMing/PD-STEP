# T10 端到端验收记录（演示形态）

对应 protocol.md 全部契约。工具：`tools/cartpole_host.py`
（`pip install pyserial`；串口经 CH343P USB）。逐项填入实测后提交。

## 用例 1：符号约定标定（protocol.md §符号约定）

| 项 | 预期 | 实测 | 处置 |
|---|---|---|---|
| `VEL 200` 实际方向记为 +x | — | 方向：__ | — |
| 同方向手掰摆顶，`theta_urad` | 增大 | __ | 不符→`CONFIG POLE_INV 1` 后复核 |
| 反向手掰摆顶，`theta_urad` | 减小（负） | __ | |
| `VEL −200` 小车方向 | 与 +x 相反 | __ | 不符→`CONFIG X_INV 1` 后复核 |
| `x_um` 随 +x 增大 | 增大 | __ | 不符→查 AS5600_INVERT_DIR |

标定结论写入端侧 `deploy/v2/edge/config.h`（POLE_ZERO_URAD 按摆杆
自然直立读数取负：`CONFIG POLE_ZERO_URAD <−θ_读数>` 使直立=0）。

## 用例 2：端侧 CLI 全形态

| 命令 | 预期 | 实测 |
|---|---|---|
| `query` | 一行当前 MODE | |
| `state` | 行率 200 Hz、字段有效 | |
| `vel 200 --hold 1` | 小车运动、无 UNKNOWN | |
| `config VEL_MAX 800` | `OK VEL_MAX 800` | |
| `mark-left`（小车在左端） | `OK MARK_LEFT <E>` | |
| `mark-right`（小车在右端） | `OK MARK_RIGHT <L>`（L ≈ 530 mm） | |
| `enable` | `MODE READY NONE` | |
| `console` 交互 | 命令/应答正常 | |

## 用例 3：看门狗（Review Focus 1）

`wdt-test`：`VEL 200` 后停止喂狗 →

| 项 | 预期 | 实测 |
|---|---|---|
| ~100 ms 内 vx_mm_s | 减到 0 | |
| `QUERY` 应答 | `MODE READY NONE`（不改模式） | |
| 恢复发送 VEL | 继续跟踪（模式不变） | |

## 用例 4：200 Hz + 460800 长稳

`state --secs 600`：

| 项 | 预期 | 实测 |
|---|---|---|
| 行率 | 199–201 Hz | |
| UNKNOWN 行 | 0 | |
| x_um 静止漂移 | < 1000 µm | |

## 用例 5：全流程（演示当日流程）

推最左 `mark-left` → 推最右 `mark-right`（核对 L ≈ 530 mm）→
`enable` → 小速度回中（`VEL 50` 至 x≈0）→ 手扶摆杆直立 →
策略接管（有模型时）或 `vel ±100` 手动换向验证 → FAULT 路径
（拔 AS5600 线）→ STATE 流不断 → `RESET` → `enable` 恢复
（RAM 标定保留，无需重新标记）。

| 项 | 实测 |
|---|---|
| 全流程走通 | |
| FAULT 后 STATE 流 | 不断（θ 恒有效） |
| RESET→ENABLE 恢复 | |

## 结论

- 五用例：__ / 5 通过
- 遗留问题与处置：
