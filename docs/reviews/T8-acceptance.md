# T8 上板验收记录：堵转回零

机械台位在位、12V 供电时执行。数据填入本表后提交。

## 前置

- 构建流：`idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" build`
- 烧录后启动日志应有 `cartfsm 流程自测 ... SELF-TEST DONE: 5/5 PASS`
  （注入式用例，不依赖机械）。

## T8-① DIAG 极性验证（未决事项 2 落点）

小车置于行程中段，`HOME` 后低速手堵小车（或顶死端部）：

| 项 | 预期 | 实测 |
|---|---|---|
| 手堵时进入 | READY → MODE FAULT STALL（或 HOMING 中直接完成回零） | |
| 不堵时 | 无误触发（连续运行 1 min 无 FAULT/STALL） | |

若极性相反：`menuconfig → cartfsm → DIAG1 堵转有效电平` 改为低有效，重烧。

## T8-② StallGuard 阈值初标（未决事项 3）

`CONFIG SGTHRS <n>` 扫 20–80：

| SGTHRS | 手堵是否触发 | 空载是否误触发 |
|---|---|---|
| 20 | | |
| 40（默认） | | |
| 60 | | |
| 80 | | |

记录不误触发且灵敏的值 → 写入端侧 `config.h` 会话推送。

## T8-③ 回零重复性（protocol.md 预期 ±1–2 mm）

连续 5 次 `HOME`（两次间 `RESET`），从 STATE 流记录每次回零完成后
静止 `x_um`：

| 次 | x_um | 与首次的差（µm） |
|---|---|---|
| 1 | | 0 |
| 2 | | |
| 3 | | |
| 4 | | |
| 5 | | |

判定：最大散布 ≤ 2000 µm 通过；超出则提高 HOME_VEL 或按
protocol.md 未决事项路径改判据（C−E 停增）。

## 结果

- 流程自测：__/5 PASS
- T8-①：通过 / 不通过（极性：高 / 低）
- T8-②：选定 SGTHRS = __
- T8-③：最大散布 __ µm（通过 / 不通过）
