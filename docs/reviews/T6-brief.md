# T6 任务包 · 步进引擎（step_engine，GPTimer ISR）

> 分发对象：执行 Agent（Claude Code）· 分支：`task/T6-engine`（已建好，勿做任何 git 操作——沙箱禁止，用户负责提交）
> 上游事实源：PLAN.md（§4.4 立即停语义、§4.7 数值与单位、T6/T10 节）、components/bsp/include/bsp.h、components/protocol/include/protocol.h、main/app_main.c

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 实现 `components/step_engine/`（API 见 §3，算法见 §4）
3. app_main 接线（见 §5）
4. 自测（§6）+ 双配置构建验证（§7）
5. 最终答复：改动文件清单、算法选型推导（含定点精度估算）、自测预期输出、构建证据

**红线（越界即验收不通过）**：
- 禁止改动：`components/protocol/`、`components/gcode_parser/`、`components/settings/`、`components/tmc2209/`（T5 正在并行开发，勿碰）、`components/bsp/`
- STEP/DIR GPIO 建议直接使用：bsp 已把 GPIO5/GPIO6 初始化为输出（T2 完成），本组件经 init 参数收引脚号即可，**不重复初始化、不碰 EN**
- 禁止 git 操作
- **自测运行时 TMC 必须处于失能态（EN 高）**——自测只驱动 STEP/DIR 电平，不得使电机转动（电机可见运动留到 T5 合并后的联合验收）

---

## 1. 任务边界（与 T7 的分工）

本任务交付"执行层"：消费已预计算好梯形边界的**段（segment）**，ISR 逐发脉冲。梯形求解、mm↔steps 换算、软限位、F/A 钳制全部属于 T7（规划器，后续任务）——**本任务不做这些**。自测代码里允许用 float 数学手工构造测试段（不引入 planner 抽象）。

## 2. 硬件/平台事实

| 项 | 值 |
|---|---|
| STEP / DIR | GPIO5 / GPIO6（bsp 已初始化为输出 LOW） |
| 步频范围 | 最慢 ~1 step/s（超长间隔）到默认配置上限 40kHz（max_feed 30000mm/min × 80 steps/mm）；PLAN 性能门限 50kHz（RMT 备选方案**不实现**，仅在报告注明判据核算：默认上限 40kHz < 50kHz，无需 RMT） |
| ISR 数学 | **只允许整数/定点运算，禁止浮点**（规避 IDF 在 Xtensa ISR 中 FPU 使用的限制与不确定性；段入队时在任务上下文完成 float→定点换算） |
| 定时器 | GPTimer（IDF v6 驱动 API），建议 1MHz 分辨率、one-shot alarm + ISR 内重装；**S3 定时器位宽按 IDF 文档核实（16-bit 可能性高）**——最慢 1 step/s = 1s 间隔远超 16-bit@1MHz 的 65ms 范围，**必须做链式等待**（大间隔拆为 ≤60ms 的连续 alarm） |
| 核亲和 | v1 全部 CPU0（与 comm_task 同核，SPSC 队列写法仍按跨核安全规范实现）；核间分配留作 T12 性能调优项 |

## 3. 组件 API（定稿，验收按此检查）

```c
/* components/step_engine/include/step_engine.h */

typedef struct {
    uint32_t steps;         /* 本段总步数 >0 */
    bool     dir;           /* 逻辑方向（true=正向）；$10 反相在输出层应用 */
    uint32_t accel_steps;   /* 加速段步数 */
    uint32_t cruise_steps;  /* 巡航段步数 */
    uint32_t decel_steps;    /* 减速段步数（=steps-前两者，显式携带） */
    float    accel;          /* steps/s^2 >0 */
    float    v_cruise;       /* steps/s >0 */
} step_seg_t;                /* 三段之和必须 == steps，enqueue 校验，违例拒绝 */

esp_err_t step_engine_init(int step_gpio, int dir_gpio);
bool      step_engine_enqueue(const step_seg_t *seg);   /* 队列满返回 false（T7 背压依据） */

/* 立即停（PLAN §4.4）：清队列、停脉冲、位置计数冻结在已执行值。
 * 从任务上下文调用（comm 任务 '!' 回调）。必须做到：从调用时刻起
 * 不再发出任何 STEP 脉冲，与 ISR 内在途处理竞态安全。 */
void      step_engine_stop(void);

/* 遥测快照（T9 消费）。position 为上电以来累计（方向感知，含 $10 反相）。
 * 32-bit 对齐读在 Xtensa 上原子，无需锁。 */
int32_t   step_engine_position_steps(void);   /* 已执行步数（带符号累计） */
uint32_t  step_engine_velocity_steps(void);   /* 当前瞬时速度 steps/s，静止=0 */
bool      step_engine_busy(void);             /* 队列非空或脉冲在途 */
void      step_engine_set_dir_invert(bool invert);  /* $10 运行时变更 */
```

队列：静态 SPSC 环形，深度 16（2 的幂），任务侧写 / ISR 侧读，volatile 头尾索引，写后发布（先写数据再更新 head）。ISR 侧 pop 后才更新 tail。

## 4. 算法规格

- **ISR 逐步递推（PLAN 指定物理模型）**：`v += a×dt; dt = 1/v`，加速/巡航/减速三段边界（步数）入队时预计算，ISR 按已走步数判段切换
- 定点实现：Q 格式与位宽自选（如 v 用 Q8/Q16、dt 用 µs），**报告必须包含推导与精度估算**；减速段为加速镜像（v 递减，低于最低限幅时用限幅值走完剩余步——纯降速到极低会 dt 爆炸）
- 首步间隔：从静止加速的首步 dt0 = √(2/a)（µs），入队时预计算
- STEP 脉冲：ISR 内拉高 → 忙等 ≥1µs（`esp_rom_delay_us(1)`）→ 拉低；脉宽要求来源：PLAN T6 验收项（TMC2209 实际要求远低于此，1µs 留裕量）
- DIR：段首与上段方向不同时，先设 DIR、忙等 ≥2µs 建立时间，再发首步；$10 反相在输出层异或
- 定时器：one-shot alarm，ISR 回调内重装下一间隔；间隔 > 定时器量程时链式等待；**alarm 到达间隔统计**在自测里做（见 §6 判据 2）

## 5. app_main 接线（合并冲突控制：只动指定锚点）

1. `#include "step_engine.h"`
2. `protocol_start()` 成功后（parser 注册之前或之后均可，选后者邻近处）：`step_engine_init(CONFIG_BSP_STEP_PIN, CONFIG_BSP_DIR_PIN);`
3. 新增一个停止回调函数 `static void cb_on_stop(void) { step_engine_stop(); }` 并注册：`protocol_register_stop_cb(cb_on_stop);`（替换 protocol 默认"仅打日志"占位——PLAN T6 节指定接入点；protocol.c 本身不改）
4. `cb_on_move` 占位**保持不动**（T7 planner 才接线，本任务不接运动命令）

## 5b. Kconfig / CMake

- `components/step_engine/Kconfig`：`CONFIG_STEP_ENGINE_SELF_TEST`（bool，默认 n）
- `main/CMakeLists.txt`：`PRIV_REQUIRES` 加 step_engine（只加不改）

## 6. 自测（`CONFIG_STEP_ENGINE_SELF_TEST=y`，上板自跑，≥20 例）

app_main 中置于既有自测段之后（与 settings/gcode 自测同位置插入）。用例组：

1. **步数精确性**：已知梯形（1000 步 0→1000Hz→0、加速/巡航/减速边界手算）入队→执行完毕→`position_steps` 变化量精确 =1000；多组速度/步数变体（含纯三角形的减速中途截断为巡航=0 的退化形）
2. **周期准确性**：ISR 记录相邻 alarm 到达间隔，分段统计（加速段/巡航段各取 ≥50 个间隔），与理论值误差 **<0.5%**（巡航段：间隔恒定=1/v_cruise；加速段：与递推理论曲线比对）
3. **立即停**：分别在加速中/巡航中/减速中触发 stop——从 stop() 调用时间戳到 ISR 记录的最后一个 STEP 脉冲时间戳 **<1ms**；busy() 立即为 false；position 冻结（stop 后 100ms 再读不变）；**stop 后再入队新段可正常执行**（不受上次 stop 影响，PLAN T10 验收项前移）
4. **队列行为**：满 16 段后 enqueue 返回 false；段边界校验（三段之和≠steps 拒绝）
5. **连续段**：两段同向/反向衔接——位置累计正确；反向衔接 DIR 建立时间生效（用例断言：方向变化步的 alarm 间隔 ≥ 正常间隔+2µs 建立窗）
6. **极慢速度**：1~2 steps/s 长间隔（链式等待路径）——间隔正确、无丢步
7. **反相标志**：set_dir_invert(true) 后位置符号翻转
8. **ISR 执行时间**：用 ccount 记录 ISR 内最大周期数，报告打印（判据：40kHz 步频预算 25µs，ISR 应 <5µs）

输出行：`PASS <case>` 逐例 + 末行 `SELF-TEST DONE: <n>/<total> PASS`。总时长控制在 30s 内（极慢速度用例用短步数）。

## 7. 构建验证

```
powershell -File idf_env.ps1 build     # 默认配置零错误
```
自测配置：sdkconfig 末尾追加 `CONFIG_STEP_ENGINE_SELF_TEST=y` 再构建。两个构建产物都要确认。

## 8. 完成报告格式

① 改动文件清单；② 定点格式与位宽、递推推导、精度估算（对照 <0.5% 判据）；③ 定时器位宽核实结论与链式等待实现说明；④ 自测用例数与预期上板输出；⑤ 双配置构建证据；⑥ RMT 判据核算（50kHz 门限 vs 默认 40kHz 上限）；⑦ 风险与未尽事项。
