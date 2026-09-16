# T7 任务包 · 运动规划器（planner）

> 分发对象：执行 Agent · 分支：`task/T7-planner`（主工作区直接开发，已由用户建好，勿做任何 git 操作——沙箱禁止，用户负责提交）
> 上游事实源：PLAN.md（§4.3 命令集、§4.6 错误码、§4.7 数值与单位、§5 参数表、T7 节 + v1.12 上游事实）、components/step_engine/include/step_engine.h、components/gcode_parser/include/gcode_parser.h、components/settings/include/settings.h、main/app_joint_test.c（段构造参考实现）、main/app_main.c

---

## 0. 执行指令（按序）

1. 通读本文件全文
2. 实现 `components/planner/`（API 见 §3，算法见 §4）
3. gcode_parser 接口变更（§5，本任务**唯一**许可触碰的既有组件）
4. app_main 接线（§6）+ Kconfig/CMake（§7）
5. 自测（§8，≥20 例）+ 三配置构建验证（§9）
6. 最终答复：改动文件清单、算法规格推导（含钳制链与取整规则）、自测预期输出、构建证据

**红线（越界即验收不通过）**：
- 禁止改动：`components/protocol/`、`components/step_engine/`、`components/settings/`、`components/tmc2209/`、`components/bsp/`、`main/app_joint_test.c`（联合验收参考实现，保持原样）
- `components/gcode_parser/` 仅限 §5 列出的四处改动；其余解析行为一字不动
- `main/app_main.c` 仅限 §6 列出的锚点
- 禁止 git 操作
- **自测运行时 TMC 必须处于失能态**：自测块在 app_main 中位于 tmc2209_init 之前、bsp_early_init 已置 EN 高——自测只驱动 STEP/DIR 电平，不得使电机转动（真运动验证留给上板验收）
- 规划器组件不得依赖 tmc2209 / protocol / bsp（CMake `REQUIRES` 仅 step_engine settings freertos）
- 全部逻辑在任务上下文，不引入浮点入 ISR（T6 ISR 纯整数路径，勿破坏——本任务天然满足，列此为红线防误加 ISR 回调）

---

## 1. 任务边界

本任务交付"规划层"：接收解析器输出的**绝对目标坐标（mm）**，完成单位换算、软限位校验、F/A 钳制、梯形/三角形速度曲线求解，构造 `step_seg_t` 入队给 step_engine（T6 已验收）。

不属于本任务：ISR 脉冲执行（T6 已交付）、状态机与 '!' 后新命令门控（T10）、状态帧遥测（T9）、编码器位置（T8）、电机使能联锁（M17/M18 已由 T5 接线，规划器不管使能状态直接入队）。

G0 与 G1 本版行为一致（PLAN §4.3），`rapid` 标志规划器忽略（预留）。

## 2. 上游事实（T5/T6 验收沉淀，实现前逐条核对）

| # | 事实 | 来源/锚点 |
|---|---|---|
| 1 | `step_seg_t`：`steps/accel_steps/cruise_steps/decel_steps` 三段之和**必须==steps**（uint32 加法），`steps>0`、`accel>0`、`v_cruise>0`（float 层）——enqueue 校验违例拒收 | step_engine.c `step_engine_enqueue`；T6 上板 P0#3 教训：此处笔误编译零告警 |
| 2 | **Q16.16 边界**：引擎换算 `float_to_q16(v)=(uint32)(v*65536+0.5f)`。v/a 必须 **<65536**（恰好 65536 → 0x100000000 回绕到 0），钳制上限统一用 `65535.0f` | step_engine.c:82 |
| 3 | **P3（v1.12 移交）**：$8 默认 2000mm/s² × 80 steps/mm = **160000 > 65535，默认参数下加速度就必须钳**；$7 极值 30000mm/min（=500mm/s）× k（$0=256 时 k=1280）= 640000 → v 同样必须钳 | PLAN v1.12 T7 节上游事实② |
| 4 | 引擎内部已有：减速段 v 限幅 10 steps/s、dt0=sqrt(2/a) 封顶 60s、>60ms 间隔链式等待——规划器**无需复刻**这些防护 | T6 定案 |
| 5 | DIR 高电平=位置递增；$10 反相在引擎输出层异或——规划器只管**逻辑方向** `dir = delta>0`，不碰 $10 | T6 定案⑤ |
| 6 | k = steps_per_mm = `settings_full_steps()×settings_microstep()/units_per_rev()`，默认 200×16/40=**80**；**订阅 $0/$1/$2 变更更新缓存** | settings.h:125；PLAN v1.12 事实③ |
| 7 | settings 订阅槽共 4：tmc2209 已占 1、planner 占 1、app_main $10 胶水占 1、**剩 1 留给 T9**；tmc2209 的订阅回调有 `!s_initialized` 守卫（自测期改 $0 安全） | tmc2209.c:475 |
| 8 | **parser 位置提交缺陷**：现行 `parse_g0_g1` 在 on_move 回调**之前**就更新 `current_pos_mm`（gcode_parser.c:280-287）——软限位拒动后坐标会错位，本任务必须改为"回调接受后才提交"（§5.2） | gcode_parser.c:280 |
| 9 | 队列深 16（SPSC）；`step_engine_busy()` = 队列非空或脉冲在途；`step_engine_stop()` 清队列后 busy 立即 false | step_engine.h |
| 10 | 段构造正确写法参考：app_joint_test.c `run_phase_a`（对称梯形：accel=decel=floor(v²/2a)）与 `seg_check_sum` | app_joint_test.c:93-134 |
| 11 | `step_engine_position_steps()`：上电累计、方向感知、32 位对齐读原子 | step_engine.h:84 |
| 12 | 模态 F/A：解析器已输出"本行生效值"（行内值或模态缓存），规划器直接消费，无需自己缓存 | gcode_parser.h:66-71 |

## 3. 组件 API（定稿，验收按此检查）

```c
/* components/planner/include/planner.h */

typedef enum {
    PLANNER_OK              = 0,
    PLANNER_ERR_SOFT_LIMIT  = 2,   /* 目标越软限位 / 段步数超上限 → error:2 */
} planner_status_t;

/* 初始化：pos_steps=0（D10 上电=0）、订阅 settings（缓存 k）、幂等。
 * 在 step_engine_init 成功之后、parser 创建之前调用。 */
esp_err_t planner_init(void);

/* 核心：规划一次到绝对目标的移动（任务上下文，protocol 行处理器调用）。
 * 管线：软限位校验 → $7/$8 mm 域钳制 → mm→steps → 梯形/三角求解
 *      （含引擎域钳制与下限）→ 入队（队列满阻塞背压）→ 位置提交。
 * 返回 PLANNER_OK / PLANNER_ERR_SOFT_LIMIT。 */
planner_status_t planner_enqueue(float target_mm, float feed_mm_min,
                                 float accel_mm_s2);

/* G92：把指令位置设为指定坐标（不产生运动；不查软限位——只约束移动目标，
 * 解析层已保证 |X|≤1e6）。 */
void planner_set_origin(float new_pos_mm);

/* 指令位置（T9 消费；单写者=comm 任务，32 位对齐读原子）。 */
int32_t planner_position_steps(void);
float   planner_position_mm(void);      /* = pos_steps / k */

/* 纯函数：为 |delta|=delta_steps（≥1）求解速度曲线填充 *out_seg（dir 由
 * 调用方填）。v_max_steps/accel_steps 为请求值（>0），函数内部完成引擎域
 * 钳制（<65536）与下限（≥1e-3）后求解。返回 false 仅当参数非法。 */
bool planner_solve(uint32_t delta_steps, float v_max_steps,
                   float accel_steps, step_seg_t *out_seg);

/* ---- 上板自测（仅 CONFIG_PLANNER_SELF_TEST=y 时存在） ---- */
#ifdef CONFIG_PLANNER_SELF_TEST
void planner_selftest_last_seg(step_seg_t *out);  /* 最近一次成功入队段的拷贝 */
int  planner_self_test_run(void);
#endif
```

## 4. 规划算法规格

### 4.1 管线（planner_enqueue）

```
k = 缓存的 steps_per_mm（订阅 $0/$1/$2）
1. 软限位：target_mm ∉ [settings_soft_min(), settings_soft_max()] → 返回 2
   （浮点含边界：≤ 上限、≥ 下限均合法）
2. target_steps = (int32_t)lroundf(target_mm × k)
3. delta = target_steps − pos_steps
   |delta| > 1e9 → 返回 2（防 uint32 溢出/荒谬行程；1e9 步@80steps/mm
   已远超 $2 最小值组合下的任何 sane 配置）
4. delta == 0 → pos_steps=target_steps，返回 0（不入队）
5. v = min(feed_mm_min,  settings_max_feed())  / 60 × k      /* $7 钳制 */
   a = min(accel_mm_s2, settings_max_accel()) × k            /* $8 钳制 */
6. planner_solve(|delta|, v, a, &seg)；seg.dir = (delta > 0)
7. 入队循环（阻塞背压 + stop 竞态处理，见 4.3）
8. 成功后 pos_steps = target_steps（指令位置=已接受命令的目标）
```

### 4.2 曲线求解（planner_solve，纯函数）

```
钳制（先于求解）：v = clamp(v, 1e-3, 65535.0)；a = clamp(a, 1e-3, 65535.0)
  —— 上限防 Q16.16 回绕（事实#2/#3）；下限防 float_to_q16 四舍五入到 0
    （v_q16=0 → ISR 除零；a_q16=0 → 永不加速）
d = v² / (2a)                       /* 加速到 v 所需步数 */
若 2d ≤ N：梯形
    accel_steps = decel_steps = (uint32)floorf(d)     /* 对称，A4 */
    cruise_steps = N − 2×accel_steps
    v_cruise = v                     /* 可为 0（2d==N 临界，引擎已验证合法） */
否则：三角（距离不足以加速到巡航速度的退化）
    accel_steps = (uint32)(N / 2)
    decel_steps = N − accel_steps    /* 奇数 N 差 1 步给减速段 */
    cruise_steps = 0
    v_cruise = sqrtf(2 × a × accel_steps)   /* 实际峰值，必 < v */
兜底：v_cruise = fmaxf(v_cruise, 1e-3f)
   —— N=1 时三角 v_cruise=sqrtf(0)=0，过不了引擎 ">0" 校验
性质（自测断言）：accel_steps+cruise_steps+decel_steps == N 恒成立；
  accel_steps=floor 路径保证 2×accel_steps ≤ N ⇒ cruise 不下溢
```

### 4.3 入队循环与 '!' 竞态（必须按此实现，报告需复述论证）

```c
while (!step_engine_enqueue(&seg)) {
    if (!step_engine_busy()) {
        /* 队列曾被填满且此刻空闲：唯一可能是 step_engine_stop() 清了队列。
         * 论证：enqueue 失败 ⇒ 队列满 ⇒ busy 必为 true；本检查与失败之间
         * 无任何延时（同次迭代，µs 级），而排空 16 段最快需 16 步 × 15µs
         * （v 上限 65535 steps/s）≈ 240µs ≫ 检查窗口，正常消化不可能为假。
         * 处理：该移动被 '!' 取消（D11 无恢复语义）——丢段、不更新位置、
         * 返回 PLANNER_OK（该 G1 行照常回 ok）。 */
        return PLANNER_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}
pos_steps = target_steps;
```

## 5. gcode_parser 接口变更（本任务唯一许可触碰的既有组件）

### 5.1 on_move 签名 void → int（头文件）

```c
/* gcode_parser.h —— 改前 */
void (*on_move)(void *ctx, float target_mm, float feed_mm_min,
                float accel_mm_s2, bool rapid);
/* 改后（返回值约定与 on_set_param 一致并扩展 2） */
int (*on_move)(void *ctx, float target_mm, float feed_mm_min,
               float accel_mm_s2, bool rapid);
/* 返回：0=接受；2=软限位拒绝（→error:2）；其余非零按 4 处理（防御） */
```

同步更新头文件注释块（回调返回值约定段落补 2=range）。

### 5.2 parse_g0_g1 位置提交时机（gcode_parser.c:280-287）

```c
    /* 改后：回调先行，接受后才提交位置（T7 软限位拒动不得错位坐标） */
    if (p->iface.on_move != NULL) {
        int ret = p->iface.on_move(p->iface.ctx, target_mm, p->modal_feed,
                                   p->modal_accel, is_g0);
        if (ret != 0) {
            return (ret == 2) ? PARSER_ERR_RANGE : PARSER_ERR_BAD_PARAM;
        }
    }
    p->current_pos_mm = target_mm;
```

模态 F/A 回写（272-278 行）保持在回调**之前**不变（F/A 行内值即使移动被拒也已生效——与 GRBL 一致，模态是行属性不是运动属性）。

### 5.3 gcode 自测同步（30 → 33 例）

- 既有 30 例的 on_move 桩改为返回 int 0（行为不变，全数应继续通过）
- 新增 ≥3 例：① G1 目标被 on_move 拒（桩返回 2）→ `error:2` 且位置不变；② G91 模式下被拒后再发相对移动，基准位置未被污染；③ 被拒行不改动模态 F/A（后续 G1 沿用旧值）

### 5.4 Python 参考模型同步（T11 教训：接口变更必须同步）

`components/gcode_parser/test/test_gcode_parser_verify.py` 同步 on_move 返回值语义与位置提交时机，并在主机重跑通过（python 直跑，构建证据里附结果）。

## 6. app_main 接线（只动指定锚点）

1. `#include "planner.h"`
2. `step_engine_init` 成功后（parser 创建之前）：`planner_init();`
3. 替换占位回调：
   ```c
   static int cb_on_move(void *ctx, float target_mm, float feed_mm_min,
                         float accel_mm_s2, bool rapid)
   {
       (void)ctx; (void)rapid;   /* G0/G1 本版行为一致（PLAN §4.3） */
       return (int)planner_enqueue(target_mm, feed_mm_min, accel_mm_s2);
   }
   static void cb_on_set_origin(void *ctx, float new_pos_mm)
   {
       (void)ctx;
       planner_set_origin(new_pos_mm);
   }
   ```
   `on_set_absolute / on_set_relative` 保持 no-op（解析器内部已换算 G90/G91 并输出绝对目标——T4 设计决策，头文件已注明）
4. **$10 补线（T6 遗留孤儿接口，5 行）**：`step_engine_set_dir_invert(settings_invert_dir());`（planner_init 后）+ app 级 settings 订阅回调，`idx==10 → step_engine_set_dir_invert(value != 0.0f)`。当前 $10 全链路无消费方（set_dir_invert 仅自测调用过），不补则运行时改 $10 无效
5. 自测块插入（step_engine 自测之后）：
   ```c
   #ifdef CONFIG_PLANNER_SELF_TEST
       planner_self_test_run();
   #endif
   ```
6. 日志镜像 `#if` 列表追加 `|| defined(CONFIG_PLANNER_SELF_TEST)`（上板验收要 12V 墙充供电，需 UART1 镜像通道；自测期 TMC 失能，PC USB 也可见）

## 7. Kconfig / CMake

- `components/planner/Kconfig`：`CONFIG_PLANNER_SELF_TEST`（bool，默认 n）
- `components/planner/CMakeLists.txt`：
  ```cmake
  idf_component_register(SRCS "planner.c"
                      INCLUDE_DIRS "include"
                      REQUIRES step_engine settings freertos)

  if(CONFIG_PLANNER_SELF_TEST)
      target_sources(${COMPONENT_LIB} PRIVATE "planner_self_test.c")
  endif()
  ```
- `main/CMakeLists.txt`：`REQUIRES` 追加 planner（只加不改）

## 8. 自测（`CONFIG_PLANNER_SELF_TEST=y`，上板自跑，21 例）

前置：`planner_self_test_run` 开头自检 `planner_init()`（幂等）与 `step_engine_init`（幂等，参照 step_engine_self_test.c:871 写法）。位置断言一律取 `step_engine_position_steps()` 相对自测起点初值的**增量**（不假设 0）；移动等待用轮询 busy + 超时，不用固定延时（T6 test 9 教训）。改 $0 的用例必须先 `settings_get` 存原值、用例后恢复（tmc2209 订阅有守卫，自测期改 $0 安全——事实#7）。结尾 `step_engine_stop()` + 等 idle 收尾。

**纯求解组（planner_solve，不碰引擎，9 例）**：

| # | 输入 | 断言 |
|---|---|---|
| 1 | N=10000, v=1000, a=1000 | 梯形 500/9000/500，v_cruise==1000 |
| 2 | N=100, v=1000, a=1000 | 三角 50/0/50，v_cruise≈sqrt(2×1000×50)≈316.23 |
| 3 | N=101, v=1000, a=1000 | 三角 50/0/51（奇数步差 1 给减速） |
| 4 | N=1000, v=1000, a=1000 | 临界 2d==N：500/0/500（cruise=0 梯形退化） |
| 5 | N=1e7, v=200000, a=1000 | v_cruise==65535.0（引擎域钳制），梯形三段和==N |
| 6 | N=10000, v=1000, a=200000 | seg.accel==65535.0（**P3 用例：$8 默认值换算 160000 必须钳**） |
| 7 | v=1e-9, a=1e-9, N=10 | v_cruise/accel ≥ 1e-3（下限），段合法 |
| 8 | N=1, v=100, a=100 | 0/0/1，v_cruise>0（兜底生效） |
| 9 | 参数表 ≥5 组循环 | 性质断言：三段和==N、accel_steps≤N、v_cruise≤min(v_max,65535) |

**集成组（planner_enqueue → step_engine 真执行，TMC 失能只走电平，12 例）**：

| # | 操作 | 断言 |
|---|---|---|
| 10 | 目标 = 初值 + 1234 步对应 mm | 引擎位置增量**精确** +1234，busy 先真后假 |
| 11 | 反向 −567 步 | dir 逻辑反向，位置 −567 |
| 12 | 三次连续移动（+1234/−567/+891） | 位置净增量精确（队列衔接不丢步） |
| 13 | 目标 == 当前位置 | 不产生段（last_seg 不更新、不 busy），位置不变 |
| 14 | +0.02mm@k=80（1.6 步）与 +0.001mm（0.08 步） | 前者 2 步（lroundf 就近取整）；后者 0 步 no-op |
| 15 | feed=9000（$7 默认 3000 钳制） | last_seg.v_cruise == 3000/60×80 = 4000 |
| 16 | accel=9000（$8 默认 2000 → 160000 → 引擎域再钳） | last_seg.accel == 65535.0（P3 端到端） |
| 17 | 目标 601mm（$13 默认 600） | 返回 2；位置不变；引擎无任何动作 |
| 18 | 目标 600.0 与 0.0（边界） | 均成功（含边界语义） |
| 19 | planner_set_origin(100) 后 enqueue(100.5) | planner_position_mm()==100；随后 +40 步 |
| 20 | settings_set($0=8) 后 +1mm，再恢复 $0=16 | 40 步（订阅重派生 k=200×8/40=40），恢复后 80 步 |
| 21 | 连续 17 段小移动（各 64 步） | 全部返回 OK、最终位置精确 17×64=1088（自然覆盖队列满阻塞路径） |

目标 mm 取值用 `期望步数/k` 反推（float 相对误差 1e-7 ≪ 0.5 步，回乘取整稳定），**期望步数硬编码在用例里**（避免用同一公式生成期望造成同义反复）。

输出行：`PASS <case>` 逐例 + 末行 `SELF-TEST DONE: <n>/<total> PASS`。总时长 ≤30s。

## 9. 构建验证（三配置，全部零错误零警告）

```
powershell -File idf_env.ps1 build                                    # ① 默认
# sdkconfig 追加 CONFIG_PLANNER_SELF_TEST=y 构建                      # ② planner 自测
# sdkconfig 再追加 CONFIG_GCODE_PARSER_SELF_TEST=y 构建                # ③ 验证 33 例解析自测
```

## 10. 完成报告格式

① 改动文件清单（对照 §0 红线自查）；② 算法推导：钳制链顺序（mm 域 $7/$8 → 引擎域 65535/下限）、梯形/三角判别式、取整规则、4.3 竞态论证复述；③ 自测用例表与预期上板输出；④ 三配置构建证据 + Python 模型重跑结果；⑤ 风险与未尽事项（T9/T10 接口预留：planner_position_mm 消费方、'!' 后新命令门控归 T10 状态机）。

---

## 附 A：与 PLAN v1.12 的偏差预登记（验收时入档）

1. `planner_enqueue` 返回 `bool` → `planner_status_t`（0/2）：软限位触发 `error:2` 是 PLAN 验收项，bool 无法表达；"队列满返回 false（comm 侧阻塞背压）"的**阻塞语义保留**，机制内化为函数内部 vTaskDelay 循环
2. 新增纯函数 `planner_solve` 与自测钩子 `planner_selftest_last_seg`（PLAN 未列，为可测性——求解与入队解耦、钳制结果可观测）
3. 单测执行路径：宿主机单测 → 上板自测编入固件（v1.8 定案沿用，本机无主机编译器）
4. gcode_parser `on_move` void→int + 位置提交时机修正：PLAN T7 节未预见，系 §4.6 `error:2` 传播的唯一通路（现行 gcode_parser.c:280 先提交后回调，拒动即错位）
