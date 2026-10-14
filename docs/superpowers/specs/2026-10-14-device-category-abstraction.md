# 器件类别抽象与解释层上收设计（magnetic_encoder / motor）

## 背景与目标

上一轮域解耦（[设计](2026-10-14-device-decoupling-design.md)）按应用域切出了 cartpos / poleang / motor 三个域组件，业务调用面已经干净。但三个域组件都把「器件本身」与「本台倒立摆对器件的解释」合在一起：

- cartpos 内嵌皮带机械比例：µm 线性单位预设了皮带传动（[as5600.h](../../components/as5600/include/as5600.h) 的 counts→µm 固定 40 mm ÷ 4096），`$2` 订阅、原点偏移也在器件侧。
- poleang 内嵌摆的直立定义：「直立 = 0」的零位与符号标定（POLE_ZERO_URAD / POLE_INV）在域实现内逐次读业务参数表。
- motor 内嵌机械换算与协议语义：mm↔steps 换算消费 `$0×$1÷$2`，`motor_set_wdt_ms` 是 [protocol.md](../../docs/protocol.md) 的 VEL 命令看门狗。
- as5600 与 mt6835 的采样任务、失败计数、速度窗结构互为抄录（[mt6835.c](../../components/mt6835/mt6835.c) 头注释自述「结构照抄 as5600」）。

本设计的目标：**器件按类别抽象（magnetic_encoder、motor），类别契约只含器件本征语义与原生单位（counts、steps），不依赖本项目任何业务概念；「拿器件做什么」全部上收到业务侧解释层**。机械比例、符号、零位、原点、mm 换算、命令看门狗参数只在解释层出现。

换型故事随之扩展：换编码器芯片 = 换类别实现（menuconfig 选择编入）；换机械（皮带轮、行程、绳长）= 只动解释层参数，器件类别层零改动；第二台位可整体复用 magnetic_encoder / motor 两个类别组件。

非目标：运行期热插拔器件；去除 ESP-IDF 类型（同板换件场景，见上一轮设计的范围界定）。

## 技术选型（现有组件核对）

2026-10-14 核对 ESP Component Registry（检索词：encoder、as5600、mt6835、stepper motor、tmc2209；MT6835 另经 Web 检索确认注册表无托管组件），并拉取 [espp/magnetic_encoder](https://github.com/esp-cpp/espp/tree/main/components/magnetic_encoder) 源码评估接口设计。结论：**无覆盖本设计组合的现成抽象，但其接口设计有直接参考价值（见「类别契约」各采纳注记）**。最接近的四个组件与各自缺口：

| 组件 | 覆盖面与缺口 |
|---|---|
| [espp/magnetic_encoder](https://components.espressif.com/components/espp/magnetic_encoder) v1.3.8 | 磁编码器 CRTP 基类（周期更新环、原始计数累计、速度估计），与本设计同旨，命名与接口词汇的参考来源；不采用其代码：C++ 框架（连带 base_peripheral / timer / logger 依赖），无 MT6835 实现，无连续失败健康语义（is_ok），且为个人主导的社区项目（非 Espressif 官方，96% 提交来自单一维护者），维护风险与本项目取向不匹配 |
| [espressif/as5600](https://components.espressif.com/components/espressif/as5600) v0.1.0 | 官方裸 I2C 寄存器驱动（原始角 / 角度 / 磁状态，句柄式）；等价于本项目 as5600_hw 层，低于类别层，无多圈计数、速度估计与健康策略 |
| [gin66/fastaccelstepper](https://components.espressif.com/components/gin66/fastaccelstepper) v2.0.0 | 成熟步进脉冲引擎（RMT/MCPWM/I2S、斜坡与命令队列、支持 IDF 6.1），是 step_engine 的真实替代候选；缺口：无命令新鲜度看门狗（WDT_MS 的透明恢复语义需在其上自包一层）、C++ API（本项目 C 组件需包装）、换用即全部运动行为需重新上板验证。当前单轴、步率远低于其优势区间，不引入；记录为未来多轴 / 高步率场景的替换候选 |
| [pedrominatel/tmc2208](https://components.espressif.com/components/pedrominatel/tmc2208) v0.1.0 | 仅 STEP/DIR（RMT），UART 寄存器面未实现且不含 TMC2209，不覆盖本项目 tmc2209 的寄存器策略层（电流 / 细分 / VSENSE 分档 / fail-fast） |

其余 encoder 检索结果为增量式正交编码器（PCNT 类）驱动，与本设计的绝对式磁编码器非同类器件。MT6835 注册表无组件，GitHub 上的独立实现（[Hotakus/mt6835](https://github.com/Hotakus/mt6835)，纯 C 跨平台；[simplefoc/Arduino-FOC-drivers](https://github.com/simplefoc/Arduino-FOC-drivers) 内的 SPI 驱动）均非注册表托管；本项目 mt6835_hw 的帧格式已经过实机抄录核对。

自研边界据此确定：类别层的采样任务、失败计数、速度窗是既有已验证代码的**重定位**，不是新实现；新增代码限于两个契约头、解释层与句柄接线。is_ok 连续失败语义与 WDT_MS 命令看门狗是本项目的协议契约（[protocol.md](../../docs/protocol.md)），外部库不提供。

## 总体结构

```
业务层        cartfsm                telemetry
              cartproto  cartparams  settings
                 │                        │
解释层        cartpos    poleang    cartaxis
（业务侧）  （counts→x） （角量→θ）  （mm↔steps）
                 │           │           │
器件类别抽象层  magnetic_encoder（契约，句柄式多实例）  motor（契约，steps 域，句柄式）
                 │                          │
实现      magnetic_encoder_as5600   magnetic_encoder_mt6835   motor_tmc2209
              │                 │               │
器件驱动核心  as5600(hw+tracker)  mt6835(hw+urad)  tmc2209 + step_engine
```

绑定方式：**句柄式运行期绑定**。每个类别实现经自己的 init 创建实例并返回句柄；解释层与组装点持有句柄调用契约函数。这是 ESP-IDF 驱动层（i2c_master / spi_master）的既有惯例，天然支持一个类别多个物理实例（本台位：两台编码器、一台电机）。

## 组件清单

| 组件 | 形态 | 职责 |
|---|---|---|
| `components/magnetic_encoder/` | 类别组件（新增） | magnetic_encoder 契约 + `magnetic_encoder_as5600.c` / `magnetic_encoder_mt6835.c` 实现（1 kHz 采样任务、失败计数、is_ok、16 ms 速度窗） |
| `components/motor/` | 类别组件（重定位） | motor 契约（steps 域）+ `motor_tmc2209.c` 实现（tmc2209 + step_engine 组合）；删除 mm 换算 |
| `components/cartaxis/` | 解释组件（新增） | mm↔steps 换算、steps_per_mm 派生、`$0` 单一写者、X_INV 取反 |
| `components/cartpos/` | 解释组件（重定位） | encoder counts → x_um / vx：`$2` 比例、原点偏移；签名不变 |
| `components/poleang/` | 解释组件（重定位） | encoder 角量 → θ / vth：零位、符号（读 cartparams）；签名不变 |
| `components/as5600/` | 器件驱动组件（瘦身） | 只留 as5600_hw（I2C 读层）+ as5600_tracker（多圈纯逻辑）；设备策略与任务移入 magnetic_encoder_as5600.c |
| `components/mt6835/` | 器件驱动组件（瘦身） | 只留 mt6835_hw（SPI 帧层）+ mt6835_urad（±π 环绕纯函数） |
| `components/tmc2209/`、`components/step_engine/` | 器件驱动组件（瘦身） | tmc2209 去 settings 订阅；step_engine 删除 `step_engine_set_dir_invert`（符号上收解释层） |

**溶解的文件**：`as5600.c`、`mt6835.c`（设备策略并入类别实现）、`cartpos_as5600.c`、`poleang_mt6835.c`（解释层不再按器件分实现——解释层拿任意 encoder 句柄都能工作）、`motor_conv.c`（换算并入 cartaxis 纯函数，宿主测试随迁）。

**不动**：cartfsm、telemetry、cartproto、cartparams、settings、bsp 的组件边界与公开 API（cartfsm / telemetry 的调用点按下文改名，语义不变）。

## 类别契约

### magnetic_encoder（绝对式磁编码器）

句柄式多实例。全部函数线程安全，可在任务上下文任意调用。命名与 count / accumulator 词汇参考 espp/magnetic_encoder。

| 函数 | 语义 |
|---|---|
| `int32_t magnetic_encoder_counts_per_rev(magnetic_encoder_handle_t)` | 每转原始计数（分辨率）：AS5600 为 4096，MT6835 为 2097152 |
| `int64_t magnetic_encoder_position_counts(magnetic_encoder_handle_t)` | 多圈累计位置（accumulator，原始 counts，带符号，原生方向，自 init 累计）；首样本前为 0。int64 承载：连续单方向旋转不溢出（参考 espp 的溢出论证；本台位 int32 即够，类别层取通用宽度） |
| `int32_t magnetic_encoder_velocity_counts_per_s(magnetic_encoder_handle_t)` | 角速度估计（counts/s，16 窗均值）。量化级 62.5 counts/s（16 ms 窗整数化）；窗口未满为 0 |
| `bool magnetic_encoder_is_ok(magnetic_encoder_handle_t)` | 连续 ≥10 次读取失败为 false；一次成功即恢复 true |

原生单位原则：契约只说 counts 与每转计数，不出现任何机械单位（mm、µrad）与已解释角度。µrad / mm 换算是纯数学或含机械参数的换算，归解释层。

**混叠上界（契约约束，参考 espp 的显式文档化）**：unwrap 以 ±每转计数/2 为阈值，单个采样周期内转过超过半转即混叠——多圈累计与速度估计都不可信。AS5600 @1 kHz 上界约 122 rev/s，MT6835 @1 kHz 上界约 403 rev/s，均远高于本台位物理速度。

预留扩展：后续版本可在本契约增加同拍快照读（位置 + 速度 + 采样时间戳一次取回）。espp 以 `run_task=false` 外驱更新循环支持调用方统一节拍驱动多台编码器，是同拍相干采样的可行机构，届时可一并评估。本期不实现，两编码器采样相位关系与现状一致（独立任务、无相干性保证）。

器件特有面留在实现头：`magnetic_encoder_as5600.h` 暴露 AS5600 自测用的原始角读取（B 组用例）；`magnetic_encoder_mt6835.h` 暴露 STATUS 位与失败分诊快照（打日志用）。公共契约不含这些。

### motor（步进电机，steps 域）

句柄式。全部函数线程安全。steps 指**当前细分档的一个微步**——微步档位变更即单位含义变更（见 `motor_set_microstep`）。

| 函数 | 语义 |
|---|---|
| `esp_err_t motor_enable(motor_handle_t, bool en)` | 驱动器使能（低有效 EN 引脚）；en=true 等待 ≥1 ms 唤醒；仅实现未初始化时返回非 ESP_OK |
| `void motor_set_vel(motor_handle_t, long steps_per_s)` | 设定带符号目标速度并喂命令看门狗；引擎内部以定点递推平滑逼近，换向经零速 |
| `void motor_halt(motor_handle_t)` | 目标清 0 斜坡停；不喂看门狗 |
| `void motor_stop(motor_handle_t)` | 急停：停脉冲、清递推状态与目标、冻结指令位置 |
| `bool motor_at_rest(motor_handle_t)` | true ⇔ 当前速度精确为零（引擎内部定点域判零，不经整数舍入） |
| `int32_t motor_position_steps(motor_handle_t)` | 指令位置（微步，自上电累计） |
| `void motor_set_position(motor_handle_t, int32_t steps)` | 指令位置重定基；前置条件：电机静止（无脉冲发出），违反为未定义 |
| `void motor_set_accel(motor_handle_t, uint32_t steps_per_s2)` / `motor_set_decel(...)` | 追踪加速度 / 减速度档 |
| `void motor_set_wdt_ms(motor_handle_t, uint32_t ms)` | 命令新鲜度看门狗：超过 ms 未收到有效速度命令，按减速档斜坡停（对任何遥控运动轴通用的安全特性） |
| `void motor_set_microstep(motor_handle_t, int ms)` | 细分档位（2^n，1–256）；前置条件：电机静止；变更后 steps 单位按新档解释，指令位置计数不重置——调用方负责重定基 |
| `void motor_set_current(motor_handle_t, int run_ma, int hold_ma)` | 运行 / 保持电流 |

契约无方向旋钮：+x 之类应用方向约定不属于步进电机本征。方向符号由解释层对指令与读数统一取反实现（与原 DIR 引脚翻转在 mm 面完全等价：速度、位置、斜坡、看门狗均对称）。

## 解释层组件

解释层属业务侧：可以依赖 settings / cartparams（这是业务数据层），不再构成反向依赖。

### cartpos（小车位置测量，重定位）

公开签名不变（消费方 cartfsm / telemetry 零改动）：

| 函数 | 语义（与现行为的关系） |
|---|---|
| `int64_t cartpos_position_um(void)` | x_um = round(counts × `$2` × 1000 ÷ counts_per_rev) − 原点偏移 µm。换算统一四舍五入；旧实现的 40 mm 整数精确路径为截断除法，两者相差 ≤1 µm（数值域兼容，见迁移不变量 4） |
| `float cartpos_velocity_mm_s(void)` | counts/s × `$2` ÷ counts_per_rev（浮点）。与旧浮点路径相差 ≤1 mm/s |
| `bool cartpos_is_ok(void)` | 转发 magnetic_encoder_is_ok（10 次失败语义不变） |
| `void cartpos_set_origin_mm(float new_pos_mm)` | 原点偏移改存 µm 整数（旧为 mm 浮点，读时取整）；声明语义不变 |

新增生命周期：`cartpos_init(magnetic_encoder_handle_t enc)`。比例随 `$2` 订阅刷新（订阅从 as5600 移入本层）。`$2` 语义由此收敛为单一权威：位置与速度同源同值，旧实现「位置走固定 40 mm 比例、速度跟 `$2`」的分裂消除。

### poleang（摆角测量，重定位）

公开签名不变。`poleang_init(magnetic_encoder_handle_t enc)`。

| 函数 | 语义 |
|---|---|
| `int32_t poleang_theta_urad(void)` | θ = ±π 环绕(counts × 2π ÷ counts_per_rev 对应角 − POLE_ZERO_URAD)，POLE_INV 为真时取反；首样本前为 0 |
| `int32_t poleang_vth_mrad_s(void)` | round(counts/s × 2π × 1000 ÷ counts_per_rev)，符号随 POLE_INV；量化级细于旧实现的 1 mrad/s，与旧值相差 ≤1 mrad/s |
| `bool poleang_is_ok(void)` | 转发 magnetic_encoder_is_ok |

零位与符号（POLE_ZERO_URAD / POLE_INV）继续逐次读 cartparams（cartparams 本就是业务侧数据层，依赖方向合法）。

### cartaxis（小车直线轴命令，新增）

继承旧 [motor.h](../../components/motor/include/motor.h) 的全部业务面签名（cartfsm / flowtest 调用点改名、语义不变）：

```
cartaxis_init(motor_handle_t m)
cartaxis_enable(bool en)
cartaxis_set_vel_mm_s(long v)      cartaxis_halt(void)      cartaxis_stop(void)
cartaxis_at_rest(void)             cartaxis_cmd_position_um(void)
cartaxis_set_position_um(long um)  /* 前置：静止 */
cartaxis_set_accel_mm_s2(long a)   cartaxis_set_decel_mm_s2(long d)
cartaxis_set_wdt_ms(long ms)
```

职责与语义：

- **换算**：mm↔steps 用 settings 派生的 steps_per_mm（`$0`×`$1`÷`$2`），即读即算；四条换算为宿主可测纯函数（原 motor_conv 迁入，mm/s→steps/s 改为 lround，不再经 Q16.16 暴露）。
- **X_INV**：为真时对指令与读数统一取反（替代旧 `motor_set_dir_invert`）。`step_engine_set_dir_invert` 随迁移删除。
- **`$0` 单一写者**：本组件是细分档位唯一的生效路径——订阅 `$0/$1/$2`，`$0` 变更时按下表处理；tmc2209 组件不再订阅任何 settings 项。旧实现「订阅回调忽略寄存器写返回值、换算与实际细分可脱钩」的缺陷（双消费者无协调）由此消除。

`$0` 变更处理（决策表；`$1/$2` 变更仅影响换算系数，即时生效，无状态动作）：

| 变更时电机状态 | 动作 |
|---|---|
| 静止（含 DISABLED / READY 静止 / FAULT 整定完） | 回调内立即执行：记录当前 cmd_position_um → `motor_set_microstep` → 按新 steps_per_mm 重定基（µm 值不变） |
| 运动中 | `motor_halt()` 并挂起变更；此后任意 cartaxis 入口在检测到静止时按上行流程应用（cartfsm 监督任务 1 kHz 调用 cartaxis 读数，应用滞后 ≤ 数拍） |

## 实现侧

- `magnetic_encoder_as5600.c`：吸收现 as5600.c 的采样任务、失败计数、速度窗、多圈 tracker 组合；init 收接线 config（I2C 总线、地址、时钟、方向镜像布尔）并创建实例返回句柄。方向镜像是磁铁安装属性，留在实现 config（组装点从 Kconfig 折算传入），不出现在契约。
- `magnetic_encoder_mt6835.c`：同理（SPI 主机、时钟、CS）；STATUS 变化告警与失败分诊日志随迁。
- `motor_tmc2209.c`：现实现去 mm 化——换算与 spm 移除，速度/加速度/位置改为 steps 域直通 step_engine；`motor_set_microstep` 转发 tmc2209 细分写、`motor_set_current` 转发电流写（含 VSENSE 分档）。EN 经 config 的 en_gpio 直驱（现状不变）。
- `tmc2209` 组件：删除 settings 订阅与回调；寄存器策略保留。`$3/$4`（电流）改由组装点订阅中转：app_main 订阅 settings，收到 `$3/$4` 变更调用 `motor_set_current`。
- `step_engine`：删除 `step_engine_set_dir_invert`（无调用者）。
- Kconfig：magnetic_encoder 组件内为两个**独立 bool**（`MAGNETIC_ENCODER_AS5600` / `MAGNETIC_ENCODER_MT6835`，默认 y——两实例并存，不是 choice）；motor 组件保留 choice（`MOTOR_IMPL_TMC2209`）。
- CMake 约束（上一轮实测的求值陷阱）：类别组件的 REQUIRES / PRIV_REQUIRES 一律**无条件字面量声明**，仅 SRCS 随 Kconfig 宏条件编入。

## 组装契约

初始化次序：`bsp_early_init → settings_init → bsp_init → magnetic_encoder_as5600_init(&cfg, &enc_cart) → magnetic_encoder_mt6835_init(&cfg, &enc_pole) → motor_tmc2209_init(&cfg, &mot) → cartpos_init(enc_cart) → poleang_init(enc_pole) → cartaxis_init(mot) → 组装点订阅 $3/$4 中转 → cartproto_start → cartfsm_start → telemetry_start`。

app_main 是唯一创建实例、分发句柄的位置：三个实现 config 仍由 app_main 读取 Kconfig 宏与 bsp 句柄组装；解释层只拿句柄。测试工程入口同样组装。

## 依赖规则

构建期可检查的规则（迁移完成后生效）：

1. cartfsm 的 REQUIRES = cartpos、poleang、cartaxis、cartparams、cartproto；不得出现 motor、magnetic_encoder、器件驱动组件或 bsp。
2. telemetry 的 REQUIRES = cartpos、poleang、cartproto。
3. 解释层（cartpos / poleang / cartaxis）REQUIRES magnetic_encoder 与/或 motor 契约组件 + settings / cartparams；不依赖器件实现与 bsp。
4. 类别契约头（magnetic_encoder.h / motor.h）零组件依赖。类别实现 REQUIRES 器件驱动组件与 ESP-IDF 驱动库；**不得 REQUIRES settings、cartparams、bsp**（业务数据反向依赖清零，旧 as5600 的 `$2` 订阅与 tmc2209 的 `$0/$3/$4` 订阅全部移除）。
5. 器件驱动组件不 REQUIRES bsp / settings / cartparams；只被类别实现与测试工程引用。
6. cartproto 现状（REQUIRES bsp）不动；app_main 引用全部实现头并承担 `$3/$4` 中转。

## 测试与验证

- **宿主测试**：cartfsm_core、cartproto_parse、mt6835_crc、mt6835_urad、vel_ramp 不动（组件路径随瘦身调整 include 路径）；motor_conv 的 15 用例随换算迁至 cartaxis 纯函数（mm/s→steps/s 断言改为 lround 语义，其余向量不变）；telemetry_format_test 的期望值构造不再 include as5600.h（counts→µm 常量移入 cartpos 后改引 cartpos 或测试内自带）。
- **测试工程（test/）**：器件 B 组自测改经实现头（magnetic_encoder_as5600.h / magnetic_encoder_mt6835.h 的器件自测 API）；flowtest 电机调用面 motor_* 改 cartaxis_*，注入钩子劫持的 cartpos_is_ok / poleang_is_ok 名不变。
- **上板回归**：STATE 流比对口径从逐字节改为**数值域**（x_um ±1 µm、vx ±1 mm/s、vth ±1 mrad/s，行格式与 reason 字符串仍逐字节）；其余沿用上一轮计划[附录 A](../plans/2026-10-14-device-decoupling.md) 七项。

## 迁移不变量

调用面替换不得改变以下端到端契约（行为的权威定义见各条链接）：

1. **FAULT 停机次序**：斜坡置 0（加速度临时切 WDT_DEC 档）→ 每拍重发并等速度精确为零 → EN 关 → 推 MODE。链路 cartfsm → cartaxis → motor，次序不变。权威定义见 [cartfsm.c](../../components/cartfsm/cartfsm.c)。
2. **重定基时序**：MARK_RIGHT 完成时编码器位置与指令位置同帧重定基；ENABLE 先重锚指令位置再发布模式。权威定义见 [cartfsm.c](../../components/cartfsm/cartfsm.c)。
3. **锁序**：cartfsm 共享状态读写持 s_lock，cartaxis 调用在锁外。权威定义见 [cartfsm.c](../../components/cartfsm/cartfsm.c)。
4. **协议数值域兼容**：行格式、字段顺序、MODE reason 值（"AS5600"、"MT6835"、"STEP_LOSS"）逐字节不变；四个数值字段因换算路径统一（四舍五入、counts/s 整数化）允许 ≤1 个旧量化级的差异（x_um ≤1 µm、vx ≤1 mm/s、vth ≤1 mrad/s、指令位置同 x_um）。权威定义见 [protocol.md](../../docs/protocol.md)。
5. **测试注入语义**：编码器失联注入与位置偏置注入的用例行为不变。权威定义见 [test_cartfsm_flow.c](../../components/cartfsm/test/test_cartfsm_flow.c)。
6. **解释层单一权威**：机械比例（`$2`）、方向（X_INV、方向镜像安装配置）、零位（POLE_ZERO_URAD / POLE_INV）、细分档（`$0`）只在解释层与组装点消费；类别层与器件层无业务参数依赖（依赖规则 4 的构建期检查即其验收）。

## 范围界定（本设计不改变归属的内容）

- MODE reason 字符串中的器件名保留（协议字节兼容是显式契约；换器件时 reason 语义过期为已知接受的耦合）。
- cartproto 对 bsp 的 UART1 接线维持现状。
- 两编码器采样的相干性（同拍快照、时间戳）仅作契约预留，本期不实现。
- 速度估计算法维持现状（16 窗均值、按标称周期折算）；espp 以实际时戳差测速并在读失败时保持时间戳的做法，记录为未来改进候选（故障恢复瞬间的窗口低估可由此消除），不在本次迁移改变估计器语义。
- `$0` 运动中变更从「未定义」收紧为「先斜坡停再应用」——这是对原有未定义边界的显式行为定义，非语义变更。
- step_engine 内部 Q16.16 定点递推保留（精确判零与斜坡平滑依赖它），契约不暴露该记法。
