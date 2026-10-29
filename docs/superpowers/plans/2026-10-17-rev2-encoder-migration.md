# rev2 板编码器迁移与骨架上提实施计划

- 定稿：2026-10-17
- 背景：硬件改版 rev2（[hardware.md](../../hardware.md)）——小车编码器 AS5600（I2C）→ 板载 MT6835（SPI 12/11/13/10），摆角编码器改外接 MT6835（SPI 47/48/38/14），两条独立 SPI 控制器；协议通道 UART1→UART0；烧录日志切原生 USB。
- 审计结论（2026-10-17 讨论，用户已接受方案乙）：「句柄式多实例」为纸面契约（实现状态/驱动 hw 层/bsp 三层单例），类别层只抽了读数接口、未抽采样骨架（两实现八成代码重复，unwrap 两套实现分裂）。
- 本计划覆盖编码器迁移主线与 rev2 其余固件适配（通道、新增外设）。**外部行为契约全部不变**：magnetic_encoder 四函数契约、counts 原生单位、线协议、遥测、参数注册表——解释层（cartpos/poleang）预期零改动。

## 已确认契约与不变量

1. 类别契约 `magnetic_encoder.h` 四函数（counts_per_rev / position_counts / velocity_counts_per_s / is_ok）签名与语义不变。
2. 速度估计语义不变：16 ms 窗（17 样本）位置差 × 62.5。
3. 失败判定语义不变：连续 ≥10 次读失败 NOT OK，一次成功恢复。
4. 解释层零改动：cartpos/poleang 不因迁移修改（µm/count 比例随 counts_per_rev 自动重算）。
5. 方向语义归解释层：小车与摆角的方向翻转只经 `X_INV`/`POLE_INV`（cartparams）；器件读层与类别层不引入方向概念。as5600 组件的 `invert_dir`（Kconfig）保留在该组件内，rev2 组装不使用。
6. 采样任务布局不变：CPU1、优先级 3、1 kHz；双实例任务名区分（如 `me_cart` / `me_pole`）。

## 设计决策

| # | 决策 | 内容 |
|---|---|---|
| D1 | 骨架上提 | 采样任务、失败计数、速度窗、原子发布、生命周期收进类别组件公共骨架（新 TU）；两实现退化为薄读层（读帧回调 + counts/rev + 失败分诊文案 + STATUS 语义）。实例状态全部进实例结构，任务 arg=实例指针——「句柄式多实例」兑现为同实现可多实例。 |
| D2 | unwrap 上提统一 | 多圈累计（单圈绝对角 unwrap）上提为类别组件的参数化纯逻辑（counts_per_rev 为参数），宿主可测。阈值规则统一采用 as5600_tracker 的非对称式（+cpr/2、−cpr/2+1，全值域恰好覆盖一次，论证注释随迁）；与 mt6835 现对称式的差异仅在物理不可达的混叠区（>403 rev/s），行为无实际变化。种子语义（首样本定义原点、零位移）进同一纯逻辑。 |
| D3 | 驱动核心层实例化 | mt6835_hw 与 as5600_hw 的设备句柄与失败分诊快照从文件 static 改为实例句柄（init 返回 handle，read 带 handle）。SPI/I2C 事务接口语义不变。 |
| D4 | 双 SPI 主机分配 | 板载小车位 = SPI2_HOST（引脚 12/11/13/10），外接摆角位 = SPI3_HOST（引脚 47/48/38/14）。bsp Kconfig 以职责命名两组宏（`BSP_CART_ENC_*` / `BSP_POLE_ENC_*`），accessor 拆为按实例的两个。器件名不入宏名（器件可换，职责不变）。 |
| D5 | 通道迁移 | console 切 USB-Serial/JTAG（sdkconfig），UART0（43/44）整让给 cartproto（波特率 460800 不变）；cartproto 配置面从 UART1+41/42 切换。上位机经 UART0 飞线直连。 |
| D6 | 新增外设范围 | io4 vbus ADC（分压与滤波见 [hardware.md §1](../../hardware.md#1-usb-c-拓扑与-pd-供电)，读数进启动日志与遥测观测）、LED（io15）心跳、按键（io7/2/1）驱动层（消抖读取 API，行为语义后续定义）。CH224Q I2C 调档组件支持设定电压档与读取 PD 握手状态，寄存器语义以 CH224DS1 数据手册为准；上电默认请求由 CFG1 电阻决定，具体接线与请求档位以硬件清单为准，I2C 调档与 vbus ADC 回读用于验证运行时请求。 |
| D7 | µrad 换算收口 | 摆角「counts → ±π µrad」换算与 mt6835_urad 的 raw_to_urad 同源两份——收成一份纯函数（位置与消费方实现时定：类别组件角度辅助或 poleang 内部），两处消费共用。预估项，实施时可依耦合情况最小化处理。 |

## 任务拆分

依赖主线：T1→T2→T3→T4→T5→T6；T7/T8 与主线正交，可后置并行。模块划分与内部接口形态为预估，实施者可按实际数据流调整，不改变上述契约与决策。

### T1 mt6835_hw 实例化
- 改：`mt6835_hw.c/.h`——设备句柄、失败分诊快照进 `mt6835_hw_handle_t` 上下文；init 出 handle、read/last_failure 带 handle。
- 验证：组件测试适配后全过；主固件构建通过。

### T2 as5600_hw 实例化 + unwrap 上提
- 改：`as5600_hw.c/.h` 同 T1 模式实例化。
- 新增：类别组件 unwrap 纯逻辑（参数化 counts_per_rev，含种子语义与非对称阈值论证注释），`as5600_tracker.c` 语义并入；`mt6835_unwrap_diff` 淘汰。
- 验证：unwrap 纯逻辑宿主测试（覆盖：种子、正向/反向跨界、边沿抖动退化为 ±1、非对称阈值全值域恰好一次、4096 与 2^21 两种 cpr）；既有 as5600 tracker 测试用例迁移合并。

### T3 magnetic_encoder 骨架
- 新增：类别组件内骨架 TU——实例结构（操作表首成员 + 骨架状态 + 实现回调指针）、生命周期（创建任务、首读种子）、采样任务循环（arg=实例）、失败计数、速度窗计算、临界区发布。失败计数与速度窗计算抽为纯函数，宿主可测。
- 改：实现回调接口（内部私有头）：读帧（出原始角 + 可选 STATUS）、counts_per_rev、失败分诊描述。
- 验证：骨架纯函数宿主测试（失败计数状态机、速度窗填充与 ×62.5、窗口未满为 0）。

### T4 两实现瘦身接入骨架
- 改：`magnetic_encoder_as5600.c` / `magnetic_encoder_mt6835.c` 退化为薄读层并接骨架；MT6835 的 STATUS 告警与失败帧 dump 保留（经回调带实例标签）。两实现均支持多次 init（各出独立句柄）。
- 验证：组件 target 测试适配；主固件构建通过。

### T5 bsp 双 SPI 与引脚宏
- 改：`bsp.c` 双主机初始化（SPI2/SPI3）；Kconfig 两组职责命名宏（默认值按 hardware.md §3）；`bsp_get_spi_host()` 拆为小车位/摆角位两个 accessor。I2C 不动（CH224Q 预留）。
- 验证：构建通过；Kconfig 默认值与 hardware.md 逐项核对。

### T6 组装点切换
- 改：`app_main.c` / `test/main/test_app_main.c`——小车位 init 调用从 as5600 实现换 mt6835 实现（板载 SPI2），摆角位换外接引脚（SPI3）；as5600 头引用移除。
- 验证：主固件与测试固件构建通过；README 工程结构节更新（滞后标注撤销）。

### T7 通道迁移（console USB + cartproto UART0）
- 改：sdkconfig console → USB-Serial/JTAG；cartproto UART1→UART0（43/44）；bsp UART 初始化同步（UART1 面删除）。
- 验证：构建通过；上板烧录/日志/协议三通路联调（上板项）。

### T8 新增外设（vbus/LED/按键/CH224Q 调档）
- 改：bsp 增 vbus ADC 读取（系数 8.407）、LED 心跳、按键消抖读取 API；启动日志输出总线电压（诱骗成功判定）。
- 新增：ch224q 器件组件（I2C 驱动核心层：init（总线句柄 + 地址注入）、设定电压档、读取握手状态；单实例即可）；bsp 或组装点提供「设档 → ADC 回读验证」闭环调用。
- 单电阻配置模式下 I2C 自动启用，CFG1 电阻决定上电默认请求；具体接线与请求档位见 [hardware.md §1](../../hardware.md#1-usb-c-拓扑与-pd-供电)。运行时电压请求、状态及电流查询接口以 [ch224q.h](../../../components/ch224q/include/ch224q.h)为准；I2C 调档后需要通过 VBUS 回读确认实际供电电压。
- 验证：构建通过；上板读数核对（20V ±分压误差内）；I2C 改档后 ADC 回读跟随（上板项）。

### T9 上板验证（hardware.md 检查清单）
按 [hardware.md](../../hardware.md) 检查清单执行：双编码器首读关卡、USB 烧录日志、UART0 协议 460800、io4 回读 ≈20V、TMC2209 抽验、按键/LED、20V 首检。另加：**同实现双实例回归**（两 MT6835 句柄同时工作、操作表各归其主、互不干扰——纸面多实例的第一次真验收）。

### T10 文档与跨仓库
- README 工程结构节、组件职责描述更新；cartpole 仓库 onboard-checklist 建议新板自测轮（跨仓库，由用户或后续任务执行）；sim2real-spec 观测噪声表标注「x 噪声依据待 rev2 实测修订」。

## 横切契约闭合审查记录

- 数据流：raw（器件帧）→ unwrap（类别纯逻辑，per 实例）→ pos_counts/vel（骨架发布，per 实例）→ 解释层（零改动）→ 遥测/协议（不变）。每段 per-instance 贯穿，无共享静态残余（T1/T3 兕底）。
- 失败路径：读帧失败 → 实现分诊 → 骨架计数 → is_ok false → cartfsm FAULT（既有链路不变）；双实例下两编码器失败计数独立，任一 NOT OK 触发对应 reason（AS5600 reason 名上板核对——遥测/协议 reason 字符串若含器件名需同步为编码器位语义，实施时核对 cartfsm/cartproto 的 reason 字面量）。
- CPU/并发：双采样任务同核同优先级分时（1 kHz × SPI polling 事务，负载远低于预算）；ISR 在 CPU0 不受扰。
- 符号链：X_INV/POLE_INV 唯一旋钮（D5 决策落地）；新板首次标定流程按 protocol.md 符号约定验证。

## 未决与预估

- 骨架内部回调接口形态、文件划分（T3）为预估，实施时按编译与测试反馈调整。
- D7 µrad 收口的最小形态实施时定。
- 按键行为语义：明确不在本计划，后续任务。
- ENN（GPIO21）外部上拉：2026-10-17 用户确认新板已做 3.3V 上拉（与 TMC2209 逻辑电源同轨），hardware.md §2 已更新，本项关闭。
- CH224Q I2C 地址 0x22/0x23：待上板扫描确认（T8）。
- 上板项（T9、T7 联调、T8 回读验证）依赖用户操作硬件。
