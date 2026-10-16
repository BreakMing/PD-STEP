# 三器件域解耦设计（cartpos / poleang / motor 域接口）

## 背景与目标

步进电机（tmc2209 + step_engine）、小车位置（as5600）、摆杆角度（mt6835）三个器件域与业务层的耦合集中在四个面：

1. 接线知识（总线句柄、引脚、地址、时钟）内嵌在器件组件的 init 里，不在组装点。
2. 业务组件（cartfsm、telemetry）编译依赖具体器件组件，不存在领域接口。
3. 电机运动语义（指令位置的 mm↔steps 换算）以胶水函数形式长在 cartfsm 上，横跨三个组件。
4. telemetry 直接调用器件函数 `mt6835_raw_to_urad()` 应用摆角标定。

本设计的目标：**换器件型号（同板、ESP32 引脚框架不变）时，业务组件（cartfsm、telemetry、cartproto、cartparams、settings）零改动**；改动面收敛为域组件内新增实现源文件与 Kconfig 选项、menuconfig 选择，以及组装点（app_main）的 config 组装与 init 调用。

非目标：换 MCU/板级支持（接口不追求去除 ESP-IDF 类型）；运行期切换器件实现（单实现单例，链接期绑定）。

## 总体结构

```
                     业务层
        cartfsm                telemetry
          │                        │
          └──────┬────────┬────────┘
             域接口层（契约头文件，链接期绑定，无器件知识）
        cartpos            poleang            motor
   （域组件：契约头 + 实现源文件，Kconfig choice 选择实现）
        │                    │                  │
 cartpos_as5600.c    poleang_mt6835.c    motor_tmc2209.c + motor_conv.c
        │                    │                  │
      as5600              mt6835         tmc2209 + step_engine（器件驱动组件）
        │                    │                  │
       （*_hw 驱动核心层，保持不动）
```

绑定方式为链接期绑定：域组件的契约头只含接口声明，实现源文件（同组件内）定义接口函数；业务组件调用接口函数，链接器完成绑定。

## 组件清单

**新增**：

| 组件 | 形态 | 职责 |
|---|---|---|
| `components/cartpos/` | 域组件 | 契约头 + as5600 实现（`cartpos_as5600.c`）+ Kconfig choice |
| `components/poleang/` | 域组件 | 契约头 + mt6835 实现（`poleang_mt6835.c`，应用摆角标定）+ Kconfig choice |
| `components/motor/` | 域组件 | 契约头 + tmc2209 实现（`motor_tmc2209.c`）+ mm↔steps 换算纯函数（`motor_conv.c`，宿主可测）+ Kconfig choice |

域组件内的实现按 `<域>_<器件>` 命名源文件与 init/config；实现选择经各域组件的 Kconfig choice（menuconfig）完成。新增器件实现 = 域组件内新增源文件与 Kconfig 选项并切换选择，业务组件与 app_main 的 REQUIRES 均不变（app_main 仅改 config 组装与 init 调用行）。

**修改**：`as5600.c`、`mt6835.c`（init 改收 config 结构体）、`tmc2209.c`（EN 引脚经 config 注入）、`cartfsm.c`、`telemetry.c`（调用面切换）、`main/app_main.c`、`test/main/test_app_main.c`（组装）、各组件 CMakeLists（依赖收紧）、cartfsm 测试钩子（劫持点切换）；上述被改文件的头部注释同步按本设计的职责表述更新。

**不动**：三个 `*_hw` 驱动核心层、`as5600_tracker.c`、`mt6835_urad.c`、`vel_ramp.c`、`cartfsm_core.c`、`settings`、`cartparams`、`cartproto`、`bsp`（角色不变，见「组装契约」）。

## 域接口契约

Q16.16 指 16 位整数位加 16 位小数位的定点数表示（沿用 step_engine 现有 API 的记法）。

三个接口的共同约定：所有函数线程安全，可在任务上下文任意调用。器件未初始化时的返回语义沿袭迁移前设备组件行为、本设计不改变：数值读函数返回 0（状态静态零初始化）；`motor_at_rest` 与 `*_is_ok` 返回 true——速度与连续失败计数均处于零状态，判停与判正常都成立，且组装次序（见「组装契约」）保证消费者任务晚于域 init 启动，该初值不会外露为故障。连续读取失败期间，数值读函数返回最后一次成功样本的值，不额外冻结或清零。

### cartpos（小车位置，编码器实测位置）

| 函数 | 语义 |
|---|---|
| `int64_t cartpos_position_um(void)` | 连续位置（多圈、带符号、含回零原点偏移），µm 整数精确；首样本前为 0 |
| `float cartpos_velocity_mm_s(void)` | 16 ms 窗口平滑速度；窗口未满为 0 |
| `bool cartpos_is_ok(void)` | 连续 ≥10 次读取失败为 false；一次成功即恢复 true |
| `void cartpos_set_origin_mm(float new_pos_mm)` | 声明当前位置为 new_pos_mm（纯偏移建立，不产生运动）；任意时刻可调用 |

### poleang（摆角 θ）

| 函数 | 语义 |
|---|---|
| `int32_t poleang_theta_urad(void)` | **已标定**摆角：实现内按 cartparams 的 POLE_ZERO_URAD（零位）与 POLE_INV（符号）换算，±π 环绕；首样本前为 0 |
| `int32_t poleang_vth_mrad_s(void)` | 已标定角速度，符号随 POLE_INV 翻转；窗口未满为 0 |
| `bool poleang_is_ok(void)` | 同 cartpos 失败语义 |

标定应用从 telemetry 组帧点移入本域实现。STATE 帧行为不变（[protocol.md](../../docs/protocol.md) 数据面定义；同一纯函数、同一参数，仅调用位置重排）。

### motor（电机运动，指令位置）

| 函数 | 语义 |
|---|---|
| `esp_err_t motor_enable(bool en)` | 驱动器使能引脚（EN）使能/关断，en=true 时等待 ≥1 ms 唤醒；仅实现未初始化时返回非 ESP_OK |
| `void motor_set_vel_mm_s(long v)` | 设定带符号目标速度并喂命令看门狗；实现内换算 mm/s → Q16.16 steps/s |
| `void motor_halt(void)` | 目标清 0 斜坡停；不喂狗 |
| `void motor_stop(void)` | 急停：停脉冲、清递推状态与目标、冻结指令位置 |
| `bool motor_at_rest(void)` | true ⇔ 当前速度精确为零（Q16.16 域判零，不经 mm/s 整数舍入） |
| `int64_t motor_cmd_position_um(void)` | 指令位置，steps → µm 四舍五入 |
| `void motor_set_position_um(long um)` | 指令位置重定基（µm → steps）；前置条件：电机停止（无脉冲发出） |
| `void motor_set_accel_mm_s2(long a)` / `motor_set_decel_mm_s2(long d)` / `motor_set_wdt_ms(long t)` / `motor_set_dir_invert(bool inv)` | 参数作用点，语义与现 step_engine 对应函数一致；0 值防御按各自现行为 |

`motor_at_rest` 不以整数 mm/s 暴露当前速度：FAULT 停机次序依赖精确判零，Q16.16 域的微小残余换算为整数 mm/s 会舍入到 0、导致提前关 EN。

**换算契约**：mm↔steps 换算使用 settings 细分（SET_MICROSTEP）/每转整步数（SET_FULL_STEP）/每转行程（SET_MM_PER_REV）派生的 steps_per_mm；任一参数变更后，后续 `motor_set_vel_mm_s` / `motor_cmd_position_um` 调用即用新值（即读即算或订阅刷新由实现选择，语义等同）。

## 器件实现侧

**config 注入**（照 tmc2209 现有模式；接线参数经所选实现的 init 传入，最终到达器件驱动）：

- as5600 路径：config 字段为 I2C 总线句柄、从机地址、总线时钟、方向镜像布尔；器件侧不再调用 `bsp_get_i2c_bus()`，不再读取 `CONFIG_BSP_AS5600_I2C_ADDR` 等宏。
- mt6835 路径：config 字段为 SPI 主机号、时钟、CS 引脚；同理去除 bsp 调用与 `CONFIG_BSP_MT6835_*` 宏读取。
- `tmc2209_config_t` 增加 `en_gpio` 字段；`tmc2209_enable` 直接 `gpio_set_level`，不再调用 `bsp_tmc_en_set`。bsp 仍在 `bsp_init` 中配置该引脚为输出，`bsp_early_init` 的上电拉高安全次序不变。
- 实现 config 与所含器件 config 的对应关系（透传或重组）为实现细节。

**域组件内部结构**（三个域组件同构）：

- 契约头（`include/<域>.h`）只含域接口声明，不依赖任何组件。
- 每个域组件一个 Kconfig choice（`CARTPOS_IMPL` / `POLEANG_IMPL` / `MOTOR_IMPL`），当前各只有一项（AS5600 / MT6835 / TMC2209）且为默认；CMake 按所选宏编入对应实现源文件。后续新实现加入同组件并经 menuconfig 切换。
- 每个实现自带 init 与 config（如 `cartpos_as5600_init(const cartpos_as5600_config_t *cfg)`），实现头放域组件 `include/`；app_main 调用所选实现的 init。
- `cartpos_as5600.c`：init 内部完成 as5600 设备初始化；四个域函数转发设备 API。
- `poleang_mt6835.c`：同理；theta = `mt6835_raw_to_urad(mt6835_last_raw(), pole_zero_urad, pole_inv)`，vth 按 pole_inv 翻转符号；标定值读 cartparams。
- `motor_tmc2209.c`：init 内部完成 tmc2209 与 step_engine 初始化；motor 接口函数转发/组合两者；mm↔steps 换算放 `motor_conv.c` 纯函数（mm/s→Q16.16、mm/s²→steps/s²、steps→µm、µm→steps，公式与现 cartfsm 换算函数逐式一致），并新增对应宿主测试。

as5600、mt6835 的设备公共 API 保留（上板自测仍直接使用器件驱动）。

**settings 订阅归属不变**：细分/运行电流/保持电流订阅留在 tmc2209 组件（tmc2209.c）、每转行程订阅留在 as5600 组件（as5600.c）——硬件参数映射是器件实现细节，域接口不暴露电流、细分等概念。

## 组装契约

初始化次序不变：`bsp_early_init → settings_init → bsp_init → cartpos_as5600_init(&cfg) → poleang_mt6835_init(&cfg) → motor_tmc2209_init(&cfg) → cartproto_start → cartfsm_start → telemetry_start`。app_main 只 REQUIRES 三个域组件与业务组件，不直接调用器件驱动 init（原 `tmc2209_init` + `step_engine_init` 两步由 `motor_tmc2209_init` 内部完成）；init 调用对象为 menuconfig 选中的实现。

三个实现的 config 全部由 app_main（及测试工程入口）构建：总线句柄取自 bsp，引脚/地址/时钟取自 Kconfig 宏。宏定义位置不变（bsp Kconfig 的 `CONFIG_BSP_*`、as5600 Kconfig 的方向镜像、tmc2209 Kconfig 的采样电阻），Kconfig 仍是接线参数的唯一权威源；app_main 是唯一读取宏并组装 config 的位置。测试工程入口 `test_app_main.c` 同样组装。

## 依赖规则

以下为可检查的构建期规则，迁移完成后生效：

1. 业务组件（cartfsm、telemetry）的 REQUIRES 只含域组件与业务组件：cartfsm → cartpos、poleang、motor、cartparams、cartproto；telemetry → cartpos、poleang、cartproto。不得出现 bsp、settings 或任何器件驱动组件。
2. 域组件的契约头不依赖任何组件；域组件的 REQUIRES 由所选实现决定（器件驱动组件，及实现所需的数据组件如 cartparams、settings）。
3. 器件驱动组件（as5600、mt6835、tmc2209、step_engine）不得 REQUIRES bsp；总线句柄与引脚一律经 config 注入。对 ESP-IDF 驱动库（esp_driver_i2c 等）的依赖不受限。
4. 器件驱动组件只被域组件与测试工程 REQUIRES；main 不 REQUIRES 器件驱动组件。
5. cartproto 的现状（REQUIRES bsp）不在本次范围，不受规则 3 约束。

## 测试与验证

- **宿主测试**：现有 host_tests 全部保持通过；新增 `motor_conv` 换算纯函数的宿主测试（四条换算公式的边界与舍入）。
- **测试工程（test/）**：`test_app_main.c` 改为新的组装方式；各器件组件自测走设备自有 API，不受影响；cartfsm 流程自测的注入钩子（CONFIG_CARTFSM_TEST_HOOKS）劫持点从 `as5600_is_ok` / `mt6835_is_ok` 改为 `cartpos_is_ok` / `poleang_is_ok`，E 偏置注入点从 `as5600_position_um` 调用处改为 `cartpos_position_um` 调用处，注入机制与用例语义不变。
- **上板回归**：迁移完成后执行一次完整上板回归（含编码器读数、流程自测、手动/硬件段）。此前驱动核心拆分遗留的真机回归一并补记。验证记录注明日期、环境与范围。

## 迁移不变量

调用面替换不得改变以下端到端契约。本节为迁移验收依据；行为的权威定义见各条末尾链接指向的文件，此处不重复展开：

1. **FAULT 停机次序**：目标斜坡置 0（引擎加速度临时切 WDT_DEC 档）→ 监督任务每拍重发斜坡停并等待速度精确为零 → EN 关 → 推送 MODE 行。权威定义见 [cartfsm.c](../../components/cartfsm/cartfsm.c)、[step_engine.h](../../components/step_engine/include/step_engine.h)。
2. **重定基时序**：MARK_RIGHT 完成时编码器位置与指令位置同帧重定基（两标中点为 x=0）；ENABLE 时指令位置先重锚到编码器当前位置再发布模式（避免监督首拍误报 STEP_LOSS）。权威定义见 [cartfsm.c](../../components/cartfsm/cartfsm.c) 的 exec_mark_right / exec_enable。
3. **锁序**：cartfsm 共享状态的读写持 s_lock，域接口调用在 s_lock 之外执行（不与电机实现内部的锁嵌套）。权威定义见 [cartfsm.c](../../components/cartfsm/cartfsm.c)。
4. **协议字节兼容**：STATE 帧字段与数值来源、MODE 帧 reason 值（"AS5600"、"MT6835"、"STEP_LOSS"）保持不变；reason 字符串是协议契约，不随器件更换改名。权威定义见 [protocol.md](../../docs/protocol.md)、[telemetry.c](../../components/telemetry/telemetry.c)、[cartfsm.c](../../components/cartfsm/cartfsm.c)（reason_str）。
5. **测试注入语义**：编码器失联注入与编码器位置偏置注入的用例行为不变。权威定义见 [test_cartfsm_flow.c](../../components/cartfsm/test/test_cartfsm_flow.c)。

## 范围界定（本设计不改变归属的内容）

- cartfsm 继续持有手动两点标定状态机（CAL_NONE / CAL_LEFT / CAL_DONE）、丢步判定（编码器实测位置与指令位置的偏差）、软限位窗口钳制、FAULT 停机次序与命令×模式分派；模式机行为契约的权威定义在 [cartfsm_core](../../components/cartfsm/cartfsm_core.c)，本设计不修改。本设计对 cartfsm 只做两件事：器件调用改为域接口调用，删除 mm↔steps 换算胶水。
- cartproto 的 UART1 接线维持现状（仍直接读取引脚宏）。
- cartfsm 的监督任务与命令执行实现（cartfsm.c）的宿主测试需要 FreeRTOS 仿真层，不在本设计范围。
