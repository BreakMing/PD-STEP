# T5 任务包 · TMC2209 驱动组件（单线 UART）

> 分发对象：执行 Agent（Claude Code）· 分支：`task/T5-tmc`（已建好，勿做任何 git 操作——沙箱禁止，用户负责提交）
> 上游事实源：PLAN.md（§5 参数表、T5 节）、docs/hardware.md、components/settings/include/settings.h、components/bsp/include/bsp.h、main/app_main.c

---

## 0. 执行指令（按序）

1. 通读本文件全文；不确定处按"数据手册为准"原则处理并在报告中记录依据
2. **先做资料核实**（见 §3 核实清单）——CRC 初值与寄存器位布局是本任务的硬正确性风险点，未核实前不得写实现
3. 实现 `components/tmc2209/`（API 见 §4）
4. BSP 补一个函数 `bsp_tmc_en_set(bool)`（见 §5，这是 T5 唯一允许的 BSP 触碰点）
5. app_main 接线（见 §6）
6. 自测与上板 bring-up（见 §7）
7. 双配置构建验证（默认 + 自测开启），在最终答复里给出：改动文件清单、核实清单结论、自测预期输出、构建证据

**红线（越界即验收不通过）**：
- 禁止改动：`components/protocol/`、`components/gcode_parser/`、`components/settings/`、`components/step_engine/`（T6 正在并行开发，勿碰）、`components/bsp/` 除 §5 指定函数外的任何内容
- 禁止 git commit / branch / 任何 .git 写操作
- 禁止写 AS5600 的任何寄存器（一次性 OTP，只读都不必做——本任务根本不碰 I2C）
- IRUN 上板从 ≤500mA 起步（PLAN T5 注意事项）

---

## 1. 硬件事实（不得更改）

| 项 | 值 | 来源 |
|---|---|---|
| TMC2209 形态 | PCB 集成芯片 | hardware.md §2 |
| UART TX | GPIO17，经板载 1kΩ 接 PDN_UART 节点 | hardware.md §3 |
| UART RX | GPIO18，直连 PDN_UART 节点 | hardware.md §3 |
| 拓扑 | **两线 UART（非单引脚半双工）**：TX→1kΩ→节点，节点→RX 直连 | PLAN T5 节 |
| 波特率 | 115200 8N1（默认，按数据手册核对） | PLAN T5 节 |
| EN 引脚 | GPIO21，低有效；BSP 已在上电最早拉高（失能） | bsp.h |
| STEP/DIR | GPIO5/GPIO6（BSP 已初始化为输出）——**本任务不碰** | bsp.h |
| VM | 12V（PD 诱骗直供） | hardware.md §2 |
| 电机 | 42SHDC3025-24B，1.8°=200 步/圈，额定 1.68A | hardware.md §2 |
| ESP32-S3 可用 UART | UART0=控制台(43/44)、UART1=协议(41/42)，**TMC 用 UART2(17/18)** | 项目现状 |

**回波陷阱（必须处理）**：TX 经 1kΩ 连到 PDN_UART 节点、RX 直连同节点——ESP 发送的每个字节会**同时出现在自己的 RX 上**（Trinamic 应用笔记明确此现象）。实现读操作时：发送 8 字节请求 → **丢弃 RX 中回显的 8 字节** → 再等待从机 8 字节应答（超时 5ms，超时判失败）。写操作同样有回显，发送后清空 RX 即可。

---

## 2. 交付物

`components/tmc2209/`：
- 寄存器读写（帧编解码 + CRC8 纯函数）
- `tmc2209_set_current(run_ma, hold_ma)`（mA→CS 换算为纯函数）
- `tmc2209_set_microstep(ms)`
- StealthChop/斩波配置（上电应用一组数据手册推荐值）
- `tmc2209_read_status()`（DRV_STATUS：温度/过流/StallGuard 原始值解码）
- `tmc2209_enable(bool)`（EN 控制，经 BSP）
- 自测（Kconfig 门控）+ bring-up（Kconfig 门控，见 §7）

---

## 3. 数据手册核实清单（实现前完成，结论写入报告）

依据源（双源交叉）：**官方 TMC2209 数据手册**（ADI/Trinamic 最新 PDF，可 WebSearch 获取）+ **TMCStepper 开源库**（github.com/teemuatlut/TMCStepper，事实标准参考实现）。

| # | 待核实项 | 备注 |
|---|---|---|
| 1 | 数据报帧结构与总长 | 我方预期：地址字节（7bit 从机地址<<1 \| RW）+ 数据 + CRC；**总 8 字节**、寄存器数据 32bit。以数据手册 §UART 章节为准 |
| 2 | CRC8 多项式与**初值** | 多项式预期 0x07（x^8+x^2+x+1）；**初值 0x00 与 0xFF 两个候选——必须用手册+TMCStepper 双源定案**，报告写明依据 |
| 3 | 从机地址 | 预期默认 0；读请求地址字节含 RW=1 |
| 4 | IOIN 寄存器（0x06）VERSION 字段 | 预期 [31:24]，TMC2209 = **0x40**（芯片识别标准判据；不符则如实上报读到的值） |
| 5 | GCONF（0x00）位定义 | 预期：pdn_disable(bit6)=**1 必须置位**（UART 控制模式）、en_spreadcycle(bit2)=0 即 StealthChop、shaft(bit3)、mstep_reg_select(bit7)=1（MRES 由 CHOPCONF 控制） |
| 6 | IHOLD_IRUN（0x03）位布局 | 预期 IHOLD[4:0]、IRUN[12:8]、IHOLDDELAY[19:16] |
| 7 | CHOPCONF（0x6C）位布局 | 预期 TOFF[3:0]（0=驱动关闭）、TBL[6:4]、HSTRT[9:7]、HEND[13:10]、MRES[27:24]（0=256 … 4=16 … 8=整步）、VSENSE(bit17)、INTPOL(bit28) |
| 8 | DRV_STATUS（0x6F）旗标位 | stst/olb/ola/s2ga/s2gb/otpw/ot/CS_ACTUAL 各位序 |
| 9 | TPWMTHRS（0x13）与 StealthChop 切换语义 | 期望纯 StealthChop 低速运行 |
| 10 | 电流公式与 V_FS | 预期 I_RMS = (CS+1)/32 × V_FS/(R_SENSE+0.02Ω) × 1/√2；V_FS=0.325V(VSENSE=1)/0.18V(VSENSE=0)；**R_SENSE 板上实际值未知——Kconfig 项 CONFIG_TMC2209_RSENSE 默认 0.11，报告中标注"待需求方按原理图/BOM 核实"** |
| 11 | 上电推荐斩波参数 | TOFF/TBL/HSTRT/HEND/IHOLDDELAY 取数据手册推荐值，报告逐项记录选择理由 |

其余寄存器（GSTAT 0x01、TPOWERDOWN 0x04、PWMCONF 0x70、SGTHRS 0x74、SG_RESULT 0x75）地址核对即可。

**核实结果的判定锚点**：若帧/CRC 实现？上板读 IOIN 得 VERSION=0x40 即为通过；读不到任何应答 → 先查核实清单再查接线假设，报告如实记录。

---

## 4. 组件 API（定稿，验收按此检查）

```c
/* components/tmc2209/include/tmc2209.h */

typedef struct {
    int      uart_num;     /* 默认 UART_NUM_2 */
    int      tx_pin;       /* 17 */
    int      rx_pin;       /* 18 */
    int      baud;         /* 115200 */
    uint8_t  slave_addr;   /* 0 */
    float    r_sense;      /* CONFIG_TMC2209_RSENSE, 默认 0.11 */
} tmc2209_config_t;

esp_err_t tmc2209_init(const tmc2209_config_t *cfg);   /* UART2 驱动安装 + 参数加载 */

/* 寄存器层（bool=通信成功且 CRC 校验通过） */
bool tmc2209_read_reg(uint8_t reg, uint32_t *val);
bool tmc2209_write_reg(uint8_t reg, uint32_t val);

/* 功能层 */
esp_err_t tmc2209_set_current(int run_ma, int hold_ma);   /* IHOLD_IRUN + VSENSE 自动选档 */
esp_err_t tmc2209_set_microstep(int ms);                   /* 1..256 → MRES，非法值 return ESP_ERR_INVALID_ARG */
esp_err_t tmc2209_enable(bool en);                        /* EN 低有效；en=true→GPIO 低 + ≥1ms 等待后生效 */

/* 状态层（T9 消费） */
typedef struct {
    uint32_t raw;
    bool     stst, ola, olb, s2ga, s2gb, otpw, ot;
    int      cs_actual;
} tmc2209_status_t;
bool tmc2209_read_status(tmc2209_status_t *st);

/* 纯函数（自测对象，无硬件依赖） */
uint8_t tmc2209_crc8(const uint8_t *data, unsigned len);
int     tmc2209_ma_to_cs(int ma, float r_sense);          /* 换算+clamp 0..31 */
int     tmc2209_microstep_to_mres(int ms);                 /* 1→8, 2→7, 4→6, … 256→0；非法→-1 */
```

设计约束：
- 所有 UART2 读写经一把互斥锁（comm 任务与 settings 订阅回调都会调）
- 读操作必须按 §1 处理 8 字节回显
- SHAFT 位（GCONF.bit3）恒置 0——**方向反转移交 $10 invert_dir（T6 输出层处理），避免双重反转**
- microstep/电流初始值：`tmc2209_init` 内经 `settings_microstep()/settings_run_current_ma()/settings_hold_current_ma()` 读取并写入寄存器（UART 寄存器访问在 EN 失能时同样有效，可预配置）
- 运行时参数变更：`settings_subscribe()` 订阅，收到 $0/$3/$4 变更通知即重写对应寄存器（回调在 comm 任务上下文执行，互斥锁保证安全）

---

## 5. BSP 唯一触碰点

`components/bsp/include/bsp.h` + `bsp.c` 新增：

```c
/* TMC2209 ENN 控制（低有效）。enable=true → GPIO 输出低。 */
void bsp_tmc_en_set(bool enable);
```

实现读 Kconfig 既有 EN 引脚符号（T2 已有），不得新增 Kconfig 项。

---

## 6. app_main 接线（合并冲突控制：只动指定锚点）

1. `#include "tmc2209.h"`
2. `cb_on_motor_enable` 占位函数体替换为：`tmc2209_enable(enable)`（M17→使能+按 §4 已写寄存器运动；M18/M84→EN 高失能）
3. `bsp_init()` 成功后、`protocol_start()` 前：`tmc2209_config_t cfg = { UART_NUM_2, 17, 18, 115200, 0, CONFIG_TMC2209_RSENSE }; tmc2209_init(&cfg);`（引脚改读 BSP Kconfig 符号亦可，保持单一事实来源）
4. settings 订阅在 `tmc2209_init` 内部完成，app_main 不写订阅代码

## 6b. Kconfig / CMake

- `components/tmc2209/Kconfig`：`CONFIG_TMC2209_RSENSE`（float，默认 0.11）、`CONFIG_TMC2209_SELF_TEST`（bool，默认 n）、`CONFIG_TMC2209_BRINGUP`（bool，默认 n）、`CONFIG_TMC2209_SPIN_TEST`（bool，默认 n，依赖 BRINGUP）
- `main/CMakeLists.txt`：`PRIV_REQUIRES` 加 tmc2209（与既有行合并，只加不改）

---

## 7. 测试（两层）

**层 1 · 纯逻辑自测 `CONFIG_TMC2209_SELF_TEST=y`**（≥20 例，Kconfig 开关编入固件上板自跑，项目惯例——本机无宿主编译器）：
- CRC8：空/短序列/全 0xFF/递增序列向量（向量由核实后的实现与 TMCStepper 输出比对生成，写死进用例）
- 帧编码：写请求/读请求地址字节构造、整帧组装长度
- mA→CS：典型值、边界（0、超上限→clamp 31）、R_SENSE 影响项
- microstep↔MRES 全表（1..256 九档）+ 非法值
- IHOLD_IRUN 位打包、DRV_STATUS 旗标解码（合成原始值→布尔位）
- 运行输出行：`PASS <case>` 逐例 + 末行 `SELF-TEST DONE: <n>/<total> PASS`

**层 2 · 硬件 bring-up `CONFIG_TMC2209_BRINGUP=y`**（默认关；app_main 自测之后运行，逐项打印）：
1. 读 IOIN → VERSION==0x40？
2. 写 GCONF → 读回一致？
3. 写 IHOLD_IRUN（300mA/150mA）→ 读回一致？
4. 写 CHOPCONF（8 细分）→ 读回一致？
5. 读 DRV_STATUS → 数值合理（无 OT/断相，CS_ACTUAL 0..31 内）？
6. （`CONFIG_TMC2209_SPIN_TEST=y` 时）EN 低 → 2000 个 STEP 脉冲 @500Hz（直接 GPIO5 操作，1µs 脉宽）→ 电机应平稳转动一圈的 1/8（8 细分下 2000 步=10 整步）→ EN 高收尾

层 2 结束打印 `BRINGUP: PASS n/6` 或逐项 FAIL 详情。

---

## 8. 构建验证（两层都要）

```
powershell -File idf_env.ps1 build          # 默认配置零错误
```
自测配置：sdkconfig **末尾追加** `CONFIG_TMC2209_SELF_TEST=y`（追加优先级高于注释态，T11 定案技巧）再构建。两个构建产物都要在报告中确认。

## 9. 完成报告格式

最终答复包含：① 核实清单 11 项逐项结论+依据来源；② 改动文件清单；③ 纯逻辑自测用例数与预期上板输出；④ 双配置构建证据；⑤ R_SENSE 与斩波参数选择说明；⑥ 未尽事项/风险。
