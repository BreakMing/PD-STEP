# T11 任务包 · 复制以下全部内容给执行 Agent

````
# 任务：ESP32-S3 单轴运动控制器 · T11 设置与 NVS

## 背景
- T0–T4 已完成验收合并。当前 `$$` / `$n=v` 在 main/app_main.c 是占位回调
  （不落 NVS、不校验），本任务把 $ 命令接到真实参数系统，并顺手落地两项
  已定案决策（见下）。
- **必读**（按序）：PLAN.md §4.3（命令集，含 v1.10 新增 M502/M503）、
  §4.6（错误码）、§5（参数表——编号 $0–$13 与合法域已定稿，**这是本任务的
  实现规格，不要自行发明校验规则**）、§6 T11 节（含解析器接口四处变更预登记）、
  §12 执行约定；docs/reviews/T4-fix-acceptance.md §5.1（$n? 不可达根因）。
- 两项已定案决策（2026-09-15，规划 Agent 定，你只负责落地）：
  - **`$n?` 移除**：协议层实时 `?`（§4.4）在行装配中途被拦截，解析器的
    `$n?` 分支永远不可达（上板实测 `$5?` = 状态帧 + error:4）。删除死代码。
  - **M503 保留**：`$$` 别名，已上板可用、已有测试，PLAN §4.3 已补录，
    无需代码改动。

## 环境
- ESP-IDF v6.0.2，构建配方见 PLAN §12-6。
- 本机无主机 C 编译器、无 QEMU——**单测唯一真实执行路径 = 编入固件 +
  Kconfig 开关上板跑**（T4 定案先例：GCODE_PARSER_SELF_TEST，照抄该模式）。
- git：新分支 `task/T11-settings`。若 git 对象写入报 Permission denied：
  重试 2 次，仍失败则在汇报中明确报告（提交由需求方代执行），不许静默放弃。

## 设计定案（按此实现，不要另行发明）

### 1. 组件与 API（components/settings/，含 include/settings.h）

```c
typedef enum {
    SETTINGS_OK = 0,
    SETTINGS_INVALID,    /* 值非法 / 未知 idx → error:4 */
    SETTINGS_STORAGE,    /* NVS 写/擦失败     → error:5 */
} settings_status_t;

esp_err_t     settings_init(void);      /* nvs_flash_init(标准满页/新版本擦除重试) + 加载 */
settings_status_t settings_set(int idx, float value);   /* 校验→NVS 写→通知 */
float          settings_get(int idx);   /* 有效 idx 0..13 */
bool           settings_is_valid(int idx, float value); /* 纯校验无副作用，自测用 */
settings_status_t settings_reload(void);/* 不重跑 nvs_flash_init，只重读全部键 */
settings_status_t settings_reset_defaults(void); /* 清键→重载默认→逐项通知 */
int            settings_count(void);    /* =14 */
const char    *settings_name(int idx);  /* "microstep" 等，日志用 */
/* 命名取值（app/T5/T7/T9 消费；派生换算按 §4.7） */
int   settings_microstep(void);   int   settings_full_steps(void);
float settings_units_per_rev(void);     float settings_steps_per_mm(void);
int   settings_run_current_ma(void);    int   settings_hold_current_ma(void);
float settings_default_feed(void);      float settings_default_accel(void);
float settings_max_feed(void);          float settings_max_accel(void);
int   settings_report_hz(void);        bool  settings_invert_dir(void);
int   settings_uart_baud(void);        float settings_soft_min(void);
float settings_soft_max(void);
/* 变更通知：最多 4 个订阅槽；set 成功与 reset_defaults 逐项触发 */
typedef void (*settings_change_cb_t)(int idx, float new_value);
bool  settings_subscribe(settings_change_cb_t cb);
```
- 内置轻量互斥（FreeRTOS mutex 即可）保护并发读写——comm 任务写，
  后续 monitor 任务读（T9），写入计划非假设。
- settings 组件**不得**依赖 protocol / gcode_parser / bsp（分层：纯数据层）。
  所有串口输出由 app_main 胶水层完成。

### 2. NVS 布局与容错
- namespace `settings`，键 `p0`–`p13`，统一按 u32 存储（float 用 memcpy 位拷贝）。
- 首次启动键缺失 → 静默用默认值（不逐键写回，set 时才写）。
- 键存在但值非法（旧固件残留，如 p0=7）→ 内存值回退默认 + ESP_LOGW 一行
  （带 idx 与坏值），NVS 键保留不动（下次 set 覆盖）。

### 3. 校验规则
- 逐参数合法域**逐字按 PLAN §5 表实现**（$0–$13，含整数参数非整数拒绝）。
- 交叉校验以其余参数**当前内存值**为准：$4≤$3、$5≤$7、$6≤$8、$12<$13，
  两个方向都要拦（先放大后缩小时序无关）。
- idx 越界（<0 或 >13，含 14/15）→ SETTINGS_INVALID（error:4）；
  解析层对 >15 的 idx 回 error:2 的现状**不变**（那是解析器自己的范围检查）。

### 4. 解析器接口变更（四处，PLAN T11 节已预登记，允许触碰 gcode_parser）
1. `gcode_parser_create(float init_feed_mm_min, float init_accel_mm_s2)`——
   模态 F/A 初值来自 settings（§4.3 G0 行规定"启动初值取 NVS"，T4 时以编译期
   宏兜底）。头文件里 `PARSER_DEFAULT_F/A` 宏删除（消除第二事实来源）。
2. `parser_output_iface_t` 删 `on_get_param` 成员（$n? 死代码定案移除）。
3. `on_set_param` 返回类型 void→int：回调返回 0=成功；4=值非法/未知 idx；
   5=存储失败。**解析器对非 0 返回值直接回 `error:<ret>`**（line handler
   现有 ok/error 逻辑不变——解析器把它折算成 PARSER_ERR_BAD_PARAM/新枚举
   或等价机制均可，对外行为必须是 error:4 / error:5）。
4. 新增 `int (*on_restore_defaults)(void *ctx)`（返回 0/5），M502 命令触发。
- M502 解析：与现有 M 码分支同构；M503 无需改动（已 fire on_list_params）。
- **M502 后解析器模态 F/A 运行值不重置**（重启后取新默认）——在头文件
  注释里写明此行为。

### 5. app_main 接线（占位回调清除）
- 启动序调整为：bsp_early_init → **settings_init** → 横幅 → bsp_init →
  protocol_start → `gcode_parser_create(settings_default_feed(),
  settings_default_accel())` 注册。
- on_set_param → `settings_set` 映射 OK→0 / INVALID→4 / STORAGE→5。
- on_list_params（$$ 与 M503 共用）→ 循环 0..13 发 `$%d=%.3f\r\n`
  （protocol_send 系列，行尾 \r\n，§4.2），14 行后 line handler 统一补 ok。
- on_restore_defaults → `settings_reset_defaults` 映射 0/5。
- 删除 on_get_param 占位与相关死代码。

### 6. protocol 组件（唯一许可触碰点）
- UART1 波特率来源改为 `settings_uart_baud()`；settings 未初始化（如极端
  启动序）回退现有 Kconfig 默认 115200。仅此一处，其余 protocol 逻辑不动。

## 工作内容（按序）
1. 建分支 `task/T11-settings`；创建 components/settings/（含 Kconfig 开关
   `SETTINGS_SELF_TEST`，bool 默认 n，照抄 GCODE_PARSER_SELF_TEST 模式）。
2. 实现 settings 组件（§1–§3）+ 单测 test/settings_self_test.c（见下）。
3. 解析器接口四处变更（§4）+ 同步更新 gcode_parser 自测与 Unity 测试：
   - 删所有依赖 `$n?`/`on_get_param` 的用例（至少 test_dollar_query_param）；
   - 补：裸 `$5` → error:4；`on_set_param` 桩返回 4/5 → error:4/error:5 映射；
     M502 → on_restore_defaults 触发。
   - **gcode_parser 自测总数保持 30 例，断言不得削弱**。
4. app_main 接线 + protocol 波特率来源（§5–§6）。
5. 构建两次验证：默认配置零错误；SETTINGS_SELF_TEST=y（删 sdkconfig 后）零错误。
6. git 提交（沙箱失败条款见"环境"节）。

### settings 自测用例（≥24 例，编入固件上板跑）
- 首启默认值抽查（$0=16、$5=600、$8=2000、$13=600）
- 14 参数各一组合法 set→get 往返
- 非法矩阵：$0∈{7,0,512,16.5}、$1=300、$2∈{0,501}、$3∈{0,2001}、
  $4>$3、$5∈{0, 大于 $7 现值}、$6∈{0, 大于 $8 现值}、$9∈{0,51}、$10=2、
  $11=123456、$12≥$13 → 均 INVALID
- settings_set(14,1) / settings_set(-1,1) → INVALID（未知 idx）
- 持久性：set $5=1000 → settings_reload() → 仍 1000
- 恢复默认：settings_reset_defaults() → 全部回默认
- 通知：subscribe 后 set $0=32 → 回调收到 (0, 32.0)；reset_defaults 逐项触发
- 派生：$0=32 → settings_steps_per_mm()==200*32/40==160.0
- 存储坏值回退：测试内直接 nvs_open 写 p0=7 → reload → 回默认 16 + W 日志
- 自测收尾调用 settings_reset_defaults()，板子留干净状态
- 最终行格式：`SELF-TEST DONE: <n>/<total> PASS`（tag: settings_selftest）

## 范围红线
- 不动 bsp 组件（含其 UART1 Kconfig 初始化）；protocol 仅 §6 一处。
- gcode_parser 改动仅限预登记四处接口 + 对应用例更新，不重写解析逻辑。
- 不实现 T5/T6/T7/T9 功能（订阅机制就位但零消费者即达标——不写任何
  "通知消费者"占位任务）。
- 不动 sdkconfig.defaults / sdkconfig.release（新 Kconfig 项默认值即可）。
- 不碰 TMC2209 / AS5600 相关任何代码。

## 完成定义（全部满足才算完成）
- 两种配置构建零错误（附两次构建输出尾部）
- settings 自测 ≥24 例、gcode_parser 自测仍 30 例（两份用例清单附汇报）
- 代码级证据（文件+行号）：`$0=7`→error:4、`$5=0`→error:4、`$14=1`→error:4、
  `$7=500`→error:4（交叉校验）、`M502`→ok、`$$` 14 行+ok、裸 `$5`→error:4
- app_main 占位参数回调已清除、启动序符合 §5
- 单一清晰提交（或按沙箱条款报告失败原因）；`git log --oneline -3` 附汇报
- 汇报：每项工作的文件+行号证据、构建输出、遗留 TODO

## 上板验收预告（需求方执行，你不用做）
1. SETTINGS_SELF_TEST=y 固件烧录 → `SELF-TEST DONE: n/n PASS`
2. 默认固件 VOFA+ 探针：`$$` 14 行+ok；`$5=1000`→ok→`$$` 确认→断电重启→
   `$$` 仍 1000（NVS 持久）；`$0=7`/`$5=0`/`$14=1`/`$7=500`→error:4；
   `M502`→ok→`$$` 回默认；`$5?`→状态帧+error:4（T4 实测行为保持不变）
````
