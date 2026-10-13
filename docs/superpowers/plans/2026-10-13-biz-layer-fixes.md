# 业务层评审缺陷修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落实 2026-10-13 业务层评审报告的剩余缺陷（A1/A3/A4、B1–B6、B8、B10），完成 cartfsm 并发收口、EN 使能收口、参数权威化、ISR 插桩收口与注释/风格清理。

**Architecture:** 全部改动落在既有 9 个组件内部，不新增组件、不改线协议。行为变更加仅两处已拍板项：A1（ENABLE 命令经 `tmc2209_enable` 获得 2 ms settle）与 A3（FAULT settling 每拍重发 `vel_halt`）；其余任务均为行为等价重构或注释修正。依赖方向不变（cartfsm→tmc2209 由本计划 Task 3 从"声明未用"转正为真实使用）。

**Tech Stack:** ESP-IDF v6.1（CMake 组件）、Unity（test/ 独立测试工程）、宿主 gcc（wsl，`-std=c11 -Wall -Wextra -Werror`）。

**Spec:** 本计划规格 = 2026-10-13 会话交付的业务层评审报告（未落盘，缺陷清单与修复建议已完整内嵌到各任务的"缺陷与契约"块）+ 用户已拍板决策（A2 fail-fast、B7 cartparams 搬家、B9 幽灵引用清扫——三项已实施：c88316e/fabaeb3/fa761c6）。

## Global Constraints

- 注释与提交信息以中文为主体，保留准确标识符、协议字面量与通行英文术语（AGENTS.md 语言规则）。
- 行为等价重构不得改变任何可观察行为（日志文案、返回码、时序、守卫条件）。仅 A1/A3 为已拍板的行为变更，其余任务出现行为差异即缺陷。
- 参数名的唯一权威是 `components/cartparams/params.c` 的注册表；cartfsm 外壳不得再写字面量副本（AGENTS.md 单一权威）。
- 每任务完成时的验证门禁（命令须真实执行并看到成功输出，禁止 `> NUL` 重定向式假门禁）：
  - 主工程：`powershell.exe -NoProfile -Command "Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue; . 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'; Set-Location 'C:\Users\84580\workspace\cld\PD-STEP'; idf.py -B build build"`（看到 `Project build complete`）
  - 测试工程：同环境 `idf.py -C test build`（禁止携带 `-B`；看到生成 `test\build\cartpole-test.bin` 的 flash 提示）
  - 宿主六组：见 Task 4 步骤中的脚本模板（`MSYS_NO_PATHCONV=1 wsl bash …`）
- 上板用例（flowtest 3c、step_engine 分段计时对照）需要硬件在场；无硬件时构建验证必须完成，上板项在台账标注"待上板"，不得声称已验证。
- 执行中发现计划与代码事实不一致、或进入计划未定义的状态，停止该任务并报告证据（AGENTS.md 实施规则）。

## Review Focus

规格未显式覆盖、但最可能咬到使用者的五类输入/条件（每行注明归属任务与测试锚点）：

1. FAULT settling 期间晚到的非零速度目标（exec 锁外 `vel_target` 晚于 `enter_fault` 的 halt）：期望不依赖 WDT 兜底即在减速时间内停机、EN 关、推 MODE FAULT。——Task 2，flowtest 用例 3c（确定性红-绿，WDT 拉满后 300 ms 断言窗）。
2. 软限位边界处 VEL 命令与监督钳制拍交错：期望最终一致（最多丢一条边界命令，下一监督拍自愈），且 `s_last_cmd_v` 全部读写路径持 `s_lock`。——Task 2，flowtest 用例 1 回归 + 访问点 grep 审计锚。
3. `tmc2209_enable(true)` 在 comm_task（CPU0，订阅任务看门狗）里阻塞 2 ms：期望远低于看门狗超时、不误触发。——Task 3，代码审计说明（2 ms vs 30 s TWDT，无数量级风险）。
4. 解析器改用 `strncmp` 后的前缀近似词（`VELX`/`VEL `/`CONF`）与 `CONFIG?` 问号形态：期望词法判定与改造前逐位一致。——Task 7，宿主 cartproto_parse_test 57 用例回归（含边界前缀用例）。
5. ISR 重构后链式等待分支（dt > 60 ms 的长间隔拆链）：期望链式拍仍只记 `max_isr_cycles`、不进分段均值（与原语义一致）。——Task 5，test 工程 `[step_engine]` 用例回归 + 上板分段计时对照。

---

### Task 1: cartfsm exec_cmd 机械拆分（B6 前半）

**缺陷与契约：** `exec_cmd`（components/cartfsm/cartfsm.c:181-331）150 行大 switch 承担 7 种命令的分派与全部业务细节，新命令只能继续往里塞。本任务把每个 case 体搬进独立的 `exec_*` static 函数，`exec_cmd` 只留分派；行为逐字等价。同时把 MARK 两处重复的 TEST_HOOKS 偏置注入抽成 `consume_test_bias`（fsm_task 判定拍的注入块同用）。

**Files:**
- Modify: `components/cartfsm/cartfsm.c`

**Interfaces:**
- Consumes: 现有 `cart_req_t`（cartproto.h）、`push_mode`、`soft_window`、`mmps_to_q16`、`um_to_cmd_steps`。
- Produces（后续任务在本任务产物上改动，签名如下）:
  - `static void exec_query(void);`
  - `static void exec_mark_left(void);`
  - `static void exec_mark_right(void);`
  - `static void exec_enable(void);`
  - `static void exec_reset(void);`
  - `static void exec_vel(int32_t v_mm_s);`
  - `static void exec_config_set(const cart_req_t *req);`
  - `static void exec_config_dump(void);`
  - `#ifdef CONFIG_CARTFSM_TEST_HOOKS  static long consume_test_bias(long e_um); #endif`

- [ ] **Step 1: 重写命令处理段**

将 cartfsm.c 中 `/* ---- 命令执行（cartproto exec，CPU0） ---- */` 起至 `exec_cmd` 函数结束的整段，替换为：

```c
/* ---- 命令处理（cartproto exec 回调，CPU0） ----
 * exec_cmd 只做分派；各命令的业务细节在 exec_* 函数。 */

#ifdef CONFIG_CARTFSM_TEST_HOOKS
/* 流程测试注入：读数偏置一次性消费（MARK 标记与丢步判定拍共用） */
static long consume_test_bias(long e_um)
{
    e_um += s_test_loss_bias_um;
    s_test_loss_bias_um = 0;
    return e_um;
}
#endif

static void exec_query(void)
{
    cm_mode_t m;
    cm_reason_t r;
    portENTER_CRITICAL(&s_lock);
    m = s_mode;
    r = s_reason;
    portEXIT_CRITICAL(&s_lock);
    push_mode(m, r);
}

static void exec_mark_left(void)
{
    if (s_mode != CM_DISABLED) {
        cartproto_send_err("MARK_LEFT");   /* 规格 §2 回执原则 */
        return;
    }
    long e = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_TEST_HOOKS
    e = consume_test_bias(e);
#endif
    s_e_left_um = e;           /* 重复标记即覆盖重开（规格 §2） */
    s_calib = CAL_LEFT;
    cartproto_send_ok("MARK_LEFT", e);
}

static void exec_mark_right(void)
{
    if (s_mode != CM_DISABLED) {
        cartproto_send_err("MARK_RIGHT");
        return;
    }
    long e = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_TEST_HOOKS
    e = consume_test_bias(e);
#endif
    long l = e - s_e_left_um;
    if (s_calib != CAL_LEFT || !cm_calib_valid(l)) {
        cartproto_send_err("MARK_RIGHT");  /* 状态保持（规格 §2） */
        return;
    }
    s_calib_l_um = l;
    s_calib = CAL_DONE;
    /* E/C 同帧重定基：两标中点为 x=0（引擎停止时整定）。
     * set_origin 的契约是「当前物理位置此后读 v」——此刻小车
     * 在右端，传 +L/2 即得「右端=+L/2、中点=0、左端=−L/2」；
     * C 同步到 E 的新读数，保持 E−C=0 */
    as5600_set_origin((float)l / 2.0f / 1000.0f);
    step_engine_set_position(um_to_cmd_steps(l / 2));
    cartproto_send_ok("MARK_RIGHT", l);
}

static void exec_enable(void)
{
    if (s_mode != CM_DISABLED) {
        return;   /* READY/FAULT 忽略（规格 §5 矩阵） */
    }
    if (s_calib != CAL_DONE) {
        cartproto_send_err("ENABLE");
        return;
    }
    cm_mode_t m = s_mode;
    cm_reason_t r = CR_NONE;
    if (!cm_step(&m, &r, CE_CMD_ENABLE)) {
        return;
    }
    /* C 重锚到 E：先于模式发布——监督任务见 READY 即按拍判定，
     * 携旧 C 会在首拍 STEP_LOSS 误报（标定后手挪过的窗口） */
    step_engine_set_position(um_to_cmd_steps((long)as5600_position_um()));
    bsp_tmc_en_set(true);
    portENTER_CRITICAL(&s_lock);
    s_mode = CM_READY;
    s_reason = CR_NONE;
    s_last_cmd_v = 0;
    portEXIT_CRITICAL(&s_lock);
    push_mode(CM_READY, CR_NONE);
}

static void exec_reset(void)
{
    cm_mode_t m = s_mode;
    cm_reason_t r = s_reason;
    if (!cm_step(&m, &r, CE_CMD_RESET)) {
        return;   /* 仅 FAULT 生效 */
    }
    portENTER_CRITICAL(&s_lock);
    s_mode = CM_DISABLED;
    s_reason = CR_NONE;
    s_fault_settling = false;
    s_last_cmd_v = 0;
    portEXIT_CRITICAL(&s_lock);

    step_engine_stop();          /* 斜坡未完也立即急停 */
    bsp_tmc_en_set(false);
    params_apply_engine();       /* 恢复 ACC_MAX 档等 */
    push_mode(CM_DISABLED, CR_NONE);
}

static void exec_vel(int32_t v_mm_s)
{
    portENTER_CRITICAL(&s_lock);
    cm_mode_t cur = s_mode;
    portEXIT_CRITICAL(&s_lock);
    if (cur != CM_READY) {
        return;   /* 仅 READY 执行；持锁复查防 FAULT 进入瞬间的在途命令 */
    }
    const fw_params_t *p = fw_params();
    long v = v_mm_s;
    if (v > p->vel_max) {
        v = p->vel_max;
    } else if (v < -p->vel_max) {
        v = -p->vel_max;
    }
    long xmin, xmax;
    soft_window(p, &xmin, &xmax);
    long eff = cm_soft_limit_clamp((long)as5600_position_um(),
                                   xmin, xmax, v);
    s_last_cmd_v = eff;
    step_engine_vel_target(mmps_to_q16(eff));
}

static void exec_config_set(const cart_req_t *req)
{
    if (!cm_config_set_allowed(s_mode)) {
        cartproto_send_err(req->name);   /* FAULT 拒绝 */
        return;
    }
    if (fw_params_set(req->name, req->value)) {
        params_apply_one(req->name);
        cartproto_send_ok(req->name, req->value);   /* 回显归一化值 */
    } else {
        cartproto_send_err(req->name);
    }
}

static void exec_config_dump(void)
{
    for (int i = 0; i < fw_params_count(); i++) {
        cartproto_send_ok(fw_params_name(i), fw_params_value(i));
    }
}

static void exec_cmd(const cart_req_t *req)
{
    switch (req->cmd) {
    case CART_CMD_QUERY:       exec_query(); break;
    case CART_CMD_MARK_LEFT:   exec_mark_left(); break;
    case CART_CMD_MARK_RIGHT:  exec_mark_right(); break;
    case CART_CMD_ENABLE:      exec_enable(); break;
    case CART_CMD_RESET:       exec_reset(); break;
    case CART_CMD_VEL:         exec_vel(req->v_mm_s); break;
    case CART_CMD_CONFIG_SET:  exec_config_set(req); break;
    case CART_CMD_CONFIG_DUMP: exec_config_dump(); break;
    default:
        break;
    }
}
```

- [ ] **Step 2: fsm_task 判定拍的注入块改用 consume_test_bias**

fsm_task 中丢步判定段：

```c
                long e_um = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_TEST_HOOKS
                e_um += s_test_loss_bias_um;
                s_test_loss_bias_um = 0;   /* 一次性注入：判定拍消费即清 */
#endif
```

改为：

```c
                long e_um = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_TEST_HOOKS
                e_um = consume_test_bias(e_um);
#endif
```

注意：`consume_test_bias` 定义在 exec_* 段（fsm_task 之前），先声明后使用顺序成立。

- [ ] **Step 3: 构建验证**

主工程构建（Global Constraints 命令）看到 `Project build complete`；测试工程构建看到 `cartpole-test.bin`。此任务无宿主可失败测试（纯机械搬移，TDD 不适用，属等价重构裁定），行为等价锚 = Task 2 完成后的 flowtest 全量回归。

- [ ] **Step 4: Commit**

```bash
git add components/cartfsm/cartfsm.c
git commit -m "cartfsm：exec_cmd 按 command 拆为 exec_* 处理函数，分派与业务细节分离；测试偏置注入抽 consume_test_bias"
```

---

### Task 2: cartfsm 并发与钳制收口（A3 + A4 + B1）

**缺陷与契约：**
- A3：`exec_vel` 持锁复查模式后在锁外下发 `vel_target`，CPU1 的 `enter_fault`（halt）可被随后的 vel_target 覆盖；监督任务 FAULT settling 分支只轮询 `vel_now==0` 不再 halt，兜底只剩 WDT_MS。**拍板契约：settling 分支每拍重发 `step_engine_vel_halt()`，吃掉任何在途目标写入，不依赖 WDT 兜底即完成停机次序（EN 关 → 推 MODE）。**
- A4：`s_last_cmd_v` 访问点 141 持锁读、150/302 裸写、381-384 裸读改写，约定不统一。**契约：`s_last_cmd_v` 的全部读与写路径持 `s_lock`；引擎调用（halt/vel_target）一律在 `s_lock` 之外（避免 cartfsm s_lock 与 step_engine spinlock 嵌套）。**
- B1：软限位钳制（soft_window → clamp → 下发）在 exec_vel / params_apply_one(VEL_MAX) / fsm_task 三处重复。**契约：收口为两个辅助函数，三处共用。**

**Files:**
- Modify: `components/cartfsm/cartfsm.c`
- Test: `components/cartfsm/test/test_cartfsm_flow.c`

**Interfaces:**
- Consumes: Task 1 的 `exec_vel(int32_t)`；现有 `soft_window`、`cm_soft_limit_clamp`、`step_engine_vel_halt/target`、`as5600_position_um`。
- Produces:
  - `static void clamp_and_commit_velocity(long v_mm_s);`（exec_vel 与 VEL_MAX 收紧共用）
  - `static void supervise_soft_limit(const fw_params_t *p);`（监督拍持续钳制）

- [ ] **Step 1: 写失败测试（flowtest 3c）**

在 `components/cartfsm/test/test_cartfsm_flow.c` 用例 3 之后追加（文件头注释的用例清单同步加一行 `3c. FAULT settling 吃掉在途目标`）：

```c
/* 3c. FAULT settling 期间晚到的非零目标：监督每拍重发 halt，
 * 不依赖 WDT 兜底即完成停机次序（EN 关）。
 * 模拟 A3 场景：exec 锁外 vel_target 晚于 enter_fault 的 halt。
 * WDT 拉满 1000 ms——旧实现（无重发 halt）下 300 ms 断言窗内
 * 引擎持续运动、EN 不关，本用例必红。 */
TEST_CASE("流程：FAULT settling 吃掉在途目标（WDT 不兜底）", "[cartfsm][flowtest]")
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_READY, 200), "进入 READY");

    cart_req_t wdt = { .cmd = CART_CMD_CONFIG_SET, .name = "WDT_MS",
                       .value = 1000 };
    cartfsm_test_exec(&wdt);          /* READY 下 CONFIG SET 允许 */

    cartfsm_test_inject_encoder_fail(1);
    TEST_ASSERT_TRUE_MESSAGE(wait_mode(CM_FAULT, 100), "失联应 FAULT");

    /* 在途写入：settling 进行中注入引擎目标（A3 穿透形态） */
    step_engine_vel_target(200 << 16);
    TEST_ASSERT_TRUE_MESSAGE(wait_fault_settled(300),
                             "settling 应每拍重发 halt，300 ms 内 EN 关");
    cartfsm_test_inject_encoder_fail(0);
    to_disabled();
}
```

- [ ] **Step 2: 测试工程构建确认可编译（无硬件时到此为止，台账标注待上板）**

`idf.py -C test build` 看到 `cartpole-test.bin`。上板执行 `[flowtest]` 用例 3c：**预期 FAIL**（旧实现不重发 halt；EN 不关）。无硬件时跳过执行，仅以 Step 5 的实现审查 + 后续上板补证。

- [ ] **Step 3: 实现收口**

3a. `params_apply_one` 上方新增两个辅助函数：

```c
/* 软限位钳制收口：读当前位置、按窗口钳制 v、持锁提交 s_last_cmd_v、
 * 锁外下发引擎。exec_vel 与 VEL_MAX 运行中收紧共用。
 * 引擎调用在 s_lock 之外（不与 step_engine 的 spinlock 嵌套）。 */
static void clamp_and_commit_velocity(long v_mm_s)
{
    const fw_params_t *p = fw_params();
    long xmin, xmax;
    soft_window(p, &xmin, &xmax);
    long eff = cm_soft_limit_clamp((long)as5600_position_um(),
                                   xmin, xmax, v_mm_s);

    portENTER_CRITICAL(&s_lock);
    s_last_cmd_v = eff;
    portEXIT_CRITICAL(&s_lock);

    step_engine_vel_target(mmps_to_q16(eff));
}

/* 监督拍软限位持续钳制：当前目标被窗口钳为不同值时斜坡停并清
 * 目标。读/写 s_last_cmd_v 持锁，引擎调用在锁外。 */
static void supervise_soft_limit(const fw_params_t *p)
{
    portENTER_CRITICAL(&s_lock);
    long cmd = s_last_cmd_v;
    portEXIT_CRITICAL(&s_lock);

    long xmin, xmax;
    soft_window(p, &xmin, &xmax);
    long eff = cm_soft_limit_clamp((long)as5600_position_um(),
                                   xmin, xmax, cmd);
    if (eff != cmd) {
        step_engine_vel_halt();
        portENTER_CRITICAL(&s_lock);
        s_last_cmd_v = 0;
        portEXIT_CRITICAL(&s_lock);
    }
}
```

3b. `exec_vel`（Task 1 产物）尾段替换——从 `const fw_params_t *p = fw_params();` 到函数末尾改为：

```c
    const fw_params_t *p = fw_params();
    long v = v_mm_s;
    if (v > p->vel_max) {
        v = p->vel_max;
    } else if (v < -p->vel_max) {
        v = -p->vel_max;
    }
    clamp_and_commit_velocity(v);
```

3c. `params_apply_one` 的 VEL_MAX 分支替换为：

```c
    } else if (strcmp(name, "VEL_MAX") == 0) {
        /* 运行中收紧 VEL_MAX 立即约束当前运动（protocol.md 收紧语义） */
        portENTER_CRITICAL(&s_lock);
        long cur = s_last_cmd_v;
        portEXIT_CRITICAL(&s_lock);
        if (cur > p->vel_max || cur < -p->vel_max) {
            long v = (cur > 0) ? p->vel_max : -p->vel_max;
            clamp_and_commit_velocity(v);
        }
    }
```

（strcmp 分派保持原样，Task 4 统一改 idx。）

3d. `fsm_task` 监督段丢步判定的 else 分支（原软限位持续钳制内联段）替换为：

```c
                } else {
                    /* 软限位持续钳制（运行中收紧立即生效） */
                    supervise_soft_limit(p);
                }
```

3e. `fsm_task` 的 FAULT settling 分支改为：

```c
        } else if (mode == CM_FAULT && s_fault_settling) {
            /* 次序 ②③：每拍重发斜坡停——吃掉 FAULT 进入后仍在途的
             * 目标写入（exec_vel 锁外 vel_target 可晚于 enter_fault
             * 的 halt，穿透后不能只靠 WDT 兜底）→ 等零 → EN 关 →
             * 推 MODE */
            step_engine_vel_halt();
            if (step_engine_vel_now_q16() == 0) {
                bsp_tmc_en_set(false);
                s_fault_settling = false;
                cm_reason_t r = s_reason;
                push_mode(CM_FAULT, r);
            }
        }
```

- [ ] **Step 4: A4 审计锚**

`grep -n "s_last_cmd_v" components/cartfsm/cartfsm.c` 逐点核对：每个读/写都在 `portENTER_CRITICAL(&s_lock)`/`EXIT` 配对内（Task 1/2 后应仅剩 clamp_and_commit_velocity 写、supervise_soft_limit 读写、exec_enable/exec_reset 清零共 4 处，全部持锁）。

- [ ] **Step 5: 验证**

上板跑 `[flowtest]` 全量：3c 转 PASS，其余 6 个用例回归不变。宿主 cartfsm_core_test 回归（不含本任务改动面，确认未误伤）。双工程构建门禁。

- [ ] **Step 6: Commit**

```bash
git add components/cartfsm/cartfsm.c components/cartfsm/test/test_cartfsm_flow.c
git commit -m "cartfsm 并发收口：FAULT settling 每拍重发 halt 吃掉在途目标写入；s_last_cmd_v 全路径持锁；软限位钳制三处收口为 clamp_and_commit_velocity/supervise_soft_limit"
```

---

### Task 3: EN 使能收口 + CMake 依赖清理（A1 + B5）

**缺陷与契约：** A1——`tmc2209_enable`（含 EN 低后 2 ms settle，tmc2209.h 契约）生产零调用；cartfsm 直接 `bsp_tmc_en_set` 绕过时序。**拍板契约：cartfsm 的全部 EN 操作经 `tmc2209_enable`（enable 方向获得 settle；disable 方向语义等价）。** B5——CMake 多余依赖清理：`cartfsm REQUIRES tmc2209` 由本任务转正（真实使用）；`step_engine PRIV_REQUIRES bsp` 删除（源码未用）；`tmc2209 REQUIRES driver`（umbrella）改细粒度。

**Files:**
- Modify: `components/cartfsm/cartfsm.c`（exec_enable、exec_reset、fsm_task settling、cartfsm_start 四处 `bsp_tmc_en_set`）
- Modify: `components/cartfsm/CMakeLists.txt`（无变化，依赖已存在）
- Modify: `components/step_engine/CMakeLists.txt`（删 PRIV_REQUIRES bsp）
- Modify: `components/tmc2209/CMakeLists.txt`（driver → 细粒度）

**Interfaces:**
- Consumes: `esp_err_t tmc2209_enable(bool en);`（tmc2209.h，已存在）。
- Produces: 无新接口；cartfsm→tmc2209 依赖自此为真实使用。

- [ ] **Step 1: cartfsm.c 四处替换 + include**

cartfsm.c 顶部 include 块加 `#include "tmc2209.h"`（置于 `#include "step_engine.h"` 之前，按字母序）。四处替换：
- exec_enable 内 `bsp_tmc_en_set(true);` → `tmc2209_enable(true);`（2 ms settle 在 comm_task 上下文阻塞，远低于 30 s 任务看门狗，无数量级风险）
- exec_reset 内 `bsp_tmc_en_set(false);` → `tmc2209_enable(false);`
- fsm_task settling 分支 `bsp_tmc_en_set(false);` → `tmc2209_enable(false);`
- cartfsm_start 内 `bsp_tmc_en_set(false);` → `tmc2209_enable(false);`

- [ ] **Step 2: CMake 清理**

`components/step_engine/CMakeLists.txt`：`PRIV_REQUIRES bsp` 一行删除（SRCS 行的 REQUIRES 其余不动）。
`components/tmc2209/CMakeLists.txt`：

```cmake
idf_component_register(
    SRCS "tmc2209_hw.c" "tmc2209.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart freertos bsp settings
)
```

（tmc2209_hw.c 用 driver/uart.h；tmc2209.c 用 freertos/task.h；esp_log 为公共组件免声明。）

- [ ] **Step 3: 验证**

双工程构建门禁。上板 flowtest 回归（ENABLE 路径多 2 ms settle，用例 1/2/5 的 `wait_mode(CM_READY, 200)` 断言窗不受影响）；上板确认项台账记录"ENABLE 后立即 VEL 的首步完整性"（一次性人工核对）。

- [ ] **Step 4: Commit**

```bash
git add components/cartfsm/cartfsm.c components/step_engine/CMakeLists.txt components/tmc2209/CMakeLists.txt
git commit -m "EN 使能收口：cartfsm 全部 EN 操作经 tmc2209_enable（ENABLE 命令获得 2ms settle）；CMake 清理 step_engine 冗余 bsp 依赖、tmc2209 改细粒度 esp_driver_uart"
```

---

### Task 4: 参数权威化 + 订阅枚举（B3 + B4）

**缺陷与契约：** B3——参数名权威在 cartparams 注册表，cartfsm.c:130-138 又用 strcmp 字面量分派（违反单一权威）。**契约：cartparams 导出参数索引枚举 `FP_*` 与按名查索引 `fw_params_find`；注册表数组改指定初始化器与枚举锁定一致；cartfsm 作用点分派改 `switch (fw_params_find(name))`。** B4——订阅回调用魔数（as5600 `idx == 2`、tmc2209 `case 0/3/4`），改用 settings.h 枚举。

**Files:**
- Modify: `components/cartparams/include/params.h`
- Modify: `components/cartparams/params.c`
- Modify: `components/cartfsm/cartfsm.c`（params_apply_one）
- Modify: `components/as5600/as5600.c`、`components/tmc2209/tmc2209.c`（订阅回调）
- Test: `host_tests/cartfsm_core_test.c`

**Interfaces:**
- Produces:
  - `enum { FP_VEL_MAX = 0, FP_ACC_MAX, FP_WDT_MS, FP_WDT_DEC, FP_X_MARGIN_UM, FP_POLE_ZERO_URAD, FP_POLE_INV, FP_X_INV, FP_STEP_LOSS_UM, FP_COUNT };`（params.h）
  - `int fw_params_find(const char *name);`（未知/NULL 返回 -1）

- [ ] **Step 1: 写失败测试（宿主）**

`host_tests/cartfsm_core_test.c` 在既有 `test_*` 函数区追加，并按该文件现有模式在 `main` 中登记调用：

```c
/* ---- fw_params_find（B3 权威化） ---- */
static void test_fw_params_find(void)
{
    CHECK(fw_params_find("VEL_MAX") == FP_VEL_MAX, "find VEL_MAX");
    CHECK(fw_params_find("ACC_MAX") == FP_ACC_MAX, "find ACC_MAX");
    CHECK(fw_params_find("WDT_DEC") == FP_WDT_DEC, "find WDT_DEC");
    CHECK(fw_params_find("STEP_LOSS_UM") == FP_STEP_LOSS_UM, "find 尾项");
    CHECK(fw_params_find("NOPE") == -1, "未知名 -1");
    CHECK(fw_params_find("") == -1, "空名 -1");
    CHECK(fw_params_find(NULL) == -1, "NULL -1");
    CHECK(fw_params_count() == FP_COUNT, "枚举数与注册表一致");
}
```

（main 内追加 `test_fw_params_find();` 一行。）

- [ ] **Step 2: 跑宿主测试确认失败**

```bash
cat > /c/Users/84580/AppData/Local/Temp/ht_t4.sh <<'EOF'
set -e
cd /mnt/c/Users/84580/workspace/cld/PD-STEP
gcc -std=c11 -Wall -Wextra -Werror -Ihost_tests/stub \
    -Icomponents/cartfsm/include -Icomponents/cartparams/include \
    host_tests/cartfsm_core_test.c components/cartfsm/cartfsm_core.c \
    components/cartparams/params.c -o /tmp/ht_t4 && /tmp/ht_t4
EOF
MSYS_NO_PATHCONV=1 wsl bash /mnt/c/Users/84580/AppData/Local/Temp/ht_t4.sh; rm -f /c/Users/84580/AppData/Local/Temp/ht_t4.sh
```

预期：编译失败（`fw_params_find`/`FP_*` 未定义）。

- [ ] **Step 3: 实现**

3a. `params.h` 在 `fw_params_t` 定义之前加：

```c
/* 参数索引（注册表顺序，CONFIG? dump 按此序）。
 * 注册表数组以指定初始化器与本枚举锁定一致；作用点分派一律
 * 经 fw_params_find，不得在外壳重复参数名字面量。 */
enum {
    FP_VEL_MAX = 0, FP_ACC_MAX, FP_WDT_MS, FP_WDT_DEC,
    FP_X_MARGIN_UM, FP_POLE_ZERO_URAD, FP_POLE_INV,
    FP_X_INV, FP_STEP_LOSS_UM, FP_COUNT,
};
```

并在核心 API 区加声明：

```c
/**
 * @brief 按名查参数索引。未知 NAME 或 NULL 返回 -1。
 */
int fw_params_find(const char *name);
```

3b. `params.c`：include 改为 `#include <stddef.h>` + `#include <string.h>`；`s_entries` 改指定初始化器：

```c
static const param_entry_t s_entries[FP_COUNT] = {
    [FP_VEL_MAX]        = { "VEL_MAX",        &s_params.vel_max,         1,       2000    },
    [FP_ACC_MAX]        = { "ACC_MAX",        &s_params.acc_max,         100,     30000   },
    [FP_WDT_MS]         = { "WDT_MS",         &s_params.wdt_ms,          10,      1000    },
    [FP_WDT_DEC]        = { "WDT_DEC",        &s_params.wdt_dec,         100,     30000   },
    [FP_X_MARGIN_UM]    = { "X_MARGIN_UM",    &s_params.x_margin_um,     1000,    50000   },
    [FP_POLE_ZERO_URAD] = { "POLE_ZERO_URAD", &s_params.pole_zero_urad,  -3141592, 3141592 },
    [FP_POLE_INV]       = { "POLE_INV",       &s_params.pole_inv,        0,       1       },
    [FP_X_INV]          = { "X_INV",          &s_params.x_inv,           0,       1       },
    [FP_STEP_LOSS_UM]   = { "STEP_LOSS_UM",   &s_params.step_loss_um,    500,     20000   },
};
```

`fw_params_set` 的手写逐字符比较替换为 `strcmp(s_entries[i].name, name) == 0`（匹配分支内逻辑不变）；新增：

```c
int fw_params_find(const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (int i = 0; i < ENTRY_COUNT; i++) {
        if (strcmp(s_entries[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}
```

3c. `cartfsm.c` 的 `params_apply_one` 分派整体替换为：

```c
static void params_apply_one(const char *name)
{
    const fw_params_t *p = fw_params();
    switch (fw_params_find(name)) {
    case FP_ACC_MAX:
        step_engine_vel_set_accel(accel_to_steps(p->acc_max));
        break;
    case FP_WDT_DEC:
        step_engine_vel_set_decel(accel_to_steps(p->wdt_dec));
        break;
    case FP_WDT_MS:
        step_engine_vel_set_wdt_us((uint32_t)p->wdt_ms * 1000u);
        break;
    case FP_X_INV:
        step_engine_set_dir_invert(p->x_inv != 0);
        break;
    case FP_VEL_MAX:
        /* 运行中收紧 VEL_MAX 立即约束当前运动（protocol.md 收紧语义） */
        portENTER_CRITICAL(&s_lock);
        long cur = s_last_cmd_v;
        portEXIT_CRITICAL(&s_lock);
        if (cur > p->vel_max || cur < -p->vel_max) {
            long v = (cur > 0) ? p->vel_max : -p->vel_max;
            clamp_and_commit_velocity(v);
        }
        break;
    default:
        break;   /* 其余参数（X_MARGIN_UM/标定量）为读取点即时生效 */
    }
}
```

3d. `as5600.c` 订阅回调 `if (idx == 2)` → `if (idx == SET_MM_PER_REV)`；`tmc2209.c` `settings_change_cb` 的 `case 0:` → `case SET_MICROSTEP:`、`case 3:` → `case SET_RUN_MA:`、`case 4:` → `case SET_HOLD_MA:`（注释 `/* microstep */` 等随之删除，枚举名自明）。

- [ ] **Step 4: 跑宿主测试确认通过**

重跑 Step 2 命令。预期：`cartfsm_core_test: NN/NN 断言通过`，0 失败。

- [ ] **Step 5: 全量验证**

宿主六组全跑（Global Constraints 门禁）+ 双工程构建。上板 `[flowtest]` 回归（CONFIG SET 路径）。

- [ ] **Step 6: Commit**

```bash
git add components/cartparams components/cartfsm/cartfsm.c components/as5600/as5600.c components/tmc2209/tmc2209.c host_tests/cartfsm_core_test.c
git commit -m "参数权威化：cartparams 导出 FP_* 索引枚举与 fw_params_find，注册表指定初始化器锁定顺序；cartfsm 作用点改索引分派；settings 订阅回调用枚举替代魔数"
```

---

### Task 5: step_engine ISR 插桩收口（B2）

**缺陷与契约：** `gptimer_alarm_cb`（step_engine.c:173-319）被 TEST_HOOKS 插桩切碎，`!moving` 分支（258-289）与正常分支（294-316）有 26 行几乎相同的分段统计。**契约：统计收口为 `record_isr_timings` 单出口辅助，行为逐字等价——链式等待分支仍只更新 `s_max_isr_cycles`（不进分段统计）；`!moving` 分支的计时采样点仍在 `gptimer_stop` 之后（alarm 段含停表耗时）；正常分支采样点在 `isr_set_alarm` 之后。** ISR 分支结构变化可能影响周期数，须上板对照。

**Files:**
- Modify: `components/step_engine/step_engine.c`

**Interfaces:**
- Produces（仅 `#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS` 内可见）:
  - `static inline void record_isr_timings(uint32_t start, uint32_t pulse_end, uint32_t ramp_end, uint32_t now);`

- [ ] **Step 1: 新增辅助并重构 ISR**

在 `gptimer_alarm_cb` 定义之前（TEST_HOOKS 状态声明区之后）加：

```c
#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
/* 分段计时更新（ISR 发步出口调用一次；max 段与 sum 段口径同旧：
 * total=now-start，pulse=pulse_end-start，ramp=ramp_end-pulse_end，
 * alarm=now-ramp_end）。 */
static inline void record_isr_timings(uint32_t start, uint32_t pulse_end,
                                      uint32_t ramp_end, uint32_t now)
{
    uint32_t total = now - start;
    if (total > s_max_isr_cycles) {
        s_max_isr_cycles = total;
    }
    uint32_t pulse = pulse_end - start;
    if (pulse > s_max_cyc_pulse) {
        s_max_cyc_pulse = pulse;
    }
    uint32_t ramp = ramp_end - pulse_end;
    if (ramp > s_max_cyc_ramp) {
        s_max_cyc_ramp = ramp;
    }
    uint32_t alarm = now - ramp_end;
    if (alarm > s_max_cyc_alarm) {
        s_max_cyc_alarm = alarm;
    }
    s_sum_cyc_pulse += pulse;
    s_sum_cyc_ramp += ramp;
    s_sum_cyc_alarm += alarm;
    s_isr_pulse_count++;
}
#endif /* CONFIG_STEP_ENGINE_TEST_HOOKS */
```

ISR 主体改为（`#ifdef` 之外的行与现状逐字相同）：

```c
static bool gptimer_alarm_cb(gptimer_handle_t timer,
                             const gptimer_alarm_event_data_t *edata,
                             void *user_ctx)
{
    (void)timer;
    (void)user_ctx;

    s_cb_alarm_base = edata->alarm_value;
    s_cb_guard_floor = edata->count_value + ALARM_GUARD_US;

#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
    uint32_t isr_start_ccount = esp_cpu_get_cycle_count();
#endif

    /* 停止标志最优先 */
    if (s_stopped) {
        gptimer_stop(s_timer);
        s_timer_running = false;
        return false;
    }

    /* 链式等待：本次 alarm 不发步，续排下一块（不发步不进分段统计，
     * 只更新整段最大值，口径同旧） */
    if (s_chain_remaining_us > 0) {
        uint32_t chunk = (s_chain_remaining_us > MAX_ALARM_US)
                       ? MAX_ALARM_US : s_chain_remaining_us;
        s_chain_remaining_us -= chunk;
        arm_alarm(s_cb_alarm_base, s_cb_guard_floor, chunk);
#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
        uint32_t cend = esp_cpu_get_cycle_count();
        if (cend - isr_start_ccount > s_max_isr_cycles) {
            s_max_isr_cycles = cend - isr_start_ccount;
        }
#endif
        return false;
    }

    /* ---- 发 STEP 脉冲 ---- */
    gpio_set_level((gpio_num_t)s_step_gpio, 1);
    esp_rom_delay_us(1);
    gpio_set_level((gpio_num_t)s_step_gpio, 0);

    /* 位置计数：物理域（DIR 高 = 正） */
    if (s_dir ^ s_dir_invert) {
        s_position++;
    } else {
        s_position--;
    }

#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
    if (s_sample_buf && s_sample_count && *s_sample_count < s_sample_max) {
        uint32_t now = esp_cpu_get_cycle_count();
        if (s_last_alarm_ccount != 0) {
            s_sample_buf[*s_sample_count] = (now - s_last_alarm_ccount) / 240;
            (*s_sample_count)++;
        }
        s_last_alarm_ccount = now;
    }
    uint32_t cyc_pulse_end = esp_cpu_get_cycle_count();
#endif

    /* ---- 递推下一拍（持锁读共享目标，防 64 位撕裂读） ---- */
    uint64_t nv;
    uint32_t dt;
    bool     dchg;
    bool     moving;

    portENTER_CRITICAL_ISR(&s_spinlock);
    bool wdt_expired = (s_wdt_us > 0)
                    && (edata->count_value - s_last_kick_us > s_wdt_us);
    moving = vel_ramp_next(s_v_q16, s_dir, s_target_q16, s_target_dir,
                           s_accel, s_decel, wdt_expired,
                           &nv, &dt, &dchg);
    if (dchg) {
        s_dir = !s_dir;
        set_dir_physical(s_dir);
        dt += 2;   /* DIR setup 2 µs，并入第一拍间隔 */
    }
    s_v_q16 = nv;
    if (!moving) {
        /* 停表标志并入持锁段：与 vel_target 的静止启动判定互斥，
         * 避免「目标非零但引擎自停」的竞态窗口 */
        s_stopped = true;
    }
    portEXIT_CRITICAL_ISR(&s_spinlock);

#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
    uint32_t cyc_ramp_end = esp_cpu_get_cycle_count();
#endif

    if (!moving) {
        /* 到零速且无换向目标：停表（模式语义由 cartfsm 负责） */
        gptimer_stop(s_timer);
        s_timer_running = false;
#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
        record_isr_timings(isr_start_ccount, cyc_pulse_end,
                           cyc_ramp_end, esp_cpu_get_cycle_count());
#endif
        return false;
    }

    isr_set_alarm(dt);

#ifdef CONFIG_STEP_ENGINE_TEST_HOOKS
    record_isr_timings(isr_start_ccount, cyc_pulse_end,
                       cyc_ramp_end, esp_cpu_get_cycle_count());
#endif

    return false;
}
```

等价性要点（对照旧代码自审）：链式分支的 `s_max_isr_cycles` 更新保留；`!moving` 分支的 now 取自 `gptimer_stop` 之后（同旧 cend2）；正常分支 now 取自 `isr_set_alarm` 之后（同旧 cend3）；两份 26 行重复统计由同一函数替代。

- [ ] **Step 2: 构建与回归**

双工程构建；宿主 vel_ramp 回归（48 用例，确认未触碰纯函数）；test 工程上板 `[step_engine]` 全量用例。**上板对照项（台账记录）：重构前后 `step_selftest_avg/max_cycles_*` 读数对比，均值不得劣化超噪声（±5%）。**

- [ ] **Step 3: Commit**

```bash
git add components/step_engine/step_engine.c
git commit -m "step_engine：ISR 分段统计收口 record_isr_timings 单出口，消除两份重复插桩代码；行为与采样点逐字等价"
```

---

### Task 6: tmc2209_init 拆分（B6 后半）

**缺陷与契约：** `tmc2209_init`（tmc2209.c，A2 修改后约 110 行）承担 hw_init + 六段寄存器配置 + 订阅。**契约：寄存器配置段搬入 `static esp_err_t apply_default_registers(void)`（关键三寄存器失败返回 ESP_FAIL、次要三寄存器告警——fabaeb3 已定语义），init 主体只留生命周期编排；行为逐字等价。**

**Files:**
- Modify: `components/tmc2209/tmc2209.c`

**Interfaces:**
- Produces: `static esp_err_t apply_default_registers(void);`（仅本文件可见）

- [ ] **Step 1: 拆分**

`tmc2209_init` 中从 `/* ---- Apply default register configuration ---- */` 注释块起到 `if (!tmc2209_write_reg(REG_PWMCONF, pwmconf)) { … }` 止的整段，搬入新函数（寄存器宏与局部变量随之移动，值与日志逐字不变）：

```c
/* 上电默认寄存器配置。关键寄存器（GCONF/CHOPCONF/IHOLD_IRUN）任一
 * 推送失败返回 ESP_FAIL（驱动将停留在出厂配置，电流/细分失控）；
 * 次要寄存器失败仅告警。 */
static esp_err_t apply_default_registers(void)
{
    /* ……六段配置体逐字自 tmc2209_init 搬入（GCONF → CHOPCONF →
    *   IHOLD_IRUN → TPOWERDOWN → TPWMTHRS → PWMCONF，
    *   ESP_LOGE/LOGW 文案与返回值判定不变）…… */
    return ESP_OK;
}
```

（搬移体较长，机械操作：剪切原段、粘贴入函数、原变量声明随段移动。计划不重复正文以避免双份漂移源；执行者以仓库当前 tmc2209.c 的该段为准，逐字搬移不改一字。）

init 主体变为：

```c
esp_err_t tmc2209_init(const tmc2209_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_initialized) return ESP_OK;

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* UART 驱动与寄存器读写原语（驱动核心层） */
    tmc2209_hw_config_t hw = {
        .uart_num   = cfg->uart_num,
        .tx_pin     = cfg->tx_pin,
        .rx_pin     = cfg->rx_pin,
        .baud       = cfg->baud,
        .slave_addr = cfg->slave_addr,
    };
    esp_err_t err = tmc2209_hw_init(&hw);
    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(apply_default_registers(),
                        TAG, "default register config failed");

    s_initialized = true;

    ESP_LOGI(TAG, "initialized: microstep=%d, run=%dmA, hold=%dmA, VSENSE=%d",
             settings_microstep(),
             settings_run_current_ma(),
             settings_hold_current_ma(),
             s_vsense ? 1 : 0);

    /* Subscribe to settings changes ($0, $3, $4) */
    if (!settings_subscribe(settings_change_cb)) {
        ESP_LOGW(TAG, "settings_subscribe failed (slot full)");
    }

    return ESP_OK;
}
```

- [ ] **Step 2: 处理 cs_run/cs_hold 日志变量**

原 LOGI 打印 apply 段内算出的 `cs_run`/`cs_hold` 局部变量；拆分后 init 不再可见，Step 1 的目标 LOGI 已去掉 CS 字段（见上）。apply 段内现有的逐寄存器失败日志（LOGE）保留完整信息；如需成功路径明细，在 apply 末尾补一条 `ESP_LOGD` 打印 cs_run/cs_hold（沿用 set_current 的 LOGD 风格）。裁定记录：启动日志非协议输出，去掉 CS 字段属可接受的可观察变化。

- [ ] **Step 3: 验证**

双工程构建；上板 `[bringup]` 寄存器核对回归。

- [ ] **Step 4: Commit**

```bash
git add components/tmc2209/tmc2209.c
git commit -m "tmc2209：上电寄存器配置拆入 apply_default_registers，init 只留生命周期编排"
```

---

### Task 7: 注释漂移修正 + 风格杂项（B8 + B10）

**缺陷与契约：** B8——七处注释与实现不符（逐条见下）。B10——cartproto_parse 手写逐字符比较换 `strncmp`（行为逐位等价，宿主 57 用例回归）；settings `notify_locked` 名实不符且 settings_set 内联复制循环；comm_task 不可达代码；app_main `UART_NUM_2` 无说明；host_tests 残留一处 calib-spec 引用。

**Files:**
- Modify: `components/bsp/include/bsp.h`、`components/cartproto/include/cartproto.h`、`components/step_engine/include/step_engine.h`、`components/telemetry/include/telemetry.h`、`components/as5600/include/as5600.h`（注释）
- Modify: `components/cartproto/cartproto_parse.c`、`components/settings/settings.c`、`components/cartproto/cartproto.c`、`main/app_main.c`、`host_tests/cartfsm_core_test.c`（代码/注释）

**Interfaces:** 无新接口。

- [ ] **Step 1: 注释修正（B8 七处 + host_tests 残留一处）**

1. bsp.h `bsp_init` doc 中删除 `*   - UART1 (protocol channel) at Kconfig-configured pins / baud rate,\n *     with RX buffer >= 1024 bytes.` 两行，改为在该段首加一行：` *   - UART1 不在此初始化：cartproto 是其唯一所有者（引脚/波特率\n *     的 Kconfig 定义仍留在 BSP）。`
2. cartproto.h `cartproto_start` doc 中 `UART1 以事件队列重新初始化（bsp 先装了无队列驱动），波特率取` 改为 `初始化 UART1（事件队列驱动，cartproto 是其唯一所有者），波特率取`。
3. step_engine.h init doc `GPTimer @1 MHz、one-shot alarm、ISR 钉在 CPU0` 改为 `GPTimer @1 MHz、one-shot alarm；中断装在调用 init 的核（app_main 于 CPU0 启动，CPU0 布局由此保证）`。
4. step_engine.h `step_engine_vel_now_q16` doc `近似快照（多变量非原子读）；语义上用于判零与符号。` 改为 `持锁快照（s_spinlock 内一次读齐 stopped/v/dir/invert）；语义上用于判零与符号。`
5. telemetry.h 头注释 `所有固件模式（DISABLED/HOMING/READY/FAULT）无条件发送` 改为 `所有固件模式（DISABLED/READY/FAULT）无条件发送`。
6. telemetry.h 字段来源 `theta_urad ←\n * mt6835_theta_urad()（应用 POLE_ZERO_URAD/POLE_INV 标定）；` 改为 `theta_urad ←\n * mt6835_raw_to_urad(mt6835_last_raw(), …)（组帧点应用标定）；`。
7. as5600.h 红线注释段 `its only bus operation is the read helper\n *   i2c_master_transmit_receive() (register address out, data in,\n *   one transaction).  Do not add write helpers here.` 改为 `the business layer never touches the bus directly; its only\n *   read path is as5600_hw_read_raw() (driver core; register-level\n *   red-line details live in as5600_hw.c).  Do not add write helpers.`。
8. host_tests/cartfsm_core_test.c 头注释 `参数范围边界 ±1（calib-spec §2/§4）` 改为 `参数范围边界 ±1`。

- [ ] **Step 2: cartproto_parse 改 strncmp**

cartproto_parse.c：include 区加 `#include <string.h>`；新增辅助（trailing_ws 之前）：

```c
/* s 的前 len 字符与字面量 lit 完全相等（长度相等且逐位相等） */
static bool streq_n(const char *s, size_t len, const char *lit)
{
    return strlen(lit) == len && strncmp(s, lit, len) == 0;
}
```

命令分派替换（判定条件语义逐位等价——原来"cmd_len==N 且逐字符相等"即"前 N 字符等于字面量且无第 N+1 字符"）：
- `if (cmd_len == 5) { if (line[0]=='R'…) }` 段 → `if (streq_n(line, cmd_len, "RESET")) { out->cmd = CART_CMD_RESET; return true; }` 与 `if (streq_n(line, cmd_len, "QUERY")) { … }`
- ENABLE → `if (streq_n(line, cmd_len, "ENABLE")) { … }`
- CONFIG? → `if (streq_n(line, cmd_len, "CONFIG?")) { … }`
- MARK → `if (streq_n(line, cmd_len, "MARK")) { … }`（方向词同理：`streq_n(&line[rest], rl, "LEFT")` / `"RIGHT"`）
- VEL → `if (streq_n(line, cmd_len, "VEL")) { … }`
- CONFIG → `if (streq_n(line, cmd_len, "CONFIG")) { … }`

无参命令段整体变为：

```c
    if (cmd_len == len) {
        if (streq_n(line, cmd_len, "RESET")) {
            out->cmd = CART_CMD_RESET;
            return true;
        }
        if (streq_n(line, cmd_len, "QUERY")) {
            out->cmd = CART_CMD_QUERY;
            return true;
        }
        if (streq_n(line, cmd_len, "ENABLE")) {
            out->cmd = CART_CMD_ENABLE;
            return true;
        }
        if (streq_n(line, cmd_len, "CONFIG?")) {
            out->cmd = CART_CMD_CONFIG_DUMP;
            return true;
        }
        return false;
    }
```

- [ ] **Step 3: settings 通知函数改名复用**

settings.c：`notify_locked` 改名 `notify_all_subscribers`，注释改为：

```c
/* 通知全部订阅方。必须在 s_mutex 之外调用：订阅方回调可能调用
 * settings_get/is_valid（内部取锁），持锁通知会死锁。 */
```

`settings_reset_defaults` 循环内 `notify_locked(i, s_values[i]);` → `notify_all_subscribers(i, s_values[i]);`；`settings_set` 尾部内联 for 循环替换为 `notify_all_subscribers(idx, value);`。

- [ ] **Step 4: 小项**

- cartproto.c comm_task `for (;;)` 之后的 `/* 不可达 */ esp_task_wdt_delete(NULL); vTaskDelete(NULL);` 三行删除。
- app_main.c `.uart_num = UART_NUM_2,` 行尾加注释 `/* 端口分配：UART0=烧录/日志、UART1=协议、UART2=TMC */`。

- [ ] **Step 5: 验证**

宿主六组全绿（cartproto_parse 57 用例为等价锚，重点确认前缀近似词用例通过）+ 双工程构建。

- [ ] **Step 6: Commit**

```bash
git add components/bsp/include/bsp.h components/cartproto components/step_engine/include/step_engine.h components/telemetry/include/telemetry.h components/as5600/include/as5600.h components/settings/settings.c main/app_main.c host_tests/cartfsm_core_test.c
git commit -m "注释与实现对齐（UART1 所有权/ISR 核绑定/快照语义/HOMING/红线表述）；cartproto_parse 命令词改 strncmp；settings 通知函数改名并复用；清理不可达代码与 UART 端口说明"
```

---

## 附录 A：缺陷 → 任务映射

| 评审编号 | 内容 | 任务 | 状态 |
|---|---|---|---|
| A1 | EN 使能路径分裂（settle 丢失） | Task 3 | 本计划 |
| A3 | FAULT 与在途 VEL 竞态 | Task 2 | 本计划 |
| A4 | s_last_cmd_v 访问约定不一 | Task 2 | 本计划 |
| B1 | 软限位钳制三处重复 | Task 2 | 本计划 |
| B2 | ISR 插桩切碎 + 统计重复 | Task 5 | 本计划 |
| B3 | 参数名双权威 | Task 4 | 本计划 |
| B4 | 订阅回调魔数索引 | Task 4 | 本计划 |
| B5 | CMake 多余依赖 | Task 3 | 本计划 |
| B6 | exec_cmd / tmc2209_init 长函数 | Task 1 + Task 6 | 本计划 |
| B8 | 注释漂移七处 | Task 7 | 本计划 |
| B10 | 风格杂项（strncmp/notify/死代码/UART 注释） | Task 7（params 手写 strcmp 已并入 Task 4） | 本计划 |
| A2 | tmc2209 init fail-fast | — | 已实施 fabaeb3 |
| B7 | cartparams 搬家 | — | 已实施 c88316e |
| B9 | 幽灵引用清扫 | — | 已实施 fa761c6 |

未纳入本计划（用户此前裁定单独排期）：全仓中英文注释统一翻译。

## 附录 B：TDD 适用性裁定

- Task 2 A3：可确定性红-绿（flowtest 3c，WDT 拉满后 300 ms 断言窗内旧实现必红），在板上执行。
- Task 4 B3：可确定性红-绿（宿主 fw_params_find 用例先编译失败后通过）。
- Task 1/3/5/6：行为等价重构（A1 的 +2 ms settle 与 Task 6 的日志字段变化除外，均已在上文裁定说明），无数主可失败先行测试，验证锚 = 双工程构建 + 板上用例回归 + Task 5 分段计时对照。
- Task 7 cartproto_parse：等价重构，现有 57 宿主用例为回归锚。

## 附录 C：验证记录（执行时回填）

| 任务 | 主工程 | 测试工程 | 宿主 | 上板 | 提交 |
|---|---|---|---|---|---|
| Task 1 | complete | cartpole-test.bin | — | flowtest 回归随 Task 2 待上板 | 96ad4b1 |
| Task 2 | complete | complete | cartfsm_core 79/79 | 3c 红绿 + flowtest 全量待上板 | 905cf93 |
| Task 3 | complete | complete | — | flowtest 回归 + ENABLE 后首步核对待上板 | 113d16c |
| Task 4 | complete | complete | 六组 228/228（含 find 8 条先红后绿） | flowtest CONFIG 路径待上板 | b01dfa3 |
| Task 5 | complete | complete | vel_ramp 48/48 | [step_engine] 计时对照待上板 | 8c1bfe1 |
| Task 6 | complete | complete | — | [bringup] 寄存器核对待上板 | 754a416 |
| Task 7 | complete | complete | 六组 228/228（parse 57 为等价锚） | — | b5141cf |
| 终审修复 | complete | complete | 六组 235/235（parse +7 前缀用例） | 3c 运动态红绿待上板 | 314b205/ece2070 |

终审（fresh-context 评审）：无 Critical；Important 两条已修——flowtest 3c 原静止注入无法证伪旧实现（settling 约 1-2 ms 完成、wait_mode 粒度 5 ms），改运动态注入并重写论证；supervise_soft_limit 清零窗口竞态（预存）条件清零修复。Minor 两条经用户拍板同轮修复（tmc2209_enable 调用处前置条件说明、删除 cartfsm.c 无调用的 bsp.h include）。
