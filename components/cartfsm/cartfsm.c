/*
 * cartfsm — 模式机实现：监督任务与命令执行
 *
 * 并发布局：exec 回调运行在 comm_task（CPU0）；监督任务运行在
 * CPU1。共享状态用 s_lock 自旋锁保护写入；模式枚举读取原子。
 * cartaxis / cartproto 发送均为线程安全接口。
 *
 * FAULT 停机次序（契约）：目标斜坡置 0（电机加速度临时切到
 * WDT_DEC 档）→ 监督任务等速度精确为零 → EN 关 → 推 MODE 行。
 * RESET 离开 FAULT 时恢复 ACC_MAX 档并清电机状态。
 */

#include "cartfsm.h"

#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "cartpos.h"
#include "poleang.h"
#include "cartaxis.h"
#include "cartproto.h"

static const char *TAG = "cartfsm";

#define FSM_TASK_PRIO     4
#define FSM_TASK_CORE     1
#define FSM_TASK_STACK    4096
#define FSM_PERIOD_MS     1

/* ---- 共享状态 ---- */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile cm_mode_t   s_mode = CM_DISABLED;
static volatile cm_reason_t s_reason = CR_NONE;
static volatile long s_last_cmd_v = 0;     /* 最近有效目标（软限位持续钳制） */
static volatile bool s_fault_settling = false;  /* FAULT 斜坡停机进行中 */
static bool s_started = false;

/* ---- 手动两点标定状态（RAM，断电回 CAL_NONE） ---- */
static calib_state_t s_calib = CAL_NONE;
static long s_e_left_um = 0;   /* 左标 E（µm） */
static long s_calib_l_um = 0;  /* 标定行程 L（CAL_DONE 后有效） */

/* 流程测试注入钩子（CONFIG_CARTFSM_TEST_HOOKS）：0 无、1 AS5600、2 MT6835 */
#ifdef CONFIG_CARTFSM_TEST_HOOKS
static volatile int s_test_enc_fail = 0;
static volatile long s_test_loss_bias_um = 0;   /* 丢步判定的 E 偏置注入 */
#endif

/* ---- 模式/原因字符串 ---- */
static const char *mode_str(cm_mode_t m)
{
    switch (m) {
    case CM_DISABLED: return "DISABLED";
    case CM_READY:    return "READY";
    case CM_FAULT:    return "FAULT";
    default:          return "DISABLED";
    }
}

static const char *reason_str(cm_reason_t r)
{
    switch (r) {
    case CR_NONE:      return "NONE";
    case CR_STEP_LOSS: return "STEP_LOSS";
    case CR_AS5600:    return "AS5600";
    case CR_MT6835:    return "MT6835";
    default:           return "NONE";
    }
}

/* ---- 参数作用点 ---- */

/* 软限位窗口：[-L/2+m, +L/2-m] */
static void soft_window(const fw_params_t *p, long *min_um, long *max_um)
{
    *min_um = -s_calib_l_um / 2 + p->x_margin_um;
    *max_um = s_calib_l_um / 2 - p->x_margin_um;
}

static void params_apply_engine(void)
{
    const fw_params_t *p = fw_params();
    cartaxis_set_accel_mm_s2(p->acc_max);
    cartaxis_set_decel_mm_s2(p->wdt_dec);
    cartaxis_set_wdt_ms(p->wdt_ms);
}

/* 软限位钳制收口：读当前位置、按窗口钳制 v、持锁提交 s_last_cmd_v、
 * 锁外下发电机。exec_vel 与 VEL_MAX 运行中收紧共用。
 * 电机调用在 s_lock 之外（不与电机实现内部的锁嵌套）。 */
static void clamp_and_commit_velocity(long v_mm_s)
{
    const fw_params_t *p = fw_params();
    long xmin, xmax;
    soft_window(p, &xmin, &xmax);
    long eff = cm_soft_limit_clamp((long)cartpos_position_um(),
                                   xmin, xmax, v_mm_s);

    portENTER_CRITICAL(&s_lock);
    s_last_cmd_v = eff;
    portEXIT_CRITICAL(&s_lock);

    cartaxis_set_vel_mm_s(eff);
}

/* 监督拍软限位持续钳制：当前目标被窗口钳为不同值时斜坡停并清
 * 目标。读/写 s_last_cmd_v 持锁，电机调用在锁外。 */
static void supervise_soft_limit(const fw_params_t *p)
{
    portENTER_CRITICAL(&s_lock);
    long cmd = s_last_cmd_v;
    portEXIT_CRITICAL(&s_lock);

    long xmin, xmax;
    soft_window(p, &xmin, &xmax);
    long eff = cm_soft_limit_clamp((long)cartpos_position_um(),
                                   xmin, xmax, cmd);
    if (eff != cmd) {
        /* 条件清零：仅当判定所依据的目标未被新命令覆盖时才停并
         * 清零。若在判定与清零之间 exec_vel 已提交新目标，清零会
         * 使 s_last_cmd_v 与电机实际目标脱钩（记录 0 而电机仍在
         * 运动），软限位对该目标持续失效。被覆盖则本拍不动作，
         * 下一拍以新目标为基准重新钳制。 */
        portENTER_CRITICAL(&s_lock);
        bool still_old = (s_last_cmd_v == cmd);
        if (still_old) {
            s_last_cmd_v = 0;
        }
        portEXIT_CRITICAL(&s_lock);
        if (still_old) {
            cartaxis_halt();
        }
    }
}

static void params_apply_one(const char *name)
{
    const fw_params_t *p = fw_params();
    switch (fw_params_find(name)) {
    case FP_ACC_MAX:
        cartaxis_set_accel_mm_s2(p->acc_max);
        break;
    case FP_WDT_DEC:
        cartaxis_set_decel_mm_s2(p->wdt_dec);
        break;
    case FP_WDT_MS:
        cartaxis_set_wdt_ms(p->wdt_ms);
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

/* ---- 模式转换辅助 ---- */

static void push_mode(cm_mode_t m, cm_reason_t r)
{
    cartproto_send_mode(mode_str(m), reason_str(r));
}

static void enter_fault(cm_reason_t r)
{
    portENTER_CRITICAL(&s_lock);
    s_mode = CM_FAULT;
    s_reason = r;
    s_fault_settling = true;
    portEXIT_CRITICAL(&s_lock);

    /* 次序 ①：目标斜坡置 0，以 WDT_DEC 档减速 */
    const fw_params_t *p = fw_params();
    cartaxis_set_accel_mm_s2(p->wdt_dec);
    cartaxis_halt();
    /* ②③（等 vel_now==0 → EN 关 → 推 MODE）由监督任务完成 */
}

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
    long e = (long)cartpos_position_um();
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
    long e = (long)cartpos_position_um();
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
    cartpos_set_origin_mm((float)l / 2.0f / 1000.0f);
    cartaxis_set_position_um(l / 2);
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
    cartaxis_set_position_um((long)cartpos_position_um());
    cartaxis_enable(true);
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

    cartaxis_stop();               /* 斜坡未完也立即急停 */
    cartaxis_enable(false);
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
    clamp_and_commit_velocity(v);
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

/* ---- 监督任务（1 kHz，CPU1） ---- */

static void fsm_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    bool wdt_ok = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_ok) {
        ESP_LOGW(TAG, "任务看门狗订阅失败——监督任务不受监控");
    }

    for (;;) {
        const fw_params_t *p = fw_params();
        cm_mode_t mode = s_mode;

#ifdef CONFIG_CARTFSM_TEST_HOOKS
        bool a5600_ok = (s_test_enc_fail == 1) ? false : cartpos_is_ok();
        bool m6835_ok = (s_test_enc_fail == 2) ? false : poleang_is_ok();
#else
        bool a5600_ok = cartpos_is_ok();
        bool m6835_ok = poleang_is_ok();
#endif

        if (mode == CM_READY) {
            if (!a5600_ok) {
                enter_fault(CR_AS5600);
            } else if (!m6835_ok) {
                enter_fault(CR_MT6835);
            } else {
                /* 丢步判定：µm 域 |E−C|（流程自测可注入 E 偏置）。
                 * 运行中堵转由它捕获；回零后抵挡块静止属正常——
                 * C 不前进、D 不增长，无误报 */
                long e_um = (long)cartpos_position_um();
#ifdef CONFIG_CARTFSM_TEST_HOOKS
                e_um = consume_test_bias(e_um);
#endif
                long c_um = (long)cartaxis_cmd_position_um();
                if (cm_step_loss(e_um, c_um, p->step_loss_um)) {
                    enter_fault(CR_STEP_LOSS);
                } else {
                    /* 软限位持续钳制（运行中收紧立即生效） */
                    supervise_soft_limit(p);
                }
            }
        } else if (mode == CM_FAULT && s_fault_settling) {
            /* 次序 ②③：每拍重发斜坡停——吃掉 FAULT 进入后仍在途的
             * 目标写入（exec_vel 锁外 set_vel 可晚于 enter_fault
             * 的 halt，穿透后不能只靠 WDT 兜底）→ 等零 → EN 关 →
             * 推 MODE */
            cartaxis_halt();
            if (cartaxis_at_rest()) {
                cartaxis_enable(false);
                s_fault_settling = false;
                cm_reason_t r = s_reason;
                push_mode(CM_FAULT, r);
            }
        }
        /* DISABLED：无监督动作（STATE 流照发） */

        if (wdt_ok) {
            esp_task_wdt_reset();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

/* ---- 公开 API ---- */

cm_mode_t cartfsm_mode(void)
{
    return s_mode;
}

cm_reason_t cartfsm_reason(void)
{
    return s_reason;
}

calib_state_t cartfsm_calib(void)
{
    return s_calib;
}

/* ---- 流程测试注入钩子（CONFIG_CARTFSM_TEST_HOOKS） ---- */
#ifdef CONFIG_CARTFSM_TEST_HOOKS

void cartfsm_test_exec(const cart_req_t *req)
{
    exec_cmd(req);
}

void cartfsm_test_inject_encoder_fail(int which)
{
    s_test_enc_fail = which;   /* 0 无、1 AS5600、2 MT6835 */
}

void cartfsm_test_inject_loss_bias(long bias_um)
{
    s_test_loss_bias_um = bias_um;
}

void cartfsm_test_reset_calib(void)
{
    s_calib = CAL_NONE;   /* 模拟上电：RAM 标定丢失 */
    s_e_left_um = 0;
    s_calib_l_um = 0;
}

#endif

esp_err_t cartfsm_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    /* 上电默认 DISABLED、EN 关。cartaxis_enable 的返回值仅在电机实现
     * 未初始化时非 ESP_OK，而 app_main 已先经 ESP_ERROR_CHECK(
     * motor_tmc2209_init) 完成初始化——本文件全部 EN 调用点
     * （exec_enable/exec_reset/settling）同此前提，忽略安全。 */
    cartaxis_enable(false);
    params_apply_engine();

    cartproto_register_exec(exec_cmd);

    BaseType_t ok = xTaskCreatePinnedToCore(fsm_task, "cartfsm",
                                            FSM_TASK_STACK, NULL,
                                            FSM_TASK_PRIO, NULL,
                                            FSM_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建监督任务失败");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "模式机已启动：1 kHz，优先级 %d，CPU%d，上电 DISABLED",
             FSM_TASK_PRIO, FSM_TASK_CORE);
    return ESP_OK;
}
