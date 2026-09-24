/*
 * cartfsm — 模式机外壳实现
 *
 * 并发布局：exec 回调运行在 comm_task（CPU0）；监督任务运行在
 * CPU1。共享状态用 s_lock 自旋锁保护写入；模式枚举读取原子。
 * step_engine / cartproto 发送均为线程安全接口。
 *
 * FAULT 停机次序（契约）：目标斜坡置 0（引擎加速度临时切到
 * WDT_DEC 档）→ 监督任务等 vel_now == 0 → EN 关 → 推 MODE 行。
 * RESET 离开 FAULT 时恢复 ACC_MAX 档并清引擎状态。
 */

#include "cartfsm.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "bsp.h"
#include "settings.h"
#include "tmc2209.h"
#include "step_engine.h"
#include "as5600.h"
#include "mt6835.h"
#include "cartproto.h"
#include "driver/gpio.h"

static const char *TAG = "cartfsm";

#define FSM_TASK_PRIO     4
#define FSM_TASK_CORE     1
#define FSM_TASK_STACK    4096
#define FSM_PERIOD_MS     1

/* 回零完成点（protocol.md 模式转换表：触发点记为 x = −265000 µm） */
#define HOME_ORIGIN_MM   (-265.0f)

/* DIAG 堵转有效电平（Kconfig 为 n 时 CONFIG 宏不定义，须 #ifdef 归一） */
#ifdef CONFIG_CARTFSM_DIAG_ACTIVE_HIGH
#define DIAG_ACTIVE_HIGH 1
#else
#define DIAG_ACTIVE_HIGH 0
#endif

/* ---- 共享状态 ---- */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile cm_mode_t   s_mode = CM_DISABLED;
static volatile cm_reason_t s_reason = CR_NONE;
static TickType_t s_home_start = 0;
static volatile long s_last_cmd_v = 0;     /* 最近有效目标（软限位持续钳制） */
static volatile bool s_stall_report = false;
static volatile bool s_fault_settling = false;  /* FAULT 斜坡停机进行中 */
static bool s_started = false;

/* DIAG1 堵转轮询去抖（连续高样本数；1 kHz 下 2 ms） */
#define DIAG_DEBOUNCE 2
static int s_diag_high_run = 0;
static TickType_t s_last_sg_log = 0;   /* 回零中 SG 读数日志节流 */

/* 流程自测注入（CONFIG_CARTFSM_FLOW_TEST）：0 无、1 AS5600、2 MT6835 */
#ifdef CONFIG_CARTFSM_FLOW_TEST
static volatile int s_test_enc_fail = 0;
static volatile long s_test_loss_bias_um = 0;   /* 丢步判定的 E 偏置注入 */
#endif

/* ---- 模式/原因字符串 ---- */
static const char *mode_str(cm_mode_t m)
{
    switch (m) {
    case CM_DISABLED: return "DISABLED";
    case CM_HOMING:   return "HOMING";
    case CM_READY:    return "READY";
    case CM_FAULT:    return "FAULT";
    default:          return "DISABLED";
    }
}

static const char *reason_str(cm_reason_t r)
{
    switch (r) {
    case CR_NONE:         return "NONE";
    case CR_STEP_LOSS:    return "STEP_LOSS";
    case CR_STALL:        return "STALL";
    case CR_AS5600:       return "AS5600";
    case CR_MT6835:       return "MT6835";
    case CR_HOME_TIMEOUT: return "HOME_TIMEOUT";
    default:              return "NONE";
    }
}

/* ---- 换算（任务上下文，mm 域 ↔ steps 域） ---- */

/* mm/s → Q16.16 steps/s（steps/mm 来自 settings，细分 8 下为 40） */
static int64_t mmps_to_q16(long v_mm_s)
{
    double spm = (double)settings_steps_per_mm();
    return (int64_t)llround((double)v_mm_s * spm * 65536.0);
}

/* mm/s² → steps/s² */
static uint32_t accel_to_steps(long a_mm_s2)
{
    double spm = (double)settings_steps_per_mm();
    return (uint32_t)llround((double)a_mm_s2 * spm);
}

/* 指令位置 steps → µm（与 AS5600 的 E 同域，P3 裁决） */
static long cmd_steps_to_um(int32_t steps)
{
    double spm = (double)settings_steps_per_mm();
    return (long)llround((double)steps * 1000.0 / spm);
}

/* µm → 指令位置 steps（cmd_steps_to_um 的逆；回零重定基用） */
static int32_t um_to_cmd_steps(long um)
{
    double spm = (double)settings_steps_per_mm();
    return (int32_t)llround((double)um * spm / 1000.0);
}

/* ---- 参数作用点 ---- */

static void params_apply_engine(void)
{
    const fw_params_t *p = fw_params();
    step_engine_vel_set_accel(accel_to_steps(p->acc_max));
    step_engine_vel_set_decel(accel_to_steps(p->wdt_dec));
    step_engine_vel_set_wdt_us((uint32_t)p->wdt_ms * 1000u);
    step_engine_set_dir_invert(p->x_inv != 0);
}

static void params_apply_one(const char *name)
{
    const fw_params_t *p = fw_params();
    if (strcmp(name, "ACC_MAX") == 0) {
        step_engine_vel_set_accel(accel_to_steps(p->acc_max));
    } else if (strcmp(name, "WDT_DEC") == 0) {
        step_engine_vel_set_decel(accel_to_steps(p->wdt_dec));
    } else if (strcmp(name, "WDT_MS") == 0) {
        step_engine_vel_set_wdt_us((uint32_t)p->wdt_ms * 1000u);
    } else if (strcmp(name, "X_INV") == 0) {
        step_engine_set_dir_invert(p->x_inv != 0);
    } else if (strcmp(name, "SGTHRS") == 0) {
        tmc2209_set_stall_threshold((uint8_t)p->sgthrs);
    } else if (strcmp(name, "VEL_MAX") == 0) {
        /* 运行中收紧 VEL_MAX 立即约束当前运动（protocol.md 收紧语义） */
        portENTER_CRITICAL(&s_lock);
        long cur = s_last_cmd_v;
        portEXIT_CRITICAL(&s_lock);
        if (cur > p->vel_max || cur < -p->vel_max) {
            long v = (cur > p->vel_max) ? p->vel_max
                    : (cur < -p->vel_max) ? -p->vel_max : cur;
            long eff = cm_soft_limit_clamp((long)as5600_position_um(),
                                           p->x_limit_um, v);
            s_last_cmd_v = eff;
            step_engine_vel_target(mmps_to_q16(eff));
        }
    }
    /* 其余参数（HOME_* /X_LIMIT/标定量）为读取点即时生效 */
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
    step_engine_vel_set_accel(accel_to_steps(p->wdt_dec));
    step_engine_vel_halt();
    /* ②③（等 vel_now==0 → EN 关 → 推 MODE）由监督任务完成 */
}

/* ---- 命令执行（cartproto exec，CPU0） ---- */

static void exec_cmd(const cart_req_t *req)
{
    switch (req->cmd) {
    case CART_CMD_QUERY: {
        cm_mode_t m;
        cm_reason_t r;
        portENTER_CRITICAL(&s_lock);
        m = s_mode;
        r = s_reason;
        portEXIT_CRITICAL(&s_lock);
        push_mode(m, r);
        break;
    }

    case CART_CMD_HOME: {
        cm_mode_t m = s_mode;
        cm_reason_t r = CR_NONE;
        if (!cm_step(&m, &r, CE_CMD_HOME)) {
            return;   /* 仅 DISABLED 生效，其余忽略 */
        }
        const fw_params_t *p = fw_params();
        portENTER_CRITICAL(&s_lock);
        s_mode = CM_HOMING;
        s_reason = CR_NONE;
        s_home_start = xTaskGetTickCount();
        portEXIT_CRITICAL(&s_lock);

        /* 动作：EN 开 + 堵转阈值 + 以 −HOME_VEL 起步 */
        bsp_tmc_en_set(true);
        tmc2209_set_stall_threshold((uint8_t)p->sgthrs);
        step_engine_vel_set_accel(accel_to_steps(p->acc_max));
        long hv = -(long)p->home_vel;
        s_last_cmd_v = hv;
        step_engine_vel_target(mmps_to_q16(hv));
        push_mode(CM_HOMING, CR_NONE);
        break;
    }

    case CART_CMD_RESET: {
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
        break;
    }

    case CART_CMD_VEL: {
        portENTER_CRITICAL(&s_lock);
        cm_mode_t cur = s_mode;
        portEXIT_CRITICAL(&s_lock);
        if (cur != CM_READY) {
            return;   /* 仅 READY 执行；持锁复查防 FAULT 进入瞬间的在途命令 */
        }
        const fw_params_t *p = fw_params();
        long v = req->v_mm_s;
        if (v > p->vel_max) {
            v = p->vel_max;
        } else if (v < -p->vel_max) {
            v = -p->vel_max;
        }
        long eff = cm_soft_limit_clamp((long)as5600_position_um(),
                                       p->x_limit_um, v);
        s_last_cmd_v = eff;
        step_engine_vel_target(mmps_to_q16(eff));
        break;
    }

    case CART_CMD_CONFIG_SET: {
        if (!cm_config_set_allowed(s_mode)) {
            cartproto_send_err(req->name);   /* HOMING/FAULT 拒绝 */
            return;
        }
        if (fw_params_set(req->name, req->value)) {
            params_apply_one(req->name);
            cartproto_send_ok(req->name, req->value);   /* 回显归一化值 */
        } else {
            cartproto_send_err(req->name);
        }
        break;
    }

    case CART_CMD_CONFIG_DUMP: {
        for (int i = 0; i < fw_params_count(); i++) {
            cartproto_send_ok(fw_params_name(i), fw_params_value(i));
        }
        break;
    }

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

        /* DIAG1 堵转轮询：仅在使能的运动模式（HOMING/READY）且真实
         * 运动中采样——StallGuard 静止时读数无意义（SG_RESULT 恒低），
         * 静止判定会在起步前即报警（实测：HOMING 后 2 ms 即"完成"，
         * 随后 READY 存活 2 ms 即 FAULT STALL）。HOMING 限达到回零
         * 速度 80% 后参与（加速斜坡早期同样不可信）。极性按 Kconfig，
         * 2 ms 去抖；注入报告（自测）与本轮询并联。 */
        bool diag_stall = false;
        if (mode == CM_HOMING || mode == CM_READY) {
            int64_t vnow = step_engine_vel_now_q16();
            uint64_t mag = (vnow < 0) ? (uint64_t)(-vnow) : (uint64_t)vnow;
            bool moving = (mode == CM_READY)
                ? (mag > 0)
                : (mag * 5 >= (uint64_t)mmps_to_q16((long)p->home_vel) * 4);
            if (moving) {
                int lvl = gpio_get_level(CONFIG_BSP_DIAG_GPIO);
                bool high = DIAG_ACTIVE_HIGH ? (lvl == 1) : (lvl == 0);
                s_diag_high_run = high ? s_diag_high_run + 1 : 0;
                if (s_diag_high_run >= DIAG_DEBOUNCE) {
                    diag_stall = true;
                }
            } else {
                s_diag_high_run = 0;
            }
        } else {
            s_diag_high_run = 0;
        }

        bool stalled = s_stall_report || diag_stall;
        s_stall_report = false;

#ifdef CONFIG_CARTFSM_FLOW_TEST
        bool a5600_ok = (s_test_enc_fail == 1) ? false : as5600_is_ok();
        bool m6835_ok = (s_test_enc_fail == 2) ? false : mt6835_is_ok();
#else
        bool a5600_ok = as5600_is_ok();
        bool m6835_ok = mt6835_is_ok();
#endif

        if (mode == CM_HOMING) {
            /* 回零速度按拍续喂：引擎看门狗按命令流设计（WDT_MS 窗口
             * 断流即减速），HOME 是单条命令，不续喂则 100 ms 后运动
             * 停止、HOMING 干等超时。回零是固件自主运动，由监督任务
             * 承担命令流角色；总时长上限仍由 HOME_TIMEOUT 把关。
             * 持锁复查：与 exec 的 HOMING 中止（RESET→DISABLED）
             * 交错时不得再喂，否则中止后引擎被重新起步 */
            portENTER_CRITICAL(&s_lock);
            if (s_mode == CM_HOMING) {
                step_engine_vel_target(mmps_to_q16(-(long)p->home_vel));
            }
            portEXIT_CRITICAL(&s_lock);

            /* C2 标定辅助：回零途中周期读 SG_RESULT/TSTEP——空载读数
             * 决定 SGTHRS 上限（须 2×SGTHRS+1 ≤ 空载读数），堵转读数
             * （≈0）决定下限 */
            if (xTaskGetTickCount() - s_last_sg_log >= pdMS_TO_TICKS(500)) {
                s_last_sg_log = xTaskGetTickCount();
                ESP_LOGI(TAG, "回零中：SG_RESULT=%d，TSTEP=%d",
                         tmc2209_sg_result(), tmc2209_tstep());
            }

            cm_mode_t m = mode;
            cm_reason_t r = s_reason;
            cm_event_t ev = CE_STALL;

            if (!stalled) {
                if (!a5600_ok) {
                    ev = CE_AS5600_FAIL;
                } else if (!m6835_ok) {
                    ev = CE_MT6835_FAIL;
                } else if ((xTaskGetTickCount() - s_home_start)
                           >= pdMS_TO_TICKS(p->home_timeout_ms)) {
                    ev = CE_HOME_TIMEOUT;
                } else {
                    ev = (cm_event_t)0;   /* 本拍无事件 */
                }
            }

            if (ev != 0 && cm_step(&m, &r, ev)) {
                if (m == CM_READY) {
                    /* 堵转 = 完成：急停 + E/C 同帧重定基到 HOME 原点
                     *（E、C 必须同时换到新坐标系，否则 READY 首拍
                     *   |E−C| 丢步判定立即误报 FAULT/STEP_LOSS） */
                    step_engine_stop();
                    as5600_set_origin(HOME_ORIGIN_MM);
                    step_engine_set_position(
                        um_to_cmd_steps((long)(HOME_ORIGIN_MM * 1000.0)));
                    portENTER_CRITICAL(&s_lock);
                    s_mode = CM_READY;
                    s_reason = CR_NONE;
                    s_last_cmd_v = 0;
                    portEXIT_CRITICAL(&s_lock);
                    push_mode(CM_READY, CR_NONE);
                } else {
                    if (r == CR_HOME_TIMEOUT) {
                        /* 超时诊断线索：DIAG 原始电平与活跃电平连续
                         * 拍数——堵转门限未开/接线断/极性反的区分依据
                         *（TMC2209 DIAG 堵转为高；持续撞墙期间电平
                         * 恒低=信号未产生，恒高而计数 0=极性配置反） */
                        /* SG_RESULT：0=堵转/满载、~510=空载——撞墙中
                         * 仍高=芯片未感知负载（提 SGTHRS/查电流），
                         * 低但 DIAG 恒 0=DIAG 线断（查 GPIO16 接线） */
                        ESP_LOGW(TAG, "回零超时：DIAG 电平=%d，活跃电平"
                                 "连续 %u 拍（极性=%s），SG_RESULT=%d，"
                                 "TSTEP=%d",
                                 gpio_get_level(CONFIG_BSP_DIAG_GPIO),
                                 s_diag_high_run,
                                 DIAG_ACTIVE_HIGH
                                     ? "高有效" : "低有效",
                                 tmc2209_sg_result(), tmc2209_tstep());
                    }
                    enter_fault(r);
                }
            }
        } else if (mode == CM_READY) {
            /* 堵转仅在运动中判定：StallGuard 需要运动，静止读数无
             * 意义——回零完成后小车抵着挡块由保持电流顶着，DIAG 持续
             * 有效，若静止也判故障则 READY 存活 ~2 ms 即误报（实测） */
            bool moving = step_engine_vel_now_q16() != 0;
            if (stalled && moving) {
                enter_fault(CR_STALL);
            } else if (!a5600_ok) {
                enter_fault(CR_AS5600);
            } else if (!m6835_ok) {
                enter_fault(CR_MT6835);
            } else {
                /* 丢步判定：µm 域 |E−C|（流程自测可注入 E 偏置） */
                long e_um = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_FLOW_TEST
                e_um += s_test_loss_bias_um;
#endif
                long c_um = cmd_steps_to_um(step_engine_position_steps());
                if (cm_step_loss(e_um, c_um, p->step_loss_um)) {
                    enter_fault(CR_STEP_LOSS);
                } else {
                    /* 软限位持续钳制（运行中收紧立即生效） */
                    long eff = cm_soft_limit_clamp(
                        (long)as5600_position_um(), p->x_limit_um,
                        s_last_cmd_v);
                    if (eff != s_last_cmd_v) {
                        step_engine_vel_halt();
                        s_last_cmd_v = 0;
                    }
                }
            }
        } else if (mode == CM_FAULT && s_fault_settling) {
            /* 次序 ②③：等零 → EN 关 → 推 MODE */
            if (step_engine_vel_now_q16() == 0) {
                bsp_tmc_en_set(false);
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

void cartfsm_report_stall(bool stalled)
{
    s_stall_report = stalled;
}

/* ---- 流程自测钩子（CONFIG_CARTFSM_FLOW_TEST，上板注入用） ---- */
#ifdef CONFIG_CARTFSM_FLOW_TEST

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

#endif

esp_err_t cartfsm_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    /* 上电默认 DISABLED、EN 关 */
    bsp_tmc_en_set(false);
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
