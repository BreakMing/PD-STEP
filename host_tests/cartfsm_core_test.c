/*
 * cartfsm 纯核心 + 参数注册表 宿主测试
 * 转换表逐行、CONFIG 矩阵、参数范围边界 ±1、软限位（Review Focus 3/5）
 *
 * 退出码 = 失败断言数（0 = 全过）。
 */

#include <stdio.h>
#include "cartfsm_core.h"
#include "params.h"

static int s_fail = 0;
static int s_run = 0;

#define CHECK(cond, msg) do { \
    s_run++; \
    if (!(cond)) { \
        s_fail++; \
        printf("FAIL %s:%d  %s\n", __func__, __LINE__, msg); \
    } \
} while (0)

/* 便捷：断言一次事件后的模式与原因（mode/ref 传 -1 表示断言不变） */
static cm_mode_t m;
static cm_reason_t r;

static bool step(cm_event_t ev)
{
    return cm_step(&m, &r, ev);
}

/* ---- 转换表逐行 ---- */
static void test_transitions(void)
{
    /* DISABLED + HOME → HOMING/NONE */
    m = CM_DISABLED; r = CR_NONE;
    CHECK(step(CE_CMD_HOME) && m == CM_HOMING && r == CR_NONE, "DISABLED+HOME→HOMING");

    /* DISABLED 忽略其余命令事件 */
    m = CM_DISABLED;
    CHECK(!step(CE_CMD_RESET), "DISABLED+RESET 忽略");
    CHECK(!step(CE_HOME_DONE), "DISABLED+HOME_DONE 忽略");
    CHECK(!step(CE_STEP_LOSS), "DISABLED+STEP_LOSS 忽略");
    CHECK(!step(CE_HOME_TIMEOUT), "DISABLED+TIMEOUT 忽略");
    CHECK(!step(CE_AS5600_FAIL), "DISABLED+AS5600 忽略");

    /* HOMING + 到位（位置偏差判据事件）→ READY/NONE */
    m = CM_HOMING; r = CR_NONE;
    CHECK(step(CE_HOME_DONE) && m == CM_READY && r == CR_NONE,
          "HOMING+HOME_DONE→READY");

    /* HOMING + 超时 → FAULT/HOME_TIMEOUT */
    m = CM_HOMING;
    CHECK(step(CE_HOME_TIMEOUT) && m == CM_FAULT && r == CR_HOME_TIMEOUT,
          "HOMING+TIMEOUT→FAULT/HOME_TIMEOUT");

    /* HOMING + 编码器失败 → FAULT */
    m = CM_HOMING;
    CHECK(step(CE_AS5600_FAIL) && m == CM_FAULT && r == CR_AS5600,
          "HOMING+AS5600→FAULT/AS5600");
    m = CM_HOMING;
    CHECK(step(CE_MT6835_FAIL) && m == CM_FAULT && r == CR_MT6835,
          "HOMING+MT6835→FAULT/MT6835");

    /* HOMING + RESET → DISABLED（回零中止逃生口） */
    m = CM_HOMING;
    CHECK(step(CE_CMD_RESET) && m == CM_DISABLED && r == CR_NONE,
          "HOMING+RESET→DISABLED");

    /* HOMING 忽略 HOME；未达速偏差 → FAULT/STEP_LOSS（决策表序 3） */
    m = CM_HOMING;
    CHECK(!step(CE_CMD_HOME), "HOMING+HOME 忽略");
    m = CM_HOMING; r = CR_NONE;
    CHECK(step(CE_STEP_LOSS) && m == CM_FAULT && r == CR_STEP_LOSS,
          "HOMING+STEP_LOSS→FAULT（未达速偏差）");

    /* READY 故障三源 */
    m = CM_READY;
    CHECK(step(CE_STEP_LOSS) && m == CM_FAULT && r == CR_STEP_LOSS,
          "READY+STEP_LOSS→FAULT");
    m = CM_READY;
    CHECK(step(CE_AS5600_FAIL) && m == CM_FAULT && r == CR_AS5600,
          "READY+AS5600→FAULT");
    m = CM_READY;
    CHECK(step(CE_MT6835_FAIL) && m == CM_FAULT && r == CR_MT6835,
          "READY+MT6835→FAULT");

    /* READY 忽略 HOME/RESET */
    m = CM_READY;
    CHECK(!step(CE_CMD_HOME), "READY+HOME 忽略");
    CHECK(!step(CE_CMD_RESET), "READY+RESET 忽略");
    CHECK(!step(CE_HOME_TIMEOUT), "READY+TIMEOUT 忽略");
    CHECK(!step(CE_HOME_DONE), "READY+HOME_DONE 忽略");

    /* FAULT + RESET → DISABLED/NONE */
    m = CM_FAULT; r = CR_STEP_LOSS;
    CHECK(step(CE_CMD_RESET) && m == CM_DISABLED && r == CR_NONE, "FAULT+RESET→DISABLED");

    /* FAULT 忽略其余 */
    m = CM_FAULT; r = CR_NONE;
    CHECK(!step(CE_CMD_HOME), "FAULT+HOME 忽略");
    CHECK(!step(CE_HOME_DONE), "FAULT+HOME_DONE 忽略");
    CHECK(!step(CE_AS5600_FAIL), "FAULT+AS5600 忽略");
    CHECK(!step(CE_HOME_TIMEOUT), "FAULT+TIMEOUT 忽略");
    CHECK(!step(CE_STEP_LOSS), "FAULT+STEP_LOSS 忽略");
}

/* ---- CONFIG SET 模式矩阵 ---- */
static void test_config_matrix(void)
{
    CHECK(cm_config_set_allowed(CM_DISABLED), "DISABLED 接受 SET");
    CHECK(cm_config_set_allowed(CM_READY), "READY 接受 SET");
    CHECK(!cm_config_set_allowed(CM_HOMING), "HOMING 拒绝 SET");
    CHECK(!cm_config_set_allowed(CM_FAULT), "FAULT 拒绝 SET");
}

/* ---- 软限位（Review Focus 3：回程方向放行） ---- */
static void test_soft_limit(void)
{
    CHECK(cm_soft_limit_clamp(-270000, 260000, -100) == 0,
          "越下界且向外 → 0");
    CHECK(cm_soft_limit_clamp(-270000, 260000, 100) == 100,
          "越下界但向内 → 不钳");
    CHECK(cm_soft_limit_clamp(270000, 260000, 100) == 0,
          "越上界且向外 → 0");
    CHECK(cm_soft_limit_clamp(270000, 260000, -100) == -100,
          "越上界但向内 → 不钳");
    CHECK(cm_soft_limit_clamp(0, 260000, 100) == 100, "界内不钳");
    CHECK(cm_soft_limit_clamp(260000, 260000, 100) == 100, "恰在界上不钳");
    CHECK(cm_soft_limit_clamp(-265000, 260000, 0) == 0, "零目标不动");
    /* 运行中收紧（Review Focus 5）：限位从 265000 收到 10000 */
    CHECK(cm_soft_limit_clamp(-270000, 10000, -100) == 0, "收紧后立即钳");
}

/* ---- 丢步判定 ---- */
static void test_step_loss(void)
{
    CHECK(!cm_step_loss(0, 0, 3000), "零偏差");
    CHECK(!cm_step_loss(3000, 0, 3000), "恰在阈值（不超过）");
    CHECK(cm_step_loss(3001, 0, 3000), "超 1 µm 判丢步");
    CHECK(cm_step_loss(-3001, 0, 3000), "负向超限判丢步");
    CHECK(!cm_step_loss(200000, 203000, 3000), "混合符号恰在阈值");
    CHECK(cm_step_loss(200000, 203001, 3000), "混合符号超 1 µm");
}

/* ---- 参数注册表：默认值 + 范围边界 ±1 ---- */
static void test_params_ranges(void)
{
    const fw_params_t *p = fw_params();
    CHECK(p->vel_max == 1000, "VEL_MAX 默认 1000");
    CHECK(p->acc_max == 5000, "ACC_MAX 默认 5000");
    CHECK(p->wdt_ms == 100, "WDT_MS 默认 100");
    CHECK(p->wdt_dec == 5000, "WDT_DEC 默认 5000");
    CHECK(p->home_vel == 50, "HOME_VEL 默认 50");
    CHECK(p->home_timeout_ms == 30000, "HOME_TIMEOUT 默认 30000");
    CHECK(p->x_limit_um == 260000, "X_LIMIT 默认 260000");
    CHECK(p->pole_zero_urad == 0 && p->pole_inv == 0 && p->x_inv == 0,
          "标定项默认 0");
    CHECK(p->step_loss_um == 3000, "STEP_LOSS_UM 默认 3000");

    /* 逐项边界 ±1 */
    CHECK(fw_params_set("VEL_MAX", 1), "VEL_MAX 下界 1");
    CHECK(!fw_params_set("VEL_MAX", 0), "VEL_MAX 0 拒绝");
    CHECK(fw_params_set("VEL_MAX", 2000), "VEL_MAX 上界 2000");
    CHECK(!fw_params_set("VEL_MAX", 2001), "VEL_MAX 2001 拒绝");

    CHECK(fw_params_set("ACC_MAX", 100) && fw_params_set("ACC_MAX", 30000),
          "ACC_MAX 界内");
    CHECK(!fw_params_set("ACC_MAX", 99) && !fw_params_set("ACC_MAX", 30001),
          "ACC_MAX 界外拒绝");

    CHECK(fw_params_set("WDT_MS", 10) && fw_params_set("WDT_MS", 1000),
          "WDT_MS 界内");
    CHECK(!fw_params_set("WDT_MS", 9) && !fw_params_set("WDT_MS", 1001),
          "WDT_MS 界外拒绝");

    CHECK(fw_params_set("WDT_DEC", 100) && fw_params_set("WDT_DEC", 30000),
          "WDT_DEC 界内");
    CHECK(!fw_params_set("WDT_DEC", 99), "WDT_DEC 界外拒绝");

    CHECK(fw_params_set("HOME_VEL", 5) && fw_params_set("HOME_VEL", 200),
          "HOME_VEL 界内");
    CHECK(!fw_params_set("HOME_VEL", 4) && !fw_params_set("HOME_VEL", 201),
          "HOME_VEL 界外拒绝");

    CHECK(fw_params_set("HOME_TIMEOUT", 1000)
          && fw_params_set("HOME_TIMEOUT", 120000), "HOME_TIMEOUT 界内");
    CHECK(!fw_params_set("HOME_TIMEOUT", 999)
          && !fw_params_set("HOME_TIMEOUT", 120001), "HOME_TIMEOUT 界外拒绝");

    CHECK(fw_params_set("X_LIMIT", 10000) && fw_params_set("X_LIMIT", 265000),
          "X_LIMIT 界内");
    CHECK(!fw_params_set("X_LIMIT", 9999) && !fw_params_set("X_LIMIT", 265001),
          "X_LIMIT 界外拒绝");

    CHECK(fw_params_set("POLE_ZERO_URAD", -3141592)
          && fw_params_set("POLE_ZERO_URAD", 3141592), "POLE_ZERO 界内");
    CHECK(!fw_params_set("POLE_ZERO_URAD", -3141593)
          && !fw_params_set("POLE_ZERO_URAD", 3141593), "POLE_ZERO 界外拒绝");

    CHECK(fw_params_set("POLE_INV", 0) && fw_params_set("POLE_INV", 1),
          "POLE_INV 界内");
    CHECK(!fw_params_set("POLE_INV", 2) && !fw_params_set("POLE_INV", -1),
          "POLE_INV 界外拒绝");

    CHECK(fw_params_set("X_INV", 1), "X_INV 界内");
    CHECK(!fw_params_set("X_INV", 5), "X_INV 界外拒绝");

    CHECK(fw_params_set("STEP_LOSS_UM", 500)
          && fw_params_set("STEP_LOSS_UM", 20000), "STEP_LOSS_UM 界内");
    CHECK(!fw_params_set("STEP_LOSS_UM", 499)
          && !fw_params_set("STEP_LOSS_UM", 20001), "STEP_LOSS_UM 界外拒绝");

    CHECK(!fw_params_set("NO_SUCH", 1), "未知 NAME 拒绝");
    CHECK(!fw_params_set("", 1), "空 NAME 拒绝");

    /* set→get 回读（上一组边界设置后） */
    CHECK(fw_params()->step_loss_um == 20000, "set 后回读");
}

/* ---- 注册表枚举一致性 ---- */
static void test_registry_enum(void)
{
    CHECK(fw_params_count() == 11, "十一项");
    CHECK(fw_params_name(0) != NULL && fw_params_name(10) != NULL,
          "0..10 有名");
    CHECK(fw_params_name(11) == NULL, "越界 NULL");
    /* dump 顺序的 NAME 拼写 */
    const char *expect[] = {
        "VEL_MAX", "ACC_MAX", "WDT_MS", "WDT_DEC", "HOME_VEL", "HOME_TIMEOUT",
        "X_LIMIT", "POLE_ZERO_URAD", "POLE_INV", "X_INV",
        "STEP_LOSS_UM",
    };
    for (int i = 0; i < 11; i++) {
        int ok = fw_params_name(i) != NULL;
        if (ok) {
            const char *a = fw_params_name(i);
            const char *b = expect[i];
            while (*a && *b && *a == *b) { a++; b++; }
            ok = (*a == '\0' && *b == '\0');
        }
        if (!ok) {
            printf("FAIL registry name[%d]\n", i);
            s_fail++;
        }
        s_run++;
    }
}

/* ---- HOMING 到位决策表（homing-spec §2 优先序） ---- */
static void test_homing_event(void)
{
    cm_homing_in_t in = {
        .a5600_ok = true, .m6835_ok = true, .speed_latch = true,
        .e_um = 0, .c_um = 0, .step_loss_um = 3000, .timed_out = false,
    };
    /* 序 1：AS5600 失败最优先——E 无效不得参与任何判定 */
    in.a5600_ok = false; in.e_um = 999999;
    CHECK(cm_homing_event(&in) == CE_AS5600_FAIL, "AS5600 失败最优先");
    in.a5600_ok = true;
    /* 序 2：MT6835 失败次之 */
    in.m6835_ok = false;
    CHECK(cm_homing_event(&in) == CE_MT6835_FAIL, "MT6835 失败次之");
    in.m6835_ok = true;
    /* 序 3：未达速偏差 → STEP_LOSS（起点压挡块/卡死/域反向） */
    in.speed_latch = false; in.e_um = 4000;
    CHECK(cm_homing_event(&in) == CE_STEP_LOSS, "未达速偏差→STEP_LOSS");
    /* 序 4：达速偏差 → 到位 */
    in.speed_latch = true;
    CHECK(cm_homing_event(&in) == CE_HOME_DONE, "达速偏差→到位");
    /* 序 4 优先于序 5：同拍超时也判到位 */
    in.timed_out = true;
    CHECK(cm_homing_event(&in) == CE_HOME_DONE, "到位优先于超时");
    /* 序 5：无偏差时超时（不论锁存） */
    in.e_um = 0; in.speed_latch = false;
    CHECK(cm_homing_event(&in) == CE_HOME_TIMEOUT, "未达速无偏差→超时");
    in.speed_latch = true;
    CHECK(cm_homing_event(&in) == CE_HOME_TIMEOUT, "达速无偏差→超时");
    /* 无事件与 NULL 安全 */
    in.timed_out = false;
    CHECK(cm_homing_event(&in) == 0, "常态无事件");
    CHECK(cm_homing_event(NULL) == 0, "NULL 无事件");
}

/* ---- 达速判据：含符号、0.8×HOME_VEL 门限 ---- */
static void test_speed_reached(void)
{
    CHECK(cm_speed_reached(-40, 50), "−40 达 −0.8×50 门限");
    CHECK(!cm_speed_reached(-39, 50), "−39 未达门限");
    CHECK(cm_speed_reached(-50, 50), "额定速度达标");
    CHECK(!cm_speed_reached(40, 50), "正方向不算（域反向保护）");
    CHECK(!cm_speed_reached(0, 50), "静止不算");
    CHECK(!cm_speed_reached(-40, 0), "HOME_VEL=0 非法输入不算");
    CHECK(!cm_speed_reached(-40, -50), "HOME_VEL 负值不算");
    CHECK(cm_speed_reached(-4, 5), "最小 HOME_VEL=5 的门限 −4");
}

int main(void)
{
    test_transitions();
    test_config_matrix();
    test_soft_limit();
    test_step_loss();
    test_homing_event();
    test_speed_reached();
    test_params_ranges();
    test_registry_enum();

    printf("cartfsm_core_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
