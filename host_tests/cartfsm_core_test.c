/*
 * cartfsm 纯核心 + 参数注册表 宿主测试
 * 三态转换表逐行、标定 L 合法性、软限位窗口、CONFIG 矩阵、
 * 参数范围边界 ±1
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

/* ---- 转换表逐行（三态） ---- */
static void test_transitions(void)
{
    /* DISABLED + ENABLE → READY/NONE */
    m = CM_DISABLED; r = CR_NONE;
    CHECK(step(CE_CMD_ENABLE) && m == CM_READY && r == CR_NONE,
          "DISABLED+ENABLE→READY");

    /* DISABLED 忽略其余命令/故障事件 */
    m = CM_DISABLED;
    CHECK(!step(CE_CMD_RESET), "DISABLED+RESET 忽略");
    CHECK(!step(CE_STEP_LOSS), "DISABLED+STEP_LOSS 忽略");
    CHECK(!step(CE_AS5600_FAIL), "DISABLED+AS5600 忽略");
    CHECK(!step(CE_MT6835_FAIL), "DISABLED+MT6835 忽略");

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

    /* READY 忽略 ENABLE/RESET */
    m = CM_READY;
    CHECK(!step(CE_CMD_ENABLE), "READY+ENABLE 忽略");
    CHECK(!step(CE_CMD_RESET), "READY+RESET 忽略");

    /* FAULT + RESET → DISABLED/NONE */
    m = CM_FAULT; r = CR_STEP_LOSS;
    CHECK(step(CE_CMD_RESET) && m == CM_DISABLED && r == CR_NONE,
          "FAULT+RESET→DISABLED");

    /* FAULT 忽略其余 */
    m = CM_FAULT; r = CR_NONE;
    CHECK(!step(CE_CMD_ENABLE), "FAULT+ENABLE 忽略");
    CHECK(!step(CE_STEP_LOSS), "FAULT+STEP_LOSS 忽略");
    CHECK(!step(CE_AS5600_FAIL), "FAULT+AS5600 忽略");
}

/* ---- CONFIG SET 模式矩阵 ---- */
static void test_config_matrix(void)
{
    CHECK(cm_config_set_allowed(CM_DISABLED), "DISABLED 接受 SET");
    CHECK(cm_config_set_allowed(CM_READY), "READY 接受 SET");
    CHECK(!cm_config_set_allowed(CM_FAULT), "FAULT 拒绝 SET");
}

/* ---- 标定 L 合法性（calib-spec §2：100–1000 mm，正号兼域对齐） ---- */
static void test_calib_valid(void)
{
    CHECK(!cm_calib_valid(99999), "L=99999 拒（下界−1）");
    CHECK(cm_calib_valid(100000), "L=100000 收（下界）");
    CHECK(cm_calib_valid(1000000), "L=1000000 收（上界）");
    CHECK(!cm_calib_valid(1000001), "L=1000001 拒（上界+1）");
    CHECK(!cm_calib_valid(0), "L=0 拒");
    CHECK(!cm_calib_valid(-500000), "L 负拒（域反向）");
}

/* ---- 软限位窗口钳位（calib-spec §4：向外钳 0，向内/界内不受限） ---- */
static void test_soft_limit(void)
{
    /* 下界 −250000、上界 +250000 的窗 */
    CHECK(cm_soft_limit_clamp(260000, -250000, 250000, 100) == 0,
          "越上界且向外 → 0");
    CHECK(cm_soft_limit_clamp(260000, -250000, 250000, -100) == -100,
          "越上界但向内 → 不钳");
    CHECK(cm_soft_limit_clamp(-260000, -250000, 250000, -100) == 0,
          "越下界且向外 → 0");
    CHECK(cm_soft_limit_clamp(-260000, -250000, 250000, 100) == 100,
          "越下界但向内 → 不钳");
    CHECK(cm_soft_limit_clamp(0, -250000, 250000, 100) == 100, "界内不钳");
    CHECK(cm_soft_limit_clamp(250000, -250000, 250000, 100) == 100,
          "恰在界上不钳");
    CHECK(cm_soft_limit_clamp(-250000, -250000, 250000, 0) == 0,
          "零目标不动");
    /* 运行中收紧余量（窗口收窄到 ±240000）立即生效 */
    CHECK(cm_soft_limit_clamp(250000, -240000, 240000, 100) == 0,
          "收紧后立即钳");
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
    CHECK(p->x_margin_um == 10000, "X_MARGIN_UM 默认 10000");
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

    CHECK(fw_params_set("X_MARGIN_UM", 1000)
          && fw_params_set("X_MARGIN_UM", 50000), "X_MARGIN_UM 界内");
    CHECK(!fw_params_set("X_MARGIN_UM", 999)
          && !fw_params_set("X_MARGIN_UM", 50001), "X_MARGIN_UM 界外拒绝");

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
    CHECK(fw_params_count() == 9, "九项");
    CHECK(fw_params_name(0) != NULL && fw_params_name(8) != NULL,
          "0..8 有名");
    CHECK(fw_params_name(9) == NULL, "越界 NULL");
    /* dump 顺序的 NAME 拼写 */
    const char *expect[] = {
        "VEL_MAX", "ACC_MAX", "WDT_MS", "WDT_DEC", "X_MARGIN_UM",
        "POLE_ZERO_URAD", "POLE_INV", "X_INV", "STEP_LOSS_UM",
    };
    for (int i = 0; i < 9; i++) {
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

int main(void)
{
    test_transitions();
    test_config_matrix();
    test_calib_valid();
    test_soft_limit();
    test_step_loss();
    test_params_ranges();
    test_registry_enum();
    test_fw_params_find();

    printf("cartfsm_core_test: %d/%d 断言通过\n", s_run - s_fail, s_run);
    return s_fail;
}
