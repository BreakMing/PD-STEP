/*
 * cartaxis 细分单一写者状态机宿主测试 — 评审 Critical-1 的回归锚
 *
 * 覆盖规格 cartaxis 决策表三条路径 + 每转整步数/每转行程即时生效：
 *  A 上电：applied 档建立，cmd = steps × applied 派生 spm 换算
 *  B 静止即时应用：µm 保持、步数按新 spm 重定基（settings 先提交
 *    后通知——回调里 settings 视图已新，旧档步数不得被新系数污染）
 *  C 运动中挂起 → 斜坡完成由入口应用：挂起期间读数保持旧档换算
 *  D 写失败：芯片档与换算一并保持旧值（读数自洽），重设才恢复
 *  E 每转行程变更：仅换算系数即时变化，不推细分
 *
 * 编译：gcc -std=c11 -Wall -Wextra \
 *   -Ihost_tests/stub -Icomponents/cartaxis/include \
 *   -Icomponents/motor/include -Icomponents/settings/include \
 *   -Icomponents/cartparams/include \
 *   host_tests/cartaxis_test.c components/cartaxis/cartaxis.c \
 *   components/cartaxis/cartaxis_conv.c -lm
 * 退出码 = 失败断言数（0 = 全过）。
 */
#include <stdio.h>
#include "cartaxis.h"
#include "settings.h"
#include "params.h"

/* ================= settings 桩 ================= */

static settings_change_cb_t s_cb;
static float s_vals[5];

bool settings_subscribe(settings_change_cb_t cb)
{
    s_cb = cb;
    return true;
}

int settings_microstep(void)    { return (int)s_vals[SET_MICROSTEP]; }
int settings_full_steps(void)   { return (int)s_vals[SET_FULL_STEP]; }
float settings_units_per_rev(void) { return s_vals[SET_MM_PER_REV]; }

/* 复刻真实语义（settings.c：先提交 s_values[idx] 再通知） */
static void stub_settings_set(int idx, float v)
{
    s_vals[idx] = v;
    if (s_cb) {
        s_cb(idx, v);
    }
}

/* ================= cartparams 桩 ================= */

static fw_params_t s_params;

const fw_params_t *fw_params(void)
{
    return &s_params;
}

/* ================= motor 桩 ================= */

static int32_t g_pos_steps;          /* 指令位置（芯片档微步） */
static bool    g_at_rest = true;
static int     g_chip_ms = -1;       /* 芯片实际细分档（-1 = 未写过） */
static int     g_write_fail = 0;     /* 非 0 → set_microstep 失败 */
static int     g_halt_calls = 0;

static void stub_settle(void) { g_at_rest = true; }   /* 模拟斜坡完成 */

esp_err_t motor_enable(motor_handle_t m, bool en)
{
    (void)m; (void)en;
    return ESP_OK;
}

void motor_set_vel(motor_handle_t m, long steps_per_s)
{
    (void)m; (void)steps_per_s;
}

void motor_halt(motor_handle_t m)
{
    /* 桩：halt 只下发目标清 0，静止由 stub_settle 模拟斜坡完成 */
    (void)m;
    g_halt_calls++;
}

void motor_stop(motor_handle_t m) { (void)m; }
bool motor_at_rest(motor_handle_t m) { (void)m; return g_at_rest; }
int32_t motor_position_steps(motor_handle_t m) { (void)m; return g_pos_steps; }

void motor_set_position(motor_handle_t m, int32_t steps)
{
    (void)m;
    g_pos_steps = steps;
}

void motor_set_accel(motor_handle_t m, uint32_t a) { (void)m; (void)a; }
void motor_set_decel(motor_handle_t m, uint32_t d) { (void)m; (void)d; }
void motor_set_wdt_ms(motor_handle_t m, uint32_t w) { (void)m; (void)w; }

esp_err_t motor_set_microstep(motor_handle_t m, int microstep)
{
    (void)m;
    if (g_write_fail) {
        return ESP_FAIL;
    }
    g_chip_ms = microstep;
    return ESP_OK;
}

esp_err_t motor_set_current(motor_handle_t m, int run_ma, int hold_ma)
{
    (void)m; (void)run_ma; (void)hold_ma;
    return ESP_OK;
}

/* ================= 断言 ================= */

static int s_fail = 0;
static int s_run = 0;

#define CHECK_EQ(expr, expect) do { \
    long long _e = (long long)(expect); \
    long long _a = (long long)(expr); \
    s_run++; \
    if (_e != _a) { \
        s_fail++; \
        printf("FAIL %s:%d  %s = %lld, 期望 %lld\n", \
               __func__, __LINE__, #expr, _a, _e); \
    } \
} while (0)

int main(void)
{
    /* ---- A 上电：细分=8、每转整步数=200、每转行程=40 → applied=8、spm=40 ---- */
    s_vals[SET_MICROSTEP] = 8;
    s_vals[SET_FULL_STEP] = 200;
    s_vals[SET_MM_PER_REV] = 40.0f;
    s_params.x_inv = 0;
    g_pos_steps = 1600;   /* 1600 步 × 1/40 步每 µm = 40.000 mm */

    cartaxis_init((motor_handle_t)1);
    CHECK_EQ(g_chip_ms, -1);                       /* init 不推细分（tmc2209 config 已推） */
    CHECK_EQ(cartaxis_cmd_position_um(), 40000);   /* 1600 × 1000 / 40 */

    /* ---- B 静止即时应用：细分 8→16（spm 40→80），µm 保持 ---- */
    stub_settings_set(SET_MICROSTEP, 16);
    CHECK_EQ(g_chip_ms, 16);                       /* 新档已写入芯片 */
    CHECK_EQ(g_pos_steps, 3200);                   /* 40000 µm × 80 / 1000 */
    CHECK_EQ(cartaxis_cmd_position_um(), 40000);   /* µm 不变 */

    /* ---- C 运动中挂起 → 斜坡完成由入口应用：细分 16→32（spm 80→160） ---- */
    g_at_rest = false;
    stub_settings_set(SET_MICROSTEP, 32);
    CHECK_EQ(g_halt_calls, 1);                     /* 回调已下发斜坡停 */
    CHECK_EQ(g_chip_ms, 16);                       /* 运动中不推档 */
    CHECK_EQ(cartaxis_cmd_position_um(), 40000);   /* 挂起期旧档换算，无瞬态 */
    CHECK_EQ(g_chip_ms, 16);                       /* 读数入口不误推 */

    stub_settle();                                 /* 斜坡完成 → 静止 */
    CHECK_EQ(cartaxis_cmd_position_um(), 40000);   /* 入口应用后 µm 仍不变 */
    CHECK_EQ(g_chip_ms, 32);
    CHECK_EQ(g_pos_steps, 6400);                   /* 40000 µm × 160 / 1000 */

    /* ---- D 写失败：芯片档与换算一并保持旧值 ---- */
    g_write_fail = 1;
    stub_settings_set(SET_MICROSTEP, 8);
    CHECK_EQ(g_chip_ms, 32);                       /* 芯片保持旧档 */
    CHECK_EQ(g_pos_steps, 6400);                   /* 位置不重写 */
    CHECK_EQ(cartaxis_cmd_position_um(), 40000);   /* 旧档换算读数自洽 */

    /* ---- D2 写失败后重设细分恢复（附录 C 恢复语义锚定） ---- */
    g_write_fail = 0;
    stub_settings_set(SET_MICROSTEP, 8);
    CHECK_EQ(g_chip_ms, 8);                        /* 重设即走正常应用路径 */
    CHECK_EQ(g_pos_steps, 1600);                   /* 40000 µm × 40 / 1000 */
    CHECK_EQ(cartaxis_cmd_position_um(), 40000);   /* µm 仍不变 */

    /* ---- E 每转行程变更：仅系数即时变化（spm 40→80），不推细分 ---- */
    stub_settings_set(SET_MM_PER_REV, 20.0f);
    CHECK_EQ(g_chip_ms, 8);                        /* 不推细分 */
    CHECK_EQ(cartaxis_cmd_position_um(), 20000);   /* 1600 × 1000 / 80：行程定义减半 */

    printf("%s: %d 用例, %d 失败\n", s_fail ? "FAIL" : "PASS", s_run, s_fail);
    return s_fail;
}
