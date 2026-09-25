/*
 * cartfsm — 上板自测（核心向量与宿主测试同源，不驱动电机）
 *
 * CONFIG_CARTFSM_SELF_TEST=y 时编入，cartfsm_start 之后运行。
 * 流程级注入用例（假到位/时钟）在任务 8 的上板验收中人工执行。
 */

#include "cartfsm.h"

#include "esp_log.h"

static const char *TAG = "cartfsm_selftest";

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

static bool test_transitions_spot(void)
{
    cm_mode_t m = CM_DISABLED;
    cm_reason_t r = CR_NONE;
    CHECK(cm_step(&m, &r, CE_CMD_HOME) && m == CM_HOMING, "HOME→HOMING");
    CHECK(cm_step(&m, &r, CE_HOME_DONE) && m == CM_READY && r == CR_NONE,
          "到位完成→READY");
    CHECK(cm_step(&m, &r, CE_STEP_LOSS) && m == CM_FAULT && r == CR_STEP_LOSS,
          "丢步→FAULT");
    CHECK(cm_step(&m, &r, CE_CMD_RESET) && m == CM_DISABLED, "RESET→DISABLED");
    CHECK(cm_step(&m, &r, CE_CMD_HOME) && m == CM_HOMING, "HOME→HOMING");
    CHECK(cm_step(&m, &r, CE_CMD_RESET) && m == CM_DISABLED,
          "HOMING+RESET→DISABLED（中止逃生口）");
    return true;
}

static bool test_soft_limit_spot(void)
{
    CHECK(cm_soft_limit_clamp(-270000, 260000, -100) == 0, "向外钳 0");
    CHECK(cm_soft_limit_clamp(-270000, 260000, 100) == 100, "向内放行");
    return true;
}

static bool test_config_matrix_spot(void)
{
    CHECK(cm_config_set_allowed(CM_READY), "READY 接受");
    CHECK(!cm_config_set_allowed(CM_HOMING), "HOMING 拒绝");
    CHECK(fw_params_set("X_LIMIT", 10000), "收紧 X_LIMIT 合法");
    CHECK(!fw_params_set("X_LIMIT", 9999), "越界拒绝");
    CHECK(fw_params_set("X_LIMIT", 260000), "恢复默认 X_LIMIT");
    CHECK(fw_params()->x_limit_um == 260000, "恢复生效");
    return true;
}

#define RUN_TEST(name) do { \
    if (name()) { \
        s_pass++; \
        ESP_LOGI(TAG, "  PASS  %s", #name); \
    } else { \
        s_fail++; \
    } \
} while (0)

void cartfsm_self_test_run(void)
{
    s_pass = 0;
    s_fail = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " cartfsm Self-Test");
    ESP_LOGI(TAG, "========================================");

    RUN_TEST(test_transitions_spot);
    RUN_TEST(test_soft_limit_spot);
    RUN_TEST(test_config_matrix_spot);

    int total = s_pass + s_fail;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/%d PASS", s_pass, total);
    ESP_LOGI(TAG, "========================================");
}
