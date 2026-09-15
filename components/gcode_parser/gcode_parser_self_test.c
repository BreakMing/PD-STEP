/*
 * G-code Parser — on-target self-test (no Unity dependency)
 *
 * Built into firmware when CONFIG_GCODE_PARSER_SELF_TEST=y.
 * Runs all 30 test cases on boot, prints results via ESP_LOG,
 * and returns the number of passing tests.
 *
 * Entry point: gcode_parser_self_test_run()
 * Final line: "SELF-TEST DONE: <n>/30 PASS"
 */

#include "gcode_parser.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "gcode_selftest";

/* ---- Record helpers for callback verification ---- */

#define SELFTEST_MAX_RECORDS 64

typedef enum {
    REC_NONE = 0,
    REC_MOVE,
    REC_SET_ORIGIN,
    REC_SET_ABSOLUTE,
    REC_SET_RELATIVE,
    REC_MOTOR_ENABLE,
    REC_LIST_PARAMS,
    REC_SET_PARAM,
    REC_GET_PARAM,
} rec_type_t;

typedef struct {
    rec_type_t type;
    float f1;   /* target / position / value */
    float f2;   /* feed */
    float f3;   /* accel */
    int   i1;   /* param index */
    bool  b1;   /* rapid / enable */
} rec_t;

static rec_t s_records[SELFTEST_MAX_RECORDS];
static int   s_record_count = 0;

static void rec_clear(void)
{
    memset(s_records, 0, sizeof(s_records));
    s_record_count = 0;
}

/* ---- Callback stubs ---- */

static void cb_on_move(void *ctx, float target_mm, float feed_mm_min,
                       float accel_mm_s2, bool rapid)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_MOVE;
        r->f1 = target_mm;
        r->f2 = feed_mm_min;
        r->f3 = accel_mm_s2;
        r->b1 = rapid;
        s_record_count++;
    }
}

static void cb_on_set_origin(void *ctx, float new_pos_mm)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_SET_ORIGIN;
        r->f1 = new_pos_mm;
        s_record_count++;
    }
}

static void cb_on_set_absolute(void *ctx)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        s_records[s_record_count++].type = REC_SET_ABSOLUTE;
    }
}

static void cb_on_set_relative(void *ctx)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        s_records[s_record_count++].type = REC_SET_RELATIVE;
    }
}

static void cb_on_motor_enable(void *ctx, bool enable)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_MOTOR_ENABLE;
        r->b1 = enable;
        s_record_count++;
    }
}

static void cb_on_list_params(void *ctx)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        s_records[s_record_count++].type = REC_LIST_PARAMS;
    }
}

static void cb_on_set_param(void *ctx, int idx, float value)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_SET_PARAM;
        r->i1 = idx;
        r->f1 = value;
        s_record_count++;
    }
}

static void cb_on_get_param(void *ctx, int idx)
{
    (void)ctx;
    if (s_record_count < SELFTEST_MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_GET_PARAM;
        r->i1 = idx;
        s_record_count++;
    }
}

static gcode_parser_t *create_test_parser(void)
{
    parser_output_iface_t iface = {0};
    iface.on_move         = cb_on_move;
    iface.on_set_origin   = cb_on_set_origin;
    iface.on_set_absolute = cb_on_set_absolute;
    iface.on_set_relative = cb_on_set_relative;
    iface.on_motor_enable = cb_on_motor_enable;
    iface.on_list_params  = cb_on_list_params;
    iface.on_set_param    = cb_on_set_param;
    iface.on_get_param    = cb_on_get_param;
    iface.ctx             = NULL;
    return gcode_parser_create(&iface);
}

/* ---- Test assertion macros ---- */

static int s_pass_count = 0;
static int s_fail_count = 0;

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        ESP_LOGE(TAG, "  FAIL %s: %s (line %d)", __func__, msg, __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_EQ(expected, actual) do { \
    if ((expected) != (actual)) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %d, got %d (line %d)", \
                 __func__, (int)(expected), (int)(actual), __LINE__); \
        return false; \
    } \
} while (0)

#define ASSERT_FLOAT_EQ(expected, actual) do { \
    if (fabsf((expected) - (actual)) > 0.001f) { \
        ESP_LOGE(TAG, "  FAIL %s: expected %.3f, got %.3f (line %d)", \
                 __func__, (double)(expected), (double)(actual), __LINE__); \
        return false; \
    } \
} while (0)

/* ---- 30 test cases ----
 * Each returns true on pass, false on fail.
 * Assertions mirror the Unity test_gcode_parser.c exactly. */

static bool test_g0_basic_absolute(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G0 X100");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(1, s_record_count);
    ASSERT_EQ(REC_MOVE, s_records[0].type);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f1);
    ASSERT_TRUE(s_records[0].b1, "rapid should be true for G0");
    ASSERT_FLOAT_EQ(PARSER_DEFAULT_FEED_MM_MIN, s_records[0].f2);
    ASSERT_FLOAT_EQ(PARSER_DEFAULT_ACCEL_MM_S2, s_records[0].f3);
    gcode_parser_destroy(p);
    return true;
}

static bool test_g1_with_f_and_a(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X50 F1200 A500");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(1, s_record_count);
    ASSERT_FLOAT_EQ(50.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(1200.0f, s_records[0].f2);
    ASSERT_FLOAT_EQ(500.0f, s_records[0].f3);
    ASSERT_TRUE(!s_records[0].b1, "rapid should be false for G1");
    ASSERT_FLOAT_EQ(1200.0f, gcode_parser_get_feed(p));
    ASSERT_FLOAT_EQ(500.0f, gcode_parser_get_accel(p));
    gcode_parser_destroy(p);
    return true;
}

static bool test_modal_f_a_persistence(void)
{
    gcode_parser_t *p = create_test_parser();
    gcode_parser_feed(p, "G1 X10 F100 A200");
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X20");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(1, s_record_count);
    ASSERT_FLOAT_EQ(20.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);
    ASSERT_FLOAT_EQ(200.0f, s_records[0].f3);
    gcode_parser_destroy(p);
    return true;
}

static bool test_modal_f_independent_of_a(void)
{
    gcode_parser_t *p = create_test_parser();
    gcode_parser_feed(p, "G1 X10 F500");
    ASSERT_FLOAT_EQ(500.0f, gcode_parser_get_feed(p));
    ASSERT_FLOAT_EQ(PARSER_DEFAULT_ACCEL_MM_S2, gcode_parser_get_accel(p));
    rec_clear();
    gcode_parser_feed(p, "G1 X20 A1000");
    ASSERT_FLOAT_EQ(500.0f, s_records[0].f2);
    ASSERT_FLOAT_EQ(1000.0f, s_records[0].f3);
    ASSERT_FLOAT_EQ(1000.0f, gcode_parser_get_accel(p));
    gcode_parser_destroy(p);
    return true;
}

static bool test_g91_relative_g90_switch(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G91");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_TRUE(!gcode_parser_is_absolute(p), "should be relative after G91");
    ASSERT_EQ(REC_SET_RELATIVE, s_records[0].type);
    rec_clear();
    gcode_parser_feed(p, "G1 X10 F100");
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
    rec_clear();
    gcode_parser_feed(p, "G1 X5");
    ASSERT_FLOAT_EQ(15.0f, s_records[0].f1);
    rec_clear();
    st = gcode_parser_feed(p, "G90");
    ASSERT_TRUE(gcode_parser_is_absolute(p), "should be absolute after G90");
    ASSERT_EQ(REC_SET_ABSOLUTE, s_records[0].type);
    gcode_parser_destroy(p);
    return true;
}

static bool test_g92_set_origin(void)
{
    gcode_parser_t *p = create_test_parser();
    gcode_parser_feed(p, "G1 X100 F100");
    ASSERT_FLOAT_EQ(100.0f, gcode_parser_get_position(p));
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G92 X50");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_SET_ORIGIN, s_records[0].type);
    ASSERT_FLOAT_EQ(50.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(50.0f, gcode_parser_get_position(p));
    gcode_parser_destroy(p);
    return true;
}

static bool test_negative_x_relative(void)
{
    gcode_parser_t *p = create_test_parser();
    gcode_parser_feed(p, "G91");
    gcode_parser_feed(p, "G1 X50 F100");
    ASSERT_FLOAT_EQ(50.0f, gcode_parser_get_position(p));
    rec_clear();
    gcode_parser_feed(p, "G1 X-20");
    ASSERT_FLOAT_EQ(30.0f, s_records[0].f1);
    gcode_parser_destroy(p);
    return true;
}

static bool test_decimal_values(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X12.345 F123.45 A67.89");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(12.345f, s_records[0].f1);
    ASSERT_FLOAT_EQ(123.45f, s_records[0].f2);
    ASSERT_FLOAT_EQ(67.89f, s_records[0].f3);
    gcode_parser_destroy(p);
    return true;
}

static bool test_no_space_format(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1X50F1200");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(50.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(1200.0f, s_records[0].f2);
    gcode_parser_destroy(p);
    return true;
}

static bool test_mixed_case(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "g1 x10 f100 a200");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);
    ASSERT_FLOAT_EQ(200.0f, s_records[0].f3);
    gcode_parser_destroy(p);
    return true;
}

static bool test_whitespace_tabs(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "   G1\tX5\tF100   ");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(5.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);
    gcode_parser_destroy(p);
    return true;
}

static bool test_comment_only_line(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "; this is a comment");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_empty_and_whitespace_line(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(0, s_record_count);
    st = gcode_parser_feed(p, "   \t  ");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_inline_semicolon_comment(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X10 ; move to position 10");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
    gcode_parser_destroy(p);
    return true;
}

static bool test_inline_paren_comment(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    /* PLAN §4.1: (...) at value position is legal, with or without
     * surrounding whitespace */
    parser_status_t st = gcode_parser_feed(p, "G1 X(hello)10 F100");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);
    rec_clear();
    st = gcode_parser_feed(p, "G1 X (hello) 20");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(20.0f, s_records[0].f1);
    rec_clear();
    st = gcode_parser_feed(p, "G1 X5 F (hi) 200");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_FLOAT_EQ(5.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(200.0f, s_records[0].f2);
    gcode_parser_destroy(p);
    return true;
}

static bool test_unknown_gcode(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G42 X10");
    ASSERT_EQ(PARSER_ERR_UNKNOWN_CMD, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_unknown_mcode(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "M999");
    ASSERT_EQ(PARSER_ERR_UNKNOWN_CMD, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_g1_missing_x(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 F100");
    ASSERT_EQ(PARSER_ERR_BAD_PARAM, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_bad_numeric_letters(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X10abc");
    ASSERT_EQ(PARSER_ERR_BAD_PARAM, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_double_decimal_point(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X1.2.3");
    ASSERT_EQ(PARSER_ERR_BAD_PARAM, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_x_out_of_range(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X2000000");
    ASSERT_EQ(PARSER_ERR_RANGE, st);
    ASSERT_EQ(0, s_record_count);
    gcode_parser_destroy(p);
    return true;
}

static bool test_f_out_of_range_zero(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X10 F0");
    ASSERT_EQ(PARSER_ERR_RANGE, st);
    gcode_parser_destroy(p);
    return true;
}

static bool test_a_out_of_range_large(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G1 X10 A99999");
    ASSERT_EQ(PARSER_ERR_RANGE, st);
    gcode_parser_destroy(p);
    return true;
}

static bool test_g92_missing_x(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "G92");
    ASSERT_EQ(PARSER_ERR_BAD_PARAM, st);
    gcode_parser_destroy(p);
    return true;
}

static bool test_param_index_out_of_range(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "$16=100");
    ASSERT_EQ(PARSER_ERR_RANGE, st);
    st = gcode_parser_feed(p, "$100=100");
    ASSERT_EQ(PARSER_ERR_RANGE, st);
    gcode_parser_destroy(p);
    return true;
}

static bool test_m17_m18_m84_motor_enable(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "M17");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_MOTOR_ENABLE, s_records[0].type);
    ASSERT_TRUE(s_records[0].b1, "M17 should enable motor");
    rec_clear();
    st = gcode_parser_feed(p, "M18");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_TRUE(!s_records[0].b1, "M18 should disable motor");
    rec_clear();
    st = gcode_parser_feed(p, "M84");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_MOTOR_ENABLE, s_records[0].type);
    ASSERT_TRUE(!s_records[0].b1, "M84 should disable motor");
    gcode_parser_destroy(p);
    return true;
}

static bool test_dollar_dollar_list_params(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "$$");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_LIST_PARAMS, s_records[0].type);
    gcode_parser_destroy(p);
    return true;
}

static bool test_dollar_set_param(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "$3=42.5");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_SET_PARAM, s_records[0].type);
    ASSERT_EQ(3, s_records[0].i1);
    ASSERT_FLOAT_EQ(42.5f, s_records[0].f1);
    gcode_parser_destroy(p);
    return true;
}

static bool test_dollar_query_param(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "$5?");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_GET_PARAM, s_records[0].type);
    ASSERT_EQ(5, s_records[0].i1);
    gcode_parser_destroy(p);
    return true;
}

static bool test_m503_report_params(void)
{
    gcode_parser_t *p = create_test_parser();
    rec_clear();
    parser_status_t st = gcode_parser_feed(p, "M503");
    ASSERT_EQ(PARSER_OK, st);
    ASSERT_EQ(REC_LIST_PARAMS, s_records[0].type);
    gcode_parser_destroy(p);
    return true;
}

/* ---- Test runner ---- */

#define RUN_TEST(name) do { \
    if (name()) { \
        s_pass_count++; \
        ESP_LOGI(TAG, "  PASS  %s", #name); \
    } else { \
        s_fail_count++; \
    } \
} while (0)

int gcode_parser_self_test_run(void)
{
    s_pass_count = 0;
    s_fail_count = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " G-code Parser Self-Test (30 cases)");
    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "Basic commands:");
    RUN_TEST(test_g0_basic_absolute);
    RUN_TEST(test_g1_with_f_and_a);

    ESP_LOGI(TAG, "Modal semantics:");
    RUN_TEST(test_modal_f_a_persistence);
    RUN_TEST(test_modal_f_independent_of_a);

    ESP_LOGI(TAG, "Coordinate modes:");
    RUN_TEST(test_g91_relative_g90_switch);
    RUN_TEST(test_g92_set_origin);
    RUN_TEST(test_negative_x_relative);

    ESP_LOGI(TAG, "Format variations:");
    RUN_TEST(test_decimal_values);
    RUN_TEST(test_no_space_format);
    RUN_TEST(test_mixed_case);
    RUN_TEST(test_whitespace_tabs);

    ESP_LOGI(TAG, "Comments & empty lines:");
    RUN_TEST(test_comment_only_line);
    RUN_TEST(test_empty_and_whitespace_line);
    RUN_TEST(test_inline_semicolon_comment);
    RUN_TEST(test_inline_paren_comment);

    ESP_LOGI(TAG, "Error paths:");
    RUN_TEST(test_unknown_gcode);
    RUN_TEST(test_unknown_mcode);
    RUN_TEST(test_g1_missing_x);
    RUN_TEST(test_bad_numeric_letters);
    RUN_TEST(test_double_decimal_point);
    RUN_TEST(test_x_out_of_range);
    RUN_TEST(test_f_out_of_range_zero);
    RUN_TEST(test_a_out_of_range_large);
    RUN_TEST(test_g92_missing_x);
    RUN_TEST(test_param_index_out_of_range);

    ESP_LOGI(TAG, "M-codes & parameters:");
    RUN_TEST(test_m17_m18_m84_motor_enable);
    RUN_TEST(test_dollar_dollar_list_params);
    RUN_TEST(test_dollar_set_param);
    RUN_TEST(test_dollar_query_param);
    RUN_TEST(test_m503_report_params);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " SELF-TEST DONE: %d/30 PASS", s_pass_count);
    ESP_LOGI(TAG, "========================================");

    return s_pass_count;
}
