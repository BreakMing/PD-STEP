/*
 * G-code Parser — ESP-IDF Unity unit tests
 *
 * On-target test: runs on the ESP32-S3 via idf.py test.
 * Uses the built-in Unity test framework from ESP-IDF.
 *
 * Run on target:
 *   idf.py test
 *
 * Test count: 30 cases covering all commands, modal semantics,
 * format variations, comments, and error paths.
 */

#include "unity.h"
#include "gcode_parser.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ---- Record helpers for callback verification ---- */

#define MAX_RECORDS 64

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

static rec_t s_records[MAX_RECORDS];
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
    if (s_record_count < MAX_RECORDS) {
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
    if (s_record_count < MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_SET_ORIGIN;
        r->f1 = new_pos_mm;
        s_record_count++;
    }
}

static void cb_on_set_absolute(void *ctx) { (void)ctx; if (s_record_count < MAX_RECORDS) { s_records[s_record_count++].type = REC_SET_ABSOLUTE; } }
static void cb_on_set_relative(void *ctx) { (void)ctx; if (s_record_count < MAX_RECORDS) { s_records[s_record_count++].type = REC_SET_RELATIVE; } }

static void cb_on_motor_enable(void *ctx, bool enable)
{
    (void)ctx;
    if (s_record_count < MAX_RECORDS) {
        rec_t *r = &s_records[s_record_count];
        r->type = REC_MOTOR_ENABLE;
        r->b1 = enable;
        s_record_count++;
    }
}

static void cb_on_list_params(void *ctx) { (void)ctx; if (s_record_count < MAX_RECORDS) { s_records[s_record_count++].type = REC_LIST_PARAMS; } }

static void cb_on_set_param(void *ctx, int idx, float value)
{
    (void)ctx;
    if (s_record_count < MAX_RECORDS) {
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
    if (s_record_count < MAX_RECORDS) {
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

#define ASSERT_FLOAT_EQ(a, b) TEST_ASSERT_FLOAT_WITHIN(0.001f, (b), (a))

/* ================================================================
 * Test cases
 * ================================================================ */

static gcode_parser_t *s_p = NULL;

void setUp(void)
{
    s_p = create_test_parser();
    rec_clear();
}

void tearDown(void)
{
    gcode_parser_destroy(s_p);
    s_p = NULL;
}

/* --- Basic commands --- */

TEST_CASE("G0 basic absolute move", "[gcode][move]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G0 X100");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(1, s_record_count);
    TEST_ASSERT_EQUAL(REC_MOVE, s_records[0].type);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f1);
    TEST_ASSERT_TRUE(s_records[0].b1);  /* rapid = true */
    ASSERT_FLOAT_EQ(PARSER_DEFAULT_FEED_MM_MIN, s_records[0].f2);
    ASSERT_FLOAT_EQ(PARSER_DEFAULT_ACCEL_MM_S2, s_records[0].f3);
}

TEST_CASE("G1 with F and A", "[gcode][move]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X50 F1200 A500");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(1, s_record_count);
    ASSERT_FLOAT_EQ(50.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(1200.0f, s_records[0].f2);
    ASSERT_FLOAT_EQ(500.0f, s_records[0].f3);
    TEST_ASSERT_FALSE(s_records[0].b1);
    ASSERT_FLOAT_EQ(1200.0f, gcode_parser_get_feed(s_p));
    ASSERT_FLOAT_EQ(500.0f, gcode_parser_get_accel(s_p));
}

/* --- Modal semantics --- */

TEST_CASE("Modal F and A persist across lines", "[gcode][modal]")
{
    gcode_parser_feed(s_p, "G1 X10 F100 A200");
    rec_clear();

    parser_status_t st = gcode_parser_feed(s_p, "G1 X20");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(1, s_record_count);
    ASSERT_FLOAT_EQ(20.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);  /* F modal retained */
    ASSERT_FLOAT_EQ(200.0f, s_records[0].f3);  /* A modal retained */
}

TEST_CASE("Modal F updates independently of A", "[gcode][modal]")
{
    gcode_parser_feed(s_p, "G1 X10 F500");
    ASSERT_FLOAT_EQ(500.0f, gcode_parser_get_feed(s_p));
    ASSERT_FLOAT_EQ(PARSER_DEFAULT_ACCEL_MM_S2, gcode_parser_get_accel(s_p));

    rec_clear();
    gcode_parser_feed(s_p, "G1 X20 A1000");
    ASSERT_FLOAT_EQ(500.0f, s_records[0].f2);   /* F stays 500 */
    ASSERT_FLOAT_EQ(1000.0f, s_records[0].f3);  /* A updated to 1000 */
    ASSERT_FLOAT_EQ(1000.0f, gcode_parser_get_accel(s_p));
}

/* --- Coordinate modes --- */

TEST_CASE("G91 relative mode and G90 switch back", "[gcode][coords]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G91");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_FALSE(gcode_parser_is_absolute(s_p));
    TEST_ASSERT_EQUAL(REC_SET_RELATIVE, s_records[0].type);

    rec_clear();
    gcode_parser_feed(s_p, "G1 X10 F100");
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);

    rec_clear();
    gcode_parser_feed(s_p, "G1 X5");
    ASSERT_FLOAT_EQ(15.0f, s_records[0].f1);  /* 10 + 5 */

    rec_clear();
    st = gcode_parser_feed(s_p, "G90");
    TEST_ASSERT_TRUE(gcode_parser_is_absolute(s_p));
    TEST_ASSERT_EQUAL(REC_SET_ABSOLUTE, s_records[0].type);
}

TEST_CASE("G92 set origin", "[gcode][coords]")
{
    gcode_parser_feed(s_p, "G1 X100 F100");
    ASSERT_FLOAT_EQ(100.0f, gcode_parser_get_position(s_p));

    rec_clear();
    parser_status_t st = gcode_parser_feed(s_p, "G92 X50");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_SET_ORIGIN, s_records[0].type);
    ASSERT_FLOAT_EQ(50.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(50.0f, gcode_parser_get_position(s_p));
}

TEST_CASE("Negative X in relative mode", "[gcode][coords]")
{
    gcode_parser_feed(s_p, "G91");
    gcode_parser_feed(s_p, "G1 X50 F100");
    ASSERT_FLOAT_EQ(50.0f, gcode_parser_get_position(s_p));

    rec_clear();
    gcode_parser_feed(s_p, "G1 X-20");
    ASSERT_FLOAT_EQ(30.0f, s_records[0].f1);  /* 50 + (-20) */
}

/* --- Format variations --- */

TEST_CASE("Decimal values", "[gcode][format]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X12.345 F123.45 A67.89");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(12.345f, s_records[0].f1);
    ASSERT_FLOAT_EQ(123.45f, s_records[0].f2);
    ASSERT_FLOAT_EQ(67.89f, s_records[0].f3);
}

TEST_CASE("No-space format G1X50F1200", "[gcode][format]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1X50F1200");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(50.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(1200.0f, s_records[0].f2);
}

TEST_CASE("Mixed case g1 x10 f100", "[gcode][format]")
{
    parser_status_t st = gcode_parser_feed(s_p, "g1 x10 f100 a200");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);
    ASSERT_FLOAT_EQ(200.0f, s_records[0].f3);
}

TEST_CASE("Leading/trailing whitespace and tabs", "[gcode][format]")
{
    parser_status_t st = gcode_parser_feed(s_p, "   G1\tX5\tF100   ");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(5.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);
}

/* --- Comments & empty lines --- */

TEST_CASE("Comment-only line returns ok no callback", "[gcode][comment]")
{
    parser_status_t st = gcode_parser_feed(s_p, "; this is a comment");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("Empty line and whitespace-only line", "[gcode][comment]")
{
    parser_status_t st = gcode_parser_feed(s_p, "");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(0, s_record_count);

    st = gcode_parser_feed(s_p, "   \t  ");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("Inline semicolon comment", "[gcode][comment]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X10 ; move to position 10");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
}

TEST_CASE("Inline parenthesis comment", "[gcode][comment]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X(hello)10 F100");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(10.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(100.0f, s_records[0].f2);

    st = gcode_parser_feed(s_p, "G1 X (hello) 20");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(20.0f, s_records[0].f1);

    st = gcode_parser_feed(s_p, "G1 X5 F (hi) 200");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    ASSERT_FLOAT_EQ(5.0f, s_records[0].f1);
    ASSERT_FLOAT_EQ(200.0f, s_records[0].f2);
}

/* --- Error paths --- */

TEST_CASE("Unknown G-code returns error:1", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G42 X10");
    TEST_ASSERT_EQUAL(PARSER_ERR_UNKNOWN_CMD, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("Unknown M-code returns error:1", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "M999");
    TEST_ASSERT_EQUAL(PARSER_ERR_UNKNOWN_CMD, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("G1 missing X returns error:4", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 F100");
    TEST_ASSERT_EQUAL(PARSER_ERR_BAD_PARAM, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("Bad numeric (letters in value) returns error:4", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X10abc");
    TEST_ASSERT_EQUAL(PARSER_ERR_BAD_PARAM, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("Double decimal point returns error:4", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X1.2.3");
    TEST_ASSERT_EQUAL(PARSER_ERR_BAD_PARAM, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("X out of range returns error:2", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X2000000");
    TEST_ASSERT_EQUAL(PARSER_ERR_RANGE, st);
    TEST_ASSERT_EQUAL(0, s_record_count);
}

TEST_CASE("F out of range (zero) returns error:2", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X10 F0");
    TEST_ASSERT_EQUAL(PARSER_ERR_RANGE, st);
}

TEST_CASE("A out of range (too large) returns error:2", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G1 X10 A99999");
    TEST_ASSERT_EQUAL(PARSER_ERR_RANGE, st);
}

TEST_CASE("G92 missing X returns error:4", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "G92");
    TEST_ASSERT_EQUAL(PARSER_ERR_BAD_PARAM, st);
}

TEST_CASE("Parameter index out of range returns error:2", "[gcode][error]")
{
    parser_status_t st = gcode_parser_feed(s_p, "$16=100");
    TEST_ASSERT_EQUAL(PARSER_ERR_RANGE, st);

    st = gcode_parser_feed(s_p, "$100=100");
    TEST_ASSERT_EQUAL(PARSER_ERR_RANGE, st);
}

/* --- M-codes --- */

TEST_CASE("M17 enables motor, M18/M84 disables", "[gcode][mcode]")
{
    parser_status_t st = gcode_parser_feed(s_p, "M17");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_MOTOR_ENABLE, s_records[0].type);
    TEST_ASSERT_TRUE(s_records[0].b1);

    rec_clear();
    st = gcode_parser_feed(s_p, "M18");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_FALSE(s_records[0].b1);

    rec_clear();
    st = gcode_parser_feed(s_p, "M84");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_MOTOR_ENABLE, s_records[0].type);
    TEST_ASSERT_FALSE(s_records[0].b1);
}

/* --- Parameter commands --- */

TEST_CASE("$$ lists all parameters", "[gcode][params]")
{
    parser_status_t st = gcode_parser_feed(s_p, "$$");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_LIST_PARAMS, s_records[0].type);
}

TEST_CASE("$n=v sets parameter", "[gcode][params]")
{
    parser_status_t st = gcode_parser_feed(s_p, "$3=42.5");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_SET_PARAM, s_records[0].type);
    TEST_ASSERT_EQUAL(3, s_records[0].i1);
    ASSERT_FLOAT_EQ(42.5f, s_records[0].f1);
}

TEST_CASE("$n? queries parameter", "[gcode][params]")
{
    parser_status_t st = gcode_parser_feed(s_p, "$5?");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_GET_PARAM, s_records[0].type);
    TEST_ASSERT_EQUAL(5, s_records[0].i1);
}

TEST_CASE("M503 reports parameters (same as $$)", "[gcode][params]")
{
    parser_status_t st = gcode_parser_feed(s_p, "M503");
    TEST_ASSERT_EQUAL(PARSER_OK, st);
    TEST_ASSERT_EQUAL(REC_LIST_PARAMS, s_records[0].type);
}
