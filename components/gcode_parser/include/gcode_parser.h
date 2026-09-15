/*
 * G-code Parser — pure logic component
 *
 * Parses a subset of G-code (PLAN §4.3 command set):
 *   G0 / G1   — linear move (X required, F/A optional & modal)
 *   G90 / G91 — absolute / relative coordinate mode
 *   G92       — set current position (origin offset)
 *   M17 / M18 — motor enable / disable (M84 alias for M18)
 *   $$        — list all parameters
 *   $<n>=<v>  — set parameter value
 *   $<n>?     — query single parameter value
 *   M503      — report parameters (same as $$, placeholder)
 *
 * Features:
 *   - Case insensitive (g1 == G1)
 *   - Whitespace tolerant (spaces, tabs, multiple separators)
 *   - Comment handling: ';' to end of line, '(' inline comments
 *   - Modal state: F (feed) and A (accel) persist across lines
 *   - Numeric clamping: |X|<=1e6, F in (0,30000], A in (0,50000]
 *   - Error codes per PLAN §4.6: 1=unknown cmd, 2=soft-limit/range,
 *     4=bad parameter / missing arg
 *
 * Output is delivered via a callback interface (parser_output_iface_t).
 * T6 step_engine consumes on_move, T11 settings consumes on_set_param /
 * on_get_param / on_list_params.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Parser status / error codes (PLAN §4.6) ---- */
typedef enum {
    PARSER_OK              = 0,   /* Parsed successfully */
    PARSER_ERR_UNKNOWN_CMD = 1,   /* Unknown / unsupported command */
    PARSER_ERR_RANGE       = 2,   /* Value out of allowed range (soft-limit style) */
    PARSER_ERR_BAD_PARAM   = 4,   /* Bad parameter value / missing required arg */
} parser_status_t;

/* ---- Output callback interface ----
 *
 * All callbacks receive the user-provided ctx pointer as first argument.
 * Callbacks are optional — set unused ones to NULL.
 */
typedef struct {
    /* Motion command (G0 / G1).
     * target_mm : target X position (mm).  Always absolute — parser
     *             handles G90/G91 internally and reports absolute target.
     * feed_mm_min: effective feed rate (mm/min).  Modal value if not
     *             specified on the line.
     * accel_mm_s2: effective acceleration (mm/s²).  Modal value if not
     *             specified on the line.
     * rapid     : true for G0 (rapid), false for G1 (linear feed).
     *             In this firmware both behave the same, but the flag
     *             is passed through for semantic distinction. */
    void (*on_move)(void *ctx, float target_mm, float feed_mm_min,
                    float accel_mm_s2, bool rapid);

    /* Set origin offset (G92 X<pos>).
     * new_pos_mm : the new "current position" value (mm). */
    void (*on_set_origin)(void *ctx, float new_pos_mm);

    /* Switch to absolute coordinate mode (G90). */
    void (*on_set_absolute)(void *ctx);

    /* Switch to relative coordinate mode (G91). */
    void (*on_set_relative)(void *ctx);

    /* Motor enable / disable (M17 = enable, M18/M84 = disable). */
    void (*on_motor_enable)(void *ctx, bool enable);

    /* List all parameters ($$ or M503). */
    void (*on_list_params)(void *ctx);

    /* Set parameter ($<n>=<v>).
     * idx   : parameter index (0..15 per PLAN §5).
     * value : new value. */
    void (*on_set_param)(void *ctx, int idx, float value);

    /* Query single parameter ($<n>?).
     * idx : parameter index. */
    void (*on_get_param)(void *ctx, int idx);

    void *ctx;   /* user data passed through to every callback */
} parser_output_iface_t;

/* ---- Opaque parser state ---- */
typedef struct gcode_parser_s gcode_parser_t;

/* ---- Default modal values ----
 * Used before any F/A is specified on a line.
 * Real defaults should come from NVS (T11); these are safe fallbacks. */
#define PARSER_DEFAULT_FEED_MM_MIN   300.0f
#define PARSER_DEFAULT_ACCEL_MM_S2   200.0f

/* ---- Range limits (clamp with error) ---- */
#define PARSER_MAX_POS_MM            1.0e6f   /* |X| <= 1e6 mm */
#define PARSER_MAX_FEED_MM_MIN       30000.0f /* F <= 30000 mm/min */
#define PARSER_MIN_FEED_MM_MIN       0.0f     /* F > 0 */
#define PARSER_MAX_ACCEL_MM_S2       50000.0f /* A <= 50000 mm/s² */
#define PARSER_MIN_ACCEL_MM_S2       0.0f     /* A > 0 */
#define PARSER_MAX_PARAM_INDEX       15       /* $0..$15 */

/* ================================================================
 * API
 * ================================================================ */

/**
 * @brief Create and initialize a parser instance.
 *
 * Allocates a parser struct and sets modal defaults:
 *   absolute mode (G90), feed = PARSER_DEFAULT_FEED_MM_MIN,
 *   accel = PARSER_DEFAULT_ACCEL_MM_S2.
 *
 * @param iface  Output callback interface (copied internally; may be
 *               on the stack).  Unused callbacks can be NULL.
 *
 * @return Parser handle, or NULL on allocation failure.
 */
gcode_parser_t *gcode_parser_create(const parser_output_iface_t *iface);

/**
 * @brief Destroy a parser instance and free memory.
 *
 * @param parser  Parser handle (may be NULL — no-op).
 */
void gcode_parser_destroy(gcode_parser_t *parser);

/**
 * @brief Feed one line of G-code to the parser.
 *
 * The line should NOT include the trailing \n or \r\n (T3 strips those).
 * Leading / trailing whitespace is fine.
 *
 * If the line is empty or a pure comment, PARSER_OK is returned
 * (and no callback is fired).
 *
 * @param parser  Parser handle.
 * @param line    Null-terminated G-code line.
 *
 * @return PARSER_OK on success, or an error code (see parser_status_t).
 */
parser_status_t gcode_parser_feed(gcode_parser_t *parser, const char *line);

/**
 * @brief Get the current absolute position (mm) tracked by the parser.
 *
 * The parser tracks position so that G91 (relative) and G92 (origin
 * offset) work correctly.  This is the "instruction position" — not
 * the actual machine position.
 *
 * @param parser  Parser handle.
 * @return Current X position in mm (absolute coordinates).
 */
float gcode_parser_get_position(const gcode_parser_t *parser);

/**
 * @brief Get current modal feed rate (mm/min).
 */
float gcode_parser_get_feed(const gcode_parser_t *parser);

/**
 * @brief Get current modal acceleration (mm/s²).
 */
float gcode_parser_get_accel(const gcode_parser_t *parser);

/**
 * @brief Returns true if the parser is in absolute (G90) mode.
 */
bool gcode_parser_is_absolute(const gcode_parser_t *parser);

/**
 * @brief Run all 30 parser self-tests on-target.
 *
 * Only available when CONFIG_GCODE_PARSER_SELF_TEST=y.
 * Prints PASS/FAIL per test and a summary line to ESP_LOG.
 *
 * @return Number of passing tests (0..30).
 */
#ifdef CONFIG_GCODE_PARSER_SELF_TEST
int gcode_parser_self_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
