/*
 * G-code Parser — pure logic component
 *
 * Parses a subset of G-code (PLAN §4.3 command set):
 *   G0 / G1   — linear move (X required, F/A optional & modal)
 *   G90 / G91 — absolute / relative coordinate mode
 *   G92       — set current position (origin offset)
 *   M17 / M18 — motor enable / disable (M84 alias for M18)
 *   M502      — restore parameters to factory defaults
 *   M503      — report parameters (same as $$)
 *   $$        — list all parameters
 *   $<n>=<v>  — set parameter value
 *
 * Features:
 *   - Case insensitive (g1 == G1)
 *   - Whitespace tolerant (spaces, tabs, multiple separators)
 *   - Comment handling: ';' to end of line, '(' inline comments
 *     (both word-level and at value position — PLAN §4.1)
 *   - Modal state: F (feed) and A (accel) persist across lines
 *   - Numeric clamping: |X|<=1e6, F in (0,30000], A in (0,50000]
 *   - Error codes per PLAN §4.6: 1=unknown cmd, 2=range,
 *     4=bad parameter, 5=storage failure
 *
 * Output is delivered via a callback interface (parser_output_iface_t).
 * T6 step_engine consumes on_move, T11 settings consumes on_set_param /
 * on_list_params / on_restore_defaults.
 *
 * Note on M502: restores default parameter values but does NOT reset
 * the parser's modal F/A runtime values.  New modal defaults take
 * effect after reboot (parser created with settings_default_feed/accel).
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
    PARSER_ERR_RANGE       = 2,   /* Value out of allowed range */
    PARSER_ERR_BAD_PARAM   = 4,   /* Bad parameter value / missing arg */
    PARSER_ERR_STORAGE     = 5,   /* NVS write / erase failure */
} parser_status_t;

/* ---- Output callback interface ----
 *
 * All callbacks receive the user-provided ctx pointer as first argument.
 * Callbacks are optional — set unused ones to NULL.
 *
 * Callbacks that return int use the convention:
 *   0 = success
 *   2 = range rejection (soft limit)  — on_move only
 *   4 = invalid value / unknown index
 *   5 = storage failure
 * The parser propagates non-zero returns as PARSER_ERR_RANGE,
 * PARSER_ERR_BAD_PARAM or PARSER_ERR_STORAGE respectively.
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
     *
     * Returns: 0 = accepted (position committed, motion planned);
     *          2 = rejected by soft limits (→ error:2, position NOT
     *              committed, so a rejected move cannot skew G91);
     *          any other non-zero is treated as 4 (defensive).
     * The parser fires this BEFORE committing its tracked position —
     * see parse_move(). */
    int (*on_move)(void *ctx, float target_mm, float feed_mm_min,
                   float accel_mm_s2, bool rapid);

    /* Set origin offset (G92 X<pos>). */
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
     * Returns: 0=ok, 4=invalid, 5=storage failure. */
    int (*on_set_param)(void *ctx, int idx, float value);

    /* Restore all parameters to factory defaults (M502).
     * Returns: 0=ok, 5=storage failure. */
    int (*on_restore_defaults)(void *ctx);

    void *ctx;   /* user data passed through to every callback */
} parser_output_iface_t;

/* ---- Opaque parser state ---- */
typedef struct gcode_parser_s gcode_parser_t;

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
 * Allocates a parser struct and sets initial modal values:
 *   absolute mode (G90), feed = init_feed_mm_min,
 *   accel = init_accel_mm_s2.
 *
 * @param init_feed_mm_min   Initial modal feed rate (mm/min).
 *                           Typically settings_default_feed().
 * @param init_accel_mm_s2   Initial modal acceleration (mm/s²).
 *                           Typically settings_default_accel().
 * @param iface              Output callback interface (copied internally).
 *
 * @return Parser handle, or NULL on allocation failure.
 */
gcode_parser_t *gcode_parser_create(float init_feed_mm_min,
                                     float init_accel_mm_s2,
                                     const parser_output_iface_t *iface);

/**
 * @brief Destroy a parser instance and free memory.
 */
void gcode_parser_destroy(gcode_parser_t *parser);

/**
 * @brief Feed one line of G-code to the parser.
 *
 * The line should NOT include the trailing \n or \r\n (T3 strips those).
 * Leading / trailing whitespace is fine.
 *
 * @return PARSER_OK on success, or an error code (see parser_status_t).
 */
parser_status_t gcode_parser_feed(gcode_parser_t *parser, const char *line);

/* ---- State inspection ---- */

float gcode_parser_get_position(const gcode_parser_t *parser);
float gcode_parser_get_feed(const gcode_parser_t *parser);
float gcode_parser_get_accel(const gcode_parser_t *parser);
bool  gcode_parser_is_absolute(const gcode_parser_t *parser);

/* ---- Self-test (on-target, 30 cases) ---- */
#ifdef CONFIG_GCODE_PARSER_SELF_TEST
int gcode_parser_self_test_run(void);
#endif

#ifdef __cplusplus
}
#endif
