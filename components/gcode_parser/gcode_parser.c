/*
 * G-code Parser implementation
 *
 * Line-level parser for a subset of G-code (PLAN §4.3).
 * Pure logic — no hardware dependencies, runs on host and on target.
 */

#include "gcode_parser.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>

/* ---- Internal parser state struct ---- */
struct gcode_parser_s {
    parser_output_iface_t iface;

    /* Modal state */
    bool  absolute_mode;   /* true = G90 (absolute), false = G91 (relative) */
    float modal_feed;      /* mm/min */
    float modal_accel;     /* mm/s² */

    /* Current instruction position (absolute, mm).
     * Needed for G91 (relative → absolute) and G92 (origin offset). */
    float current_pos_mm;
};

/* ================================================================
 * Helpers — whitespace / comment skipping
 * ================================================================ */

/* Skip whitespace characters (space, tab).
 * Returns pointer to first non-whitespace char, or to '\0'. */
static const char *skip_ws(const char *p)
{
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    return p;
}

/* Skip to end of line comment.
 * If *p == ';' or '(', skip the rest of the comment.
 *   ';' → skip to end of string
 *   '(' → skip to matching ')' (or end of string)
 * Returns pointer to first char after the comment. */
static const char *skip_comment(const char *p)
{
    if (*p == ';') {
        /* Line comment — skip to end */
        while (*p) p++;
        return p;
    }
    if (*p == '(') {
        /* Inline comment — skip to ')' */
        p++;  /* skip '(' */
        while (*p && *p != ')') {
            p++;
        }
        if (*p == ')') {
            p++;  /* skip ')' */
        }
        return p;
    }
    return p;
}

/* ================================================================
 * Helpers — numeric parsing
 * ================================================================ */

/* Parse a floating-point number starting at *p.
 *
 * Accepts: optional sign, digits, optional decimal point and fractional part.
 * Does NOT accept scientific notation (1.5e3).  If 'e' or 'E' is found,
 * parsing stops before it (so G-code E-word doesn't confuse things —
 * though our command set doesn't use E anyway).
 *
 * On success: *out_val is set, *endp points to first char not consumed.
 * Returns true on success, false if no valid number was found.
 */
static bool parse_float(const char *p, float *out_val, const char **endp)
{
    const char *start = p;
    bool has_digits = false;

    /* Optional sign */
    if (*p == '+' || *p == '-') {
        p++;
    }

    /* Integer part */
    while (*p && isdigit((unsigned char)*p)) {
        has_digits = true;
        p++;
    }

    /* Fractional part */
    if (*p == '.') {
        p++;
        while (*p && isdigit((unsigned char)*p)) {
            has_digits = true;
            p++;
        }
    }

    if (!has_digits) {
        *endp = start;
        return false;
    }

    /* Use strtof for the actual conversion (handles all edge cases).
     * We've already validated the shape above. */
    *out_val = strtof(start, (char **)&p);
    *endp = p;
    return true;
}

/* Parse an integer number starting at *p.
 *
 * Accepts: optional sign, digits.
 * Returns true on success. */
static bool parse_int(const char *p, int *out_val, const char **endp)
{
    const char *start = p;
    bool has_digits = false;
    int sign = 1;

    if (*p == '+') {
        p++;
    } else if (*p == '-') {
        sign = -1;
        p++;
    }

    while (*p && isdigit((unsigned char)*p)) {
        has_digits = true;
        p++;
    }

    if (!has_digits) {
        *endp = start;
        return false;
    }

    /* Convert manually to avoid pulling in strtol (and for clarity). */
    int val = 0;
    const char *q = start;
    if (*q == '+' || *q == '-') q++;
    while (isdigit((unsigned char)*q)) {
        val = val * 10 + (*q - '0');
        q++;
    }
    *out_val = sign * val;
    *endp = p;
    return true;
}

/* ================================================================
 * Helpers — range checking
 * ================================================================ */

static bool in_range_excl(float val, float min, float max)
{
    return (val > min) && (val <= max);
}

static bool in_range_abs(float val, float max_abs)
{
    return (val >= -max_abs) && (val <= max_abs);
}

/* ================================================================
 * Command parsers
 * ================================================================
 *
 * Each parser function takes the position in the line after the command
 * letter+number (e.g. after "G1"), parses the remaining words, fires
 * callbacks, and returns a status code.
 */

/* Parse G0 / G1 (linear move).
 * Expected words: X (required), F (optional, modal), A (optional, modal). */
static parser_status_t parse_move(gcode_parser_t *p, const char *line,
                                  bool is_g0)
{
    const char *s = line;
    bool has_x = false;
    float x_val = 0.0f;
    bool has_f = false;
    float f_val = p->modal_feed;
    bool has_a = false;
    float a_val = p->modal_accel;

    while (*s) {
        s = skip_ws(s);
        if (*s == '\0') break;

        /* Check for end-of-line or inline comment */
        if (*s == ';' || *s == '(') {
            s = skip_comment(s);
            continue;
        }

        char letter = (char)toupper((unsigned char)*s);
        s++;  /* skip letter */

        s = skip_ws(s);

        /* Skip inline (...) comment at value position (PLAN §4.1),
         * plus any whitespace after it.  ';' skips to end of line,
         * leaving no value → parse_float fails → error:4 (unchanged). */
        s = skip_comment(s);
        s = skip_ws(s);

        if (letter == 'X') {
            if (!parse_float(s, &x_val, &s)) {
                return PARSER_ERR_BAD_PARAM;  /* X without a value */
            }
            has_x = true;
        } else if (letter == 'F') {
            if (!parse_float(s, &f_val, &s)) {
                return PARSER_ERR_BAD_PARAM;
            }
            has_f = true;
        } else if (letter == 'A') {
            if (!parse_float(s, &a_val, &s)) {
                return PARSER_ERR_BAD_PARAM;
            }
            has_a = true;
        } else {
            /* Unknown word letter — treat as bad parameter / malformed line */
            return PARSER_ERR_BAD_PARAM;
        }
    }

    /* X is required for G0/G1 */
    if (!has_x) {
        return PARSER_ERR_BAD_PARAM;
    }

    /* Range check: X (soft limit style) */
    if (!in_range_abs(x_val, PARSER_MAX_POS_MM)) {
        return PARSER_ERR_RANGE;
    }

    /* Range check: F (must be > 0 and <= max) */
    if (!in_range_excl(f_val, PARSER_MIN_FEED_MM_MIN, PARSER_MAX_FEED_MM_MIN)) {
        return PARSER_ERR_RANGE;
    }

    /* Range check: A (must be > 0 and <= max) */
    if (!in_range_excl(a_val, PARSER_MIN_ACCEL_MM_S2, PARSER_MAX_ACCEL_MM_S2)) {
        return PARSER_ERR_RANGE;
    }

    /* Compute absolute target position */
    float target_mm;
    if (p->absolute_mode) {
        target_mm = x_val;
    } else {
        target_mm = p->current_pos_mm + x_val;
    }

    /* Range check final target too */
    if (!in_range_abs(target_mm, PARSER_MAX_POS_MM)) {
        return PARSER_ERR_RANGE;
    }

    /* Update modal state if F/A were given on this line */
    if (has_f) {
        p->modal_feed = f_val;
    }
    if (has_a) {
        p->modal_accel = a_val;
    }

    /* Update tracked position */
    p->current_pos_mm = target_mm;

    /* Fire callback */
    if (p->iface.on_move != NULL) {
        p->iface.on_move(p->iface.ctx, target_mm, p->modal_feed,
                         p->modal_accel, is_g0);
    }

    return PARSER_OK;
}

/* Parse G92 (set origin / current position).
 * Expected words: X (required). */
static parser_status_t parse_g92(gcode_parser_t *p, const char *line)
{
    const char *s = line;
    bool has_x = false;
    float x_val = 0.0f;

    while (*s) {
        s = skip_ws(s);
        if (*s == '\0') break;
        if (*s == ';' || *s == '(') {
            s = skip_comment(s);
            continue;
        }

        char letter = (char)toupper((unsigned char)*s);
        s++;
        s = skip_ws(s);

        /* Skip inline (...) comment at value position (PLAN §4.1),
         * plus any whitespace after it. */
        s = skip_comment(s);
        s = skip_ws(s);

        if (letter == 'X') {
            if (!parse_float(s, &x_val, &s)) {
                return PARSER_ERR_BAD_PARAM;
            }
            has_x = true;
        } else {
            return PARSER_ERR_BAD_PARAM;
        }
    }

    if (!has_x) {
        return PARSER_ERR_BAD_PARAM;
    }

    /* Range check */
    if (!in_range_abs(x_val, PARSER_MAX_POS_MM)) {
        return PARSER_ERR_RANGE;
    }

    /* Set current position (origin offset).
     * Parser just updates its tracked position; actual coordinate
     * offset in encoder/planner is handled by the callback consumer. */
    p->current_pos_mm = x_val;

    if (p->iface.on_set_origin != NULL) {
        p->iface.on_set_origin(p->iface.ctx, x_val);
    }

    return PARSER_OK;
}

/* Parse $<n>=<v> (parameter set).
 * $<n>? has been removed (PLAN v1.10 — unreachable due to real-time '?').
 * Expects s to point just past the '$'. */
static parser_status_t parse_dollar(gcode_parser_t *p, const char *s)
{
    /* Parse the parameter index */
    int idx = 0;
    if (!parse_int(s, &idx, &s)) {
        return PARSER_ERR_BAD_PARAM;
    }
    if (idx < 0 || idx > PARSER_MAX_PARAM_INDEX) {
        return PARSER_ERR_RANGE;
    }

    s = skip_ws(s);

    if (*s == '=') {
        /* $<n>=<v> — set parameter */
        s++;  /* skip '=' */
        s = skip_ws(s);

        float val = 0.0f;
        if (!parse_float(s, &val, &s)) {
            return PARSER_ERR_BAD_PARAM;
        }

        /* Make sure nothing follows (except comment / whitespace) */
        s = skip_ws(s);
        if (*s != '\0' && *s != ';' && *s != '(') {
            return PARSER_ERR_BAD_PARAM;
        }

        if (p->iface.on_set_param != NULL) {
            int rc = p->iface.on_set_param(p->iface.ctx, idx, val);
            if (rc == 4) return PARSER_ERR_BAD_PARAM;
            if (rc == 5) return PARSER_ERR_STORAGE;
            if (rc != 0) return PARSER_ERR_BAD_PARAM;
        }
        return PARSER_OK;

    } else if (*s == '\0' || *s == ';' || *s == '(') {
        /* Bare "$<n>" (no '=') — invalid.
         * Note: $<n>? is dead code — '?' intercepted as real-time char
         * by protocol layer before line assembly (see T4-fix §5.1). */
        return PARSER_ERR_BAD_PARAM;
    } else {
        return PARSER_ERR_BAD_PARAM;
    }
}

/* Parse $$ (list all parameters).
 * Expects s to point at the second '$' or just past "$$". */
static parser_status_t parse_dollar_dollar(gcode_parser_t *p, const char *s)
{
    (void)s;  /* we already consumed both $$ */

    if (p->iface.on_list_params != NULL) {
        p->iface.on_list_params(p->iface.ctx);
    }
    return PARSER_OK;
}

/* ================================================================
 * Main line dispatcher
 * ================================================================ */

parser_status_t gcode_parser_feed(gcode_parser_t *p, const char *line)
{
    if (p == NULL || line == NULL) {
        return PARSER_ERR_BAD_PARAM;
    }

    const char *s = skip_ws(line);

    /* Empty line → ok, no callback */
    if (*s == '\0') {
        return PARSER_OK;
    }

    /* Comment-only line (starts with ';' or '(') → ok, no callback */
    if (*s == ';' || *s == '(') {
        return PARSER_OK;
    }

    /* --- $ commands --- */
    if (*s == '$') {
        s++;  /* skip '$' */
        s = skip_ws(s);

        if (*s == '$') {
            /* $$ — list all parameters */
            s++;
            s = skip_ws(s);
            /* Allow trailing comment only */
            if (*s != '\0' && *s != ';' && *s != '(') {
                return PARSER_ERR_UNKNOWN_CMD;
            }
            return parse_dollar_dollar(p, s);
        }

        if (isdigit((unsigned char)*s)) {
            /* $<n>=<v> or $<n>? */
            return parse_dollar(p, s);
        }

        /* Unknown $ variant */
        return PARSER_ERR_UNKNOWN_CMD;
    }

    /* --- G-commands --- */
    char first = (char)toupper((unsigned char)*s);
    if (first == 'G') {
        s++;  /* skip 'G' */
        s = skip_ws(s);

        int gcode = 0;
        if (!parse_int(s, &gcode, &s)) {
            return PARSER_ERR_BAD_PARAM;  /* G without number */
        }

        switch (gcode) {
        case 0:  /* G0 — rapid move */
            return parse_move(p, s, true);
        case 1:  /* G1 — linear feed move */
            return parse_move(p, s, false);
        case 90:  /* G90 — absolute mode */
            s = skip_ws(s);
            if (*s != '\0' && *s != ';' && *s != '(') {
                return PARSER_ERR_BAD_PARAM;
            }
            p->absolute_mode = true;
            if (p->iface.on_set_absolute != NULL) {
                p->iface.on_set_absolute(p->iface.ctx);
            }
            return PARSER_OK;
        case 91:  /* G91 — relative mode */
            s = skip_ws(s);
            if (*s != '\0' && *s != ';' && *s != '(') {
                return PARSER_ERR_BAD_PARAM;
            }
            p->absolute_mode = false;
            if (p->iface.on_set_relative != NULL) {
                p->iface.on_set_relative(p->iface.ctx);
            }
            return PARSER_OK;
        case 92:  /* G92 — set origin */
            return parse_g92(p, s);
        default:
            return PARSER_ERR_UNKNOWN_CMD;
        }
    }

    /* --- M-commands --- */
    if (first == 'M') {
        s++;  /* skip 'M' */
        s = skip_ws(s);

        int mcode = 0;
        if (!parse_int(s, &mcode, &s)) {
            return PARSER_ERR_BAD_PARAM;
        }

        s = skip_ws(s);
        if (*s != '\0' && *s != ';' && *s != '(') {
            return PARSER_ERR_BAD_PARAM;
        }

        switch (mcode) {
        case 17:  /* M17 — motor enable */
            if (p->iface.on_motor_enable != NULL) {
                p->iface.on_motor_enable(p->iface.ctx, true);
            }
            return PARSER_OK;
        case 18:  /* M18 — motor disable */
        case 84:  /* M84 — alias for M18 */
            if (p->iface.on_motor_enable != NULL) {
                p->iface.on_motor_enable(p->iface.ctx, false);
            }
            return PARSER_OK;
        case 502:  /* M502 — restore factory defaults */
            if (p->iface.on_restore_defaults != NULL) {
                int rc = p->iface.on_restore_defaults(p->iface.ctx);
                if (rc == 5) return PARSER_ERR_STORAGE;
                if (rc != 0) return PARSER_ERR_BAD_PARAM;
            }
            /* Note: M502 does NOT reset parser modal F/A values.
             * New defaults take effect after reboot. */
            return PARSER_OK;
        case 503:  /* M503 — report parameters (same as $$) */
            if (p->iface.on_list_params != NULL) {
                p->iface.on_list_params(p->iface.ctx);
            }
            return PARSER_OK;
        default:
            return PARSER_ERR_UNKNOWN_CMD;
        }
    }

    /* First char is not G, M, or $ — unknown command */
    return PARSER_ERR_UNKNOWN_CMD;
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

gcode_parser_t *gcode_parser_create(float init_feed_mm_min,
                                     float init_accel_mm_s2,
                                     const parser_output_iface_t *iface)
{
    gcode_parser_t *p = (gcode_parser_t *)calloc(1, sizeof(gcode_parser_t));
    if (p == NULL) {
        return NULL;
    }

    if (iface != NULL) {
        p->iface = *iface;
    } else {
        memset(&p->iface, 0, sizeof(p->iface));
    }

    /* Default modal state */
    p->absolute_mode = true;   /* G90 — absolute by default (GRBL convention) */
    p->modal_feed = init_feed_mm_min;
    p->modal_accel = init_accel_mm_s2;
    p->current_pos_mm = 0.0f;

    return p;
}

void gcode_parser_destroy(gcode_parser_t *p)
{
    if (p != NULL) {
        free(p);
    }
}

/* ================================================================
 * Accessors
 * ================================================================ */

float gcode_parser_get_position(const gcode_parser_t *p)
{
    return p ? p->current_pos_mm : 0.0f;
}

float gcode_parser_get_feed(const gcode_parser_t *p)
{
    return p ? p->modal_feed : 0.0f;
}

float gcode_parser_get_accel(const gcode_parser_t *p)
{
    return p ? p->modal_accel : 0.0f;
}

bool gcode_parser_is_absolute(const gcode_parser_t *p)
{
    return p ? p->absolute_mode : true;
}
