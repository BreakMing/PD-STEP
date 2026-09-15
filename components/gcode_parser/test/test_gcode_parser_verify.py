#!/usr/bin/env python3
"""
G-code Parser — Python logic verification (reference model of C implementation)

Python port of the C gcode_parser, kept in sync with components/gcode_parser/
interface (T11 version: gcode_parser_create(init_feed, init_accel, iface),
no $n? syntax, on_set_param returns int, M502 restore defaults).

Run:
    python test_gcode_parser_verify.py

This is NOT a replacement for the on-target C self-test
(gcode_parser_self_test.c).  It's a host-side logic verification that the
parsing algorithm behaves as expected across all 30 test cases.
"""

import sys
import math

# ---- Error codes (PLAN §4.6) ----
PARSER_OK = 0
PARSER_ERR_UNKNOWN_CMD = 1
PARSER_ERR_RANGE = 2
PARSER_ERR_BAD_PARAM = 4
PARSER_ERR_STORAGE = 5

# ---- Defaults & limits ----
# Test-only fallback initial modal values.  In firmware these come from
# NVS settings (settings_default_feed / settings_default_accel, PLAN §4.3).
DEFAULT_FEED = 300.0
DEFAULT_ACCEL = 200.0
MAX_POS_MM = 1.0e6
MAX_FEED = 30000.0
MIN_FEED = 0.0
MAX_ACCEL = 50000.0
MIN_ACCEL = 0.0
MAX_PARAM_INDEX = 15


class GCodeParser:
    def __init__(self, init_feed_mm_min=DEFAULT_FEED,
                 init_accel_mm_s2=DEFAULT_ACCEL):
        self.absolute_mode = True
        self.modal_feed = init_feed_mm_min
        self.modal_accel = init_accel_mm_s2
        self.current_pos_mm = 0.0
        self.callbacks = {}

    def set_callback(self, name, fn):
        self.callbacks[name] = fn

    def _fire(self, name, *args):
        """Fire a callback.  Returns the callback's return value (or None).
        Integer-returning callbacks (set_param, restore_defaults) use
        the C convention: 0=ok, 4=invalid, 5=storage failure."""
        if name in self.callbacks:
            return self.callbacks[name](*args)
        return None

    def feed(self, line):
        s = line.strip()

        # Empty or comment-only line
        if not s or s[0] == ';' or s[0] == '(':
            return PARSER_OK

        # $ commands
        if s[0] == '$':
            s = s[1:].lstrip()
            if s and s[0] == '$':
                # $$ — list all params
                rest = s[1:].lstrip()
                if rest and rest[0] not in ('\0', ';', '(') and rest != '':
                    # Allow trailing comment only
                    rest2 = rest.split(';', 1)[0].strip()
                    if rest2:
                        return PARSER_ERR_UNKNOWN_CMD
                self._fire('list_params')
                return PARSER_OK

            if s and s[0].isdigit():
                return self._parse_dollar(s)
            return PARSER_ERR_UNKNOWN_CMD

        # G / M commands
        first = s[0].upper()

        if first == 'G':
            rest = s[1:].lstrip()
            num_str = ''
            i = 0
            if i < len(rest) and rest[i] in '+-':
                num_str += rest[i]
                i += 1
            while i < len(rest) and rest[i].isdigit():
                num_str += rest[i]
                i += 1
            if not num_str or not any(c.isdigit() for c in num_str):
                return PARSER_ERR_BAD_PARAM
            gcode = int(num_str)
            rest = rest[i:]

            if gcode == 0:
                return self._parse_move(rest, rapid=True)
            elif gcode == 1:
                return self._parse_move(rest, rapid=False)
            elif gcode == 90:
                rest = rest.strip()
                if rest and rest[0] not in (';', '('):
                    return PARSER_ERR_BAD_PARAM
                self.absolute_mode = True
                self._fire('set_absolute')
                return PARSER_OK
            elif gcode == 91:
                rest = rest.strip()
                if rest and rest[0] not in (';', '('):
                    return PARSER_ERR_BAD_PARAM
                self.absolute_mode = False
                self._fire('set_relative')
                return PARSER_OK
            elif gcode == 92:
                return self._parse_g92(rest)
            else:
                return PARSER_ERR_UNKNOWN_CMD

        elif first == 'M':
            rest = s[1:].lstrip()
            num_str = ''
            i = 0
            while i < len(rest) and rest[i].isdigit():
                num_str += rest[i]
                i += 1
            if not num_str:
                return PARSER_ERR_BAD_PARAM
            mcode = int(num_str)
            rest = rest[i:].strip()

            if rest and rest[0] not in (';', '('):
                return PARSER_ERR_BAD_PARAM

            if mcode == 17:
                self._fire('motor_enable', True)
                return PARSER_OK
            elif mcode in (18, 84):
                self._fire('motor_enable', False)
                return PARSER_OK
            elif mcode == 502:
                # M502 — restore factory defaults.  Does NOT reset the
                # parser's modal F/A runtime values (reboot picks up the
                # new defaults).  Callback returns 0=ok, 5=storage failure.
                rc = self._fire('restore_defaults')
                if rc == 5:
                    return PARSER_ERR_STORAGE
                if rc is not None and rc != 0:
                    return PARSER_ERR_BAD_PARAM
                return PARSER_OK
            elif mcode == 503:
                self._fire('list_params')
                return PARSER_OK
            else:
                return PARSER_ERR_UNKNOWN_CMD

        return PARSER_ERR_UNKNOWN_CMD

    def _strip_comment(self, s):
        """Strip inline ( ) and ; comments, return code-only portion."""
        result = []
        i = 0
        while i < len(s):
            if s[i] == ';':
                break
            if s[i] == '(':
                # find matching ')'
                j = s.find(')', i + 1)
                if j == -1:
                    break
                i = j + 1
                continue
            result.append(s[i])
            i += 1
        return ''.join(result)

    def _parse_float(self, s):
        """Parse float from string, return (value, chars_consumed)."""
        if not s:
            return None, 0
        i = 0
        if s[i] in '+-':
            i += 1
        start = i
        has_digits = False
        while i < len(s) and s[i].isdigit():
            has_digits = True
            i += 1
        if i < len(s) and s[i] == '.':
            i += 1
            while i < len(s) and s[i].isdigit():
                has_digits = True
                i += 1
        if not has_digits:
            return None, 0
        try:
            val = float(s[:i])
            return val, i
        except ValueError:
            return None, 0

    def _parse_move(self, rest, rapid):
        # Strip inline paren comments, then parse words
        code = self._strip_comment(rest)

        has_x = False
        x_val = 0.0
        has_f = False
        f_val = self.modal_feed
        has_a = False
        a_val = self.modal_accel

        i = 0
        while i < len(code):
            # skip whitespace
            while i < len(code) and code[i] in ' \t':
                i += 1
            if i >= len(code):
                break

            letter = code[i].upper()
            i += 1

            # skip whitespace between letter and value
            while i < len(code) and code[i] in ' \t':
                i += 1

            val, consumed = self._parse_float(code[i:])
            if val is None:
                return PARSER_ERR_BAD_PARAM
            i += consumed

            if letter == 'X':
                x_val = val
                has_x = True
            elif letter == 'F':
                f_val = val
                has_f = True
            elif letter == 'A':
                a_val = val
                has_a = True
            else:
                return PARSER_ERR_BAD_PARAM

        if not has_x:
            return PARSER_ERR_BAD_PARAM

        # Range checks
        if abs(x_val) > MAX_POS_MM:
            return PARSER_ERR_RANGE
        if not (MIN_FEED < f_val <= MAX_FEED):
            return PARSER_ERR_RANGE
        if not (MIN_ACCEL < a_val <= MAX_ACCEL):
            return PARSER_ERR_RANGE

        # Compute target
        if self.absolute_mode:
            target = x_val
        else:
            target = self.current_pos_mm + x_val

        if abs(target) > MAX_POS_MM:
            return PARSER_ERR_RANGE

        # Update modal state
        if has_f:
            self.modal_feed = f_val
        if has_a:
            self.modal_accel = a_val

        self.current_pos_mm = target
        self._fire('move', target, self.modal_feed, self.modal_accel, rapid)
        return PARSER_OK

    def _parse_g92(self, rest):
        code = self._strip_comment(rest)
        has_x = False
        x_val = 0.0

        i = 0
        while i < len(code):
            while i < len(code) and code[i] in ' \t':
                i += 1
            if i >= len(code):
                break
            letter = code[i].upper()
            i += 1
            while i < len(code) and code[i] in ' \t':
                i += 1
            val, consumed = self._parse_float(code[i:])
            if val is None:
                return PARSER_ERR_BAD_PARAM
            i += consumed
            if letter == 'X':
                x_val = val
                has_x = True
            else:
                return PARSER_ERR_BAD_PARAM

        if not has_x:
            return PARSER_ERR_BAD_PARAM
        if abs(x_val) > MAX_POS_MM:
            return PARSER_ERR_RANGE

        self.current_pos_mm = x_val
        self._fire('set_origin', x_val)
        return PARSER_OK

    def _parse_dollar(self, s):
        # parse index
        idx_str = ''
        i = 0
        if i < len(s) and s[i] in '+-':
            idx_str += s[i]
            i += 1
        while i < len(s) and s[i].isdigit():
            idx_str += s[i]
            i += 1
        if not idx_str or not any(c.isdigit() for c in idx_str):
            return PARSER_ERR_BAD_PARAM
        idx = int(idx_str)

        rest = s[i:].lstrip()

        if rest and rest[0] == '=':
            rest = rest[1:].lstrip()
            val, consumed = self._parse_float(rest)
            if val is None:
                return PARSER_ERR_BAD_PARAM
            rest2 = rest[consumed:].strip()
            # Allow trailing comment
            if rest2 and rest2[0] not in (';', '('):
                rest3 = rest2.split(';', 1)[0].strip()
                if rest3:
                    return PARSER_ERR_BAD_PARAM

            if idx < 0 or idx > MAX_PARAM_INDEX:
                return PARSER_ERR_RANGE

            # Callback returns 0=ok, 4=invalid, 5=storage failure
            rc = self._fire('set_param', idx, val)
            if rc == 4:
                return PARSER_ERR_BAD_PARAM
            if rc == 5:
                return PARSER_ERR_STORAGE
            if rc is not None and rc != 0:
                return PARSER_ERR_BAD_PARAM
            return PARSER_OK

        # Bare "$<n>" with no '=' — invalid.  ($<n>? was removed in T11:
        # '?' never reaches the parser — the protocol layer intercepts
        # it as a real-time character, PLAN §4.4.)
        return PARSER_ERR_BAD_PARAM


# ================================================================
# Test framework
# ================================================================

total = 0
passed = 0
failed = 0


def run_test(name, fn):
    global total, passed, failed
    total += 1
    try:
        fn()
        passed += 1
        print(f"  {name:<55} PASS")
    except AssertionError as e:
        failed += 1
        print(f"  {name:<55} FAIL: {e}")
    except Exception as e:
        failed += 1
        print(f"  {name:<55} ERROR: {e}")


def assert_eq(a, b, msg=""):
    if a != b:
        raise AssertionError(f"{msg}: expected {b}, got {a}")


def assert_float_eq(a, b, eps=0.001, msg=""):
    if abs(a - b) > eps:
        raise AssertionError(f"{msg}: expected {b}, got {a} (eps {eps})")


# ---- Test recorder ----

class Recorder:
    def __init__(self):
        self.events = []
        # Configurable return values for integer-returning callbacks
        # (mirrors C stub behavior in gcode_parser_self_test.c)
        self.set_param_return = 0
        self.restore_return = 0

    def clear(self):
        self.events = []

    def on_move(self, target, feed, accel, rapid):
        self.events.append(('move', target, feed, accel, rapid))

    def on_set_origin(self, pos):
        self.events.append(('set_origin', pos))

    def on_set_absolute(self):
        self.events.append(('set_absolute',))

    def on_set_relative(self):
        self.events.append(('set_relative',))

    def on_motor_enable(self, enable):
        self.events.append(('motor_enable', enable))

    def on_list_params(self):
        self.events.append(('list_params',))

    def on_set_param(self, idx, val):
        self.events.append(('set_param', idx, val))
        return self.set_param_return

    def on_restore_defaults(self):
        self.events.append(('restore_defaults',))
        return self.restore_return


def make_parser():
    p = GCodeParser()
    r = Recorder()
    p.set_callback('move', r.on_move)
    p.set_callback('set_origin', r.on_set_origin)
    p.set_callback('set_absolute', r.on_set_absolute)
    p.set_callback('set_relative', r.on_set_relative)
    p.set_callback('motor_enable', r.on_motor_enable)
    p.set_callback('list_params', r.on_list_params)
    p.set_callback('set_param', r.on_set_param)
    p.set_callback('restore_defaults', r.on_restore_defaults)
    return p, r


# ================================================================
# 30 test cases
# ================================================================

def test_g0_basic_absolute():
    p, r = make_parser()
    st = p.feed("G0 X100")
    assert_eq(st, PARSER_OK)
    assert_eq(len(r.events), 1)
    assert_eq(r.events[0][0], 'move')
    assert_float_eq(r.events[0][1], 100.0)
    assert_eq(r.events[0][4], True)  # rapid


def test_g1_with_f_and_a():
    p, r = make_parser()
    st = p.feed("G1 X50 F1200 A500")
    assert_eq(st, PARSER_OK)
    assert_eq(len(r.events), 1)
    assert_float_eq(r.events[0][1], 50.0)
    assert_float_eq(r.events[0][2], 1200.0)
    assert_float_eq(r.events[0][3], 500.0)
    assert_eq(r.events[0][4], False)
    assert_float_eq(p.modal_feed, 1200.0)
    assert_float_eq(p.modal_accel, 500.0)


def test_modal_f_a_persistence():
    p, r = make_parser()
    p.feed("G1 X10 F100 A200")
    r.clear()
    st = p.feed("G1 X20")
    assert_eq(st, PARSER_OK)
    assert_eq(len(r.events), 1)
    assert_float_eq(r.events[0][1], 20.0)
    assert_float_eq(r.events[0][2], 100.0)
    assert_float_eq(r.events[0][3], 200.0)


def test_modal_f_update():
    p, r = make_parser()
    p.feed("G1 X10 F500")
    assert_float_eq(p.modal_feed, 500.0)
    assert_float_eq(p.modal_accel, DEFAULT_ACCEL)
    r.clear()
    p.feed("G1 X20 A1000")
    assert_float_eq(r.events[0][2], 500.0)
    assert_float_eq(r.events[0][3], 1000.0)
    assert_float_eq(p.modal_accel, 1000.0)


def test_g91_relative_mode():
    p, r = make_parser()
    st = p.feed("G91")
    assert_eq(st, PARSER_OK)
    assert_eq(p.absolute_mode, False)
    assert_eq(r.events[0][0], 'set_relative')
    r.clear()
    p.feed("G1 X10 F100")
    assert_float_eq(r.events[0][1], 10.0)
    r.clear()
    p.feed("G1 X5")
    assert_float_eq(r.events[0][1], 15.0)
    r.clear()
    p.feed("G90")
    assert_eq(p.absolute_mode, True)
    assert_eq(r.events[0][0], 'set_absolute')


def test_g92_set_origin():
    p, r = make_parser()
    p.feed("G1 X100 F100")
    assert_float_eq(p.current_pos_mm, 100.0)
    r.clear()
    st = p.feed("G92 X50")
    assert_eq(st, PARSER_OK)
    assert_eq(r.events[0][0], 'set_origin')
    assert_float_eq(r.events[0][1], 50.0)
    assert_float_eq(p.current_pos_mm, 50.0)


def test_negative_x_relative():
    p, r = make_parser()
    p.feed("G91")
    p.feed("G1 X50 F100")
    assert_float_eq(p.current_pos_mm, 50.0)
    r.clear()
    p.feed("G1 X-20")
    assert_float_eq(r.events[0][1], 30.0)


def test_decimal_values():
    p, r = make_parser()
    st = p.feed("G1 X12.345 F123.45 A67.89")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[0][1], 12.345)
    assert_float_eq(r.events[0][2], 123.45)
    assert_float_eq(r.events[0][3], 67.89)


def test_no_space_format():
    p, r = make_parser()
    st = p.feed("G1X50F1200")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[0][1], 50.0)
    assert_float_eq(r.events[0][2], 1200.0)


def test_mixed_case():
    p, r = make_parser()
    st = p.feed("g1 x10 f100 a200")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[0][1], 10.0)
    assert_float_eq(r.events[0][2], 100.0)
    assert_float_eq(r.events[0][3], 200.0)


def test_whitespace_tolerant():
    p, r = make_parser()
    st = p.feed("   G1\tX5\tF100   ")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[0][1], 5.0)
    assert_float_eq(r.events[0][2], 100.0)


def test_comment_line():
    p, r = make_parser()
    st = p.feed("; this is a comment")
    assert_eq(st, PARSER_OK)
    assert_eq(len(r.events), 0)


def test_empty_line():
    p, r = make_parser()
    assert_eq(p.feed(""), PARSER_OK)
    assert_eq(len(r.events), 0)
    assert_eq(p.feed("   \t  "), PARSER_OK)
    assert_eq(len(r.events), 0)


def test_inline_comment_semicolon():
    p, r = make_parser()
    st = p.feed("G1 X10 ; move to position 10")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[0][1], 10.0)


def test_inline_paren_comment():
    p, r = make_parser()
    st = p.feed("G1 X(hello)10 F100")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[0][1], 10.0)
    assert_float_eq(r.events[0][2], 100.0)
    st = p.feed("G1 X (hello) 20")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[1][1], 20.0)
    st = p.feed("G1 X5 F (hi) 200")
    assert_eq(st, PARSER_OK)
    assert_float_eq(r.events[2][1], 5.0)
    assert_float_eq(r.events[2][2], 200.0)


def test_unknown_gcode():
    p, r = make_parser()
    st = p.feed("G42 X10")
    assert_eq(st, PARSER_ERR_UNKNOWN_CMD)
    assert_eq(len(r.events), 0)


def test_unknown_mcode():
    p, r = make_parser()
    st = p.feed("M999")
    assert_eq(st, PARSER_ERR_UNKNOWN_CMD)
    assert_eq(len(r.events), 0)


def test_g1_missing_x():
    p, r = make_parser()
    st = p.feed("G1 F100")
    assert_eq(st, PARSER_ERR_BAD_PARAM)
    assert_eq(len(r.events), 0)


def test_bad_numeric():
    p, r = make_parser()
    st = p.feed("G1 X10abc")
    assert_eq(st, PARSER_ERR_BAD_PARAM)
    assert_eq(len(r.events), 0)


def test_double_decimal():
    p, r = make_parser()
    st = p.feed("G1 X1.2.3")
    assert_eq(st, PARSER_ERR_BAD_PARAM)
    assert_eq(len(r.events), 0)


def test_x_out_of_range():
    p, r = make_parser()
    # 2,000,000 mm > 1e6 mm max → error:2 (range)
    st = p.feed("G1 X2000000")
    assert_eq(st, PARSER_ERR_RANGE)
    assert_eq(len(r.events), 0)


def test_f_out_of_range():
    p, r = make_parser()
    assert_eq(p.feed("G1 X10 F0"), PARSER_ERR_RANGE)
    assert_eq(p.feed("G1 X10 F-5"), PARSER_ERR_RANGE)


def test_a_out_of_range():
    p, r = make_parser()
    st = p.feed("G1 X10 A99999")
    assert_eq(st, PARSER_ERR_RANGE)
    assert_eq(len(r.events), 0)


def test_g92_missing_x():
    p, r = make_parser()
    st = p.feed("G92")
    assert_eq(st, PARSER_ERR_BAD_PARAM)
    assert_eq(len(r.events), 0)


def test_param_index_out_of_range():
    p, r = make_parser()
    assert_eq(p.feed("$16=100"), PARSER_ERR_RANGE)
    assert_eq(p.feed("$100=100"), PARSER_ERR_RANGE)


def test_m17_m18_motor_enable():
    p, r = make_parser()
    assert_eq(p.feed("M17"), PARSER_OK)
    assert_eq(r.events[0], ('motor_enable', True))
    r.clear()
    assert_eq(p.feed("M18"), PARSER_OK)
    assert_eq(r.events[0], ('motor_enable', False))
    r.clear()
    assert_eq(p.feed("M84"), PARSER_OK)
    assert_eq(r.events[0], ('motor_enable', False))


def test_dollar_dollar_list():
    p, r = make_parser()
    assert_eq(p.feed("$$"), PARSER_OK)
    assert_eq(r.events[0][0], 'list_params')


def test_dollar_set_param_and_return():
    # rc 0 → PARSER_OK; rc 4 → error:4; rc 5 → error:5 (C convention)
    p, r = make_parser()
    r.set_param_return = 0
    assert_eq(p.feed("$3=42.5"), PARSER_OK)
    assert_eq(r.events[0][0], 'set_param')
    assert_eq(r.events[0][1], 3)
    assert_float_eq(r.events[0][2], 42.5)

    r.clear()
    r.set_param_return = 4
    assert_eq(p.feed("$3=999"), PARSER_ERR_BAD_PARAM)

    r.clear()
    r.set_param_return = 5
    assert_eq(p.feed("$3=999"), PARSER_ERR_STORAGE)


def test_m502_restore_defaults():
    # rc 0 → PARSER_OK; modal F/A not reset; rc 5 → error:5
    p, r = make_parser()
    p.feed("G1 X10 F1200 A800")
    r.clear()

    r.restore_return = 0
    assert_eq(p.feed("M502"), PARSER_OK)
    assert_eq(r.events[0][0], 'restore_defaults')
    # Modal values survive M502 (reboot picks up new defaults)
    assert_float_eq(p.modal_feed, 1200.0)
    assert_float_eq(p.modal_accel, 800.0)

    r.clear()
    r.restore_return = 5
    assert_eq(p.feed("M502"), PARSER_ERR_STORAGE)


def test_m503_and_bare_dollar_n():
    # M503 → list_params; bare $5 (no '=') → error:4
    # ($n? syntax removed in T11 — '?' intercepted by protocol layer)
    p, r = make_parser()
    assert_eq(p.feed("M503"), PARSER_OK)
    assert_eq(r.events[0][0], 'list_params')

    r.clear()
    assert_eq(p.feed("$5"), PARSER_ERR_BAD_PARAM)
    assert_eq(p.feed("$5?"), PARSER_ERR_BAD_PARAM)


# ================================================================
# Main
# ================================================================

def main():
    print("G-code Parser — Python Logic Verification")
    print("=========================================\n")

    print("Basic commands:")
    run_test("G0 basic absolute", test_g0_basic_absolute)
    run_test("G1 with F and A", test_g1_with_f_and_a)

    print("\nModal semantics:")
    run_test("Modal F/A persistence", test_modal_f_a_persistence)
    run_test("Modal F update (A unchanged)", test_modal_f_update)

    print("\nCoordinate modes:")
    run_test("G91 relative + G90 switch", test_g91_relative_mode)
    run_test("G92 set origin", test_g92_set_origin)
    run_test("Negative X in relative mode", test_negative_x_relative)

    print("\nFormat variations:")
    run_test("Decimal values", test_decimal_values)
    run_test("No-space format G1X50F1200", test_no_space_format)
    run_test("Mixed case g1 x10 f100", test_mixed_case)
    run_test("Whitespace + tabs tolerant", test_whitespace_tolerant)

    print("\nComments & empty lines:")
    run_test("Comment-only line", test_comment_line)
    run_test("Empty / whitespace-only line", test_empty_line)
    run_test("Inline ; comment", test_inline_comment_semicolon)
    run_test("Inline (paren) comment", test_inline_paren_comment)

    print("\nError paths:")
    run_test("Unknown G-code → error:1", test_unknown_gcode)
    run_test("Unknown M-code → error:1", test_unknown_mcode)
    run_test("G1 missing X → error:4", test_g1_missing_x)
    run_test("Bad numeric (letters) → error:4", test_bad_numeric)
    run_test("Double decimal → error:4", test_double_decimal)
    run_test("X out of range → error:2", test_x_out_of_range)
    run_test("F out of range → error:2", test_f_out_of_range)
    run_test("A out of range → error:2", test_a_out_of_range)
    run_test("G92 missing X → error:4", test_g92_missing_x)
    run_test("Param index out of range → error:2", test_param_index_out_of_range)

    print("\nM-codes & parameters:")
    run_test("M17/M18/M84 motor enable", test_m17_m18_motor_enable)
    run_test("$$ list params", test_dollar_dollar_list)
    run_test("$n=v set param + rc propagation", test_dollar_set_param_and_return)
    run_test("M502 restore defaults", test_m502_restore_defaults)
    run_test("M503 + bare $n → error:4", test_m503_and_bare_dollar_n)

    print(f"\n=========================================")
    print(f"Total: {total}  Passed: {passed}  Failed: {failed}")

    if failed > 0:
        print("\n*** SOME TESTS FAILED ***")
        return 1
    else:
        print("\nAll 30 tests passed!")
        return 0


if __name__ == '__main__':
    sys.exit(main())
