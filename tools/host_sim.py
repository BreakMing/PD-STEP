#!/usr/bin/env python3
"""T12 - host simulator and end-to-end acceptance over the UART1 protocol.

Sends G-code, parses ``ok`` / ``error:<n>`` / ``<STATE|C|E|V|T>`` frames /
``$n=<value>`` lines, runs the eight PLAN T12 acceptance cases and writes a
Markdown report (default ``docs/acceptance.md``).

Wiring (board side, docs/hardware.md 3):
    CH343P TX -> board GPIO42 (UART1 RX)
    CH343P RX -> board GPIO41 (UART1 TX)
    GND       -> GND

The board MUST run the NORMAL firmware (every CONFIG_*_SELF_TEST and
CONFIG_APP_*_TEST off).  Test builds mirror ESP_LOG to UART1, which would
interleave log text with the protocol stream; the parser below tolerates
that defensively, but the acceptance premise is a clean channel.

Usage:
    python tools/host_sim.py COM7 --phase 1        # cases 1-6, 8 + case 7 step 1
    python tools/host_sim.py COM7 --phase 2        # after the power cycle
    python tools/host_sim.py COM7 --skip-power-cycle --fast

Exit code: 0 if every executed case passed, 1 otherwise.
"""

import argparse
import datetime
import os
import re
import sys
import time

try:
    import serial
except ImportError:                      # environment guard
    sys.exit("pyserial is required: pip install pyserial")

try:                                     # never crash on a legacy console
    sys.stdout.reconfigure(errors="replace")
except (AttributeError, ValueError):     # code page that cannot encode the
    pass                                 # Chinese report strings


# ================================================================ constants

BAUD = 115200
DEFAULT_REPORT = os.path.join("docs", "acceptance.md")
EXPECTED_PARAMS = 14

# Protocol line grammar (PLAN 4.2 / 4.5 / 5)
RE_OK = re.compile(r"^ok$")
RE_ERROR = re.compile(r"^error:(\d+)$")
RE_FRAME = re.compile(
    r"^<(?P<state>[A-Z]+)\|C:(?P<c>-?\d+\.\d+)\|E:(?P<e>-?\d+\.\d+)"
    r"\|V:(?P<v>-?\d+\.\d+)\|T:(?P<t>[A-Z0-9]+)>$")
RE_PARAM = re.compile(r"^\$(\d+)=(-?\d+(?:\.\d+)?)$")
RE_PHASE1_MARK = re.compile(
    r"<!--\s*host_sim:phase1\s+\$5_original=(?P<orig>-?\d+\.\d+)"
    r"\s+\$5_set=(?P<set>-?\d+\.\d+)\s*-->")

# Judgement tolerances
TOL_POS = 0.05          # mm - commanded position vs target
TOL_ENC = 0.10          # mm - encoder / |C-E| deviation (PLAN T12 case 3)

# Motion parameters (mm domain, inside the default soft limits)
SETUP_TRAVEL = 300.0    # bring the axis to mid-travel, then declare it
SETUP_FEED = 3000.0
SETUP_ACCEL = 100.0

STREAM_FEEDS = (1200.0, 1800.0)
STREAM_ACCELS = (200.0, 500.0)
STREAM_STEP = 2.0       # mm per segment (+2 / -2 alternating)

LONG_TRAVEL = 120.0     # case 4 - long move (slow feed => ~12 s of motion)
LONG_FEED = 600.0
LONG_ACCEL = 100.0

TELEM_TRAVEL = 40.0     # case 8 - long move for a 2 s frame window (~4 s)
TELEM_FEED = 600.0
TELEM_ACCEL = 200.0
TELEM_WINDOW_S = 2.0
TELEM_SILENCE_S = 1.0


# ================================================================ exceptions

class Timeout(Exception):
    """No expected protocol line arrived before the deadline."""


class ProtocolError(Exception):
    """The board answered with an error: line where success was required."""


# ================================================================ serial I/O

class Board:
    """Line-oriented, log-tolerant client for the UART1 protocol."""

    def __init__(self, port):
        self.port = port
        self.noise = []                 # skipped non-protocol lines
        self.noise_count = 0
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.3)
        except (serial.SerialException, ValueError, OSError) as exc:
            sys.exit(
                f"cannot open {port}: {exc}\n"
                f"  - use the CH343P port (not the on-board USB port)\n"
                f"  - close any other terminal / VOFA+ instance holding it\n"
                f"  - check the port name, e.g. COM7")
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- raw ----

    def send(self, text):
        """Write raw bytes (no line terminator added)."""
        self.ser.write(text.encode("ascii"))

    def send_line(self, cmd):
        self.send(cmd + "\r\n")

    def drain_input(self, settle_s=0.05):
        """Discard buffered RX bytes, twice, with a settle pause between.

        Used before measuring "standing still must emit nothing": while the
        axis was moving, monitor_task emitted frames at $9 Hz and wait_idle()
        only consumes one line per poll, so the tail of the move can still be
        in flight.  Draining does NOT mask a real defect — a firmware that
        emitted frames at standstill would still be counted afterwards.
        """
        self.ser.reset_input_buffer()
        time.sleep(settle_s)
        self.ser.reset_input_buffer()

    def _read_raw(self):
        line = self.ser.readline()
        if not line:
            return None
        return line.decode("ascii", "replace").rstrip("\r\n")

    @staticmethod
    def classify(line):
        if RE_OK.match(line):
            return "ok", None
        m = RE_ERROR.match(line)
        if m:
            return "error", int(m.group(1))
        m = RE_FRAME.match(line)
        if m:
            return "frame", {
                "state": m.group("state"),
                "c": float(m.group("c")),
                "e": float(m.group("e")),
                "v": float(m.group("v")),
                "t": m.group("t"),
                "raw": line,
            }
        m = RE_PARAM.match(line)
        if m:
            return "param", (int(m.group(1)), float(m.group(2)))
        return "noise", line

    def read_event(self, deadline):
        """Next protocol event before `deadline`, else None.

        Non-protocol lines (ESP_LOG mirror text, boot banner, ...) are
        skipped and counted - see T12 fact #1.
        """
        while time.monotonic() < deadline:
            line = self._read_raw()
            if not line:
                continue
            kind, val = self.classify(line)
            if kind == "noise":
                self.noise_count += 1
                if len(self.noise) < 20:
                    self.noise.append(line)
                continue
            return kind, val
        return None

    # ---- protocol helpers ----

    def expect(self, cmd, timeout_s=3.0):
        """Send `cmd`, return ('ok', None) / ('error', code).

        Status frames arriving from an ongoing move are skipped (they are
        valid protocol traffic, just not the response we are waiting for).
        """
        self.send_line(cmd)
        deadline = time.monotonic() + timeout_s
        while True:
            ev = self.read_event(deadline)
            if ev is None:
                raise Timeout(cmd)
            if ev[0] in ("ok", "error"):
                return ev

    def expect_ok(self, cmd, timeout_s=3.0):
        kind, val = self.expect(cmd, timeout_s)
        if kind != "ok":
            raise ProtocolError(f"{cmd} -> error:{val}")
        return val

    def query(self, timeout_s=1.5):
        """Send '?' and return the next status frame dict."""
        self.send("?")
        deadline = time.monotonic() + timeout_s
        while True:
            ev = self.read_event(deadline)
            if ev is None:
                raise Timeout("?")
            if ev[0] == "frame":
                return ev[1]

    def wait_idle(self, timeout_s=20.0, poll_s=0.05):
        """Poll '?' until STATE == IDLE.  Returns the frame, or None."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.query()
            if frame["state"] == "IDLE":
                return frame
            time.sleep(poll_s)
        return None

    def params(self, timeout_s=3.0):
        """Send '$$' and return {index: value} from the $n=... reply block."""
        self.send_line("$$")
        deadline = time.monotonic() + timeout_s
        out = {}
        while True:
            ev = self.read_event(deadline)
            if ev is None:
                raise Timeout("$$")
            if ev[0] == "param":
                out[ev[1][0]] = ev[1][1]
            elif ev[0] == "ok":
                return out
            elif ev[0] == "error":
                raise ProtocolError(f"$$ -> error:{ev[1]}")


# ================================================================ results

R = {}          # case key -> result dict
SKIPPED = []    # (case key, reason)


def record(key, name, criterion, ok, measured, evidence=()):
    R[key] = {
        "name": name,
        "criterion": criterion,
        "measured": measured,
        "verdict": "PASS" if ok else "FAIL",
        "evidence": list(evidence),
    }
    print(f"  {'PASS' if ok else 'FAIL'} case {key} - {name}: {measured}")
    for line in evidence:
        print(f"        {line}")


def skip(key, name, reason):
    SKIPPED.append((key, name, reason))


def case_failed(key, name, criterion, exc):
    record(key, name, criterion, False, f"异常：{exc}")


# ================================================================ setup

def probe(board):
    """Cheap liveness check with a friendly failure message."""
    try:
        board.expect_ok("G90", timeout_s=2.0)
    except Timeout:
        board.close()
        sys.exit(
            "板卡无应答 —— 请检查：\n"
            "  - 接线：CH343P TX->GPIO42、RX->GPIO41、GND<->GND\n"
            "  - 板上烧的是正常固件（所有测试开关关闭）\n"
            "  - UART1 波特率（$11）为 115200，且板卡已上电")


def read_params(board):
    ps = board.params()
    if len(ps) != EXPECTED_PARAMS:
        print(f"  WARNING: $$ returned {len(ps)} parameter lines, "
              f"expected {EXPECTED_PARAMS}")
    return ps


def fmt_params(ps):
    return " ".join(f"${k}={ps[k]:.3f}" for k in sorted(ps))


def setup_axis(board, ps):
    """M17 + G92 X0 + a move to mid-travel, then declare that point.

    Every later case works around the declared mid-travel coordinate, which
    keeps all targets inside [$12, $13] and the physical travel inside the
    600 mm rail (T12 5 preamble / 附 A-7).
    """
    soft_min, soft_max = ps.get(12, 0.0), ps.get(13, 600.0)
    print(f"  参数：{fmt_params(ps)}")
    print(f"  注意：开始前滑台须位于行程左端 —— setup 会先向 +{SETUP_TRAVEL:.0f} mm "
          f"方向移动，再把该处声明为中段坐标")

    board.expect_ok("M17")
    board.expect_ok("G92 X0")                       # declare current point 0
    board.expect_ok(f"G1 X{SETUP_TRAVEL:.3f} F{SETUP_FEED:.0f} "
                    f"A{SETUP_ACCEL:.0f}")
    f = board.wait_idle(30.0)
    if f is None:
        raise Timeout("setup move did not finish")
    board.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")     # mid-travel = SETUP_TRAVEL
    f = board.query()
    print(f"  setup 完成：C={f['c']:.3f} E={f['e']:.3f} mm"
          f"（声明 {SETUP_TRAVEL:.0f}，软限位 [{soft_min:.0f}, {soft_max:.0f}]）")
    return f


# ================================================================ case 1

def case1_flow_control(board):
    key, name = "1", "ok 流控窗口（连续 20 行不读响应）"
    criterion = "20 行注释 → 恰好 20 个 ok、零丢失、无 error:*"
    n = 20
    board.ser.reset_input_buffer()
    t0 = time.monotonic()
    for _ in range(n):
        board.send_line(";")            # legal, answered immediately
    oks, errs, frames, timeouts = 0, [], 0, 0
    deadline = time.monotonic() + 5.0
    while oks < n and time.monotonic() < deadline:
        ev = board.read_event(deadline)
        if ev is None:
            timeouts += 1
            break
        if ev[0] == "ok":
            oks += 1
        elif ev[0] == "error":
            errs.append(ev[1])
        elif ev[0] == "frame":
            frames += 1
    dt = time.monotonic() - t0
    ok = (oks == n) and not errs
    record(key, name, criterion, ok,
           f"{oks}/{n} ok，用时 {dt:.2f} s，errors={errs or '无'}，"
           f"超时={'是' if timeouts else '否'}",
           [f"连续发送 {n} 行不读取（固件窗口 16 行）→ 收到 {oks} 个 ok",
            f"空闲突发期间意外状态帧：{frames} 个"])


# ================================================================ cases 2+3

def case2_3_stream(board, fast):
    n = 100 if fast else 1000
    key2, name2 = "2", f"{n} 段混合 F/A 流式执行"
    crit2 = "每段都回 ok，无 error:*，无超时"
    key3, name3 = "3", "每段静止后 C 与 E 偏差（无失步判据）"
    crit3 = f"每段 |C-E| ≤ {TOL_ENC:.1f} mm，且无随段数递增趋势"

    board.expect_ok("G91")
    devs, fails = [], []
    t0 = time.monotonic()

    for i in range(n):
        sign = 1.0 if (i % 2 == 0) else -1.0
        feed = STREAM_FEEDS[i % 2]
        accel = STREAM_ACCELS[(i // 2) % 2]
        cmd = (f"G1 X{sign * STREAM_STEP:.3f} F{feed:.0f} A{accel:.0f}")
        try:
            board.expect_ok(cmd, timeout_s=5.0)
        except (Timeout, ProtocolError) as exc:
            fails.append(f"segment {i}: {exc}")
            break
        frame = board.wait_idle(15.0)
        if frame is None:
            fails.append(f"segment {i}: never returned to IDLE")
            break
        devs.append(abs(frame["c"] - frame["e"]))
        if devs[-1] > TOL_ENC:
            fails.append(f"segment {i}: |C-E|={devs[-1]:.4f} mm")

    board.expect_ok("G90")
    dt = time.monotonic() - t0
    done = len(devs)

    record(key2, name2, crit2, (done == n) and not fails,
           f"{done}/{n} 段全部 ok，用时 {dt:.1f} s"
           + (f"，首个失败：{fails[0]}" if fails else ""),
           [f"F ∈ {STREAM_FEEDS} mm/min，A ∈ {STREAM_ACCELS} mm/s²，"
            f"步长 ±{STREAM_STEP:.0f} mm，G91 相对模式"])

    if not devs:
        record(key3, name3, crit3, False, "未采集到段数据")
        return

    worst = max(devs)
    worst_i = devs.index(worst)
    decile = max(1, len(devs) // 10)
    first_max = max(devs[:decile])
    last_max = max(devs[-decile:])
    trend_bad = (last_max > 2.0 * first_max) and (last_max > 0.05)
    ok = (worst <= TOL_ENC) and not fails and not trend_bad
    record(key3, name3, crit3, ok,
           f"最大 |C-E|={worst:.4f} mm（第 {worst_i}/{len(devs)} 段）"
           + ("，疑似丢步：偏差随段数增长" if trend_bad else ""),
           [f"首 10% 段最大 {first_max:.4f} mm，末 10% 段最大 {last_max:.4f} mm，"
            f"平均 {sum(devs) / len(devs):.4f} mm",
            f"超差段数：{sum(1 for d in devs if d > TOL_ENC)}",
            "判据只在上位机侧（D6）：固件只上报、不做判断"])


# ================================================================ case 4

def case4_immediate_stop(board, ps):
    key, name = "4", "'!' 立即停并从冻结位置继续"
    criterion = ("C 冻结（500 ms 后不变）、显著小于被取消的目标，"
                 "后续命令从冻结位置正常执行")
    ev = []
    board.expect_ok("G90")
    cur = board.query()
    soft_max = ps.get(13, 600.0)
    target = min(cur["c"] + LONG_TRAVEL, soft_max - 20.0)

    board.expect_ok(f"G1 X{target:.3f} F{LONG_FEED:.0f} A{LONG_ACCEL:.0f}")
    time.sleep(1.2)
    moving = board.query()
    ev.append(f"运动中：{moving['raw']}")
    board.send("!")                       # real-time char, no line terminator
    stopped = board.wait_idle(8.0)
    if stopped is None:
        record(key, name, criterion, False, "'!' 后未回到 IDLE", ev)
        return
    c_stop = stopped["c"]

    time.sleep(0.5)
    again = board.query()
    ev.append(f"C 冻结值 {c_stop:.3f} mm，500 ms 后再查 {again['c']:.3f} mm"
              f"（差 {abs(again['c'] - c_stop):.3f}）")

    if c_stop > 50.0:
        nxt = 50.0
    else:
        nxt = c_stop + 20.0
    e0 = again["e"]
    board.expect_ok(f"G1 X{nxt:.3f} F1200")
    final = board.wait_idle(30.0)
    if final is None:
        record(key, name, criterion, False, "恢复命令未完成", ev)
        return
    travel_want = abs(nxt - c_stop)
    travel_got = abs(final["e"] - e0)
    ev.append(f"被取消的目标 {target:.3f} mm，实际停位 {c_stop:.3f} mm"
              f"（少走 {target - c_stop:.1f} mm）")
    ev.append(f"恢复命令：G1 X{nxt:.3f} → C={final['c']:.3f} mm，"
              f"E 位移 {travel_got:.2f} mm（期望 {travel_want:.2f} mm）")

    ok = (moving["state"] == "RUN"
          and c_stop < target - 20.0
          and abs(again["c"] - c_stop) <= 0.001   # frozen
          and abs(final["c"] - nxt) <= TOL_POS    # reached the target
          and abs(travel_got - travel_want) <= 1.0)  # no runaway
    record(key, name, criterion, ok,
           f"停在 {c_stop:.3f} mm（目标 {target:.3f}），"
           f"冻结={abs(again['c'] - c_stop) <= 0.001}，"
           f"恢复后 C={final['c']:.3f}（目标 {nxt:.3f}）",
           ev)


# ================================================================ case 5

def timed_segment(board, cmd, timeout_s=20.0):
    t0 = time.monotonic()
    board.expect_ok(cmd)
    frame = board.wait_idle(timeout_s)
    t1 = time.monotonic()
    if frame is None:
        raise Timeout(f"{cmd}: never returned to IDLE")
    return t1 - t0, frame


def case5_modal_fa(board):
    key, name = "5", "F/A 模态沿用"
    criterion = ("段1（显式 F600）≈ 段2（省略 F/A，沿用模态），"
                 "段3（F2400）< 段2 的 60%")
    ev = []
    board.expect_ok("G90")
    board.expect_ok("G92 X0")

    t1, f1 = timed_segment(board, "G1 X20 F600 A100")
    t2, f2 = timed_segment(board, "G1 X40")            # modal F600 / A100
    t3, f3 = timed_segment(board, "G1 X60 F2400")      # F explicit, A modal
    ev.append(f"段1：G1 X20 F600 A100 → {t1:.2f} s（C={f1['c']:.3f}）")
    ev.append(f"段2：G1 X40（省略 F/A）  → {t2:.2f} s（C={f2['c']:.3f}）")
    ev.append(f"段3：G1 X60 F2400        → {t3:.2f} s（C={f3['c']:.3f}）")

    modal_same = abs(t1 - t2) <= 0.25 * max(t1, t2)
    faster = t3 < 0.6 * t2
    reached = (abs(f1["c"] - 20.0) <= TOL_POS
               and abs(f2["c"] - 40.0) <= TOL_POS
               and abs(f3["c"] - 60.0) <= TOL_POS)
    ok = modal_same and faster and reached
    record(key, name, criterion, ok,
           f"t1={t1:.2f} s，t2={t2:.2f} s（比值 {t2 / t1:.2f}），"
           f"t3={t3:.2f} s（t2 的 {100.0 * t3 / t2:.0f}%）",
           ev + ["模态生效以耗时对比间接判定：协议无 F/A 读回接口"
                 "（用例 5 偏差说明）"])


# ================================================================ case 6

def case6_g92_shift(board):
    key, name = "6", "G92 建零后 C/E 同步平移"
    criterion = ("G92 X0 后 C/E 均≈0；G92 X123.456 后均≈123.456；"
                 "|C-E| 始终 ≤ 0.1 mm")
    f0 = board.query()
    board.expect_ok("G92 X0")
    f1 = board.query()
    board.expect_ok("G92 X123.456")
    f2 = board.query()

    dc = f1["c"] - f0["c"]
    de = f1["e"] - f0["e"]
    ev = [f"G92 前：      C={f0['c']:.3f} E={f0['e']:.3f}",
          f"G92 X0 后：   C={f1['c']:.3f} E={f1['e']:.3f} "
          f"（平移 dC={dc:.3f} dE={de:.3f}）",
          f"G92 X123.456：C={f2['c']:.3f} E={f2['e']:.3f} "
          f"（|C-E|={abs(f2['c'] - f2['e']):.3f}）"]

    ok = (abs(f1["c"]) <= TOL_POS and abs(f1["e"]) <= TOL_ENC
          and abs(f2["c"] - 123.456) <= TOL_POS
          and abs(f2["e"] - 123.456) <= TOL_ENC
          and abs(f2["c"] - f2["e"]) <= TOL_ENC
          and abs(dc - de) <= TOL_ENC)
    record(key, name, criterion, ok,
           f"G92 X0 后 C/E={f1['c']:.3f}/{f1['e']:.3f}，"
           f"G92 X123.456 后 {f2['c']:.3f}/{f2['e']:.3f}",
           ev)


# ================================================================ case 7

def case7_nvs_phase1(board):
    key, name = "7", "掉电重启参数保持（NVS）"
    criterion = "阶段 1 写入的 $5 经真实断电后严格相等"
    ps = read_params(board)
    original = ps[5]
    target = 1400.0 if abs(original - 1500.0) < 1e-6 else 1500.0
    board.expect_ok(f"$5={target:.3f}")
    ps2 = board.params()
    ok = abs(ps2[5] - target) < 1e-6

    record(key, name, criterion + "【阶段 1 · 写入侧】", ok,
           f"$5 {original:.3f} → {target:.3f}，$$ 复核通过",
           [f"报告已写入跨阶段标记：original={original:.3f} "
            f"set={target:.3f}",
            "阶段 2 必须在真实断电后运行——G92 / 软复位无法证明 NVS 持久性"])
    return original, target


def case7_nvs_phase2(board, report_path):
    key, name = "7", "掉电重启参数保持（NVS）"
    criterion = "$5 断电重启后 == 阶段 1 写入值"
    try:
        with open(report_path, "r", encoding="utf-8") as fh:
            text = fh.read()
    except OSError as exc:
        sys.exit(f"无法读取 {report_path}：{exc} —— 请先运行 --phase 1")

    m = RE_PHASE1_MARK.search(text)
    if not m:
        sys.exit(f"{report_path} 中没有阶段 1 记录 —— 断电前请先用"
                 f"同一个 --report 路径运行 --phase 1")
    original = float(m.group("orig"))
    target = float(m.group("set"))

    ps = read_params(board)
    survived = abs(ps[5] - target) < 1e-6
    record(key, name, criterion + "【阶段 2 · 读取侧】", survived,
           f"断电重启后 $5={ps[5]:.3f}（阶段 1 写入 {target:.3f}）",
           [f"重启后全部参数：{fmt_params(ps)}"])

    board.expect_ok(f"$5={original:.3f}")
    ps2 = board.params()
    restored = abs(ps2[5] - original) < 1e-6
    print(f"  已恢复 $5={ps2[5]:.3f}（原值 {original:.3f}）"
          f"{'ok' if restored else '失败'}")
    R[key]["evidence"].append(
        f"已恢复 $5 为 {original:.3f}"
        f"（{'$$ 复核通过' if restored else '复核未通过'}）")


# ================================================================ case 8

def case8_telemetry(board, ps):
    key, name = "8", "遥测帧频率、格式与 T 旗标"
    hz = int(round(ps.get(9, 10.0)))
    criterion = (f"运动中 ≈{hz} 帧/秒（±2）、每帧符合 §4.5 语法且 STATE=RUN、"
                 f"T=OK；静止 1 s 无自发帧")
    ev = []
    board.expect_ok("G90")
    board.expect_ok("G92 X0")
    cur = board.query()
    target = cur["c"] + TELEM_TRAVEL
    board.expect_ok(f"G1 X{target:.3f} F{TELEM_FEED:.0f} A{TELEM_ACCEL:.0f}")

    # --- 2 s silent capture window (receive only) ---
    t0 = time.monotonic()
    frames, others = [], []
    end = t0 + TELEM_WINDOW_S
    while time.monotonic() < end:
        ev_ = board.read_event(end + 0.05)
        if ev_ is None:
            break
        (frames if ev_[0] == "frame" else others).append(ev_[1])
    window = time.monotonic() - t0

    run_frames = [f for f in frames if f["state"] == "RUN"]
    flag_bad = [f for f in frames if f["t"] != "OK"]
    ev.append(f"{window:.2f} s 内收到 {len(frames)} 帧"
              f"（{len(frames) / window:.1f} Hz，$9={hz} Hz）")
    if frames:
        ev.append(f"首帧={frames[0]['raw']}  末帧={frames[-1]['raw']}")
    ev.append(f"STATE=RUN：{len(run_frames)}/{len(frames)}，"
              f"T≠OK 的帧：{len(flag_bad)}"
              + (f"（{[f['t'] for f in flag_bad][:3]}）" if flag_bad else ""))
    ev.append(f"窗口内非帧协议行：{len(others)}")

    end_frame = board.wait_idle(30.0)
    if end_frame is None:
        record(key, name, criterion, False,
               "运动未结束，无法核对静止帧", ev)
        return

    # --- idle: 1 s with no traffic must produce no frame ---
    # First drop frames still queued from the tail of the move (see
    # Board.drain_input) so only *newly* emitted frames are counted.
    board.drain_input()
    t1 = time.monotonic()
    idle_frames = 0
    while time.monotonic() < t1 + TELEM_SILENCE_S:
        ev_ = board.read_event(t1 + TELEM_SILENCE_S)
        if ev_ is None:
            break
        if ev_[0] == "frame":
            idle_frames += 1
    ev.append(f"静止 {TELEM_SILENCE_S:.0f} s 内自发帧：{idle_frames} 个"
              f"（必须为 0，D12）")
    alive = board.query()
    ev.append(f"静止时 '?' 应答：{alive['raw']}"
              f"（运动结束后 {time.monotonic() - t1:.2f} s）")

    count_ok = abs(len(frames) - hz * window) <= 2.0
    ok = (count_ok
          and frames
          and len(run_frames) == len(frames)
          and not flag_bad
          and idle_frames == 0
          and end_frame["state"] == "IDLE")
    record(key, name, criterion, ok,
           f"{window:.2f} s 内 {len(frames)} 帧（期望 {hz * window:.0f}±2），"
           f"RUN={len(run_frames)}/{len(frames)}，静止自发帧={idle_frames}",
           ev)


# ================================================================ report

def write_report(path, header_lines, results, skipped, footer="", mode="w"):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    total = sum(1 for r in results.values() if r["verdict"] in ("PASS", "FAIL"))
    passed = sum(1 for r in results.values() if r["verdict"] == "PASS")

    lines = []
    if mode == "a":
        lines.append("")
    else:
        lines.append("# 端到端验收报告（T12 / host_sim.py）")
        lines.append("")
    lines.extend(header_lines)
    lines.append("")
    lines.append("## 结果表")
    lines.append("")
    lines.append("| 用例 | 判据 | 实测 | 结论 |")
    lines.append("|---|---|---|---|")
    for key in sorted(results):
        r = results[key]
        lines.append(f"| {key}. {r['name']} | {r['criterion']} | "
                     f"{r['measured']} | {r['verdict']} |")
    for key, name, reason in skipped:
        lines.append(f"| {key}. {name} | - | {reason} | SKIP |")
    lines.append("")
    lines.append("## 证据摘录")
    lines.append("")
    for key in sorted(results):
        r = results[key]
        lines.append(f"### 用例 {key} · {r['name']} — {r['verdict']}")
        lines.append("")
        for item in r["evidence"]:
            lines.append(f"- {item}")
        lines.append("")
    lines.append(f"## 汇总：TOTAL {passed}/{total} PASS")
    lines.append("")
    if skipped:
        lines.append("### 未执行项")
        lines.append("")
        for key, name, reason in skipped:
            lines.append(f"- 用例 {key} {name}：{reason}")
        lines.append("")
    if footer:
        lines.append(footer)
        lines.append("")

    with open(path, mode, encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def phase1_marker(original, target):
    return (f"<!-- host_sim:phase1 $5_original={original:.3f} "
            f"$5_set={target:.3f} -->")


# ================================================================ main

def run_phase1(board, args):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[host_sim] T12 端到端验收 · {board.port} · {now}")
    probe(board)
    ps = read_params(board)

    header = [
        f"- 日期时间：{now}",
        f"- 串口：{board.port} @ {BAUD}-8N1（UART1 协议通道）",
        f"- 运行参数：phase=1 fast={'是' if args.fast else '否'} "
        f"断电项={'跳过' if args.skip_power_cycle else '待执行'}"
        + (f" 仅运行={args.only}" if args.only else ""),
        f"- 参数快照（运行期读 `$$`，未硬编码）：`{fmt_params(ps)}`",
        "- 环境：正常固件（所有测试开关关闭）",
        f"- 协议行过滤：跳过非协议行 {board.noise_count} 行"
        + (f"（示例：{board.noise[:2]}）" if board.noise else
           "（UART1 纯净，未见日志镜像）"),
    ]

    try:
        setup_axis(board, ps)
        cases = [
            ("1", lambda: case1_flow_control(board)),
            ("2/3", lambda: case2_3_stream(board, args.fast)),
            ("4", lambda: case4_immediate_stop(board, ps)),
            ("5", lambda: case5_modal_fa(board)),
            ("6", lambda: case6_g92_shift(board)),
            ("8", lambda: case8_telemetry(board, ps)),
        ]
        meta = {"1": ("1", "ok 流控窗口"),
                "2/3": ("2/3", "流式执行 + C/E 偏差"),
                "4": ("4", "立即停"),
                "5": ("5", "F/A 模态"),
                "6": ("6", "G92 平移"),
                "8": ("8", "遥测")}

        # --only LIST : run a subset (unselected cases are simply absent from
        # the report, so a partial run still reports TOTAL n/n PASS when the
        # selected ones all pass).
        want = None
        if args.only:
            want = {s for s in re.split(r"[,\s]+", args.only.strip()) if s}
            if "2" in want or "3" in want:
                want.add("2/3")
            print(f"  仅运行用例：{sorted(want)}")

        for tag, fn in cases:
            if want is not None and tag not in want:
                continue
            try:
                fn()
            except (Timeout, ProtocolError) as exc:
                key, name = meta[tag]
                if tag == "2/3":
                    case_failed("2", name, "见任务包 §5.2", exc)
                    case_failed("3", name, "见任务包 §5.3", exc)
                else:
                    case_failed(key, name, "见任务包 §5", exc)
                print(f"  FAIL case {tag}: {exc}")

        if args.skip_power_cycle:
            skip("7", "掉电重启参数保持（NVS）",
                 "--skip-power-cycle 指定跳过")
        elif want is None or "7" in want:
            original, target = case7_nvs_phase1(board)
            print()
            print("*** 需要人工操作 **********************************************")
            print("  1. 断开 12V 电源与 USB 线")
            print("  2. 等待 5 秒")
            print("  3. 重新上电，并重新插上本串口")
            print(f"  4. 运行：python tools/host_sim.py {board.port} --phase 2")
            print("*************************************************************")
            print()
            header.append("- 用例 7 阶段 1 完成，等待人工断电重启后运行 "
                          "`--phase 2`")
            header.append(phase1_marker(original, target))
    finally:
        header[5] = (f"- 协议行过滤：跳过非协议行 {board.noise_count} 行"
                     + (f"（示例：{board.noise[:2]}）" if board.noise else
                        "（UART1 纯净，未见日志镜像）"))
        write_report(args.report, header, R, SKIPPED)
        print(f"[host_sim] 报告已写入 {args.report}")

    total = sum(1 for r in R.values() if r["verdict"] in ("PASS", "FAIL"))
    passed = sum(1 for r in R.values() if r["verdict"] == "PASS")
    print(f"TOTAL: {passed}/{total} PASS")
    return 0 if (passed == total and not SKIPPED) else 1


def run_phase2(board, args):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[host_sim] T12 端到端验收 · 阶段 2 · {board.port} · {now}")
    probe(board)
    case7_nvs_phase2(board, args.report)
    header = [
        "",
        "---",
        "",
        f"## 阶段 2（断电重启后）",
        "",
        f"- 日期时间：{now}",
        f"- 串口：{board.port} @ {BAUD}-8N1",
    ]
    write_report(args.report, header, R, SKIPPED, mode="a")
    print(f"[host_sim] 阶段 2 结果已追加到 {args.report}")
    ok = all(r["verdict"] == "PASS" for r in R.values())
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description="T12 host simulator / 端到端验收脚本（UART1 协议）")
    ap.add_argument("port", help="CH343P 串口，例如 COM7")
    ap.add_argument("--skip-power-cycle", action="store_true",
                    help="跳过用例 7（NVS 掉电重启）")
    ap.add_argument("--phase", choices=["1", "2"],
                    help="1 = 用例 1-6、8 + 用例 7 写入侧（默认）；"
                         "2 = 断电重启后验证用例 7 读取侧")
    ap.add_argument("--fast", action="store_true",
                    help="用例 2 由 1000 段缩短为 100 段（快速自检）")
    ap.add_argument("--only", metavar="LIST",
                    help="只运行指定用例（逗号分隔，如 8 或 4,5,8；用例 2/3 用 2 或 3）")
    ap.add_argument("--report", default=DEFAULT_REPORT,
                    help=f"报告输出路径（默认 {DEFAULT_REPORT}）")
    args = ap.parse_args()

    board = Board(args.port)
    try:
        if args.phase == "2":
            return run_phase2(board, args)
        return run_phase1(board, args)
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())
