#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
max_accel_run — 最大加速度实测执行器（方案：docs/max-accel-exploration-plan.md）

依赖：pip install pyserial；与 cartpole_host.py 同目录运行（复用 Rig 串口会话）。

子命令：
  calib          引导两点标定（人工推车到两端，各按回车确认）
  smoke          冒烟段：ACC_MAX 5000、50 mm/s 一次往返
  matrix         实验矩阵：4 速度点 × 6 加速度档 × 3 段往返（可参数化）
  step-response  VEL 阶跃响应记录（τ / 实际加速度测量，回填 sim2real-spec）

用法示例：
  python max_accel_run.py --port COM7 calib
  python max_accel_run.py --port COM7 matrix
  python max_accel_run.py --port COM7 matrix --speeds 50 --accs 5000 10000
  python max_accel_run.py --port COM7 step-response

产出（--out 目录，默认 results/maxaccel-<启动时间>/）：
  raw.log            全部串口行（带相对时间戳）
  segments.csv       每段：速度、加速度、段序、判定、净偏移、备注
  step-response.csv  阶跃记录：t、x_um、vx_mm_s
  summary.txt        可行域矩阵与组合判定

判定规则（方案「判定规则」节）：
  FAULT                                        → FAIL
  |往返净偏移| > 300 µm（12 步）                → FAIL
  |净偏移| ≤ 100 µm                            → PASS
  100–300 µm                                   → MARGIN
  组合判定：3 段中 FAIL ≥ 2 或 MARGIN ≥ 2 → 不可行，PASS ≥ 2 → 可行，
  其余（PASS 1 + FAIL 1 + MARGIN 1 等）→ 存疑按不可行处理（保守侧）。
"""

import argparse
import csv
import os
import sys
import time
from collections import deque

from cartpole_host import Rig

# ---- 方案固定值（docs/max-accel-exploration-plan.md） ----

SPEEDS_DEFAULT = [50, 120, 300, 550]          # mm/s
ACCS_DEFAULT = [5000, 10000, 15000, 20000, 25000, 30000]  # mm/s²
REPS_DEFAULT = 3                               # 每组合段数

OFFSET_PASS_UM = 100    # 净偏移 ≤ 此值判 PASS
OFFSET_FAIL_UM = 300    # 净偏移 > 此值判 FAIL（300 µm = 12 步 @25 µm/step）

FEED_PERIOD_S = 0.05    # 喂狗周期（WDT_MS 默认 100 ms）
VX_NOISE_SIGMA = 5      # 固件速度估计静止噪声指标 σ ≤ 5 mm/s
VX_SETTLED = 2 * VX_NOISE_SIGMA   # 停稳判据 |vx| ≤ 2σ（协议指标）
SETTLE_RUN = 8          # 连续拍数（@50 ms pump ≈ 0.4 s）

# 每速度点：单方向目标位移（mm）、按最不利档（最低加速度）的巡航保持时间（s）
LEG_PROFILE = {
    50:  (60,  1.20),
    120: (80,  0.64),
    300: (100, 0.27),
    550: (150, 0.16),
}


# ---- 纯逻辑（单元测试覆盖） ----

def classify_offset(offset_um, faulted):
    """单段判定：FAULT 优先；净偏移按 100/300 µm 三分。"""
    if faulted:
        return "FAIL"
    a = abs(offset_um)
    if a > OFFSET_FAIL_UM:
        return "FAIL"
    if a <= OFFSET_PASS_UM:
        return "PASS"
    return "MARGIN"


def combine_verdicts(verdicts):
    """组合判定：多数 FAIL 或多数 MARGIN → 不可行；多数 PASS → 可行；
    三段各占一种 → 存疑，按不可行处理（保守侧）。返回 (ok, label)。"""
    n = len(verdicts)
    if verdicts.count("FAIL") * 2 > n or verdicts.count("MARGIN") * 2 > n:
        return False, "不可行"
    if verdicts.count("PASS") * 2 > n:
        return True, "可行"
    return False, "存疑（按不可行）"


def reached(vx_mm_s, target_v):
    """加速到达判据：固件速度估计带噪声（σ≤5）与 ≤10 ms 群延迟，
    容差取 max(2σ, 目标的 5%)。"""
    return abs(vx_mm_s - target_v) <= max(2 * VX_NOISE_SIGMA, target_v * 0.05)


def settled_run(vx_samples, run=SETTLE_RUN):
    """停稳判定：连续 run 个样本 |vx| ≤ 2σ。输入为最近样本序列（旧→新），
    list 或 deque 均可（deque 不支持切片，先转 list）。"""
    if len(vx_samples) < run:
        return False
    return all(abs(v) <= VX_SETTLED for v in list(vx_samples)[-run:])


def leg_times(v, acc, hold_s):
    """超时预算：斜坡时间 v/acc（s，均为 mm 单位制）加裕量。"""
    ramp = v / float(acc)
    return ramp + 0.5, ramp + 1.0   # (到达超时, 停稳超时)


def hold_time(v):
    """该速度点的巡航保持时间（方案单段时序表）。未知速度点线性内插。"""
    if v in LEG_PROFILE:
        return LEG_PROFILE[v][1]
    ks = sorted(LEG_PROFILE)
    lo, hi = ks[0], ks[-1]
    if v <= lo:
        return LEG_PROFILE[lo][1]
    if v >= hi:
        return LEG_PROFILE[hi][1]
    for a, b in zip(ks, ks[1:]):
        if a <= v <= b:
            fa, fb = LEG_PROFILE[a][1], LEG_PROFILE[b][1]
            return fa + (fb - fa) * (v - a) / (b - a)
    raise ValueError(v)


# ---- 串口会话扩展 ----

class TestRig(Rig):
    """Rig 之上加：带时间戳状态采样、FAULT 事件、原始行日志。"""

    def __init__(self, port, baud=460800, log_path=None):
        super().__init__(port, baud)
        self.samples = deque(maxlen=600000)   # (t, x_um, theta_urad, vx, vth)
        self.fault_events = []                # [(t, reason)]
        self._t0 = time.monotonic()
        self._log = open(log_path, "a", encoding="ascii") if log_path else None

    def now(self):
        return time.monotonic() - self._t0

    def _dispatch(self, line):
        if self._log:
            self._log.write("%.3f %s\n" % (self.now(), line))
        super()._dispatch(line)
        parts = line.split()
        if len(parts) == 5 and parts[0] == "STATE":
            try:
                x, th, vx, vth = (int(p) for p in parts[1:])
            except ValueError:
                return
            self.samples.append((time.monotonic(), x, th, vx, vth))
        elif len(parts) == 3 and parts[0] == "MODE" and parts[1] == "FAULT":
            self.fault_events.append((time.monotonic(), parts[2]))

    def last_state_tuple(self):
        """最近一帧 (x_um, vx_mm_s)；无样本返回 None。"""
        if not self.samples:
            return None
        _, x, _, vx, _ = self.samples[-1]
        return x, vx

    def close(self):
        if self._log:
            self._log.flush()
            self._log.close()
        super().close()


# ---- 会话操作 ----

def ensure_ready(rig, timeout=5.0):
    m = rig.query()
    if m and m[0] == "READY":
        return True
    print("当前模式 %s，非 READY——先完成标定与 ENABLE（calib 子命令）" % (m,))
    return False


def set_config(rig, name, value):
    rig.send_line("CONFIG %s %d" % (name, value))
    rig.pump(0.5)
    r = rig.last_reply
    if r and r[0] == "OK" and r[1] == name:
        return True
    print("CONFIG %s %d 未确认（%s）" % (name, value, r))
    return False


def recover_from_fault(rig):
    """FAULT → RESET → ENABLE（RAM 标定保留，protocol.md 操作流程第 8 步）。"""
    rig.send_line("RESET")
    m = rig.wait_mode("DISABLED", 3.0)
    if m is None:
        print("RESET 未回到 DISABLED（当前 %s）" % (rig.last_mode,))
        return False
    rig.send_line("ENABLE")
    m = rig.wait_mode("READY", 5.0)
    if m is None:
        print("恢复失败：ENABLE 超时（当前 %s）" % (rig.last_mode,))
        return False
    return True


def move_to_center(rig, band_um=2000, v_slow=100, timeout_s=20.0):
    """以低速把小车移回原点附近（标定原点 = 两标中点），给下一组合干净起点。"""
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        st = rig.last_state_tuple()
        if st is None:
            rig.pump(0.05)
            continue
        x, vx = st
        if abs(x) <= band_um and abs(vx) <= VX_SETTLED:
            rig.send_line("VEL 0")
            rig.pump(0.2)
            return True
        v = -v_slow if x > 0 else v_slow
        rig.send_line("VEL %d" % v)
        rig.pump(FEED_PERIOD_S)
        if rig.last_mode and rig.last_mode[0] == "FAULT":
            print("回中途中 FAULT（%s）" % (rig.last_mode[1],))
            return recover_from_fault(rig)
    print("回中超时（当前位置 %s）" % (rig.last_state_tuple() and rig.last_state_tuple()[0],))
    return False


# ---- 段执行 ----

def run_leg(rig, v, hold_s, acc):
    """单程：加速到 ±v → 巡航 hold_s → VEL 0 停稳。
    返回 (x_start_um, x_end_um, faulted, settled, note)。"""
    n_fault0 = len(rig.fault_events)
    st = rig.last_state_tuple()
    x_start = st[0] if st else None
    note = ""
    settled = False

    # 加速 + 巡航：每拍喂狗；到达判定带超时兜底（vx 估计异常时不无限等待）
    reach_to, _ = leg_times(v, acc, hold_s)
    t_reach_deadline = time.monotonic() + reach_to
    reached_flag = False
    t_hold_end = None
    while True:
        rig.send_line("VEL %d" % v)
        rig.pump(FEED_PERIOD_S)
        if len(rig.fault_events) > n_fault0:
            return x_start, None, True, False, "FAULT: %s" % rig.fault_events[-1][1]
        st = rig.last_state_tuple()
        if st is None:
            continue
        _, vx = st
        if not reached_flag:
            if reached(vx, abs(v)):
                reached_flag = True
                t_hold_end = time.monotonic() + hold_s
            elif time.monotonic() > t_reach_deadline:
                reached_flag = True
                t_hold_end = time.monotonic() + hold_s
                note = "reach-timeout"
        elif time.monotonic() >= t_hold_end:
            break

    # 减速停稳
    _, stop_to = leg_times(v, acc, hold_s)
    t_stop_deadline = time.monotonic() + stop_to
    vx_run = deque(maxlen=SETTLE_RUN)
    while time.monotonic() < t_stop_deadline:
        rig.send_line("VEL 0")
        rig.pump(FEED_PERIOD_S)
        if len(rig.fault_events) > n_fault0:
            return x_start, None, True, False, "FAULT: %s" % rig.fault_events[-1][1]
        st = rig.last_state_tuple()
        if st is None:
            continue
        vx_run.append(st[1])
        if settled_run(vx_run):
            settled = True
            break
    else:
        note = (note + ";" if note else "") + "settle-timeout"

    st = rig.last_state_tuple()
    x_end = st[0] if st else None
    return x_start, x_end, False, settled, note


def run_segment(rig, v, acc):
    """一个试探段 = 对称往返两程。返回 dict（CSV 行）。"""
    hold = hold_time(v)
    legs = []
    for direction in (+1, -1):
        r = run_leg(rig, direction * v, hold, acc)
        legs.append(r)
        if r[2]:   # FAULT，段终止
            break
    x0, x1 = legs[0][0], legs[-1][1]
    faulted = any(l[2] for l in legs)
    offset = (x1 - x0) if (x0 is not None and x1 is not None) else None
    verdict = classify_offset(offset if offset is not None else 0, faulted)
    notes = ";".join(l[4] for l in legs if l[4])
    return {
        "v_mm_s": v, "acc_mm_s2": acc, "offset_um": offset,
        "verdict": verdict, "faulted": faulted, "notes": notes,
    }


# ---- 子命令 ----

def cmd_calib(rig, args):
    input("把小车推到最左端，停稳后按回车发 MARK LEFT …")
    rig.send_line("MARK LEFT")
    rig.pump(1.0)
    r = rig.last_reply
    if not (r and r[0] == "OK"):
        print("MARK LEFT 被拒：%s" % (r,))
        return 1
    left = r[2]
    input("把小车推到最右端，停稳后按回车发 MARK RIGHT …")
    rig.send_line("MARK RIGHT")
    rig.pump(1.0)
    r = rig.last_reply
    if not (r and r[0] == "OK"):
        print("MARK RIGHT 被拒：%s" % (r,))
        return 1
    print("标定行程 L = %d µm（%.1f mm），回执已入 raw.log" % (abs(r[2] - left),
                                                          abs(r[2] - left) / 1000.0))
    rig.send_line("ENABLE")
    m = rig.wait_mode("READY", 5.0)
    if m is None:
        print("ENABLE 超时（%s）" % (rig.last_mode,))
        return 1
    print("READY。可运行 matrix / step-response / smoke。")
    return 0


def cmd_smoke(rig, args):
    if not ensure_ready(rig):
        return 1
    if not set_config(rig, "ACC_MAX", 5000):
        return 1
    seg = run_segment(rig, 50, 5000)
    print("冒烟段：%s（净偏移 %s µm，备注 %s）" %
          (seg["verdict"], seg["offset_um"], seg["notes"] or "无"))
    return 0 if seg["verdict"] == "PASS" else 1


def cmd_matrix(rig, args):
    if not ensure_ready(rig):
        return 1
    rows = []
    consecutive_all_fail = 0
    aborted = False
    for v in args.speeds:
        if not move_to_center(rig):
            aborted = True
            break
        for acc in args.accs:
            if not set_config(rig, "ACC_MAX", acc):
                aborted = True
                break
            verdicts = []
            for rep in range(args.reps):
                seg = run_segment(rig, v, acc)
                seg.update(seq=rep + 1)
                rows.append(seg)
                print("v=%-3d acc=%-5d 段%d：%-6s 偏移=%-7s %s" %
                      (v, acc, rep + 1, seg["verdict"],
                       seg["offset_um"] if seg["offset_um"] is not None else "—",
                       seg["notes"] or ""))
                verdicts.append(seg["verdict"])
                if seg["faulted"]:
                    if not recover_from_fault(rig):
                        aborted = True
                        break
                    if not move_to_center(rig):
                        aborted = True
                        break
            if aborted:
                break
            ok, label = combine_verdicts(verdicts)
            print("  组合 v=%d acc=%d → %s" % (v, acc, label))
            if all(x == "FAIL" for x in verdicts):
                consecutive_all_fail += 1
                if consecutive_all_fail >= 2:
                    print("连续 2 个组合全部失败，按方案安全规则自动中止矩阵。")
                    aborted = True
            else:
                consecutive_all_fail = 0
        if aborted:
            break
    write_segments_csv(os.path.join(args.outdir, "segments.csv"), rows)
    write_summary(os.path.join(args.outdir, "summary.txt"), rows, args, aborted)
    print("结果已写入 %s" % args.outdir)
    if aborted:
        print("矩阵提前中止（数据保留已跑部分）。")
    return 0


def cmd_step_response(rig, args):
    """0 → v → 0 阶跃，记录 vx(t)（τ 与实际加速度分析依据）。"""
    if not ensure_ready(rig):
        return 1
    if not set_config(rig, "ACC_MAX", 5000):
        return 1
    v = args.step_v
    path = os.path.join(args.outdir, "step-response.csv")
    with open(path, "w", newline="", encoding="ascii") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_um", "vx_mm_s"])
        for phase_vel, dur in ((0, 0.5), (v, 3.0), (0, 3.0)):
            t_end = time.monotonic() + dur
            while time.monotonic() < t_end:
                rig.send_line("VEL %d" % phase_vel)
                rig.pump(FEED_PERIOD_S)
                st = rig.last_state_tuple()
                if st:
                    w.writerow(["%.3f" % rig.now(), st[0], st[1]])
    print("阶跃记录已写入 %s（分析回填 sim2real-spec a_max/τ）" % path)
    return 0


# ---- 输出 ----

def write_segments_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["v_mm_s", "acc_mm_s2", "seq", "verdict",
                    "offset_um", "faulted", "notes"])
        for r in rows:
            w.writerow([r["v_mm_s"], r["acc_mm_s2"], r.get("seq", ""),
                        r["verdict"], r["offset_um"] if r["offset_um"] is not None else "",
                        int(r["faulted"]), r["notes"]])


def write_summary(path, rows, args, aborted):
    combos = {}
    for r in rows:
        combos.setdefault((r["v_mm_s"], r["acc_mm_s2"]), []).append(r["verdict"])
    with open(path, "w", encoding="utf-8") as f:
        f.write("最大加速度实测结果（VM 20V、800/400 mA、细分 8）\n")
        f.write("日期：%s  中止：%s\n\n" % (time.strftime("%Y-%m-%d %H:%M"),
                                        "是" if aborted else "否"))
        f.write("v\\acc    " + "".join("%-8d" % a for a in args.accs) + "\n")
        for v in args.speeds:
            cells = []
            for a in args.accs:
                vs = combos.get((v, a))
                if not vs:
                    cells.append("%-8s" % "—")
                    continue
                ok, _ = combine_verdicts(vs)
                cells.append("%-8s" % ("可行" if ok else "不可行"))
            f.write("%-8d" % v + "".join(cells) + "\n")
        f.write("\n每速度点最高可行加速度：\n")
        for v in args.speeds:
            best = None
            for a in args.accs:
                vs = combos.get((v, a))
                if vs and combine_verdicts(vs)[0]:
                    best = a
            if best == args.accs[-1]:
                f.write("  %d mm/s：≥ %d mm/s²（软件墙，本矩阵未探到边界）\n" % (v, best))
            elif best is None:
                f.write("  %d mm/s：本矩阵内无可行档\n" % v)
            else:
                f.write("  %d mm/s：%d mm/s²\n" % (v, best))


def main():
    ap = argparse.ArgumentParser(description="最大加速度实测执行器")
    ap.add_argument("--port", required=True, help="串口（如 COM7）")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--out", default=None, help="结果目录（默认 results/maxaccel-<ts>/）")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("calib")
    sub.add_parser("smoke")
    p = sub.add_parser("matrix")
    p.add_argument("--speeds", type=int, nargs="+", default=SPEEDS_DEFAULT)
    p.add_argument("--accs", type=int, nargs="+", default=ACCS_DEFAULT)
    p.add_argument("--reps", type=int, default=REPS_DEFAULT)
    p = sub.add_parser("step-response")
    p.add_argument("--step-v", type=int, default=50)

    args = ap.parse_args()
    outdir = args.out or os.path.join(
        "results", "maxaccel-" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    log_path = os.path.join(outdir, "raw.log")
    rig = TestRig(args.port, args.baud, log_path=log_path)
    table = {"calib": cmd_calib, "smoke": cmd_smoke,
             "matrix": cmd_matrix, "step-response": cmd_step_response}
    args.outdir = outdir
    try:
        rc = table[args.cmd](rig, args)
    except KeyboardInterrupt:
        print("\n用户中断；已跑数据保留在 %s" % outdir)
        rc = 130
    finally:
        rig.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
