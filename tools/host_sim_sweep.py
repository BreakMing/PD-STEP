#!/usr/bin/env python3
"""速度 / 加速度扫描 —— 梯形往复，逐步提速找出失步边界。

对每一组 (F, A)：
  1. 从工作区间下界出发，G92 建零
  2. 满行程往复 cycles 次（每次都是完整的梯形：加速→巡航→减速）
  3. 每次回到起点后读 `?`，记录 |E - 起点| —— 这就是**累积失步量**

判据（两条分开看）：
  * 单程误差 |E目标 - E实测|：含"运动滞后 + 采样时序"，高速下天然变大，
    本身**不是失步**，只作参考。
  * 净误差 |E回起点 - 起点|：往复抵消了滞后，**只剩真实的累积丢步**。
    这是判定失步的主判据。

用法：
    python tools/host_sim_sweep.py COM18                     # 默认阶梯
    python tools/host_sim_sweep.py COM18 --quick             # 少量档位，快速
    python tools/host_sim_sweep.py COM18 --speeds 6000,12000,24000
    python tools/host_sim_sweep.py COM18 --accels 500,2000,8000 --speeds 12000
    python tools/host_sim_sweep.py COM18 --report docs/sweep.md

安全：脚本会自动把 $7/$8 抬高到扫描上限（否则高档位会被钳制），
      结束时恢复原值；`--stop-on-fail`（默认开）在首次失步后立即停止。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from host_sim import Board, Timeout, ProtocolError, fmt_params, probe  # noqa: E402

# 速度阶梯（mm/min）：50 → 500 mm/s，步频 4k → 40k（@k=80）
SPEED_LADDER = [3000, 4500, 6000, 9000, 12000, 15000, 18000, 21000, 24000, 27000, 30000]
QUICK_SPEEDS = [3000, 6000, 12000, 18000]
ACCEL_LADDER = [2000]

ENTRY_FEED, ENTRY_ACCEL = 3000.0, 1000.0   # 入场/复位用的温和参数
LOSS_TOL_MIN_MM = 0.20                     # 累计丢步容差下限（实际取 max(此值, 2×步长)）
STEP_RATE_LIMIT = 40000.0                  # 引擎实际可用的步频门限（超过必然丢步）

BAND_LO, BAND_HI = 75.0, 425.0             # 工作区间，setup 时按软限位重算
UPR = 40.0                                 # $2 units_per_rev（mm/圈），setup 时读取
RESULTS = []


# ================================================================ 工具

def rec(label, ok, measured, evidence=()):
    RESULTS.append((label, ok, measured, list(evidence)))
    print(f"  {'PASS' if ok else 'FAIL'} {label}: {measured}")
    for line in evidence:
        print(f"        {line}")


def clean_state(b):
    """停脉冲 + 排空缓冲 + 等到真 IDLE（避免读到积压的旧帧）。"""
    try:
        b.send("!")
        time.sleep(0.15)
        b.drain_input()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            fr = b.query(timeout_s=1.0)
            if fr["state"] == "IDLE":
                break
        b.drain_input()
    except Exception:
        pass


def move_and_wait(b, target_mm, feed, accel, timeout_s):
    """发一条移动命令，等到真的停下，返回耗时（秒）。"""
    b.drain_input()
    t0 = time.monotonic()
    b.expect_ok(f"G1 X{target_mm:.3f} F{feed:.0f} A{accel:.0f}", timeout_s=timeout_s)
    if b.wait_idle(timeout_s) is None:
        raise Timeout(f"移动未结束（X{target_mm:.0f} F{feed:.0f} A{accel:.0f}）")
    dt = time.monotonic() - t0
    b.drain_input()
    return dt


# ================================================================ setup

def setup_axis(b, ps):
    """走到工作区间中段并声明坐标；区间按实际软限位计算。"""
    global BAND_LO, BAND_HI, UPR

    UPR = ps[2] if ps[2] > 0 else 40.0
    soft_lo, soft_hi = ps[12], ps[13]
    span = soft_hi - soft_lo
    if span < 200.0:
        sys.exit(f"软限位跨度仅 {span:.0f} mm（[{soft_lo:.0f}, {soft_hi:.0f}]），"
                 f"不足以做扫描 —— 请确认 $12/$13 与机械行程一致")
    margin = span * 0.15
    BAND_LO = soft_lo + margin
    BAND_HI = soft_hi - margin
    mid = soft_lo + span / 2.0

    print(f"  软限位 [{soft_lo:.0f}, {soft_hi:.0f}] mm → 扫描区间 "
          f"[{BAND_LO:.0f}, {BAND_HI:.0f}]（单程 {BAND_HI - BAND_LO:.0f} mm，"
          f"首尾各留 {margin:.0f} mm）")
    print(f"  ! 请确认软限位与机械行程一致；扫描会满行程高速往复，"
          f"务必确认机械紧固、行程通畅")

    b.expect_ok("M17")
    b.expect_ok("G92 X0")
    move_and_wait(b, mid, ENTRY_FEED, ENTRY_ACCEL, 60.0)
    b.expect_ok(f"G92 X{mid:.3f}")
    fr = b.query()
    print(f"  setup 完成：C={fr['c']:.3f} E={fr['e']:.3f} mm")


# ================================================================ 单档扫描

def sweep_one(b, feed, accel, cycles, k):
    """一档 (F, A)：满行程往复 cycles 次。

    k = steps/mm（由当前 $0/$1/$2 派生）。容差按步长缩放：
    1 细分时 1 步 = 0.2mm，拿 0.2mm 当容差就等于"允许丢 1 步"，太严。

    判据用**累计位移差**：把每段运动的 |Δ指令| 与 |Δ实测| 分别累加，
    两者之差就是这段时间里丢掉的行程。

    为什么不用"往返净误差"：丢步在每次运动时都会发生，去程丢、回程也丢，
    正反方向的净残差会**互相抵消**（离线测试实测：每次丢 0.2mm，往返净误差恒为 0）。
    累计量只增不减，没有这个盲区。

    返回 (ok, measured, evidence)
    """
    # 入场：温和速度回到起点并建零
    move_and_wait(b, BAND_LO, ENTRY_FEED, ENTRY_ACCEL, 60.0)
    b.expect_ok(f"G92 X{BAND_LO:.3f}")

    fr = b.query()
    prev_c, prev_e = fr["c"], fr["e"]
    total_cmd = total_enc = 0.0
    net_devs, times = [], []
    span = BAND_HI - BAND_LO

    for _ in range(cycles):
        for tgt in (BAND_HI, BAND_LO):
            times.append(move_and_wait(b, tgt, feed, accel, 120.0))
            fr = b.query()
            total_cmd += abs(fr["c"] - prev_c)
            total_enc += abs(fr["e"] - prev_e)
            prev_c, prev_e = fr["c"], fr["e"]
        net_devs.append(abs(prev_e - BAND_LO))

    loss = total_cmd - total_enc            # >0 表示实测走的比指令少（丢步）
    n_moves = cycles * 2
    step_mm = 1.0 / k if k > 0 else 0.0125
    tol = max(LOSS_TOL_MIN_MM, 2.0 * step_mm)
    ok = loss <= tol
    mmps = feed / 60.0
    rpm = mmps / UPR * 60.0
    steps_s = mmps * k
    over_rate = steps_s > STEP_RATE_LIMIT

    measured = (f"F{feed:.0f} ({mmps:.0f} mm/s, {rpm:.0f} rpm, "
                f"{steps_s / 1000:.1f}k 步/s{', ⚠已超引擎门限' if over_rate else ''}) "
                f"A{accel:.0f}：累计丢步 {loss:.3f} mm "
                f"({'无失步' if ok else '★ 超出容差'})")

    avg_t = sum(times) / len(times) if times else 0.0
    speed_txt = (f"{span * 2 / avg_t:.0f} mm/s" if avg_t > 0.01 else "n/a（耗时过短）")
    evidence = [
        f"累计指令位移 {total_cmd:.2f} mm，累计实测位移 {total_enc:.2f} mm "
        f"（{n_moves} 段运动）",
        f"→ 累计丢步 {loss:.3f} mm，平均每次 {loss / n_moves:.4f} mm"
        f"（容差 {tol:.3f} mm = max({LOSS_TOL_MIN_MM:.2f}, 2×{step_mm:.4f} 步长)）",
        f"当前 k={k:.1f} 步/mm（1 步 = {step_mm:.4f} mm），"
        f"步频 {steps_s:.0f} 步/s（引擎门限 {STEP_RATE_LIMIT / 1000:.0f}k）",
        f"回起点的净残差：{', '.join(f'{d:.3f}' for d in net_devs)} mm"
        f"（仅供参考，正反丢步会互相抵消）",
        f"单程 {span:.0f} mm，每段平均 {avg_t:.2f} s → 实测均速 {speed_txt}",
    ]
    return ok, measured, evidence


# ================================================================ 主流程

def run(board, args):
    ps = board.params()
    print(f"[sweep] 速度/加速度扫描 · {board.port}")

    # 可选：先把软限位上限对齐到当前机械行程
    if args.set_soft_max is not None:
        board.expect_ok(f"$13={args.set_soft_max:.0f}")
        ps = board.params()
        print(f"  已将 $13 设为 {ps[13]:.0f}")

    # 可选：切换细分（$0），用于对比不同细分下的可达速度
    if args.microstep is not None:
        board.expect_ok(f"$0={args.microstep:.0f}")
        ps = board.params()
        print(f"  已将 $0 设为 {ps[0]:.0f} 细分（合法值：2^n，1–256）")

    orig_f, orig_a = ps[7], ps[8]        # 抬高之前记下原值，退出时恢复
    orig_u = ps[0]                       # 细分原值

    want_f = max(args.speeds)
    want_a = max(args.accels)

    # 抬高 $7/$8，否则高档位会被 max_feed/max_accel 钳制
    raised = []
    if ps[7] < want_f:
        board.expect_ok(f"$7={want_f:.0f}")
        raised.append(f"$7: {ps[7]:.0f} → {want_f:.0f}")
    if ps[8] < want_a:
        board.expect_ok(f"$8={want_a:.0f}")
        raised.append(f"$8: {ps[8]:.0f} → {want_a:.0f}")
    ps = board.params()
    if raised:
        print(f"  已抬高上限以便扫描：{'; '.join(raised)}")
    print(f"  参数：{fmt_params(ps)}")

    k = (ps[1] * ps[0]) / ps[2] if ps[2] > 0 else 80.0
    tol = max(LOSS_TOL_MIN_MM, 2.0 / k)
    print(f"  k = {k:.1f} 步/mm（$1×$0÷$2 = {ps[1]:.0f}×{ps[0]:.0f}÷{ps[2]:.0f}）"
          f" → 1 步 = {1.0 / k:.4f} mm，一圈 {ps[2]:.1f} mm = {ps[1] * ps[0]:.0f} 步")
    print(f"  累计丢步容差 = max({LOSS_TOL_MIN_MM:.2f}, 2 步长) = {tol:.3f} mm")

    setup_axis(board, ps)

    limit = None
    try:
        for a in args.accels:
            print(f"\n=== 加速度 A{a:.0f} mm/s² ===")
            for f in args.speeds:
                try:
                    ok, measured, evidence = sweep_one(board, f, a, args.cycles, k)
                except (Timeout, ProtocolError) as exc:
                    rec(f"A{a:.0f} F{f:.0f}", False, f"异常：{exc}")
                    clean_state(board)
                    limit = (f, a, f"通信异常：{exc}")
                    if args.stop_on_fail:
                        break
                    continue
                rec(f"A{a:.0f} F{f:.0f}", ok, measured, evidence)
                if not ok:
                    limit = (f, a, f"累计丢步超过 {LOSS_TOL_MM:.2f} mm 容差")
                    if args.stop_on_fail:
                        print(f"\n  → 首次失步出现在 F{f:.0f} / A{a:.0f}，"
                              f"按 --stop-on-fail 停止扫描")
                        break
                clean_state(board)
            if limit and args.stop_on_fail:
                break
    finally:
        # 恢复 $7/$8 原值（无论扫描是否异常中断）
        try:
            if ps[7] != orig_f:
                board.expect_ok(f"$7={orig_f:.0f}")
            if ps[8] != orig_a:
                board.expect_ok(f"$8={orig_a:.0f}")
            if ps[0] != orig_u:
                board.expect_ok(f"$0={orig_u:.0f}")     # 恢复细分
        except Exception:
            pass

    # 恢复现场
    try:
        clean_state(board)
        move_and_wait(board, BAND_LO + (BAND_HI - BAND_LO) / 2.0,
                      ENTRY_FEED, ENTRY_ACCEL, 60.0)
        board.expect_ok("M18")
    except Exception:
        pass

    passed = sum(1 for r in RESULTS if r[1])
    total = len(RESULTS)
    print(f"\n{'=' * 60}")
    print(f"扫描完成：{passed}/{total} 档无失步")
    if limit:
        print(f"失步边界：F{limit[0]:.0f} mm/min（{limit[0] / 60:.0f} mm/s）"
              f" @ A{limit[1]:.0f} mm/s² —— {limit[2]}")
    else:
        print("全部档位均无失步：扫描上限内未触及电机能力边界，可继续提高上限")

    if args.report:
        write_report(args.report, board, ps, passed, total, limit)
        print(f"[sweep] 报告已写入 {args.report}")
    return 0 if passed == total else 1


def write_report(path, board, ps, passed, total, limit):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    lines = [
        "# 速度 / 加速度扫描报告（host_sim_sweep.py）",
        "",
        f"- 时间：{time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 串口：{board.port}",
        f"- 参数快照：`{fmt_params(ps)}`",
        f"- 扫描区间：{BAND_LO:.0f} – {BAND_HI:.0f} mm（单程 {BAND_HI - BAND_LO:.0f} mm）",
        f"- 细分 $0 = {ps[0]:.0f}，k = {(ps[1] * ps[0]) / ps[2]:.1f} 步/mm",
        f"- 累计丢步容差：max({LOSS_TOL_MIN_MM:.2f} mm, 2×步长)",
        "",
        "## 结果",
        "",
        "| 档位 | 实测 | 结论 |",
        "|---|---|---|",
    ]
    for label, ok, measured, _ in RESULTS:
        lines.append(f"| {label} | {measured} | {'PASS' if ok else '**FAIL**'} |")
    lines.append("")
    lines.append("## 明细")
    lines.append("")
    for label, _ok, _m, evidence in RESULTS:
        lines.append(f"### {label}")
        lines.append("")
        for line in evidence:
            lines.append(f"- {line}")
        lines.append("")
    lines.append("## 结论")
    lines.append("")
    if limit:
        lines.append(f"失步边界：**F{limit[0]:.0f} mm/min"
                     f"（{limit[0] / 60:.0f} mm/s）@ A{limit[1]:.0f} mm/s²** —— {limit[2]}")
    else:
        lines.append(f"**全部 {total} 档无失步**：扫描上限内未触及电机能力边界。")
    lines.append("")
    with open(path, "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines))


# ================================================================ main

def main():
    ap = argparse.ArgumentParser(description="速度/加速度扫描（梯形往复，找失步边界）")
    ap.add_argument("port", help="串口名，如 COM18")
    ap.add_argument("--speeds", help="速度列表 mm/min，如 3000,6000,12000")
    ap.add_argument("--accels", help="加速度列表 mm/s²，如 2000,8000")
    ap.add_argument("--cycles", type=int, default=3, help="每档往复次数（默认 3）")
    ap.add_argument("--quick", action="store_true",
                    help=f"快速模式：只用 {QUICK_SPEEDS} mm/min")
    ap.add_argument("--no-stop-on-fail", dest="stop_on_fail",
                    action="store_false", default=True,
                    help="首次失步后继续扫描其余档位（默认停止）")
    ap.add_argument("--report", help="把报告写到该路径，如 docs/sweep.md")
    ap.add_argument("--set-soft-max", type=float, metavar="MM",
                    help="运行前把 $13 设为该值（如 500）并校验；$12 保持不动")
    ap.add_argument("--microstep", type=int, metavar="N",
                    help="运行前把 $0 设为 N 细分（2 的幂，1–256），结束时恢复原值。"
                         "N=1 即全步：k=5 步/mm，步频最低（50mm/s 只要 250 步/s），"
                         "适合探更高的机械速度上限")
    args = ap.parse_args()

    args.speeds = ([float(x) for x in args.speeds.replace(",", " ").split()]
                   if args.speeds else (QUICK_SPEEDS if args.quick else SPEED_LADDER))
    args.accels = ([float(x) for x in args.accels.replace(",", " ").split()]
                   if args.accels else ACCEL_LADDER)

    try:
        board = Board(args.port)
    except Exception as exc:
        print(f"cannot open {args.port}: {exc}")
        print("  - use the CH343P port (not the on-board USB port)")
        print("  - close any other terminal / VOFA+ instance holding it")
        print("  - check the port name with [System.IO.Ports.SerialPort]::GetPortNames()")
        return 1

    try:
        probe(board)
        return run(board, args)
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())
