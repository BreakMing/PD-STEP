#!/usr/bin/env python3
"""T12+ 暴力压力测试 — 极限参数下的稳定性、无失步与恢复能力。

在 host_sim.py 的 8 项验收用例之外，用**拉满的参数**与**随机化负载**再压一轮。
协议层直接复用 host_sim.py（Board / Timeout / ProtocolError），不重复实现。

!!! 安全提示 !!!
本脚本会把速度与加速度拉到 $7/$8 的上限，并反复急启/急停、随机打断、
洪流灌命令。电机与机械承受的压力明显高于常规验收。
运行前请确保：行程通畅、联轴器与同步轮紧固、12V 供电稳定、滑台起始于左端。

运行前提（与 host_sim.py 相同）：
    CH343P TX -> board GPIO42, RX -> board GPIO41, GND <-> GND
    板上为正常固件（所有测试开关关闭）
    滑台初始位于行程左端

用法：
    python tools/host_sim_stress.py COM18                   # 默认档（约 3-4 分钟）
    python tools/host_sim_stress.py COM18 --quick           # 快速档（约 1 分钟）
    python tools/host_sim_stress.py COM18 --duration 600    # 长跑 10 分钟
    python tools/host_sim_stress.py COM18 --skip-long       # 跳过长跑项
    python tools/host_sim_stress.py COM18 --only 5          # 只跑某一项
    python tools/host_sim_stress.py COM18 --report docs/stress.md

退出码：0 = 全部执行项通过，1 = 有失败
"""

import argparse
import os
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from host_sim import Board, Timeout, ProtocolError, fmt_params, probe
except ImportError as exc:                      # pragma: no cover
    sys.exit(f"需要与 host_sim.py 同目录（且已装 pyserial）：{exc}")

try:
    sys.stdout.reconfigure(errors="replace")
except (AttributeError, ValueError):
    pass


# ================================================================ 参数

TOL_POS = 0.05          # mm — C 与目标
TOL_ENC = 0.10          # mm — |C-E|（与 T12 用例 3 同判据）
TOL_TRAVEL = 1.0        # mm — E 位移与期望位移

SETUP_FEED = 3000.0
SETUP_ACCEL = 100.0

# 工作区间与中段位置：由 setup_axis() 依据**实际软限位 $12/$13** 重算
# （首尾各留 15% 余量），因此机械行程 500mm / 600mm 都不需要改本脚本。
SETUP_TRAVEL = 250.0        # 中段（setup 走到这里并声明坐标）
BAND_LO = 75.0              # 大幅往返的下界
BAND_HI = 425.0             # 大幅往返的上界

RESULTS = []            # (name, ok, measured, evidence[])


def rec(name, ok, measured, evidence=()):
    RESULTS.append((name, ok, measured, list(evidence)))
    print(f"  {'PASS' if ok else 'FAIL'} {name}: {measured}")
    for line in evidence:
        print(f"        {line}")


def clean_state(b):
    """用例之间的清场：停脉冲 → 排空积压帧 → 等到真的 IDLE → 再排空。

    必须的：任何一个用例异常退出（例如命令未被正确应答）都会留下
    ① 仍在执行的残留段、② 缓冲区里成百上千条未读的 ok/状态帧。
    前者让引擎一直 busy，后者让后续用例的 `?` 读到**旧帧**而误判位置
    ——两者叠加会污染其后每一个用例。这里把它们彻底清掉。
    """
    try:
        b.send("!")                     # 实时字符，停脉冲、清段队列
        time.sleep(0.15)
        b.drain_input()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            fr = b.query(timeout_s=1.0)
            if fr["state"] == "IDLE":
                break
        b.drain_input()
    except Exception as exc:
        print(f"        （清场时出现异常，已忽略：{exc}）")


# ================================================================ setup

def setup_axis(board, ps):
    """M17 + G92 X0 + 走到中段并声明坐标；工作区间按实际软限位计算。"""
    global SETUP_TRAVEL, BAND_LO, BAND_HI

    soft_lo, soft_hi = ps[12], ps[13]
    span = soft_hi - soft_lo
    if span < 200.0:
        sys.exit(f"软限位跨度仅 {span:.0f} mm（[{soft_lo:.0f}, {soft_hi:.0f}]），"
                 f"不足以做暴力测试 —— 请确认 $12/$13 与机械行程一致")
    margin = span * 0.15
    BAND_LO = soft_lo + margin
    BAND_HI = soft_hi - margin
    SETUP_TRAVEL = soft_lo + span / 2.0

    print(f"  参数：{fmt_params(ps)}")
    print(f"  软限位 [{soft_lo:.0f}, {soft_hi:.0f}] mm（跨度 {span:.0f}）"
          f" → 工作区间 [{BAND_LO:.0f}, {BAND_HI:.0f}]，中段 {SETUP_TRAVEL:.0f}"
          f"（首尾各留 {margin:.0f} mm 余量）")
    print(f"  ! 请确认软限位与机械行程一致 —— 行程 500mm 时 $13 应为 500，"
          f"否则脚本会按过大的区间工作并撞到机械限位")
    print(f"  注意：滑台须位于行程左端 —— setup 会先向 +{SETUP_TRAVEL:.0f} mm 移动")
    board.expect_ok("M17")
    board.expect_ok("G92 X0")
    board.expect_ok(f"G1 X{SETUP_TRAVEL:.3f} F{SETUP_FEED:.0f} A{SETUP_ACCEL:.0f}")
    if board.wait_idle(30.0) is None:
        raise Timeout("setup 移动未结束")
    board.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")
    fr = board.query()
    print(f"  setup 完成：C={fr['c']:.3f} E={fr['e']:.3f} mm")


# ================================================================ 用例

def case1_max_speed(b, ps, rounds):
    """F/A 拉满，在 100–500 mm 间大幅往返。"""
    f_max, a_max = ps[7], ps[8]
    devs, terr = [], []
    b.expect_ok("G90")
    for i in range(rounds):
        tgt = BAND_HI if i % 2 == 0 else BAND_LO
        b.expect_ok(f"G1 X{tgt:.3f} F{f_max:.0f} A{a_max:.0f}", timeout_s=10)
        # 高频查询会在缓冲里留下大量未读的 `?` 响应帧。不清掉的话，下面的
        # wait_idle 按 FIFO 会先读到**旧帧**，可能把运动前的 IDLE 当成当前状态
        # ——这正是首轮跑出 175mm 假偏差的原因（175 = 工作区间跨度）。
        b.drain_input()
        fr = b.wait_idle(30)
        if fr is None:
            return False, f"第 {i} 轮未回到 IDLE", []
        devs.append(abs(fr["c"] - fr["e"]))
        terr.append(abs(fr["c"] - tgt))
    ok = max(terr) <= TOL_POS and max(devs) <= TOL_ENC
    return ok, (f"{rounds} 轮 @ F{f_max:.0f} A{a_max:.0f}，"
                f"最大位置误差 {max(terr):.4f} mm，最大 |C-E| {max(devs):.4f} mm"), [
        f"工作区间 [{BAND_LO:.0f}, {BAND_HI:.0f}] mm，往复 {rounds} 次",
        f"|C-E| 平均 {statistics.fmean(devs):.4f} mm"
        + (f"，末 3 轮 {[round(d, 4) for d in devs[-3:]]}" if len(devs) >= 3 else "")]


def case2_hard_accel(b, ps, rounds):
    """短距离 + A 拉满 → 梯形退化为三角形，考急加减速。"""
    f_max, a_max = ps[7], ps[8]
    accel_steps = 20.0          # mm
    devs = []
    b.expect_ok("G90")
    b.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")
    for i in range(rounds * 2):
        sign = 1.0 if i % 2 == 0 else -1.0
        b.expect_ok(f"G1 X{SETUP_TRAVEL + sign * accel_steps:.3f} "
                    f"F{f_max:.0f} A{a_max:.0f}", timeout_s=10)
        fr = b.wait_idle(30)
        if fr is None:
            return False, f"第 {i} 次未回到 IDLE", []
        devs.append(abs(fr["c"] - fr["e"]))
    ok = max(devs) <= TOL_ENC
    return ok, (f"{rounds * 2} 次 ±{accel_steps:.0f} mm 急加减速，"
                f"最大 |C-E| {max(devs):.4f} mm"), [
        f"A = $8 上限 {a_max:.0f} mm/s²，F = $7 上限 {f_max:.0f} mm/min",
        f"|C-E| 平均 {statistics.fmean(devs):.4f} mm"]


def case3_flood(b, ps, n, batch=10):
    """命令洪流（背压）：分批连发，每批 batch 条不等响应，批间收齐。

    为什么分批到 10 条：固件 UART1 驱动缓冲 256 B，单条命令约 25 B，
    10 条正好压到缓冲边缘并留有余量。**刻意不超限**——超过硬件缓冲导致的
    丢字节是 UART 的物理限制，不是固件缺陷，测它没有意义。
    批内不等响应仍然是"洪流"，达到压背压窗口的目的。
    """
    f_max, a_max = ps[7], ps[8]
    step = 0.0125               # mm = 1 步 @k=80
    b.expect_ok("G91")
    oks, errs = 0, []
    t0 = time.monotonic()
    sent = 0
    while sent < n:
        m = min(batch, n - sent)
        for i in range(m):
            sign = 1.0 if (sent + i) % 2 == 0 else -1.0
            b.send_line(f"G1 X{sign * step:.4f} F{f_max:.0f} A{a_max:.0f}")
        sent += m
        got = 0
        deadline = time.monotonic() + 10.0
        while got < m and time.monotonic() < deadline:
            ev = b.read_event(deadline)
            if ev is None:
                break
            if ev[0] == "ok":
                oks += 1
                got += 1
            elif ev[0] == "error":
                errs.append(ev[1])
                got += 1
    dt = time.monotonic() - t0
    b.expect_ok("G90")
    ok = (oks == n) and not errs
    batches = (n + batch - 1) // batch
    return ok, (f"{oks}/{n} ok（分 {batches} 批，每批 ≤{batch} 条不等响应），"
                f"用时 {dt:.1f} s，errors={errs or '无'}"), [
        f"单条命令 ≈25 B，固件 UART1 驱动缓冲 256 B → 每批 10 条压到边缘",
        f"吞吐 {n / dt:.0f} 条/秒（含每批的等待）"]


def case4_query_storm(b, ps, rounds):
    """运动中高频 `?` 穿插，验证不干扰运动。"""
    f, a = ps[7], ps[8]
    devs, frames = [], 0
    b.expect_ok("G90")
    b.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")
    for i in range(rounds):
        tgt = BAND_LO if i % 2 == 0 else BAND_HI
        b.expect_ok(f"G1 X{tgt:.3f} F{f:.0f} A{a:.0f}", timeout_s=10)
        # 在运动途中高频查询（query() 内部已发送 '?'）
        t_end = time.monotonic() + 0.6
        while time.monotonic() < t_end:
            try:
                b.query(timeout_s=0.5)
                frames += 1
            except Timeout:
                break
        fr = b.wait_idle(30)
        if fr is None:
            return False, f"第 {i} 轮未回到 IDLE（高频查询后）", []
        devs.append(abs(fr["c"] - fr["e"]))
    ok = len(devs) == rounds and max(devs) <= TOL_ENC
    return ok, (f"{rounds} 轮运动中穿插查询，共 {frames} 次 `?`，"
                f"最大 |C-E| {max(devs):.4f} mm"), [
        "验证高频实时字符不破坏运动与位置精度"]


def case5_random_stop(b, ps, rounds, seed):
    """随机时刻发 `!`，验证每次都能停稳、冻结并可恢复。"""
    f, a = 1500.0, 300.0
    rng = random.Random(seed)
    stops, recover_ok = [], 0
    b.expect_ok("G90")
    for i in range(rounds):
        b.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")     # 每轮重置坐标基准
        tgt = BAND_HI                               # 走到工作区间上界（行程自适应）
        b.expect_ok(f"G1 X{tgt:.3f} F{f:.0f} A{a:.0f}", timeout_s=10)
        time.sleep(rng.uniform(0.3, 1.4))           # 随机打断时刻
        b.send("!")                                 # 实时字符，不带行尾
        fr = b.wait_idle(8.0)
        if fr is None:
            return False, f"第 {i} 轮 `!` 后未回到 IDLE", stops
        c_stop = fr["c"]
        time.sleep(0.15)
        again = b.query()
        frozen = abs(again["c"] - c_stop) <= 0.001
        stops.append((c_stop, frozen))
        # 恢复：从冻结位置回到中段
        try:
            b.expect_ok(f"G1 X{SETUP_TRAVEL:.3f} F{f:.0f} A{a:.0f}", timeout_s=15)
            if b.wait_idle(20) is None:
                return False, f"第 {i} 轮恢复未完成", stops
            recover_ok += 1
        except (Timeout, ProtocolError) as exc:
            return False, f"第 {i} 轮恢复失败：{exc}", stops
    all_frozen = all(x[1] for x in stops)
    ok = (recover_ok == rounds) and all_frozen
    uniq = len({round(s[0], 1) for s in stops})
    return ok, (f"{rounds} 轮随机打断：全部停稳并恢复，"
                f"冻结全部成立={all_frozen}，停位种类 {uniq}"), [
        f"停位样本（mm）：{[round(s[0], 1) for s in stops[:8]]}"
        + (" ..." if len(stops) > 8 else ""),
        "每轮均由 `G92` 重置基准，验证 rebase 后系统可反复复用"]


def case6_microstep_switch(b, ps, rounds):
    """极限切换 $0（16 ↔ 128，k 变化 8 倍），验证位置始终正确。"""
    full, upr = ps[1], ps[2]
    seq = [16, 128, 16]
    moved_ok, details = 0, []
    b.expect_ok("G90")
    try:
        for ms in seq:
            b.expect_ok(f"$0={ms}")
            k = full * ms / upr
            v_max_steps = (ps[7] / 60.0) * k
            if v_max_steps > 40000.0:
                details.append(f"$0={ms} 跳过：k={k:.0f} → "
                               f"v={v_max_steps:.0f} steps/s 超 40kHz 门")
                continue
            b.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")
            b.expect_ok(f"G1 X{SETUP_TRAVEL + 10.0:.3f} "
                        f"F{min(ps[7], 1200.0):.0f} A500")
            fr = b.wait_idle(20)
            if fr is None:
                return False, f"$0={ms} 时移动未结束", details
            err = abs(fr["c"] - (SETUP_TRAVEL + 10.0))
            dev = abs(fr["c"] - fr["e"])
            moved_ok += (err <= TOL_POS and dev <= TOL_ENC)
            details.append(f"$0={ms} → k={k:.0f}：走 10mm 得 C="
                           f"{fr['c'] - SETUP_TRAVEL:.4f}，|C-E|={dev:.4f}")
    finally:
        b.expect_ok(f"$0={ps[0]:.0f}")             # 恢复
    ok = moved_ok == len([s for s in seq if full * s / upr * ps[7] / 60.0 <= 40000.0])
    return ok, f"$0 切换序列 {seq}：{moved_ok} 次位置精确", details


def case7_limit_bombing(b, ps, rounds):
    """软限位边界轰炸：反复越界，验证只回 error:2 且位置不动。"""
    lo, hi = ps[12], ps[13]
    b.expect_ok("G90")
    before = b.query()
    errs_ok = 0
    for i in range(rounds):
        for target, want in ((lo - 1.0, 2), (hi + 1.0, 2)):
            kind, val = b.expect(f"G1 X{target:.3f} F1200")
            if kind == "error" and val == want:
                errs_ok += 1
    after = b.query()
    moved = abs(after["c"] - before["c"])
    ok = (errs_ok == rounds * 2) and moved <= 0.001
    return ok, (f"{rounds * 2} 次越界全部回 error:2，"
                f"位置变动 {moved:.4f} mm"), [
        f"软限位 [{lo:.0f}, {hi:.0f}]，反复发 X={lo - 1:.0f} / X={hi + 1:.0f}",
        "验证拒绝时不产生任何脉冲、不污染坐标"]


def case8_single_step(b, ps, rounds):
    """单步微动：±1 步（0.0125 mm）往返，验证不打滑、可回零。"""
    f = 600.0
    k = (ps[1] * ps[0]) / ps[2]     # steps/mm，运行期读取（用例 6 改过 $0，此处已恢复）
    step_mm = 1.0 / k               # 1 步（k=80 时为 0.0125 mm）
    b.expect_ok("G91")
    b.expect_ok(f"G1 X{step_mm:.4f} F{f:.0f}")
    b.wait_idle(10)
    b.expect_ok(f"G1 X{-step_mm:.4f} F{f:.0f}")
    fr0 = b.wait_idle(10)
    if fr0 is None:
        b.expect_ok("G90")
        return False, "单步往返未结束", []
    start = fr0["c"]
    for _ in range(rounds):
        b.expect_ok(f"G1 X{step_mm:.4f} F{f:.0f}", timeout_s=5)
        b.wait_idle(10)
        b.expect_ok(f"G1 X{-step_mm:.4f} F{f:.0f}", timeout_s=5)
        b.wait_idle(10)
    b.expect_ok("G90")
    end = b.query()
    drift = abs(end["c"] - start)
    ok = drift <= 0.001
    return ok, f"{rounds} 组 ±1 步往返，净漂移 {drift:.4f} mm", [
        f"单步 = {step_mm:.4f} mm（k=80），组数 {rounds}",
        f"验证微步定位可逆、无累积误差（C={end['c']:.4f}）"]


def case9_random_storm(b, ps, n, seed):
    """随机风暴：随机步长/速度/加速度的绝对移动，校验累计位置。"""
    rng = random.Random(seed)
    f_choices = [600.0, 1200.0, 2400.0, ps[7]]
    a_choices = [200.0, 500.0, 1000.0, ps[8]]
    b.expect_ok("G90")
    b.expect_ok(f"G92 X{SETUP_TRAVEL:.3f}")
    expect_mm = SETUP_TRAVEL
    devs = []
    for i in range(n):
        step = rng.choice([-60.0, -30.0, -10.0, -2.0, 2.0, 10.0, 30.0, 60.0])
        nxt = expect_mm + step
        if nxt < BAND_LO:                       # 夹在安全区间内
            nxt = BAND_LO + (BAND_LO - nxt)
        if nxt > BAND_HI:
            nxt = BAND_HI - (nxt - BAND_HI)
        b.expect_ok(f"G1 X{nxt:.3f} F{rng.choice(f_choices):.0f} "
                    f"A{rng.choice(a_choices):.0f}", timeout_s=10)
        fr = b.wait_idle(20)
        if fr is None:
            return False, f"第 {i} 次随机移动未结束", []
        devs.append(abs(fr["c"] - fr["e"]))
        expect_mm = nxt
    final = b.query()
    err = abs(final["c"] - expect_mm)
    ok = err <= TOL_POS and max(devs) <= TOL_ENC
    return ok, (f"{n} 次随机移动：末位置误差 {err:.4f} mm，"
                f"最大 |C-E| {max(devs):.4f} mm"), [
        f"步长 ∈ ±{{2,10,30,60}} mm，F ∈ {[int(x) for x in f_choices]}，"
        f"A ∈ {[int(x) for x in a_choices]}",
        f"期望末位置 {expect_mm:.3f} mm，实测 {final['c']:.3f} mm"]


def case10_endurance(b, ps, duration_s, sample_every=10):
    """长跑：连续大幅往返 duration 秒，监控 |C-E| 是否随时间漂移。"""
    f, a = ps[7], ps[8]
    b.expect_ok("G90")
    devs, rounds, t0 = [], 0, time.monotonic()
    while time.monotonic() - t0 < duration_s:
        tgt = BAND_HI if rounds % 2 == 0 else BAND_LO
        b.expect_ok(f"G1 X{tgt:.3f} F{f:.0f} A{a:.0f}", timeout_s=10)
        fr = b.wait_idle(30)
        if fr is None:
            return False, f"第 {rounds} 轮未回到 IDLE（长跑）", devs
        devs.append(abs(fr["c"] - fr["e"]))
        rounds += 1
    elapsed = time.monotonic() - t0
    if len(devs) < 4:
        return True, f"长跑 {elapsed:.0f}s 仅完成 {rounds} 轮，样本过少", devs
    head = statistics.fmean(devs[:sample_every // 2 or 1])
    tail = statistics.fmean(devs[-sample_every // 2 or -1:])
    worst = max(devs)
    trend = tail > 2.0 * head and tail > 0.05
    ok = (worst <= TOL_ENC) and not trend
    return ok, (f"{elapsed:.0f}s 内 {rounds} 轮 @ F{f:.0f} A{a:.0f}，"
                f"最大 |C-E| {worst:.4f} mm，首/末均值 {head:.4f}/{tail:.4f}"), [
        f"时间上无漂移趋势（末均值 {tail:.4f} vs 首均值 {head:.4f}）",
        f"累计往复 {rounds} 次（每次 {BAND_HI - BAND_LO:.0f} mm）"]


# ================================================================ main

CASES = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]


def run(board, args):
    print(f"[stress] T12+ 暴力压力测试 · {board.port}")
    ps = board.params()

    # 可选：先把软限位上限对齐到当前机械行程（避免按过大的区间工作）
    if args.set_soft_max is not None:
        board.expect_ok(f"$13={args.set_soft_max:.0f}")
        ps = board.params()
        print(f"  已将 $13 设为 {ps[13]:.0f}（$12={ps[12]:.0f} 未改动）")

    setup_axis(board, ps)

    selected = set(CASES)
    if args.only:
        selected = {s.strip() for s in args.only.replace(",", " ").split()}
    if args.skip_long:
        selected.discard("10")

    rounds = 5 if args.quick else args.rounds
    flood = 150 if args.quick else args.flood
    storm = 60 if args.quick else 200
    duration = min(args.duration, 45) if args.quick else args.duration

    plan = [
        ("1", "速度极限大幅往返", lambda: case1_max_speed(board, ps, rounds)),
        ("2", "急加减速（A 拉满）", lambda: case2_hard_accel(board, ps, rounds)),
        ("3", "命令洪流（背压）", lambda: case3_flood(board, ps, flood)),
        ("4", "高频 `?` 穿插", lambda: case4_query_storm(board, ps, rounds)),
        ("5", "`!` 随机打断", lambda: case5_random_stop(board, ps, rounds, args.seed)),
        ("6", "极限 $0 切换", lambda: case6_microstep_switch(board, ps, rounds)),
        ("7", "软限位边界轰炸", lambda: case7_limit_bombing(board, ps, rounds)),
        ("8", "单步微动", lambda: case8_single_step(board, ps, rounds)),
        ("9", "随机风暴", lambda: case9_random_storm(board, ps, storm, args.seed)),
        ("10", "长跑漂移监控", lambda: case10_endurance(board, ps, duration)),
    ]

    for key, name, fn in plan:
        if key not in selected:
            continue
        print(f"\n--- 用例 {key} · {name} ---")
        clean_state(board)          # 清掉上一个用例可能残留的段与未读帧
        try:
            ok, measured, evidence = fn()
        except (Timeout, ProtocolError) as exc:
            rec(f"{key} {name}", False, f"异常：{exc}")
            clean_state(board)      # 异常退出后同样要清场，否则污染后续用例
            continue
        rec(f"{key} {name}", ok, measured, evidence)

    # 收尾：停脉冲、失能（'!' 为实时字符，不带行尾）
    try:
        board.send("!")
        time.sleep(0.2)
        board.drain_input()
        board.expect_ok("M18")
    except Exception:
        pass

    passed = sum(1 for r in RESULTS if r[1])
    total = len(RESULTS)
    print(f"\nTOTAL: {passed}/{total} PASS")
    if args.report:
        write_report(args.report, board, ps, passed, total)
        print(f"[stress] 报告已写入 {args.report}")
    return 0 if passed == total else 1


def write_report(path, board, ps, passed, total):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    lines = [
        "# 暴力压力测试报告（host_sim_stress.py）",
        "",
        f"- 时间：{time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 串口：{board.port}",
        f"- 参数快照：`{fmt_params(ps)}`",
        f"- 协议行过滤：跳过非协议行 {board.noise_count} 行",
        "",
        "## 结果",
        "",
        "| 用例 | 实测 | 结论 |",
        "|---|---|---|",
    ]
    for name, ok, measured, _ in RESULTS:
        lines.append(f"| {name} | {measured} | {'PASS' if ok else 'FAIL'} |")
    lines += ["", "## 证据", ""]
    for name, ok, measured, evidence in RESULTS:
        lines.append(f"### {name} — {'PASS' if ok else 'FAIL'}")
        lines.append("")
        for e in evidence:
            lines.append(f"- {e}")
        lines.append("")
    lines.append(f"## 汇总：TOTAL {passed}/{total} PASS")
    lines.append("")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(
        description="T12+ 暴力压力测试（复用 host_sim.py 协议层）")
    ap.add_argument("port", help="CH343P 串口，例如 COM18")
    ap.add_argument("--rounds", type=int, default=12,
                    help="各用例的往复/打断轮数（默认 12）")
    ap.add_argument("--flood", type=int, default=500,
                    help="命令洪流的条数（默认 500）")
    ap.add_argument("--duration", type=int, default=120,
                    help="长跑秒数（默认 120）")
    ap.add_argument("--seed", type=int, default=20260917,
                    help="随机种子（默认固定，保证可复现）")
    ap.add_argument("--quick", action="store_true",
                    help="快速档：轮数/条数/时长全部缩减（约 1 分钟）")
    ap.add_argument("--skip-long", action="store_true", help="跳过用例 10（长跑）")
    ap.add_argument("--only", metavar="LIST",
                    help="只运行指定用例，如 5 或 1,3,9")
    ap.add_argument("--set-soft-max", type=float, metavar="MM",
                    help="运行前把 $13 设为该值（如 500）并校验；$12 保持不动。"
                         "用于让软限位与当前机械行程一致")
    ap.add_argument("--report", metavar="PATH", help="输出 Markdown 报告")
    args = ap.parse_args()

    board = Board(args.port)
    try:
        probe(board)                            # 活性检查 + 友好的失败提示
        return run(board, args)
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())
