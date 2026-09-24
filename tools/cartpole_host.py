#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cartpole_host — cartpole 固件串口联调工具（PC 直连 ESP32-S3 UART1/CH343P）

依赖：pip install pyserial

用法（示例）：
  python cartpole_host.py --port COM7 query
  python cartpole_host.py --port COM7 home
  python cartpole_host.py --port COM7 vel 200 --hold 2
  python cartpole_host.py --port COM7 config VEL_MAX 800
  python cartpole_host.py --port COM7 dump
  python cartpole_host.py --port COM7 state --secs 10
  python cartpole_host.py --port COM7 wdt-test
  python cartpole_host.py --port COM7 console

对应 protocol.md：STATE/MODE/OK/ERR 行、460800 8N1。
"""

import argparse
import sys
import time

import serial


class Rig:
    def __init__(self, port, baud=460800, timeout=0.2):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.last_mode = None      # (mode, reason)
        self.last_reply = None     # (kind, name, value)
        self.state_count = 0
        self.unknown_count = 0

    def close(self):
        self.ser.close()

    def send_line(self, line):
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()

    def pump(self, duration):
        """读串口 duration 秒，解析 STATE/MODE/OK/ERR，统计 UNKNOWN。"""
        t0 = time.time()
        buf = b""
        while time.time() - t0 < duration:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                self._dispatch(line)

    def _dispatch(self, line):
        if line.startswith("STATE "):
            parts = line.split()
            if len(parts) == 5:
                try:
                    [int(p) for p in parts[1:]]
                    self.state_count += 1
                    self.last_state = parts[1:]
                    return
                except ValueError:
                    pass
            self.unknown_count += 1
            print("[UNKNOWN] %r" % line)
        elif line.startswith("MODE "):
            parts = line.split()
            if len(parts) == 3:
                self.last_mode = (parts[1], parts[2])
                print("[MODE] %s %s" % self.last_mode)
            else:
                self.unknown_count += 1
                print("[UNKNOWN] %r" % line)
        elif line.startswith("OK "):
            parts = line.split()
            if len(parts) == 3:
                try:
                    self.last_reply = ("OK", parts[1], int(parts[2]))
                    print("[OK] %s = %d" % (parts[1], int(parts[2])))
                    return
                except ValueError:
                    pass
            self.unknown_count += 1
            print("[UNKNOWN] %r" % line)
        elif line.startswith("ERR "):
            self.last_reply = ("ERR", line[4:], None)
            print("[ERR] %s" % line[4:])
        else:
            self.unknown_count += 1
            print("[UNKNOWN] %r" % line)

    def query(self):
        self.send_line("QUERY")
        self.pump(0.3)
        return self.last_mode

    def wait_mode(self, want, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.1)
            if self.last_mode and self.last_mode[0] == want:
                return self.last_mode
        return None


def cmd_query(rig, args):
    m = rig.query()
    if m is None:
        print("无 MODE 应答")
        return 1
    return 0


def cmd_home(rig, args):
    rig.send_line("HOME")
    m = rig.wait_mode("READY", args.timeout)
    if m is None:
        print("超时未到 READY（当前 %s）" % (rig.last_mode,))
        return 1
    return 0


def cmd_vel(rig, args):
    m = rig.query()
    if not m or m[0] != "READY":
        print("当前模式 %s，非 READY——先 HOME" % (m,))
        return 1
    end = time.time() + args.hold
    n = 0
    while time.time() < end:
        rig.send_line("VEL %d" % args.v)
        n += 1
        rig.pump(0.005)
    rig.pump(0.3)
    print("已发送 %d 条 VEL（%.1f Hz）" % (n, n / args.hold))
    if rig.last_state:
        print("最新 STATE: x=%s θ=%s vx=%s vth=%s" % tuple(rig.last_state))
    return 0


def cmd_config(rig, args):
    rig.send_line("CONFIG %s %d" % (args.name.upper(), args.value))
    rig.pump(0.5)
    if rig.last_reply and rig.last_reply[0] == "OK":
        return 0
    return 1


def cmd_dump(rig, args):
    rig.state_count = 0
    rig.send_line("CONFIG?")
    rig.pump(0.8)
    print("STATE 行（dump 窗口内）：%d" % rig.state_count)
    return 0


def cmd_state(rig, args):
    t0 = time.time()
    rig.state_count = 0
    rig.unknown_count = 0
    xs, ths = [], []
    while time.time() - t0 < args.secs:
        rig.pump(0.2)
        if rig.last_state:
            x, th = int(rig.last_state[0]), int(rig.last_state[1])
            xs.append(x)
            ths.append(th)
    dt = time.time() - t0
    rate = rig.state_count / dt
    print("时长 %.1f s，STATE %d 行 → %.1f Hz（目标 200）" %
          (dt, rig.state_count, rate))
    print("UNKNOWN 行：%d（要求 0）" % rig.unknown_count)
    if xs:
        print("x_um ∈ [%d, %d]（漂移 %d µm）" %
              (min(xs), max(xs), max(xs) - min(xs)))
        print("theta_urad ∈ [%d, %d]" % (min(ths), max(ths)))
    ok = 199.0 <= rate <= 201.0 and rig.unknown_count == 0
    return 0 if ok else 1


def cmd_wdt_test(rig, args):
    m = rig.query()
    if not m or m[0] != "READY":
        print("需先 HOME 到 READY")
        return 1
    end = time.time() + 1.0
    while time.time() < end:
        rig.send_line("VEL 200")
        rig.pump(0.005)
    print("停止喂狗，观察 1 s …")
    rig.pump(1.0)
    m = rig.query()
    print("QUERY → %s（预期 READY：看门狗是瞬态保护不改模式）" % (m,))
    ok = m and m[0] == "READY" and m[1] == "NONE"
    return 0 if ok else 1


def cmd_console(rig, args):
    print("交互模式：输入命令回车发送；Ctrl-C 退出。")
    try:
        while True:
            line = input("> ").strip()
            if line:
                rig.send_line(line)
            rig.pump(0.3)
    except KeyboardInterrupt:
        pass
    return 0


def main():
    ap = argparse.ArgumentParser(description="cartpole 固件串口联调工具")
    ap.add_argument("--port", required=True, help="串口（如 COM7）")
    ap.add_argument("--baud", type=int, default=460800)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("query")
    p = sub.add_parser("home"); p.add_argument("--timeout", type=float, default=35.0)
    p = sub.add_parser("vel"); p.add_argument("v", type=int)
    p.add_argument("--hold", type=float, default=1.0)
    p = sub.add_parser("config"); p.add_argument("name"); p.add_argument("value", type=int)
    sub.add_parser("dump")
    p = sub.add_parser("state"); p.add_argument("--secs", type=float, default=10.0)
    sub.add_parser("wdt-test")
    sub.add_parser("console")

    args = ap.parse_args()
    rig = Rig(args.port, args.baud)
    try:
        table = {
            "query": cmd_query, "home": cmd_home, "vel": cmd_vel,
            "config": cmd_config, "dump": cmd_dump, "state": cmd_state,
            "wdt-test": cmd_wdt_test, "console": cmd_console,
        }
        rc = table[args.cmd](rig, args)
    finally:
        rig.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
