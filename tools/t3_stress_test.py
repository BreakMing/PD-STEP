#!/usr/bin/env python3
"""T3 comm stack stress test.

Floods short comment lines (';x\\n') over the UART1 protocol channel and
counts 'ok\\r\\n' replies. Verifies zero loss and throughput.

Wiring (board side, per docs/hardware.md §3):
    CH343P TX  -> board GPIO42 (UART1 RX)
    CH343P RX  -> board GPIO41 (UART1 TX)
    GND        -> GND

Usage:
    python t3_stress_test.py COM7 [num_lines]

Criterion (PLAN.md T3): zero loss and >= 2400 lines/s.
Physics: 115200-8N1 -> 11520 B/s per direction; TX frame 'ok\\r\\n' = 4 B
-> theoretical TX ceiling 2880 lines/s; 2400 leaves ~17% headroom for
mutex / scheduler / driver overhead.
"""

import sys
import time
import threading

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 30000
CRIT_LINES_S = 2400

try:
    s = serial.Serial(PORT, 115200, timeout=0.2)
except serial.SerialException as e:
    sys.exit(f"cannot open {PORT}: {e}")

s.reset_input_buffer()
s.reset_output_buffer()


def sender():
    payload = b";x\n" * 1000
    remaining = N
    while remaining > 0:
        take = min(1000, remaining)
        s.write(payload[: take * 3])
        remaining -= take


t = threading.Thread(target=sender)
start = time.perf_counter()
t.start()

count = 0
stall_reads = 0
# receiving 'ok\r\n' x N is the bottleneck: N * 4 / 11520 s; add 15 s margin
deadline = start + N * 4 / 11520 + 15

while count < N and time.perf_counter() < deadline:
    line = s.readline()
    if line == b"ok\r\n":
        count += 1
        stall_reads = 0
    elif line == b"":
        stall_reads += 1
        if stall_reads > 25:  # ~5 s of silence -> sender must be done
            break
    else:
        print(f"UNEXPECTED RX: {line!r}")
        break

t.join()
elapsed = time.perf_counter() - start
rate = count / elapsed if elapsed > 0 else 0.0

print(f"sent {N} lines, received {count} ok, loss {N - count}")
print(f"elapsed {elapsed:.2f} s, throughput {rate:.0f} lines/s "
      f"(criterion >= {CRIT_LINES_S})")
ok = (count == N) and (rate >= CRIT_LINES_S)
print("PASS" if ok else "FAIL")
s.close()
sys.exit(0 if ok else 1)
