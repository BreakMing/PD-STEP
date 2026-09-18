# Tools directory

This directory contains host-side tools and test scripts.

- `host_sim.py` — Host simulator and end-to-end acceptance script (T12)
  Usage: `python tools/host_sim.py <PORT> --phase 1` → power-cycle the board → `python tools/host_sim.py <PORT> --phase 2` (需 pyserial；快速自检用 `--skip-power-cycle --fast`，报告写入 `docs/acceptance.md`)
