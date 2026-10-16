# -*- coding: utf-8 -*-
"""max_accel_run 纯逻辑单元测试（标准库 unittest，不触碰串口与真实文件）。

被测对象是 tools/max_accel_run.py 中具有独立行为契约的纯函数：
单段判定（FAULT 优先、100/300 µm 三分）、组合判定多数规则、
加速到达判据、停稳判据、超时预算与巡航保持时间表。串口会话
（TestRig）与段执行编排属于实机集成，不在本层覆盖。
"""

import unittest
from collections import deque

import max_accel_run as m


class ClassifyOffsetTest(unittest.TestCase):
    def test_fault_takes_priority(self):
        self.assertEqual(m.classify_offset(0, faulted=True), "FAIL")
        self.assertEqual(m.classify_offset(-5000, faulted=True), "FAIL")

    def test_pass_band(self):
        self.assertEqual(m.classify_offset(0, False), "PASS")
        self.assertEqual(m.classify_offset(100, False), "PASS")      # 边界含
        self.assertEqual(m.classify_offset(-100, False), "PASS")

    def test_margin_band(self):
        self.assertEqual(m.classify_offset(101, False), "MARGIN")
        self.assertEqual(m.classify_offset(299, False), "MARGIN")
        self.assertEqual(m.classify_offset(-300 + 1, False), "MARGIN")

    def test_fail_band(self):
        # 方案语义「净偏移 > 300 µm 判失败」：300 本身落在复测带
        self.assertEqual(m.classify_offset(300, False), "MARGIN")
        self.assertEqual(m.classify_offset(301, False), "FAIL")
        self.assertEqual(m.classify_offset(-301, False), "FAIL")


class CombineVerdictsTest(unittest.TestCase):
    def test_majority_pass(self):
        ok, label = m.combine_verdicts(["PASS", "PASS", "FAIL"])
        self.assertTrue(ok)
        self.assertEqual(label, "可行")

    def test_majority_fail(self):
        ok, _ = m.combine_verdicts(["FAIL", "FAIL", "PASS"])
        self.assertFalse(ok)

    def test_majority_margin_is_not_ok(self):
        ok, _ = m.combine_verdicts(["MARGIN", "MARGIN", "PASS"])
        self.assertFalse(ok)

    def test_all_three_distinct_is_conservative_fail(self):
        ok, label = m.combine_verdicts(["PASS", "MARGIN", "FAIL"])
        self.assertFalse(ok)
        self.assertIn("存疑", label)

    def test_single_sample(self):
        self.assertTrue(m.combine_verdicts(["PASS"])[0])
        self.assertFalse(m.combine_verdicts(["MARGIN"])[0])
        self.assertFalse(m.combine_verdicts(["FAIL"])[0])


class ReachedTest(unittest.TestCase):
    def test_within_noise_tolerance(self):
        # tol = max(2σ=10, 5%×50=2.5) = 10
        self.assertTrue(m.reached(44, 50))
        self.assertTrue(m.reached(58, 50))

    def test_outside_tolerance(self):
        self.assertFalse(m.reached(39, 50))
        self.assertFalse(m.reached(61, 50))

    def test_small_target_uses_noise_floor(self):
        # tol = max(10, 0.25) = 10：目标 5 mm/s 时 vx=-4 也算到达
        self.assertTrue(m.reached(-4, 5))


class SettledRunTest(unittest.TestCase):
    def test_needs_full_run(self):
        self.assertFalse(m.settled_run([0, 0, 0, 0, 0, 0, 0], run=8))

    def test_consecutive_low(self):
        self.assertTrue(m.settled_run([30, 10, 0, 0, 0, 0, 0, 0, 0], run=8))

    def test_last_sample_breaks_run(self):
        self.assertFalse(m.settled_run([0] * 7 + [11], run=8))

    def test_deque_input(self):
        d = deque([0] * 8, maxlen=8)
        self.assertTrue(m.settled_run(d))


class LegTimesTest(unittest.TestCase):
    def test_ramp_plus_margin(self):
        self.assertAlmostEqual(m.leg_times(550, 5000, 1.0)[0], 0.61)
        self.assertAlmostEqual(m.leg_times(550, 5000, 1.0)[1], 1.11)

    def test_high_acc_short_ramp(self):
        self.assertAlmostEqual(m.leg_times(50, 30000, 1.0)[0],
                               50 / 30000 + 0.5)


class HoldTimeTest(unittest.TestCase):
    def test_known_speeds_exact(self):
        self.assertEqual(m.hold_time(50), 1.20)
        self.assertEqual(m.hold_time(550), 0.16)

    def test_interpolated_between_known(self):
        # 200 介于 120(0.64) 与 300(0.27) 之间线性内插
        want = 0.64 + (0.27 - 0.64) * (200 - 120) / (300 - 120)
        self.assertAlmostEqual(m.hold_time(200), want)

    def test_clamps_outside_table(self):
        self.assertEqual(m.hold_time(30), 1.20)
        self.assertEqual(m.hold_time(600), 0.16)


if __name__ == "__main__":
    unittest.main()
