# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for touch_sweep.py. Run: uv run clients/python/test_touch_sweep.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from touch_sweep import first_cross, interp, trial_metrics  # noqa: E402

F = [(0.0, 0.1), (1.0, 0.5), (2.0, 4.5), (3.0, 20.0), (4.0, 10.0), (5.0, 10.0)]
Z = [(0.0, 50.0), (2.0, 40.0), (5.0, 39.0)]
assert first_cross(F, 1.0) == 1.125  # 0.5 -> 4.5 crosses 1.0 an eighth of the way
assert first_cross(F, 100) is None
assert interp(Z, -1) == 50.0 and interp(Z, 1.0) == 45.0 and interp(Z, 9) == 39.0
m = trial_metrics(F, Z, threshold=5.0, contact_n=1.0, settle_s=1.0)
assert m["contact"] and m["peak_n"] == 20.0 and m["residual_n"] == 10.0, m
assert abs(m["detect_ms"] - (2.0 + 0.5 / 15.5 - 1.125) * 1e3) < 1e-6, m
assert abs(m["overshoot_mm"] - (interp(Z, 1.125) - 39.0)) < 1e-9, m
assert trial_metrics(F[:2], Z, 5.0, 1.0, 1.0) == {"contact": False, "peak_n": 0.5}
print("ok")
