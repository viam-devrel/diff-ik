# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for press_hold.py. Run: uv run clients/python/test_press_hold.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from press_hold import hold_metrics  # noqa: E402

P = [(0.0, 0.0), (1.0, 2.0), (2.0, 14.0), (3.0, 11.5), (4.0, 10.5), (5.0, 9.5), (6.0, 10.0)]
m = hold_metrics(P, target=10, contact_n=1, band=0.1, tail_s=2.0)
assert m["contact"] and m["peak_n"] == 14.0 and abs(m["overshoot_pct"] - 40) < 1e-9, m
assert m["settle_ms"] == 3000.0, m  # 11.5 at t=3 is out of band; in band from t=4, contact at t=1
assert abs(m["mean_n"] - 10.0) < 1e-9, m
assert hold_metrics(P, 10, 1, 0.01, 1)["settle_ms"] == 5000.0  # only the last row is within ±0.1 N
assert hold_metrics([(0.0, 0.0)], 10, 1, 0.1, 1) == {"contact": False}
print("ok")
