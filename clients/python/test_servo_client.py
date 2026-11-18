# /// script
# requires-python = ">=3.11"
# ///
"""Pure-helper checks for servo_client.py. Run: uv run clients/python/test_servo_client.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_client import FOLLOW_COLS, POSE_KEYS, follow_row  # noqa: E402

fp = dict(zip(POSE_KEYS, [1.0, 2.0, 3.0, 0.0, 0.0, -1.0, 0.0]))
r = follow_row(1.23456, fp)
assert len(r) == len(FOLLOW_COLS), (len(r), len(FOLLOW_COLS))
assert r[0] == "1.2346" and r[1] == "pose" and r[2] == 1.0
assert FOLLOW_COLS[-1] == "peak_accel_ratio" and r[-1] == ""
s = follow_row(2.0, fp, {"rate_hz": 50.0, "tick_p99_ms": 3.0, "overruns": 1, "resyncs": 2, "lag_ms": 42.0, "peak_accel_ratio": 0.5})
assert len(s) == len(FOLLOW_COLS) and s[1] == "status"
assert dict(zip(FOLLOW_COLS, s))["lag_ms"] == 42.0 and dict(zip(FOLLOW_COLS, s))["resyncs"] == 2
assert dict(zip(FOLLOW_COLS, s))["peak_accel_ratio"] == 0.5
print("ok")
