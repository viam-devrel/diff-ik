# /// script
# requires-python = ">=3.11"
# ///
"""Synthetic-log checks for tag_sweep_report.py. Run: uv run clients/python/test_tag_sweep_report.py"""
import csv
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_sweep_report import accel_peak, analyze, markdown  # noqa: E402

R, V, CX, LAG, TIP = 60.0, 20.0, 220.0, 0.1, 100.0
C = 2 * math.pi * R
END = 2 * C / V  # two laps, no ramp


def tag_xy(t):
    a = V * max(t, 0.0) / R
    return CX + R * math.cos(a), R * math.sin(a)


pre = os.path.join(tempfile.mkdtemp(), "run")
with open(pre + "-tag.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "z", "speed", "seg", "seg_speed", "lap"])
    for k in range(int(END * 50)):
        t = k / 50
        x, y = tag_xy(t)
        w.writerow([t, x, y, -97, V, 0, V, int(V * t // C)])
with open(pre + "-joints.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "q0", "q1", "x", "y", "z", "o_x", "o_y", "o_z", "theta"])
    for k in range(int(END * 50)):
        t = k / 50
        x, y = tag_xy(t - LAG)
        w.writerow([t, 10 * math.sin(2 * math.pi * t), 0, x, y, 200, 0, 0, -1, 0])  # jaw tip at z = 100
with open(pre + "-ticks.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "kind", "phase", "state"])
    for k in range(int(END * 20)):
        w.writerow([k / 20, "det", "sweep", "running"])
    w.writerow([1.0, "drop", "sweep", "jump"])
with open(pre + "-follower.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "kind", "resyncs", "overruns", "lag_ms"])
    for k in range(int(END * 5)):
        w.writerow([k / 5, "status", k // 50, 0, 40.0])
        w.writerow([k / 5 + 0.1, "pose", "", "", ""])
with open(pre + "-summary.json", "w") as f:
    json.dump({"args": {"tip_mm": TIP, "camera": "wrist-cam"}, "breaking_speed_mm_s": None}, f)

res = analyze(pre)
(row,) = res["rows"]
assert row["speed"] == V and not row["lost"], row
assert abs(row["err"]["mean"] - V * LAG) < 0.2, row["err"]  # 2 mm behind on the circle
assert abs(row["lag_ms"] - 100) <= 10, row["lag_ms"]
assert row["err_at_lag"] < 0.3, row["err_at_lag"]
assert abs(row["det_hz"] - 20) < 1 and row["drops"] == 1 and row["drop_reasons"] == {"jump": 1}, row
assert row["resyncs"] == int(END * 5 - 1) // 50 and row["overruns"] == 0 and row["max_lag_ms"] == 40.0, row
peak = 10 * (2 * math.pi) ** 2  # 10 deg sine at 1 Hz
assert abs(row["peak_accel"] - peak) / peak < 0.1, row["peak_accel"]
assert abs(max(accel_peak([k / 50 for k in range(200)], [[5 * (k / 50) ** 2] for k in range(200)])) - 10) < 0.5
# 1 ms of stamp jitter must not inflate the peak (90 deg/s peak sine at 50 Hz: true 141 deg/s^2)
import random
random.seed(1)
ts = [k / 50 + random.uniform(-0.001, 0.001) for k in range(500)]
amp, om = 90 / 1.5708, 1.5708  # A*om = 90 deg/s, A*om^2 = 141 deg/s^2
jit = max(accel_peak(ts, [[amp * math.sin(om * (k / 50))] for k in range(500)]))
assert jit < 200, jit
assert accel_peak([], []) == []
md = markdown([res])
assert "wrist-cam" in md and "| 20 |" in md, md
for suffix in ("-tag.csv", "-joints.csv"):  # a run that ended before the sweep has headers only
    open(pre + suffix, "w").write("t\n")
assert analyze(pre)["rows"] == []
print("ok", {k: row[k] for k in ("lag_ms", "err_at_lag", "det_hz", "peak_accel")})
