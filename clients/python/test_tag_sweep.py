# /// script
# requires-python = ">=3.11"
# ///
"""Schedule checks for tag_sweep.py. Run: uv run clients/python/test_tag_sweep.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_sweep import circle_state, circle_xy, speeds_arg, total_time  # noqa: E402

V, LAPS, R, RAMP = [10.0, 20.0], 2, 60.0, 1.0
C = 2 * math.pi * R
T = total_time(V, LAPS, R, RAMP)
assert abs(T - (RAMP + LAPS * C / 10 + RAMP + LAPS * C / 20)) < 1e-9, T

# start: at rest at angle 0, ramping
s0 = circle_state(0.0, V, LAPS, R, RAMP)
assert s0["s"] == 0 and s0["speed"] == 0 and s0["seg"] == 0 and s0["lap"] == -1 and not s0["done"]
assert circle_xy(0.0, 220, 0, R) == (280.0, 0.0)

# continuous position and speed at every boundary
for tb in (RAMP, RAMP + LAPS * C / 10, 2 * RAMP + LAPS * C / 10):
    a, b = circle_state(tb - 1e-6, V, LAPS, R, RAMP), circle_state(tb + 1e-6, V, LAPS, R, RAMP)
    assert abs(a["s"] - b["s"]) < 1e-3 and abs(a["speed"] - b["speed"]) < 1e-2, (tb, a, b)

# holds at the segment speed; laps counted from the end of the ramp
m = circle_state(RAMP + 1.5 * C / 10, V, LAPS, R, RAMP)
assert m["speed"] == 10 and m["seg"] == 0 and m["seg_speed"] == 10 and m["lap"] == 1
r2 = circle_state(RAMP + LAPS * C / 10 + 0.5, V, LAPS, R, RAMP)
assert r2["seg"] == 1 and r2["lap"] == -1 and 10 < r2["speed"] < 20 and r2["seg_speed"] == 20

# done after the schedule, at rest
e = circle_state(T + 1, V, LAPS, R, RAMP)
assert e["done"] and e["speed"] == 0 and abs(e["s"] - (C * 2 * LAPS + RAMP * 5 + RAMP * 15)) < 1e-6, e

# a quarter turn lands at 90 degrees
x, y = circle_xy(C / 4, 220, 0, R)
assert abs(x - 220) < 1e-9 and abs(y - 60) < 1e-9
for bad in ("", "0", "10,0", "10,-5"):
    try:
        speeds_arg(bad)
        raise AssertionError(bad)
    except ValueError:
        pass
assert speeds_arg("10,20") == "10,20"
print("ok", round(T, 1), "s")
