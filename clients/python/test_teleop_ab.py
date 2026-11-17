import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_ab import analyze, tip  # noqa: E402


def test_lag_and_offset_recovered():
    rnd = random.Random(1)
    leader = lambda t: (100 * math.sin(t), 80 * math.cos(0.7 * t), 200 + 30 * math.sin(1.3 * t))
    lt = sorted(rnd.uniform(0, 20) for _ in range(600))
    ft = sorted(rnd.uniform(0, 20) for _ in range(600))
    lp = [leader(t) for t in lt]
    fp = [tuple(50 + 2.5 * c for c in leader(t - 0.12)) for t in ft]  # 120 ms behind, offset 50
    r = analyze(lt, lp, ft, fp, 2.5)
    assert r["lag_ms"] in (110, 120, 130), r
    assert r["track_rms_mm_at_lag"] < r["track_rms_mm_at_0"], r
    assert r["jitter_rms_mm"] < 5, r


def test_tip_along_tool_axis():
    assert tip({"x": 1, "y": 2, "z": 3, "o_x": 0, "o_y": 0, "o_z": -1}, 100) == (1, 2, -97)
