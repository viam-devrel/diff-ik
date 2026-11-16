# /// script
# requires-python = ">=3.11"
# ///
"""Synthetic-CSV check for teleop_lag.py. Run: uv run clients/python/test_teleop_lag.py"""
import csv
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_lag import analyze  # noqa: E402

KEYS = ("x", "y", "z", "o_x", "o_y", "o_z", "theta")


def leader(t):  # a 40 mm-radius circle at 0.5 Hz, then rest after 6 s
    if t > 6:
        t = 6
    return 200 + 40 * math.cos(math.pi * t), 40 * math.sin(math.pi * t), 150


def main():
    d = tempfile.mkdtemp()
    pre = os.path.join(d, "run")
    with open(pre + "-ticks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", *[f"l_{k}" for k in KEYS], *[f"s_{k}" for k in KEYS],
                    "held", "clamped", "state", "err_mm", "err_deg", "tick_ms"])
        for k in range(450):  # 9 s at 50 Hz
            t = k * 0.02
            x, y, z = leader(t)
            rest = t > 6.5
            pause = 2.0 <= t < 2.3  # a short held pause mid-motion: must not count as rest
            row = [x, y, z, 0, 0, -1, 0]
            w.writerow([f"{t:.4f}", "track", *row, *row, int(rest or pause), 0,
                        "converged" if rest else "running", 0.5 if rest else 8.0, 0.1, 6.0])
    with open(pre + "-follower.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "kind", *[f"f_{k}" for k in KEYS], "rate_hz", "tick_p99_ms", "overruns", "resyncs"])
        for k in range(90):  # 10 Hz, follower = leader delayed 100 ms
            t = k * 0.1
            x, y, z = leader(max(0.0, t - 0.1))
            kind = "status" if k % 2 == 0 else "pose"
            extra = [50.0, 3.0, 0, 0] if kind == "status" else ["", "", "", ""]
            w.writerow([f"{t:.4f}", kind, x, y, z, 0, 0, -1, 0, *extra])
    r = analyze(pre)
    assert abs(r["lag_ms"] - 100) <= 10, r["lag_ms"]
    assert r["track_err_mm_at_lag"] < r["track_err_mm_raw"], r
    assert r["rest_converged_frac"] >= 0.9, r
    assert r["max_tick_p99_ms"] == 3.0 and r["max_overruns"] == 0, r
    print("ok", {k: r[k] for k in ("lag_ms", "track_err_mm_raw", "track_err_mm_at_lag", "rest_converged_frac")})

    # every tick held (e.g. the self-tracking rehearsal): no motion, so lag fields are None, no crash
    rows = list(csv.reader(open(pre + "-ticks.csv")))
    hi = rows[0].index("held")
    with open(pre + "-ticks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(rows[0])
        for r in rows[1:]:
            r[hi] = "1"
            w.writerow(r)
    r = analyze(pre)
    assert r["lag_ms"] is None and r["track_err_mm_raw"] is None, r
    print("ok all-held")


if __name__ == "__main__":
    main()
