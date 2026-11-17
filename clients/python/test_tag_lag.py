# /// script
# requires-python = ">=3.11"
# ///
"""Synthetic-CSV checks for tag_lag.py. Run: uv run clients/python/test_tag_lag.py"""
import csv
import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_lag import analyze  # noqa: E402

K = ("x", "y", "z", "o_x", "o_y", "o_z", "theta")
COLS = ["t", "kind", "phase", *[f"tag_{k}" for k in K], *[f"target_{k}" for k in K], *[f"raw_{k}" for k in K],
        *[f"tagarm_{k}" for k in K],
        "state", "err_mm", "err_deg", "reach_deg", "residual_deg", "clamped", "rtt_ms"]
FCOLS = ["t", "kind", *[f"f_{k}" for k in K], "rate_hz", "tick_p99_ms", "overruns", "resyncs"]


def write(prefix, ticks, foll):
    with open(prefix + "-ticks.csv", "w", newline="") as f:
        w = csv.DictWriter(f, COLS, restval="")
        w.writeheader()
        w.writerows({"phase": "track", **r} for r in ticks)
    with open(prefix + "-follower.csv", "w", newline="") as f:
        w = csv.DictWriter(f, FCOLS, restval="")
        w.writeheader()
        w.writerows(foll)


def pose(prefix, p, o=(0, 0, 1)):
    return {f"{prefix}_{k}": v for k, v in zip(K, (*p, *o, 0))}


def circle(t):  # 40 mm radius, 0.5 Hz, still after 6 s
    t = min(t, 6)
    return 200 + 40 * math.cos(math.pi * t), 40 * math.sin(math.pi * t), 150


def follower_rows(fn, n=1000, status=True):
    rows = [{"t": f"{k * 0.01:.3f}", "kind": "pose", **pose("f", fn(k * 0.01))} for k in range(n)]
    if status:
        rows += [{"t": "3.0", "kind": "status", "tick_p99_ms": 7, "overruns": 2, "resyncs": 1},
                 {"t": "9.0", "kind": "status", "tick_p99_ms": 5, "overruns": 3, "resyncs": 1}]
    return rows


def test_lag_and_rest():
    rnd = random.Random(1)
    ticks = []
    for k in range(300):  # 10 s at 30 Hz
        t = k / 30
        tgt = circle(t)
        row = {"t": f"{t:.4f}", "kind": "det", **pose("target", tgt),
               **pose("raw", [c + rnd.gauss(0, 1) for c in tgt]), "state": "converged",
               "clamped": "1" if k % 100 == 7 else "0", "rtt_ms": f"{10 + k % 5}"}
        if k % 10 == 3:
            row = {"t": row["t"], "kind": "resend", "clamped": "0"}
        if k == 50:
            row = {"t": row["t"], "kind": "frozen"}
        if k in (60, 61):
            row = {"t": row["t"], "kind": "drop", "state": "jump"}
        if k == 62:
            row = {"t": row["t"], "kind": "drop", "state": "grazing"}
        ticks.append(row)
    pre = os.path.join(tempfile.mkdtemp(), "run")
    write(pre, ticks, follower_rows(lambda t: circle(t - 0.15)))
    r = analyze(pre)
    print(r)
    assert abs(r["lag_ms"] - 150) <= 10, r
    assert r["track_err_mm_at_lag"] < 2, r
    assert all(abs(s - 1) <= 0.3 for s in r["raw_std_mm_axes"]), r
    assert abs(r["raw_std_mm"] - 3 ** 0.5) <= 0.5, r
    assert r["raw_normal_std_deg"] == 0, r
    assert r["resends"] == 30 and r["frozen"] == 1, r
    assert r["clamped"] == 3, r
    assert r["drops"] == {"jump": 2, "grazing": 1}, r
    assert 25 < r["det_rate_hz"] < 31, r
    assert 10 <= r["rtt_ms_p50"] <= r["rtt_ms_p99"] <= 14, r
    assert (r["max_tick_p99_ms"], r["max_overruns"], r["max_resyncs"]) == (7, 3, 1), r


def test_skew():
    w = math.pi

    def f(t):  # follower: 50 mm circle, stops at 6 s
        t = min(t, 6)
        return 200 + 50 * math.cos(w * t), 50 * math.sin(w * t), 150

    def vel(t):
        return (0, 0) if t >= 6 else (-50 * w * math.sin(w * t), 50 * w * math.cos(w * t))

    p0 = f(6)
    ticks = []
    for k in range(300):
        t = k / 30
        v = vel(t)
        tgt = (p0[0] + 0.08 * v[0], p0[1] + 0.08 * v[1], p0[2])  # target deviates by v * 80 ms
        ticks.append({"t": f"{t:.4f}", "kind": "det", **pose("target", tgt), **pose("raw", tgt),
                      "state": "converged", "clamped": "0", "rtt_ms": "10"})
    pre = os.path.join(tempfile.mkdtemp(), "run")
    write(pre, ticks, follower_rows(f))
    r = analyze(pre)
    assert abs(r["skew_ms"] - 80) <= 10, r


def test_no_motion_and_blank_cells():
    rnd = random.Random(2)
    ticks = [{"t": f"{k / 30:.4f}", "kind": "det", **pose("target", (200 + rnd.gauss(0, 2), rnd.gauss(0, 2), 150 + rnd.gauss(0, 2))),
              **pose("raw", (200 + rnd.gauss(0, 1), rnd.gauss(0, 1), 150 + rnd.gauss(0, 1))),
              "state": "converged", "clamped": "0", "rtt_ms": "9"} for k in range(150)]
    ticks.insert(10, {"t": "0.3", "kind": "drop", "state": "grazing"})  # blank target/raw/rtt
    pre = os.path.join(tempfile.mkdtemp(), "run")
    write(pre, ticks, follower_rows(lambda t: (200, 0, 150), n=600, status=False))
    r = analyze(pre)  # noisy target never rests, follower never moves: None, not a crash
    assert r["lag_ms"] is None and r["skew_ms"] is None and r["raw_std_mm"] is None, r
    assert r["max_overruns"] is None and r["drops"] == {"grazing": 1}, r
    r = analyze(pre, rest="all")
    assert r["raw_std_mm"] is not None and r["lag_ms"] is None, r


def test_phases():
    rnd = random.Random(3)
    ticks = [{"t": f"{-2 + k / 30:.4f}", "kind": "det", "phase": "engage", **pose("target", (0, 0, 0)),
              **pose("raw", (0, 0, 0)), "clamped": "0", "rtt_ms": "5"} for k in range(60)]
    for k in range(300):
        t = k / 30
        tgt = circle(t)
        ticks.append({"t": f"{t:.4f}", "kind": "det", **pose("target", tgt), **pose("raw", tgt),
                      **pose("tagarm", [c + rnd.gauss(0, 0.5) for c in tgt]), "clamped": "0", "rtt_ms": "5"})
    for k in range(60):
        ticks.append({"t": f"{10 + k / 30:.4f}", "kind": "det", "phase": "hold",
                      **pose("target", (210 + rnd.gauss(0, 0.5), 5 + rnd.gauss(0, 0.5), 100 + rnd.gauss(0, 0.5))),
                      "clamped": "0", "rtt_ms": "5"})
    pre = os.path.join(tempfile.mkdtemp(), "run")
    write(pre, ticks, follower_rows(lambda t: circle(t - 0.15), n=1200))
    r = analyze(pre)
    assert abs(r["lag_ms"] - 150) <= 10 and r["resends"] == 0, r
    assert all(abs(s - 0.5) <= 0.2 for s in r["tagarm_std_mm_axes"]), r
    assert abs(r["tagarm_std_mm"] - 0.5 * 3 ** 0.5) <= 0.3, r
    assert r["hold_n"] == 60 and abs(r["hold_target_mean_mm"][0] - 210) < 0.5, r
    assert abs(r["hold_target_std_mm"] - 0.5 * 3 ** 0.5) <= 0.4, r
    assert analyze(pre, phase="engage")["tagarm_std_mm"] is None  # --phase selects rows


if __name__ == "__main__":
    test_lag_and_rest()
    test_skew()
    test_no_motion_and_blank_cells()
    test_phases()
    print("ok")
