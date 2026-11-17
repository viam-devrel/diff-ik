# /// script
# requires-python = ">=3.11"
# ///
"""Analyse a tag_servo run: uv run clients/python/tag_lag.py PREFIX [--rest all]
Reads PREFIX-ticks.csv and PREFIX-follower.csv (written by tag_servo.py --csv PREFIX).
--rest all treats the whole run as rest (the filter-off static-noise run)."""
import argparse
import bisect
import math
import statistics
import sys
from collections import Counter

from teleop_lag import _interp, _rows

XYZ = ("x", "y", "z")


def _f(r, k):
    return float(r[k]) if r.get(k, "") != "" else None


def _pct(vs, p):
    vs = sorted(vs)
    return vs[min(len(vs) - 1, int(len(vs) * p))] if vs else None


def _rest_flags(ts, ps):
    """Per det row: True if the 0.5 s trailing mean of the target moved < 0.5 mm vs the 0.5 s before it."""
    pre = [[0.0] for _ in range(3)]
    for p in ps:
        for a in range(3):
            pre[a].append(pre[a][-1] + p[a])

    def mean(lo, hi):  # rows [lo, hi)
        return [(pre[a][hi] - pre[a][lo]) / (hi - lo) for a in range(3)] if hi > lo else None

    flags = []
    for i, t in enumerate(ts):
        b = bisect.bisect_right(ts, t - 0.5)
        c = bisect.bisect_right(ts, t - 1.0)
        cur, prev = mean(b, i + 1), mean(c, b)
        flags.append(t - 1.0 >= ts[0] and prev is not None and math.dist(cur, prev) < 0.5)
    return flags


def _rest_rows(det, flags, rest):
    """Rows of rest runs lasting >= 2 s (or every det row with rest='all')."""
    if rest == "all":
        return list(range(len(det)))
    out, run = [], []
    for i in range(len(det) + 1):
        if i < len(det) and flags[i]:
            run.append(i)
            continue
        if run and float(det[run[-1]]["t"]) - float(det[run[0]]["t"]) >= 2.0:
            out += run
        run = []
    return out


def _normal_std_deg(rows):
    ax = [(_f(r, "raw_o_x"), _f(r, "raw_o_y"), _f(r, "raw_o_z")) for r in rows]
    ax = [a for a in ax if None not in a and math.hypot(*a) > 0]
    if not ax:
        return None
    ax = [tuple(c / math.hypot(*a) for c in a) for a in ax]
    m = [statistics.fmean(a[i] for a in ax) for i in range(3)]
    n = math.hypot(*m)
    if n == 0:
        return None
    m = [c / n for c in m]
    ang = [math.degrees(math.acos(max(-1.0, min(1.0, sum(c * d for c, d in zip(a, m)))))) for a in ax]
    return math.sqrt(statistics.fmean(a * a for a in ang))


def _std3(rows, pre):
    """Per-axis and total std of a pose's xyz over rows, or (None, None)."""
    rows = [r for r in rows if r.get(f"{pre}_x", "") != ""]
    if len(rows) < 2:
        return None, None
    axes = [statistics.stdev(float(r[f"{pre}_{k}"]) for r in rows) for k in XYZ]
    return [round(s, 2) for s in axes], round(math.sqrt(sum(s * s for s in axes)), 2)


def analyze(prefix: str, rest: str = "auto", phase: str = "track") -> dict:
    allrows = _rows(prefix + "-ticks.csv")
    rows = allrows
    if phase != "all" and allrows and "phase" in allrows[0]:  # old CSVs have no phase column
        rows = [r for r in allrows if r["phase"] == phase]
    foll = [r for r in _rows(prefix + "-follower.csv") if r["kind"] == "pose" and r["f_x"] != ""]
    status = [r for r in _rows(prefix + "-follower.csv") if r["kind"] == "status" and r["tick_p99_ms"] != ""]
    det = [r for r in rows if r["kind"] == "det" and r["target_x"] != ""]
    kinds = Counter(r["kind"] for r in rows)
    out = {
        "resends": kinds["resend"], "frozen": kinds["frozen"],
        "clamped": sum(r["clamped"] in ("1", "True") for r in rows if r["kind"] in ("det", "resend")),
        "drops": dict(Counter(r["state"] for r in rows if r["kind"] == "drop")),
        "max_tick_p99_ms": max(float(r["tick_p99_ms"]) for r in status) if status else None,
        "max_overruns": max(float(r["overruns"]) for r in status) if status else None,
        "max_resyncs": max(float(r["resyncs"]) for r in status) if status else None,
    }
    rtt = [float(r["rtt_ms"]) for r in rows if r.get("rtt_ms", "") != ""]
    out["rtt_ms_p50"], out["rtt_ms_p99"] = _pct(rtt, 0.5), _pct(rtt, 0.99)
    hold = [r for r in allrows if r.get("phase") == "hold" and r["kind"] != "drop" and r["target_x"] != ""]  # look-then-move holds are "frozen" rows
    out["hold_n"] = len(hold)
    out["hold_target_mean_mm"] = ([round(statistics.fmean(float(r[f"target_{k}"]) for r in hold), 2) for k in XYZ]
                                  if hold else None)
    out["hold_target_std_mm_axes"], out["hold_target_std_mm"] = _std3(hold, "target")
    none = dict.fromkeys(("det_rate_hz", "lag_ms", "track_err_mm_at_lag", "skew_ms", "raw_std_mm",
                          "raw_std_mm_axes", "raw_normal_std_deg", "rest_s", "tagarm_std_mm_axes",
                          "tagarm_std_mm", "tagarm_mean_mm"))
    if len(det) < 2 or len(foll) < 2:
        return {**none, **out}
    dt = [float(r["t"]) for r in det]
    dp = [tuple(float(r[f"target_{k}"]) for k in XYZ) for r in det]
    ft = [float(r["t"]) for r in foll]
    fp = [tuple(float(r[f"f_{k}"]) for k in XYZ) for r in foll]
    out["det_rate_hz"] = round((len(det) - 1) / (dt[-1] - dt[0]), 2) if dt[-1] > dt[0] else None

    flags = _rest_flags(dt, dp)
    rest_idx = _rest_rows(det, flags, rest)
    rs = set(rest_idx)
    moving = [i for i in range(len(det)) if i not in rs]

    # lag: follower(t) vs target(t - lag), on a 200 Hz grid over motion
    t0, t1 = max(dt[0], ft[0]) + 0.5, min(dt[-1], ft[-1]) - 0.5
    grid = []
    for k in range(max(0, int((t1 - t0) / 0.005))):
        t = t0 + k * 0.005
        j = min(bisect.bisect_left(dt, t), len(dt) - 1)
        if j > 0 and t - dt[j - 1] < dt[j] - t:
            j -= 1
        if j not in rs:
            grid.append(t)

    def err_at(lag):
        return statistics.fmean(math.dist(_interp(ft, fp, t), _interp(dt, dp, t - lag)) for t in grid)

    # mostly-at-rest runs, or a follower that never moved, have nothing to correlate
    moved = len(grid) >= 50 and max(math.dist(_interp(ft, fp, t), fp[0]) for t in grid) > 5
    best = min((k * 0.005 for k in range(101)), key=err_at) if moved else None
    out["lag_ms"] = round(best * 1000) if best is not None else None
    out["track_err_mm_at_lag"] = round(err_at(best), 2) if best is not None else None

    # skew: raw deviation from its mean while the follower is still vs follower velocity; slope = tau
    out["skew_ms"] = None
    fv = [tuple((fp[min(j + 1, len(ft) - 1)][a] - fp[max(j - 1, 0)][a]) /
                (ft[min(j + 1, len(ft) - 1)] - ft[max(j - 1, 0)]) for a in range(3)) for j in range(len(ft))]
    srows = [(_interp(ft, fv, dt[i]), tuple(float(det[i][f"raw_{k}"]) for k in XYZ))
             for i in range(len(det)) if ft[0] <= dt[i] <= ft[-1] and det[i]["raw_x"] != ""]
    still = [p for v, p in srows if math.hypot(*v) <= 5]
    if still:
        mean = [statistics.fmean(p[a] for p in still) for a in range(3)]
        num = den = 0.0
        for v, p in srows:
            if math.hypot(*v) > 5:
                num += sum((p[a] - mean[a]) * v[a] for a in range(3))
                den += sum(c * c for c in v)
        if den > 1e-9:
            out["skew_ms"] = round(num / den * 1000)

    # rest window: raw scatter
    rr = [det[i] for i in rest_idx if det[i]["raw_x"] != ""]
    out["rest_s"] = round(dt[rest_idx[-1]] - dt[rest_idx[0]], 2) if rest_idx else None
    if len(rr) >= 2:
        axes = [statistics.stdev(float(r[f"raw_{k}"]) for r in rr) for k in XYZ]
        out["raw_std_mm_axes"] = [round(s, 2) for s in axes]
        out["raw_std_mm"] = round(math.sqrt(sum(s * s for s in axes)), 2)
        n = _normal_std_deg(rr)
        out["raw_normal_std_deg"] = round(n, 2) if n is not None else None
    else:
        out["raw_std_mm_axes"] = out["raw_std_mm"] = out["raw_normal_std_deg"] = None
    out["tagarm_std_mm_axes"], out["tagarm_std_mm"] = _std3([det[i] for i in rest_idx], "tagarm")
    tr = [det[i] for i in rest_idx if det[i].get("tagarm_x", "") != ""]
    out["tagarm_mean_mm"] = [round(statistics.fmean(float(r[f"tagarm_{k}"]) for r in tr), 2) for k in XYZ] if tr else None
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix")
    ap.add_argument("--rest", choices=["auto", "all"], default="auto")
    ap.add_argument("--phase", choices=["track", "engage", "descent", "hold", "all"], default="track")
    a = ap.parse_args()
    for k, v in analyze(a.prefix, a.rest, a.phase).items():
        print(f"{k:24s} {v}")
