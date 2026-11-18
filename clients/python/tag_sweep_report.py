# /// script
# requires-python = ">=3.11"
# ///
"""Phase 4: a per-speed tracking table for tag_sweep.py runs.

  uv run clients/python/tag_sweep_report.py PREFIX [PREFIX ...] [--md OUT.md]
Reads PREFIX-tag.csv, -joints.csv, -ticks.csv, -follower.csv and -summary.json.
Spec: docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md"""
import argparse
import collections
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_lag import _interp, _rows  # noqa: E402

LAGS = [k * 0.005 for k in range(-100, 101)]  # -0.5 .. +0.5 s


def jaw_tip(r: dict, tip_mm: float) -> tuple:
    """End-effector position plus tip_mm along the tool axis (Viam's orientation vector)."""
    n = math.sqrt(sum(float(r[k]) ** 2 for k in ("o_x", "o_y", "o_z")))
    return tuple(float(r[k]) + tip_mm * float(r[o]) / n for k, o in (("x", "o_x"), ("y", "o_y"), ("z", "o_z")))


def horiz_errors(ts, tips, tag_t, tag_xy, lag=0.0) -> list:
    """Horizontal mm from each jaw tip to the tag at t - lag; samples outside the tag log are skipped."""
    out = []
    for t, p in zip(ts, tips):
        u = t - lag
        if tag_t[0] <= u <= tag_t[-1]:
            x, y = _interp(tag_t, tag_xy, u)
            out.append(math.hypot(p[0] - x, p[1] - y))
    return out


def best_lag(ts, tips, tag_t, tag_xy) -> tuple:
    def mean_at(lag):
        e = horiz_errors(ts, tips, tag_t, tag_xy, lag)
        return statistics.fmean(e) if e else math.inf

    lag = min(LAGS, key=mean_at)
    return lag, mean_at(lag)


def stats(vs) -> dict:
    s = sorted(vs)
    return {"mean": statistics.fmean(s), "p95": s[min(len(s) - 1, int(0.95 * len(s)))], "max": s[-1]}


def accel_peak(ts, qs, win=5) -> list:
    """Peak |acceleration| per joint (units/s²): a win-sample moving average of positions and timestamps,
    then central second differences. Unsmoothed stamps would amplify read-time jitter twice over."""
    if len(qs) < win + 2:
        return []
    h = win // 2
    t2 = [statistics.fmean(ts[i - h:i + h + 1]) for i in range(h, len(ts) - h)]  # smooth stamps too: jitter
    q2 = [[statistics.fmean(q[j] for q in qs[i - h:i + h + 1]) for j in range(len(qs[0]))]
          for i in range(h, len(qs) - h)]
    peak = [0.0] * len(qs[0])
    for i in range(1, len(t2) - 1):
        for j in range(len(peak)):
            v1 = (q2[i][j] - q2[i - 1][j]) / (t2[i] - t2[i - 1])
            v2 = (q2[i + 1][j] - q2[i][j]) / (t2[i + 1] - t2[i])
            peak[j] = max(peak[j], abs(v2 - v1) / ((t2[i + 1] - t2[i - 1]) / 2))
    return peak


def segments(tag) -> dict:
    """Per speed index: the speed, its whole window [t0, t1] and its steady window [s0, s1] (laps >= 1)."""
    segs = {}
    for r in tag:
        t, k, lap = float(r["t"]), int(r["seg"]), int(r["lap"])
        s = segs.setdefault(k, {"speed": float(r["seg_speed"]), "t0": t, "t1": t, "s0": None, "s1": None})
        s["t1"] = t
        if lap >= 1:
            s["s0"] = t if s["s0"] is None else s["s0"]
            s["s1"] = t
    return segs


def analyze(prefix: str) -> dict:
    with open(prefix + "-summary.json") as f:
        summ = json.load(f)
    tip = float(summ["args"]["tip_mm"])
    tag, arm = _rows(prefix + "-tag.csv"), _rows(prefix + "-joints.csv")
    if not tag or not arm:  # lost or failed before the sweep: nothing to measure
        return {"prefix": prefix, "camera": summ["args"]["camera"],
                "breaking_speed_mm_s": summ["breaking_speed_mm_s"], "rows": []}
    ticks = _rows(prefix + "-ticks.csv")
    status = [r for r in _rows(prefix + "-follower.csv") if r["kind"] == "status"]
    tag_t = [float(r["t"]) for r in tag]
    tag_xy = [(float(r["x"]), float(r["y"])) for r in tag]
    at = [float(r["t"]) for r in arm]
    tips = [jaw_tip(r, tip) for r in arm]
    qcols = [c for c in arm[0] if c.startswith("q")]
    qs = [[float(r[c]) for c in qcols] for r in arm]
    rows = []
    for _, s in sorted(segments(tag).items()):
        row = {"speed": s["speed"], "lost": summ["breaking_speed_mm_s"] == s["speed"]}
        if s["s0"] is not None:
            idx = [i for i, t in enumerate(at) if s["s0"] <= t <= s["s1"]]
            ts, tp = [at[i] for i in idx], [tips[i] for i in idx]
            e = horiz_errors(ts, tp, tag_t, tag_xy)
            if e:
                row["err"] = stats(e)
            if len(ts) > 10:
                lag, el = best_lag(ts, tp, tag_t, tag_xy)
                row["lag_ms"], row["err_at_lag"] = round(lag * 1000), el

        def inwin(r):
            return s["t0"] <= float(r["t"]) <= s["t1"]

        dur = max(s["t1"] - s["t0"], 1e-9)
        row["det_hz"] = sum(1 for r in ticks if r["kind"] == "det" and inwin(r)) / dur
        drops = [r["state"] for r in ticks if r["kind"] == "drop" and inwin(r)]
        row["drops"], row["drop_reasons"] = len(drops), dict(collections.Counter(drops))
        st = [r for r in status if inwin(r)]
        if st:
            row["resyncs"] = int(float(st[-1]["resyncs"])) - int(float(st[0]["resyncs"]))
            row["overruns"] = int(float(st[-1]["overruns"])) - int(float(st[0]["overruns"]))
            lags = [float(r["lag_ms"]) for r in st if r["lag_ms"] not in ("", "None")]
            row["max_lag_ms"] = max(lags) if lags else None
        wq = [i for i, t in enumerate(at) if s["t0"] <= t <= s["t1"]]
        if len(wq) > 7:
            row["peak_accel"] = max(accel_peak([at[i] for i in wq], [qs[i] for i in wq]))
        rows.append(row)
    return {"prefix": prefix, "camera": summ["args"]["camera"],
            "breaking_speed_mm_s": summ["breaking_speed_mm_s"], "rows": rows}


def _f(v, nd=1):
    return "–" if v is None else f"{v:.{nd}f}"


def markdown(results: list) -> str:
    out = ["| Camera | Speed (mm/s) | Error mean / p95 / max (mm) | Lag (ms) | Error at lag (mm) | Detections (Hz) "
           "| Drops | Lost | Resyncs | Overruns | Max lag_ms | Peak joint accel (°/s²) |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for res in results:
        for r in res["rows"]:
            e = r.get("err")
            err = f"{_f(e['mean'])} / {_f(e['p95'])} / {_f(e['max'])}" if e else "–"
            out.append(f"| {res['camera']} | {r['speed']:g} | {err} | {r.get('lag_ms', '–')} | "
                       f"{_f(r.get('err_at_lag'))} | {_f(r['det_hz'])} | {r['drops']} | {'yes' if r['lost'] else 'no'} | "
                       f"{r.get('resyncs', '–')} | {r.get('overruns', '–')} | {_f(r.get('max_lag_ms'), 0)} | "
                       f"{_f(r.get('peak_accel'), 0)} |")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prefixes", nargs="+")
    ap.add_argument("--md", default=None, help="also write the table to this file")
    a = ap.parse_args()
    md = markdown([analyze(p) for p in a.prefixes])
    print(md)
    if a.md:
        with open(a.md, "w") as f:
            f.write(md + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
