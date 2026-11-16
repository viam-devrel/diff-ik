# /// script
# requires-python = ">=3.11"
# ///
"""Analyse a teleop run: uv run clients/python/teleop_lag.py PREFIX
Reads PREFIX-ticks.csv and PREFIX-follower.csv (written by teleop.py --csv PREFIX)."""
import bisect
import csv
import math
import statistics
import sys


def _rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _interp(ts, vs, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return tuple(a + f * (b - a) for a, b in zip(vs[i - 1], vs[i]))


def analyze(prefix: str) -> dict:
    ticks = [r for r in _rows(prefix + "-ticks.csv") if r["phase"] == "track"]
    if len(ticks) < 2:
        return {"error": "no tracking ticks (engage failed?)"}
    foll = _rows(prefix + "-follower.csv")
    lt = [float(r["t"]) for r in ticks]
    lp = [(float(r["l_x"]), float(r["l_y"]), float(r["l_z"])) for r in ticks]
    ft = [float(r["t"]) for r in foll]
    fp = [(float(r["f_x"]), float(r["f_y"]), float(r["f_z"])) for r in foll]
    t0, t1 = max(lt[0], ft[0]), min(lt[-1], ft[-1])
    grid = [t0 + 0.5 + k * 0.005 for k in range(int((t1 - t0 - 1.0) / 0.005))]  # 200 Hz, 0.5 s trimmed

    # motion only: drop grid points inside held runs (rest)
    held_t = [float(r["t"]) for r in ticks if r["held"] == "1"]
    hs = set(round(t, 2) for t in held_t)
    grid = [t for t in grid if round(t, 2) not in hs and round(t - 0.01, 2) not in hs and round(t + 0.01, 2) not in hs]

    def err_at(lag):  # follower(t) vs leader(t - lag), during motion
        return statistics.fmean(math.dist(_interp(ft, fp, t), _interp(lt, lp, t - lag)) for t in grid)

    lags = [k * 0.005 for k in range(0, 101)]  # 0–500 ms
    best = min(lags, key=err_at) if len(grid) >= 50 else None  # mostly-at-rest runs have no motion to correlate
    # rest (spec): ticks inside a held run lasting >= 2 s, at least 1 s into it, with no clamp
    held, run = [], []
    for r in ticks + [None]:
        if r is not None and r["held"] == "1":
            run.append(r)
            continue
        if run and float(run[-1]["t"]) - float(run[0]["t"]) >= 2.0:
            held += [x for x in run if float(x["t"]) - float(run[0]["t"]) >= 1.0 and x["clamped"] == "0"]
        run = []
    status = [r for r in foll if r["kind"] == "status" and r["tick_p99_ms"] != ""]
    tick_ms = sorted(float(r["tick_ms"]) for r in ticks)
    dur = lt[-1] - lt[0]
    return {
        "lag_ms": round(best * 1000) if best is not None else None,
        "track_err_mm_raw": round(err_at(0.0), 2) if best is not None else None,
        "track_err_mm_at_lag": round(err_at(best), 2) if best is not None else None,
        "rest_converged_frac": round(sum(r["state"] == "converged" for r in held) / len(held), 3) if held else None,
        "rest_err_mm_median": round(statistics.median(float(r["err_mm"]) for r in held), 2) if held else None,
        "rest_err_deg_median": round(statistics.median(float(r["err_deg"]) for r in held), 2) if held else None,
        "clamped_ticks": sum(r["clamped"] == "1" for r in ticks),
        "client_rate_hz": round((len(ticks) - 1) / dur, 2) if dur > 0 else None,
        "client_tick_ms_p50": tick_ms[len(tick_ms) // 2],
        "client_tick_ms_p99": tick_ms[int(len(tick_ms) * 0.99)],
        "max_tick_p99_ms": max(float(r["tick_p99_ms"]) for r in status) if status else None,
        "max_overruns": max(float(r["overruns"]) for r in status) if status else None,
        "max_resyncs": max(float(r["resyncs"]) for r in status) if status else None,
    }


if __name__ == "__main__":
    for k, v in analyze(sys.argv[1]).items():
        print(f"{k:24s} {v}")
