# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""A/B a viam:teleop leader-follower service across motion backends (builtin vs diff-ik).

Records the leader tip and follower TCP from outside the service, so either backend is measured
the same way, then starts the service, runs for --duration s and stops it:
  uv run clients/python/teleop_ab.py --address <machine>.viam.cloud --service teleop --csv .runs/ab-builtin
  uv run clients/python/teleop_ab.py --address <machine>.viam.cloud --service teleop-diffik --csv .runs/ab-diffik
  uv run clients/python/teleop_ab.py --analyze .runs/ab-builtin
Needs VIAM_API_KEY / VIAM_API_KEY_ID. Tip = end position + offset along the tool axis (o_x, o_y, o_z)."""

import argparse
import asyncio
import bisect
import csv
import json
import math
import os
import statistics
import sys
import time


def tip(p: dict, off_mm: float) -> tuple[float, float, float]:
    return (p["x"] + off_mm * p["o_x"], p["y"] + off_mm * p["o_y"], p["z"] + off_mm * p["o_z"])


def _interp(ts, vs, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return tuple(a + f * (b - a) for a, b in zip(vs[i - 1], vs[i]))


def analyze(lt, lp, ft, fp, scale: float) -> dict:
    """lt/lp leader tip samples, ft/fp follower TCP samples (s, mm). The follower should track
    offset + scale × leader (identity origin rotation). Lag = the delay minimising the RMS residual
    after removing the mean offset, over moving grid points only."""
    t0, t1 = max(lt[0], ft[0]) + 0.5, min(lt[-1], ft[-1]) - 0.5
    grid = [t0 + k * 0.01 for k in range(int((t1 - t0) / 0.01))]
    sl = [tuple(scale * c for c in _interp(lt, lp, t)) for t in grid]
    # moving: scaled leader speed > 20 mm/s (over 50 ms)
    mv = [i for i in range(5, len(grid)) if math.dist(sl[i], sl[i - 5]) / 0.05 > 20]

    def resid(lag):
        if len(mv) < 50:
            return None
        d = [tuple(f - scale * l for f, l in zip(_interp(ft, fp, grid[i]), _interp(lt, lp, grid[i] - lag)))
             for i in mv]
        m = [statistics.fmean(c) for c in zip(*d)]
        return math.sqrt(statistics.fmean(sum((a - b) ** 2 for a, b in zip(v, m)) for v in d))

    lags = [k * 0.01 for k in range(0, 81)]  # 0–800 ms
    errs = {lag: resid(lag) for lag in lags}
    best = min(lags, key=lambda x: errs[x]) if errs[0] is not None else None
    # jitter: follower TCP minus its own 150 ms centred moving average, RMS (mm)
    fg = [_interp(ft, fp, t) for t in grid]
    w = 7
    jit = [math.dist(fg[i], tuple(statistics.fmean(c) for c in zip(*fg[i - w:i + w + 1])))
           for i in range(w, len(fg) - w)]
    return {
        "samples_leader_hz": round(len(lt) / (lt[-1] - lt[0]), 1),
        "samples_follower_hz": round(len(ft) / (ft[-1] - ft[0]), 1),
        "moving_s": round(len(mv) * 0.01, 1),
        "lag_ms": round(best * 1000) if best is not None else None,
        "track_rms_mm_at_lag": round(errs[best], 1) if best is not None else None,
        "track_rms_mm_at_0": round(errs[0], 1) if best is not None else None,
        "jitter_rms_mm": round(math.sqrt(statistics.fmean(j * j for j in jit)), 2) if jit else None,
    }


def load(prefix: str):
    with open(prefix + "-poses.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    side = lambda s: ([float(r["t"]) for r in rows if r["side"] == s],
                      [(float(r["x"]), float(r["y"]), float(r["z"])) for r in rows if r["side"] == s])
    return (*side("leader"), *side("follower"))


async def record(a) -> str:
    from viam.components.arm import Arm
    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions
    from viam.services.generic import Generic

    dial = DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    robot = await RobotClient.at_address(a.address, RobotClient.Options(dial_options=dial))
    leader, follower = Arm.from_robot(robot, a.leader), Arm.from_robot(robot, a.follower)
    svc = Generic.from_robot(robot, a.service)
    os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    out = open(a.csv + "-poses.csv", "w", newline="")
    w = csv.writer(out)
    w.writerow(["t", "side", "x", "y", "z"])
    t0 = time.monotonic()
    stop = asyncio.Event()

    async def poll(side, arm, off):
        while not stop.is_set():
            s = time.monotonic()
            p = await arm.get_end_position()
            e = time.monotonic()
            d = {k: getattr(p, k) for k in ("x", "y", "z", "o_x", "o_y", "o_z")}
            w.writerow([f"{(s + e) / 2 - t0:.4f}", side, *(f"{c:.3f}" for c in tip(d, off))])

    summary = {"service": a.service}
    try:
        summary["status_before"] = await svc.do_command({"command": "status"})
        if a.countdown > 0:  # non-interactive: whoever is at the station was told to stand by
            for k in range(int(a.countdown), 0, -1):
                print(f"starting '{a.service}' in {k} s", flush=True)
                await asyncio.sleep(1)
        else:
            await asyncio.to_thread(input, f"Hands on the leader; Enter starts '{a.service}' "
                                           f"(align_on_start may move the leader). ")
        tasks = [asyncio.create_task(poll("leader", leader, a.leader_tip_mm)),
                 asyncio.create_task(poll("follower", follower, a.follower_tip_mm))]
        summary["start"] = await svc.do_command({"command": "start"})
        print(f"running {a.duration:.0f} s; move the leader (Ctrl-C ends early)")
        try:
            await asyncio.sleep(a.duration)
        except asyncio.CancelledError:
            pass
    finally:
        stop.set()
        try:
            summary["status_after"] = await svc.do_command({"command": "status"})
        finally:
            await svc.do_command({"command": "stop"})
            out.close()
            with open(a.csv + "-status.json", "w") as f:
                json.dump(summary, f, indent=1, default=str)
            await robot.close()
    return a.csv


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address")
    ap.add_argument("--service", default="teleop")
    ap.add_argument("--leader", default="leader-arm")
    ap.add_argument("--follower", default="arm")
    ap.add_argument("--leader-tip-mm", type=float, default=100, help="the service's leader_tool_offset_mm z")
    ap.add_argument("--follower-tip-mm", type=float, default=105, help="gripper frame z offset on the arm")
    ap.add_argument("--scale", type=float, default=2.5)
    ap.add_argument("--duration", type=float, default=60)
    ap.add_argument("--countdown", type=float, default=0, help="start after N s instead of waiting for Enter")
    ap.add_argument("--csv", help="output prefix")
    ap.add_argument("--analyze", metavar="PREFIX", help="analyse a recorded run and exit")
    a = ap.parse_args()
    prefix = a.analyze or asyncio.run(record(a))
    print(json.dumps(analyze(*load(prefix), a.scale), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
