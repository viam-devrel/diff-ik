# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Press and hold: a twist session with servo_start `force` brings the tool down onto the table and regulates the
pressing force at --target-n. Sweeps the force gain and logs the step response: peak, overshoot, settle time and the
steady-state mean and spread.

  uv run clients/python/press_hold.py --insecure --gains 0.05,0.1,0.2 --target-n 10 --csv .runs/press
Cell: dev/ft-mujoco.json.
"""
import argparse
import math
import statistics


def hold_metrics(press: list, target: float, contact_n: float, band: float, tail_s: float) -> dict:
    """press: (t, pressing force N) rows. Contact is the first row >= contact_n. Settled is the first time after which
    every row stays within ±band·target of target. Steady stats are over the last tail_s s."""
    tc = next((t for t, f in press if f >= contact_n), None)
    if tc is None:
        return {"contact": False}
    after = [(t, f) for t, f in press if t >= tc]
    peak = max(f for _, f in after)
    settled = None
    for t, f in reversed(after):
        if abs(f - target) > band * target:
            break
        settled = t
    tail = [f for t, f in after if t >= after[-1][0] - tail_s]
    return {"contact": True, "peak_n": peak, "overshoot_pct": 100 * (peak - target) / target,
            "settle_ms": None if settled is None else (settled - tc) * 1e3,
            "mean_n": statistics.fmean(tail), "std_n": statistics.pstdev(tail)}


def floats(s: str) -> list:
    v = [float(x) for x in s.split(",")]
    if not v or not all(x > 0 and math.isfinite(x) for x in v):
        raise argparse.ArgumentTypeError("want a comma list of values > 0")
    return v


async def run(a):
    import asyncio
    import csv
    import json
    import time

    from viam.components.sensor import Sensor
    from viam.services.motion import MotionClient

    from touch_sweep import connect, goto

    conns = await connect(a, 3)
    svc, ft, svc2 = (MotionClient.from_robot(conns[0], a.service), Sensor.from_robot(conns[1], a.sensor),
                     MotionClient.from_robot(conns[2], a.service))
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731
    start = {"x": a.x, "y": a.y, "z": a.start_z, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}
    out, trace_f = [], open(f"{a.csv}-trace.csv", "w", newline="") if a.csv else None
    trace = csv.writer(trace_f) if trace_f else None
    if trace:
        trace.writerow(["gain", "target_n", "trial", "t", "press_n"])
    try:
        for gain in a.gains:
            for target in a.targets:
                for k in range(a.trials):
                    await goto(svc, a.arm, start)
                    await asyncio.sleep(0.5)
                    r0 = await ft.get_readings()
                    fz0 = r0["force"]["z"]
                    force = {"sensor": a.sensor, "target_n": target, "gain": gain, "max_mm_s": a.max_mm_s,
                             "approach_mm_s": a.approach_mm_s}
                    cmd = {"command": "servo_start", "component_name": a.arm, "frame": "world", "mode": "twist",
                           "rate_hz": a.rate_hz, "limits": {"linear_mm_s": max(a.approach_mm_s, a.max_mm_s)},
                           "timeout_ms": 500, "force": force}
                    if a.guard_n:
                        cmd["guard"] = {"sensor": a.sensor, "force_n": a.guard_n}
                    sid = (await svc.do_command(cmd))["session_id"]
                    press, done = [], False

                    async def stream():
                        while not done:
                            await svc2.do_command({"command": "servo_twist", "session_id": sid,
                                                   "linear": [0, 0, 0], "angular": [0, 0, 0]})
                            await asyncio.sleep(0.05)

                    task = asyncio.create_task(stream())
                    contact_at = None
                    end = now() + a.travel_mm / a.approach_mm_s + 2
                    while now() < end:
                        p = fz0 - (await ft.get_readings())["force"]["z"]
                        press.append((now(), p))
                        if contact_at is None and p >= a.contact_n:
                            contact_at = now()
                            end = contact_at + a.hold_s
                    done = True
                    await task
                    final = await svc.do_command({"command": "servo_stop", "session_id": sid})
                    m = hold_metrics(press, target, a.contact_n, a.band, a.tail_s)
                    m.update(gain=gain, target_n=target, trial=k, state=final.get("state"),
                             resyncs=final.get("resyncs"))
                    out.append(m)
                    print(json.dumps({q: round(x, 2) if isinstance(x, float) else x for q, x in m.items()}))
                    if trace:
                        trace.writerows([gain, target, k, f"{t:.4f}", f"{p:.3f}"] for t, p in press)
        await goto(svc, a.arm, start)
    finally:
        if trace_f:
            trace_f.close()
        if a.csv and out:
            cols = sorted({q for r in out for q in r})
            with open(f"{a.csv}-trials.csv", "w", newline="") as f:
                w = csv.DictWriter(f, cols)
                w.writeheader()
                w.writerows(out)
        for c in conns:
            await c.close()


def main():
    import asyncio
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8086")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--sensor", default="wrist-ft")
    ap.add_argument("--gains", type=floats, default=[0.5], help="force.gain values, mm/s per N")
    ap.add_argument("--targets", type=floats, default=[10.0], help="force.target_n values, N")
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--rate-hz", type=float, default=50, help="servo loop rate")
    ap.add_argument("--max-mm-s", type=float, default=5)
    ap.add_argument("--approach-mm-s", type=float, default=5)
    ap.add_argument("--guard-n", type=float, default=0, help="also pass a guard at this force (0 = none)")
    ap.add_argument("--contact-n", type=float, default=1.0, help="client's contact mark, N")
    ap.add_argument("--hold-s", type=float, default=4.0, help="log this long after contact")
    ap.add_argument("--band", type=float, default=0.1, help="settle band, fraction of the target")
    ap.add_argument("--tail-s", type=float, default=1.5, help="steady stats over the last this many s")
    ap.add_argument("--x", type=float, default=550)
    ap.add_argument("--y", type=float, default=0)
    ap.add_argument("--start-z", type=float, default=450, help="flange z at the start, mm (contact is at ~432)")
    ap.add_argument("--travel-mm", type=float, default=60)
    ap.add_argument("--csv", help="prefix: PREFIX-trials.csv and PREFIX-trace.csv")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
