# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Block sanding: press the tool onto the workpiece at --target-n with servo_start `force`, then sweep raster passes
over its top face with x/y twists while the module regulates the force along the tool axis. The block is tilted 2°,
so each 160 mm pass also has to follow 5.6 mm of height change. Logs force per pass.

  uv run clients/python/sand.py --insecure --speeds 10,20,40 --csv .runs/sand
Cell: dev/sanding-mujoco.json (localhost:8087).
"""
import argparse
import math
import statistics


def raster(x0: float, x1: float, ys: list) -> list:
    """Waypoints (x, y, pass index or None for a step between passes): along x at each y, alternating direction."""
    pts, left = [], True
    for i, y in enumerate(ys):
        a, b = (x0, x1) if left else (x1, x0)
        if i:
            pts.append((a, y, None))  # step over to the next pass at the end x
        pts.append((b, y, i))
        left = not left
    return pts


def pass_stats(press: list, target: float, band: float) -> dict:
    """press: pressing forces (N) logged during one pass."""
    return {"mean_n": statistics.fmean(press), "std_n": statistics.pstdev(press), "min_n": min(press),
            "max_n": max(press), "in_band": sum(abs(f - target) <= band * target for f in press) / len(press)}


def floats(s: str) -> list:
    v = [float(x) for x in s.split(",")]
    if not v or not all(math.isfinite(x) for x in v):
        raise argparse.ArgumentTypeError("want a comma list of numbers")
    return v


async def run(a):
    import asyncio
    import csv
    import json
    import time

    from viam.components.arm import Arm
    from viam.components.sensor import Sensor
    from viam.services.motion import MotionClient

    from touch_sweep import connect, goto

    conns = await connect(a, 3)
    svc, ft, arm = (MotionClient.from_robot(conns[0], a.service), Sensor.from_robot(conns[1], a.sensor),
                    Arm.from_robot(conns[2], a.arm))
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731
    pts = raster(a.x0, a.x1, a.ys)
    start = {"x": a.x0, "y": a.ys[0], "z": a.start_z, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}
    rows, trace_f = [], open(f"{a.csv}-trace.csv", "w", newline="") if a.csv else None
    trace = csv.writer(trace_f) if trace_f else None
    if trace:
        trace.writerow(["speed", "t", "seg", "x", "y", "z", "press_n"])
    try:
        for v in a.speeds:
            await goto(svc, a.arm, start)
            await asyncio.sleep(0.5)
            fz0 = (await ft.get_readings())["force"]["z"]
            sid = (await svc.do_command({
                "command": "servo_start", "component_name": a.arm, "frame": "world", "mode": "twist",
                "limits": {"linear_mm_s": max(v, a.max_mm_s) * 1.5}, "timeout_ms": 500,
                "force": {"sensor": a.sensor, "target_n": a.target_n, "gain": a.gain, "ki": a.ki,
                          "max_mm_s": a.max_mm_s},
                "guard": {"sensor": a.sensor, "force_n": a.guard_n}}))["session_id"]
            state = {"seg": -1, "pose": None, "done": False}
            log = []  # (t, seg, press)

            async def poll():
                while not state["done"]:
                    p = fz0 - (await ft.get_readings())["force"]["z"]
                    log.append((now(), state["seg"], p))
                    if trace and state["pose"]:
                        q = state["pose"]
                        trace.writerow([v, f"{now():.4f}", state["seg"], f"{q.x:.2f}", f"{q.y:.2f}", f"{q.z:.2f}",
                                        f"{p:.3f}"])

            async def twist(vx=0.0, vy=0.0):
                st = await svc.do_command({"command": "servo_twist", "session_id": sid, "linear": [vx, vy, 0],
                                           "angular": [0, 0, 0]})
                return st

            poller = asyncio.create_task(poll())
            try:
                # 1. press: hold still until in contact and settled
                deadline = now() + 15
                while True:
                    await twist()
                    st = await svc.do_command({"command": "servo_status", "session_id": sid})
                    if st["state"].startswith("stopped") or st["state"] == "faulted":
                        raise RuntimeError(f"session ended while pressing: {st['state']} {st.get('guard_reason', '')}")
                    if st.get("force_contact"):
                        break
                    if now() > deadline:
                        raise RuntimeError("no contact within 15 s")
                    await asyncio.sleep(0.02)
                settle_end = now() + a.settle_s
                while now() < settle_end:
                    await twist()
                    await asyncio.sleep(0.02)
                # 2. raster: a P controller on x/y toward each waypoint, capped at the sweep speed
                for k, (gx, gy, seg) in enumerate(pts):
                    state["seg"] = -2 if seg is None else seg
                    while True:
                        q = await arm.get_end_position()  # arm base frame: same x/y as world here
                        state["pose"] = q
                        ex, ey = gx - q.x, gy - q.y
                        d = math.hypot(ex, ey)
                        if d < 1.0:
                            break
                        s = min(v, a.kp * d) / d
                        r = await twist(ex * s, ey * s)
                        if r.get("state", "").startswith("stopped") or r.get("state") == "faulted":
                            raise RuntimeError(f"session ended mid-raster: {r}")
                        await asyncio.sleep(0.02)
                state["seg"] = -3
                await twist()
            finally:
                state["done"] = True
                await poller
                final = await svc.do_command({"command": "servo_stop", "session_id": sid})
            by = {}
            for t, seg, p in log:
                by.setdefault(seg, []).append(p)
            for seg in sorted(s for s in by if s >= 0):
                m = pass_stats(by[seg], a.target_n, a.band)
                m.update(speed=v, pass_=seg, state=final.get("state"))
                rows.append(m)
                print(json.dumps({q: round(x, 3) if isinstance(x, float) else x for q, x in m.items()}))
        await goto(svc, a.arm, start)
    finally:
        if trace_f:
            trace_f.close()
        if a.csv and rows:
            with open(f"{a.csv}-passes.csv", "w", newline="") as f:
                w = csv.DictWriter(f, sorted(rows[0]))
                w.writeheader()
                w.writerows(rows)
        for c in conns:
            await c.close()


def main():
    import asyncio
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8087")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--sensor", default="wrist-ft")
    ap.add_argument("--speeds", type=floats, default=[20.0], help="sweep speeds, mm/s; one full raster each")
    ap.add_argument("--target-n", type=float, default=10)
    ap.add_argument("--gain", type=float, default=0.5, help="force.gain, mm/s per N")
    ap.add_argument("--ki", type=float, default=4, help="force.ki, mm/s per N·s")
    ap.add_argument("--max-mm-s", type=float, default=5, help="force.max_mm_s")
    ap.add_argument("--guard-n", type=float, default=40, help="guard trip, N")
    ap.add_argument("--band", type=float, default=0.2, help="in-band tolerance, fraction of the target")
    ap.add_argument("--settle-s", type=float, default=1.0)
    ap.add_argument("--kp", type=float, default=3.0, help="x/y tracking gain toward each waypoint, 1/s")
    ap.add_argument("--x0", type=float, default=470)
    ap.add_argument("--x1", type=float, default=630)
    ap.add_argument("--ys", type=floats, default=[-50, -25, 0, 25, 50], help="pass y values, mm")
    ap.add_argument("--start-z", type=float, default=515, help="flange z above the first pass, mm")
    ap.add_argument("--csv", help="prefix: PREFIX-passes.csv and PREFIX-trace.csv")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
