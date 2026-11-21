# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Guarded touch-off, client side: twist the tool straight down onto the table at each speed, poll the wrist F/T
sensor as fast as gRPC allows, and servo_stop when |F| (tared) crosses --threshold-n. Logs peak force, detection
latency, overshoot past first contact, and the force left after the stop.

  uv run clients/python/touch_sweep.py --insecure --speeds 10,20,40,80 --trials 3 --csv .runs/touch-client
Cell: dev/ft-mujoco.json. Pass --guard to have the servo module stop on force instead (step 2).
"""
import argparse
import math


def first_cross(rows: list, n: float) -> float | None:
    """Time of the first (t, |F|) row with |F| >= n, linearly interpolated from the row before; None if never."""
    for (t0, f0), (t1, f1) in zip(rows, rows[1:]):
        if f1 >= n:
            return t1 if f1 == f0 or f0 >= n else t0 + (n - f0) / (f1 - f0) * (t1 - t0)
    return None


def interp(rows: list, t: float) -> float:
    """Value of (t, v) rows at time t, clamped to the ends."""
    if t <= rows[0][0]:
        return rows[0][1]
    for (t0, v0), (t1, v1) in zip(rows, rows[1:]):
        if t <= t1:
            return v0 if t1 == t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return rows[-1][1]


def trial_metrics(force: list, z: list, threshold: float, contact_n: float, settle_s: float) -> dict:
    """force: (t, |F| N) rows, z: (t, flange z mm) rows, same clock. Contact is the first |F| >= contact_n; the
    detection is the first >= threshold. Residual is the mean |F| over the last settle_s s."""
    tc, td = first_cross(force, contact_n), first_cross(force, threshold)
    if tc is None or td is None:
        return {"contact": False, "peak_n": max(f for _, f in force)}
    tail = [f for t, f in force if t >= force[-1][0] - settle_s]
    tp, peak = max(force, key=lambda r: r[1])
    zc = interp(z, tc)
    return {"contact": True, "peak_n": peak, "t_peak_ms": (tp - tc) * 1e3, "detect_ms": (td - tc) * 1e3,
            "contact_z_mm": zc, "overshoot_mm": zc - z[-1][1], "residual_n": sum(tail) / len(tail)}


def speeds_arg(s: str) -> list:
    v = [float(x) for x in s.split(",")]
    if not v or not all(x > 0 for x in v):
        raise argparse.ArgumentTypeError("--speeds needs a comma list of values > 0")
    return v


def norm(r: dict) -> float:
    f = r["force"]
    return math.sqrt(f["x"] ** 2 + f["y"] ** 2 + f["z"] ** 2)


async def connect(a, n: int):
    """n connections, so the F/T poll, the command stream and pose reads don't queue behind each other."""
    import os

    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions

    if a.insecure:
        dial = DialOptions(insecure=True, disable_webrtc=True)
    else:  # a cloud machine: same env vars as servo_client.connect
        dial = DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    return [await RobotClient.at_address(a.address, RobotClient.Options(dial_options=dial)) for _ in range(n)]


async def goto(svc, arm_name: str, pose: dict):
    """Servo to pose (world, mm) in its own pose session and stop."""
    import asyncio

    r = await svc.do_command({"command": "servo_start", "component_name": arm_name, "frame": "world", "mode": "pose",
                              "limits": {"linear_mm_s": 100}, "timeout_ms": 1000})
    sid = r["session_id"]
    try:
        for _ in range(500):
            st = await svc.do_command({"command": "servo_target", "session_id": sid, "pose": pose})
            if st["state"] == "converged":
                return
            await asyncio.sleep(0.02)
        raise RuntimeError(f"start pose not reached: {st}")
    finally:
        await svc.do_command({"command": "servo_stop", "session_id": sid})


async def run(a):
    import asyncio
    import csv
    import json
    import os
    import time

    from viam.components.arm import Arm
    from viam.components.sensor import Sensor
    from viam.services.motion import MotionClient

    conns = await connect(a, 3)
    svc = MotionClient.from_robot(conns[0], a.service)
    ft = Sensor.from_robot(conns[1], a.sensor)
    arm = Arm.from_robot(conns[2], a.arm)
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731
    start = {"x": a.x, "y": a.y, "z": a.start_z, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}

    rows_out = []
    trace_f = open(f"{a.csv}-trace.csv", "w", newline="") if a.csv else None
    trace = csv.writer(trace_f) if trace_f else None
    if trace:
        trace.writerow(["speed", "trial", "t", "kind", "value"])
    try:
        for v in a.speeds:
            for k in range(a.trials):
                await goto(svc, a.arm, start)
                await asyncio.sleep(0.5)
                await ft.do_command({"command": "tare"})
                start_cmd = {"command": "servo_start", "component_name": a.arm, "frame": "world", "mode": "twist",
                             "limits": {"linear_mm_s": v}, "timeout_ms": 500, "ramp_ms": a.ramp_ms}
                if a.guard:
                    start_cmd["guard"] = {"sensor": a.sensor, "force_n": a.threshold_n}
                sid = (await svc.do_command(start_cmd))["session_id"]
                force, z = [], []
                stop_at = None  # client time of the servo_stop (or the guard's stop seen in status)
                final = {}

                async def stream():
                    while stop_at is None:
                        await svc.do_command({"command": "servo_twist", "session_id": sid,
                                              "linear": [0, 0, -v], "angular": [0, 0, 0]})
                        await asyncio.sleep(0.02)

                async def poses():
                    while True:
                        p = await arm.get_end_position()
                        z.append((now(), p.z))

                tasks = [asyncio.create_task(stream()), asyncio.create_task(poses())]
                deadline = now() + a.travel_mm / v + 2
                while now() < deadline:
                    f = norm(await ft.get_readings())
                    force.append((now(), f))
                    if stop_at is None and f >= a.threshold_n and not a.guard:
                        stop_at = now()
                        final = await svc.do_command({"command": "servo_stop", "session_id": sid})
                        deadline = now() + a.settle_s
                    if a.guard and stop_at is None and len(force) % 20 == 0:
                        st = await svc.do_command({"command": "servo_status", "session_id": sid})
                        if st["state"].startswith("stopped"):
                            stop_at, final = now(), st
                            deadline = now() + a.settle_s
                if stop_at is None:
                    stop_at = now()
                    final = await svc.do_command({"command": "servo_stop", "session_id": sid})
                for t in tasks:
                    t.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                m = trial_metrics(force, z, a.threshold_n, a.contact_n, a.settle_s)
                m.update(speed=v, trial=k, state=final.get("state"), resyncs=final.get("resyncs"),
                         ft_hz=len(force) / (force[-1][0] - force[0][0]))
                rows_out.append(m)
                print(json.dumps({q: round(x, 2) if isinstance(x, float) else x for q, x in m.items()}))
                if trace:
                    trace.writerows([v, k, f"{t:.4f}", "force", f"{f:.3f}"] for t, f in force)
                    trace.writerows([v, k, f"{t:.4f}", "z", f"{q:.3f}"] for t, q in z)
        await goto(svc, a.arm, start)
    finally:
        if trace_f:
            trace_f.close()
        if a.csv and rows_out:
            cols = sorted({q for r in rows_out for q in r})
            with open(f"{a.csv}-trials.csv", "w", newline="") as f:
                w = csv.DictWriter(f, cols)
                w.writeheader()
                w.writerows(rows_out)
        for c in conns:
            await c.close()


def main():
    import asyncio
    import os

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8086")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--sensor", default="wrist-ft")
    ap.add_argument("--speeds", type=speeds_arg, default=[10, 20, 40, 80], help="approach speeds, mm/s")
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--threshold-n", type=float, default=5.0, help="stop when tared |F| reaches this")
    ap.add_argument("--contact-n", type=float, default=1.0, help="|F| that marks first contact (above noise)")
    ap.add_argument("--ramp-ms", type=float, default=200)
    ap.add_argument("--x", type=float, default=550)
    ap.add_argument("--y", type=float, default=0)
    ap.add_argument("--start-z", type=float, default=470, help="flange z at the start, mm (table top is 300)")
    ap.add_argument("--travel-mm", type=float, default=80, help="give up after this much commanded travel")
    ap.add_argument("--settle-s", type=float, default=1.0, help="keep logging this long after the stop")
    ap.add_argument("--guard", action="store_true", help="servo_start guard: the module stops on force (step 2)")
    ap.add_argument("--csv", help="prefix: PREFIX-trials.csv and PREFIX-trace.csv")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
