# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Peg-in-hole with a spiral search: start over the hole with a random x/y error, press the peg onto the plate with
servo_start `force`, spiral outward until the peg drops into the hole, then let the force controller push it down
until it seats. The pressing force comes from the session's status (force_press_n), so the client doesn't depend on
the sensor's reading format.

  uv run clients/python/peg.py --insecure --trials 5 --csv .runs/peg
Cell: dev/peg-mujoco.json (the demo machine's config).
"""
import argparse
import math
import random


def spiral_point(theta: float, pitch: float) -> tuple:
    """Archimedean spiral: the radius grows by pitch per turn."""
    r = pitch * theta / (2 * math.pi)
    return r * math.cos(theta), r * math.sin(theta)


def spiral_step(theta: float, pitch: float, ds: float) -> float:
    """The angle advance that moves about ds mm along the spiral (arc length ≈ r·dθ, floored near the centre)."""
    r = pitch * theta / (2 * math.pi)
    return ds / max(r, pitch)


def offsets(n: int, radius: float, seed: int) -> list:
    """n x/y start errors, uniform over a disc of the given radius."""
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        r, a = radius * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


async def run(a):
    import asyncio
    import csv
    import json
    import os
    import sys
    import time

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from touch_sweep import connect, goto
    from viam.components.arm import Arm
    from viam.services.motion import MotionClient

    conns = await connect(a, 2)
    svc, arm = MotionClient.from_robot(conns[0], a.service), Arm.from_robot(conns[1], a.arm)
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731

    def pose(x, y, z):
        return {"x": x, "y": y, "z": z, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}

    out, trace_f = [], open(f"{a.csv}-trace.csv", "w", newline="") if a.csv else None
    trace = csv.writer(trace_f) if trace_f else None
    if trace:
        trace.writerow(["trial", "t", "phase", "x", "y", "z", "press_n"])
    try:
        for k, (dx, dy) in enumerate(offsets(a.trials, a.offset_mm, a.seed)):
            sx, sy = a.hole_x + dx, a.hole_y + dy
            await goto(svc, a.arm, pose(sx, sy, a.start_z))
            await asyncio.sleep(0.3)
            sid = (await svc.do_command({
                "command": "servo_start", "component_name": a.arm, "frame": "world", "mode": "twist",
                "limits": {"linear_mm_s": max(a.speed_mm_s, a.max_mm_s) * 2}, "timeout_ms": 500,
                "force": {"sensor": a.sensor, "target_n": a.target_n, "gain": a.gain, "max_mm_s": a.max_mm_s,
                          "comply_xy": a.comply_xy},
                "guard": {"sensor": a.sensor, "force_n": a.guard_n}}))["session_id"]
            m = {"trial": k, "dx": dx, "dy": dy}
            phase, peak = "press", {"search": 0.0, "insert": 0.0}
            try:
                async def tick(vx=0.0, vy=0.0):
                    await svc.do_command({"command": "servo_twist", "session_id": sid, "linear": [vx, vy, 0],
                                          "angular": [0, 0, 0]})
                    st = await svc.do_command({"command": "servo_status", "session_id": sid})
                    if st["state"].startswith("stopped") or st["state"] == "faulted":
                        raise RuntimeError(f"session ended: {st['state']} {st.get('guard_reason', '')}")
                    p = await arm.get_end_position()  # arm base frame: same x/y as world here
                    if trace:
                        trace.writerow([k, f"{now():.3f}", phase, f"{p.x:.2f}", f"{p.y:.2f}", f"{p.z:.2f}",
                                        f"{st.get('force_press_n', 0):.2f}"])
                    return st, p

                # 1. press onto the plate and settle
                deadline = now() + 15
                while True:
                    st, p = await tick()
                    if st.get("force_contact"):
                        break
                    if now() > deadline:
                        raise RuntimeError("no contact")
                    await asyncio.sleep(0.02)
                settle = now() + a.settle_s
                while now() < settle:
                    st, p = await tick()
                    await asyncio.sleep(0.02)
                # contact below the plate's top means the peg went straight into the hole
                found = p.z < a.plate_z - a.drop_mm
                z_surface = a.plate_z if found else p.z
                # 2. spiral out from the start point until the peg drops
                phase, theta, t_search = "search", 0.0, now()
                while not found:
                    theta += spiral_step(theta, a.pitch_mm, a.speed_mm_s * 0.02)
                    ox, oy = spiral_point(theta, a.pitch_mm)
                    if math.hypot(ox, oy) > a.search_radius_mm:
                        break
                    gx, gy = sx + ox, sy + oy
                    ex, ey = gx - p.x, gy - p.y
                    d = math.hypot(ex, ey)
                    s = min(a.speed_mm_s * 1.5, a.kp * d) / d if d > 1e-6 else 0.0
                    st, p = await tick(ex * s, ey * s)
                    peak["search"] = max(peak["search"], st.get("force_press_n", 0))
                    peak["side"] = max(peak.get("side", 0.0), st.get("force_side_n", 0))
                    if z_surface - p.z > a.drop_mm:
                        found = True
                        break
                    await asyncio.sleep(0.02)
                m.update(found=found, search_s=now() - t_search, found_err_mm=math.hypot(p.x - a.hole_x, p.y - a.hole_y))
                # 3. insert: no sideways motion; the force law drives the peg down until it seats on the bottom
                if found:
                    phase, t_ins, seated = "insert", now(), False
                    while now() < t_ins + 15:
                        st, p = await tick()
                        peak["insert"] = max(peak["insert"], st.get("force_press_n", 0))
                        peak["side"] = max(peak.get("side", 0.0), st.get("force_side_n", 0))
                        depth = z_surface - p.z
                        if depth > a.depth_mm - 1.5 and st.get("force_press_n", 0) > 0.8 * a.target_n:
                            seated = True
                            break
                        await asyncio.sleep(0.02)
                    m.update(seated=seated, insert_s=now() - t_ins, depth_mm=z_surface - p.z)
                m.update(peak_search_n=peak["search"], peak_insert_n=peak["insert"], peak_side_n=peak.get("side", 0.0))
            except Exception as e:  # a failed trial is a result, not a crash
                m["error"] = str(e)
            finally:
                try:
                    await svc.do_command({"command": "servo_stop", "session_id": sid})
                except Exception:
                    pass
            out.append(m)
            print(json.dumps({q: round(x, 2) if isinstance(x, float) else x for q, x in m.items()}))
            # lift straight out of the hole before anything moves sideways
            q = await arm.get_end_position()
            await goto(svc, a.arm, pose(q.x, q.y, a.start_z))
        await goto(svc, a.arm, pose(a.hole_x, a.hole_y, a.start_z))
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

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8088")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--sensor", default="wrist-ft")
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--offset-mm", type=float, default=5, help="start error: uniform over a disc of this radius")
    ap.add_argument("--hole-x", type=float, default=550)
    ap.add_argument("--hole-y", type=float, default=180)
    ap.add_argument("--start-z", type=float, default=440, help="flange z at the start, world mm (plate top 325)")
    ap.add_argument("--target-n", type=float, default=8)
    ap.add_argument("--gain", type=float, default=0.5)
    ap.add_argument("--max-mm-s", type=float, default=8, help="force.max_mm_s: also the insertion speed")
    ap.add_argument("--guard-n", type=float, default=40)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--pitch-mm", type=float, default=1.5, help="spiral radius growth per turn")
    ap.add_argument("--speed-mm-s", type=float, default=6, help="speed along the spiral")
    ap.add_argument("--kp", type=float, default=4.0, help="x/y tracking gain toward the spiral point, 1/s")
    ap.add_argument("--search-radius-mm", type=float, default=10)
    ap.add_argument("--drop-mm", type=float, default=2.0, help="drop below the plate surface that means found")
    ap.add_argument("--plate-z", type=float, default=25, help="flange z at plate contact, arm base frame, mm")
    ap.add_argument("--comply-xy", type=float, default=1.0, help="force.comply_xy, mm/s per N (0 = off)")
    ap.add_argument("--depth-mm", type=float, default=25, help="hole depth (plate thickness)")
    ap.add_argument("--csv", help="prefix: PREFIX-trials.csv and PREFIX-trace.csv")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
