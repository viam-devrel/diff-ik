# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""peg.py's task with the builtin motion service's Move only (no teleop, no diff-ik): one blocking Move per control
cycle, with the force law, the spiral and lateral compliance all in this client, from F/T readings it takes itself.

  uv run clients/python/builtin_peg.py --insecure --trials 5 --csv .runs/bi-peg
Cell: dev/peg-mujoco.json. Every cycle advances the goal by a fixed dt, so a slow Move slows the task down instead of
making the next step bigger.
"""
import argparse
import math


def ft_vec(r: dict) -> tuple:
    """(force, torque) from either reading format: mujoco-sim's maps or the xArm sensor's flat keys."""
    if "Fx_N" in r:
        return (r["Fx_N"], r["Fy_N"], r["Fz_N"]), (r["TRx_Nm"], r["TRy_Nm"], r["TRz_Nm"])
    f, t = r["force"], r["torque"]
    return (f["x"], f["y"], f["z"]), (t["x"], t["y"], t["z"])


def to_base_side(push: tuple) -> tuple:
    """A sideways push in the tool frame, in base x/y, for the tool pointing down as commanded (o = (0, 0, -1),
    theta = 180): that orientation is 180° about x, so tool x is base x and tool y is base -y."""
    return push[0], -push[1]


async def run(a):
    import asyncio
    import csv
    import json
    import os
    import sys
    import time

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from peg import offsets, spiral_point, spiral_step
    from touch_sweep import connect
    from viam.components.arm import Arm
    from viam.components.sensor import Sensor
    from viam.proto.common import Pose, PoseInFrame
    from viam.proto.service.motion import CollisionSpecification, Constraints
    from viam.services.motion import MotionClient

    conns = await connect(a, 3)
    mo, ft, arm = (MotionClient.from_robot(conns[0], a.builtin), Sensor.from_robot(conns[1], a.sensor),
                   Arm.from_robot(conns[2], a.arm))
    allow = CollisionSpecification.AllowedFrameCollisions
    cons = Constraints(collision_specification=[CollisionSpecification(
        allows=[allow(frame1=a.gripper, frame2=o) for o in a.surfaces.split(",")])])
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731
    sign = -1 if a.press_axis == "-z" else 1  # as diff-ik's force.press_axis

    async def move(x, y, z):  # gripper TCP (the peg tip), world mm
        dest = PoseInFrame(reference_frame="world", pose=Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=180))
        await mo.move(component_name=a.gripper, destination=dest, constraints=cons)

    async def tared(f0):
        f, _ = ft_vec(await ft.get_readings())
        push = tuple(-sign * (v - b) for v, b in zip(f, f0))  # the contact force on the tool, sensor frame
        return push, -push[2]  # press: how hard the surface pushes back along the tool axis

    out, trace_f = [], open(f"{a.csv}-trace.csv", "w", newline="") if a.csv else None
    trace = csv.writer(trace_f) if trace_f else None
    if trace:
        trace.writerow(["trial", "t", "phase", "x", "y", "z", "press_n", "side_n"])
    try:
        for k, (dx, dy) in enumerate(offsets(a.trials, a.offset_mm, a.seed)):
            sx, sy = a.hole_x + dx, a.hole_y + dy
            await move(sx, sy, a.plate_top + a.above_mm)
            await asyncio.sleep(0.3)
            f0 = [0.0, 0.0, 0.0]
            for _ in range(5):
                f, _ = ft_vec(await ft.get_readings())
                f0 = [b + v / 5 for b, v in zip(f0, f)]
            m = {"trial": k, "dx": dx, "dy": dy}
            dt = 1 / a.rate_hz
            zg, cx, cy, theta, integ = a.plate_top + a.above_mm, 0.0, 0.0, 0.0, 0.0
            phase, cycles, contact_at, z_surface, t_search, t_ins = "approach", 0, None, None, None, None
            peak_side = 0.0
            try:
                trial_end = now() + 90
                while now() < trial_end:
                    push, press = await tared(f0)
                    side = to_base_side(push)
                    side_n = math.hypot(*side)
                    peak_side = max(peak_side, side_n)
                    if math.sqrt(sum(v * v for v in push)) >= a.guard_n:
                        await arm.stop()
                        raise RuntimeError(f"guard: {math.sqrt(sum(v * v for v in push)):.1f} N")
                    p = await arm.get_end_position()  # flange, arm base frame
                    if trace:
                        trace.writerow([k, f"{now():.3f}", phase, f"{p.x:.2f}", f"{p.y:.2f}", f"{p.z:.2f}",
                                        f"{press:.2f}", f"{side_n:.2f}"])
                    if phase == "approach":
                        zg -= a.approach_mm_s * dt
                        if press >= a.contact_n:
                            phase, contact_at = "settle", now()
                    else:
                        integ = max(-a.max_mm_s, min(a.max_mm_s, integ + a.ki * (a.target_n - press) * dt))
                        zg -= max(-a.max_mm_s, min(a.max_mm_s, a.gain * (a.target_n - press) + integ)) * dt
                        # lateral compliance while inserting only: during the search the goal is open loop, so
                        # yielding to the sliding friction would drift the spiral off (diff-ik's client tracks the
                        # measured pose, which pulls it back)
                        step = min(a.comply_xy * side_n, a.max_mm_s) * dt
                        if side_n > 1e-9 and phase == "insert":
                            cx, cy = cx + side[0] / side_n * step, cy + side[1] / side_n * step
                    if phase == "settle" and now() > contact_at + a.settle_s:
                        if p.z < a.plate_z - a.drop_mm:  # went straight in
                            phase, z_surface, t_search = "insert", a.plate_z, now()
                            t_ins = now()
                        else:
                            phase, z_surface, t_search = "search", p.z, now()
                    if phase == "search":
                        theta += spiral_step(theta, a.pitch_mm, a.speed_mm_s * dt)
                        if math.hypot(*spiral_point(theta, a.pitch_mm)) > a.search_radius_mm:
                            break
                        if z_surface - p.z > a.drop_mm:
                            phase, t_ins = "insert", now()
                    if phase == "insert":
                        if z_surface - p.z > a.depth_mm - 1.5 and press > 0.8 * a.target_n:
                            m.update(seated=True)
                            break
                        if now() > t_ins + 30:
                            break
                    ox, oy = spiral_point(theta, a.pitch_mm)
                    await move(sx + ox + cx, sy + oy + cy, zg)
                    cycles += 1
                m.update(found=t_ins is not None, search_s=(t_ins - t_search) if t_ins and t_search else None,
                         insert_s=(now() - t_ins) if t_ins else None, depth_mm=(z_surface - p.z) if z_surface else None,
                         peak_side_n=peak_side, cycle_hz=cycles / max(1e-9, now() - (contact_at or now())))
                m.setdefault("seated", False)
            except Exception as e:
                m["error"] = str(e)[:200]
            out.append(m)
            print(json.dumps({q: round(x, 2) if isinstance(x, float) else x for q, x in m.items()}))
            q = await arm.get_end_position()  # lift straight out (gripper TCP = flange z + 400 - 100 in this cell)
            await move(q.x, q.y, a.plate_top + 40)
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
    ap.add_argument("--builtin", default="builtin")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--gripper", default="gripper")
    ap.add_argument("--sensor", default="wrist-ft")
    ap.add_argument("--surfaces", default="plate-l,plate-r,plate-f,plate-b,table",
                    help="frames the gripper may touch (Move's collision specification)")
    ap.add_argument("--press-axis", choices=["-z", "+z"], default="-z")
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--offset-mm", type=float, default=5)
    ap.add_argument("--hole-x", type=float, default=550)
    ap.add_argument("--hole-y", type=float, default=180)
    ap.add_argument("--plate-top", type=float, default=325, help="plate top, world mm")
    ap.add_argument("--plate-z", type=float, default=25, help="flange z at plate contact, arm base frame, mm")
    ap.add_argument("--above-mm", type=float, default=10, help="start this far above the plate")
    ap.add_argument("--rate-hz", type=float, default=20, help="control rate: the fixed dt is 1/rate")
    ap.add_argument("--approach-mm-s", type=float, default=5)
    ap.add_argument("--contact-n", type=float, default=2)
    ap.add_argument("--target-n", type=float, default=8)
    ap.add_argument("--gain", type=float, default=0.5)
    ap.add_argument("--ki", type=float, default=0)
    ap.add_argument("--max-mm-s", type=float, default=8)
    ap.add_argument("--comply-xy", type=float, default=1.0)
    ap.add_argument("--guard-n", type=float, default=40)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--pitch-mm", type=float, default=1.5)
    ap.add_argument("--speed-mm-s", type=float, default=6)
    ap.add_argument("--search-radius-mm", type=float, default=10)
    ap.add_argument("--drop-mm", type=float, default=2.0)
    ap.add_argument("--depth-mm", type=float, default=25)
    ap.add_argument("--csv")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
