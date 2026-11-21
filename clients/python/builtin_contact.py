# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""The same contact tasks as touch_sweep.py and press_hold.py, done with the builtin motion service, which has no
force input: the client reads the wrist F/T sensor and closes the loop itself.

  guard-move   one Move to --depth-mm below the surface; poll F/T and arm.Stop() at --threshold-n
  guard-step   Moves of --step-mm each, reading F/T after each; stop at --threshold-n
  hold-teleop  teleop_start, then teleop_move a z goal at --rate-hz from z += (gain·e + ki·∫e)/rate, e = target − press
  hold-move    the same law, one blocking Move per cycle
  sand-teleop  hold-teleop, then two passes along x (x0 -> x1 -> x0) at --sweep-mm-s, with an integral term (--ki)
  sand-move    the same, one blocking Move per cycle (dev/sanding-mujoco.json, --surface workpiece)

  uv run clients/python/builtin_contact.py --insecure guard-move --trials 3 --csv .runs/bi-guard
Cell: dev/ft-mujoco.json. The approach speed of a Move is the arm's max_joint_deg_s; there is no per-call speed.
"""
import argparse


async def run(a):
    import asyncio
    import csv
    import json
    import os
    import sys
    import time

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from google.protobuf import json_format
    from press_hold import hold_metrics
    from touch_sweep import connect, goto, norm, trial_metrics
    from viam.components.arm import Arm
    from viam.components.sensor import Sensor
    from viam.proto.common import Pose, PoseInFrame
    from viam.proto.service.motion import CollisionSpecification, Constraints, MoveRequest
    from viam.services.motion import MotionClient

    conns = await connect(a, 4)
    servo, bi = MotionClient.from_robot(conns[0], a.servo), MotionClient.from_robot(conns[0], a.builtin)
    ft, arm, arm2 = (Sensor.from_robot(conns[1], a.sensor), Arm.from_robot(conns[2], a.arm),
                     Arm.from_robot(conns[3], a.arm))
    # pressing needs the gripper inside the table's geometry, which the planner rejects unless allowed
    # the teleop planner checks geometry labels (gripper:body, <surface>_origin), Move frame names: allow both
    allow = CollisionSpecification.AllowedFrameCollisions
    cons = Constraints(collision_specification=[CollisionSpecification(allows=[
        allow(frame1=g, frame2=o) for g in (a.gripper, f"{a.gripper}:body") for o in (a.surface, f"{a.surface}_origin")])])
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731
    start = {"x": a.x, "y": a.y, "z": a.start_z, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}

    def dest(z, x=None):  # gripper TCP pose, world mm
        return PoseInFrame(reference_frame="world", pose=Pose(x=a.x if x is None else x, y=a.y, z=z, o_x=0, o_y=0,
                                                              o_z=-1, theta=180))

    out, trace_f = [], open(f"{a.csv}-trace.csv", "w", newline="") if a.csv else None
    trace = csv.writer(trace_f) if trace_f else None
    if trace:
        trace.writerow(["trial", "t", "kind", "value"])
    try:
        for k in range(a.trials):
            await goto(servo, a.arm, start)  # diff-ik's own pose session, stopped before builtin runs
            await asyncio.sleep(0.5)
            await ft.do_command({"command": "tare"})
            force, press, z, m = [], [], [], {}

            async def poses():
                while True:
                    p = await arm.get_end_position()
                    z.append((now(), p.z))

            pt = asyncio.create_task(poses())
            try:
                if a.mode == "guard-move":
                    mv = asyncio.create_task(bi.move(component_name=a.gripper, destination=dest(a.contact_z - a.depth_mm),
                                                     constraints=cons))
                    stop_at, end = None, now() + 15
                    while now() < end:
                        f = norm(await ft.get_readings())
                        force.append((now(), f))
                        if stop_at is None and f >= a.threshold_n:
                            stop_at = now()
                            await arm2.stop()
                            mv.cancel()
                            if a.release:  # the client-side version of the guard's release: hold where the arm is
                                await arm2.move_to_joint_positions(await arm2.get_joint_positions())
                            end = now() + a.settle_s
                        elif stop_at is None and mv.done():
                            break  # the move finished without reaching the threshold
                    mv.cancel()
                    await asyncio.gather(mv, return_exceptions=True)
                elif a.mode == "guard-step":
                    zg, moves, stop_at = a.contact_z + a.above_mm, 0, None
                    t_start = now()
                    while zg > a.contact_z - a.depth_mm:
                        zg -= a.step_mm
                        await bi.move(component_name=a.gripper, destination=dest(zg), constraints=cons)
                        moves += 1
                        f = norm(await ft.get_readings())
                        force.append((now(), f))
                        if f >= a.threshold_n:
                            stop_at = now()
                            if a.release:
                                await arm2.move_to_joint_positions(await arm2.get_joint_positions())
                            break
                    m["moves"], m["mm_s"] = moves, moves * a.step_mm / (now() - t_start)
                    end = now() + a.settle_s
                    while now() < end:
                        force.append((now(), norm(await ft.get_readings())))
                else:  # hold-teleop / hold-move
                    sand = a.mode.startswith("sand")
                    teleop = a.mode.endswith("teleop")
                    zg, xg, integ, seg, sweep_t0 = a.contact_z + a.above_mm, a.x, 0.0, -1, 0.0
                    legs = [a.x1, a.x] if sand else []
                    if teleop:
                        req = MoveRequest(name=a.builtin, component_name=a.gripper, destination=dest(zg), constraints=cons)
                        await bi.do_command({"teleop_start": json_format.MessageToJson(req)})
                    contact_at, end, cycles, period = None, now() + 20, 0, 1 / a.rate_hz
                    while now() < end:
                        tick = now()
                        p = -(await ft.get_readings())["force"]["z"]  # the sensor was tared at the start pose
                        press.append((now(), p))
                        if sand:
                            press[-1] = (press[-1][0], p, seg)
                        # fixed dt for both: a measured dt lets one slow Move make the next step (and z correction) huge
                        dt = period
                        if contact_at is None:
                            zg -= a.approach_mm_s * dt
                            if p >= a.contact_n:
                                contact_at, end = now(), now() + (a.hold_s if not sand else 1e9)
                        else:
                            integ = max(-5.0, min(5.0, integ + a.ki * (a.target_n - p) * dt))
                            zg -= (a.gain * (a.target_n - p) + integ) * dt
                            if sand and now() > contact_at + a.settle_s:
                                if seg == -1:
                                    seg, sweep_t0 = 0, now()
                                if seg < len(legs):
                                    step = a.sweep_mm_s * dt
                                    xg = xg + max(-step, min(step, legs[seg] - xg))
                                    if abs(legs[seg] - xg) < 1e-6:
                                        seg += 1
                                else:
                                    break
                        if teleop:
                            await bi.do_command({"teleop_move": json_format.MessageToJson(dest(zg, xg)),
                                                 "component_name": a.gripper})
                            await asyncio.sleep(max(0.0, period - (now() - tick)))
                        else:
                            await bi.move(component_name=a.gripper, destination=dest(zg, xg), constraints=cons)
                        cycles += 1
                    if teleop:
                        await bi.do_command({"teleop_stop": True})
                    if sand:
                        from sand import pass_stats
                        for sg in (0, 1):
                            v = [r[1] for r in press if len(r) > 2 and r[2] == sg]
                            if v:
                                ps = pass_stats(v, a.target_n, 0.2)
                                m.update({f"pass{sg}_{q}": x for q, x in ps.items()})
                                m[f"pass{sg}_n"] = len(v)
                        press = [r[:2] for r in press]
                    else:
                        m = hold_metrics(press, a.target_n, a.contact_n, a.band, a.tail_s)
                    m["loop_hz"] = cycles / (press[-1][0] - press[0][0])
                    if sand and seg > 0:
                        m["sweep_mm_s_achieved"] = 2 * abs(a.x1 - a.x) / (now() - sweep_t0)
            finally:
                pt.cancel()
                await asyncio.gather(pt, return_exceptions=True)
            if force:
                # contact by height, not force: a fast Move's own acceleration reads over 1 N at the start
                tz = next((t for t, q in z if q <= a.contact_flange_z + 0.05), None)
                if tz is None:
                    m["contact"] = False
                else:
                    m.update(trial_metrics([r for r in force if r[0] >= tz - 0.02], z, a.threshold_n, a.contact_n,
                                           a.settle_s))
                    w = [(t, q) for t, q in z if tz - 0.1 <= t <= tz]
                    if len(w) > 1:  # approach speed over the 100 ms before contact
                        m["approach_mm_s"] = (w[0][1] - w[-1][1]) / (w[-1][0] - w[0][0])
            m.update(mode=a.mode, trial=k)
            out.append(m)
            print(json.dumps({q: round(x, 2) if isinstance(x, float) else x for q, x in m.items()}))
            if trace:
                trace.writerows([k, f"{t:.4f}", "force", f"{f:.3f}"] for t, f in force)
                trace.writerows([k, f"{t:.4f}", "press", f"{f:.3f}"] for t, f in press)
                trace.writerows([k, f"{t:.4f}", "z", f"{q:.3f}"] for t, q in z)
        await goto(servo, a.arm, start)
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
    ap.add_argument("mode", choices=["guard-move", "guard-step", "hold-teleop", "hold-move", "sand-teleop", "sand-move"])
    ap.add_argument("--address", default="localhost:8086")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--builtin", default="builtin")
    ap.add_argument("--servo", default="servo", help="diff-ik service, used only to reach the start pose")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--gripper", default="gripper")
    ap.add_argument("--surface", default="table", help="frame the gripper may collide with")
    ap.add_argument("--sensor", default="wrist-ft")
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--x", type=float, default=550)
    ap.add_argument("--y", type=float, default=0)
    ap.add_argument("--start-z", type=float, default=470, help="flange z at the start, world mm")
    ap.add_argument("--contact-z", type=float, default=302, help="gripper TCP z at contact, world mm")
    ap.add_argument("--contact-flange-z", type=float, default=32.0,
                    help="flange z at contact in the arm's base frame, mm (from touch_sweep runs)")
    ap.add_argument("--no-release", dest="release", action="store_false",
                    help="guard: don't command the measured joints after the stop")
    ap.add_argument("--depth-mm", type=float, default=15, help="guard: how far below contact the goal is")
    ap.add_argument("--above-mm", type=float, default=10, help="step/hold: start this far above contact")
    ap.add_argument("--step-mm", type=float, default=0.5)
    ap.add_argument("--threshold-n", type=float, default=5)
    ap.add_argument("--contact-n", type=float, default=1)
    ap.add_argument("--settle-s", type=float, default=1.0)
    ap.add_argument("--rate-hz", type=float, default=10, help="control rate: teleop goal rate, and the fixed dt (1/rate) of every mode's force law")
    ap.add_argument("--approach-mm-s", type=float, default=5)
    ap.add_argument("--gain", type=float, default=0.5, help="mm/s per N, as diff-ik's force.gain")
    ap.add_argument("--target-n", type=float, default=10)
    ap.add_argument("--hold-s", type=float, default=4)
    ap.add_argument("--band", type=float, default=0.1)
    ap.add_argument("--tail-s", type=float, default=1.5)
    ap.add_argument("--ki", type=float, default=0, help="mm/s per N·s, as diff-ik's force.ki")
    ap.add_argument("--sweep-mm-s", type=float, default=20)
    ap.add_argument("--x1", type=float, default=630, help="sand: far end of the passes; --x is the near end")
    ap.add_argument("--csv")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
