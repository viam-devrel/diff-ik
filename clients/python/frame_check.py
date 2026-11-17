# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 3c3 B1 sim check (not a shipped tool): camera-frame targets through the servo session.

  uv run clients/python/frame_check.py --address 127.0.0.1:<port> --insecure
Asserts: a start pose round-trips through the camera frame (<= 0.1 mm), a +30 mm camera-frame target
moves the arm up 30 mm, a below-floor target is clamped, apply:false never moves the arm, and a zero
pose in the arm's end-effector frame previews as the current end-effector pose."""

import argparse
import asyncio
import math
import sys

from servo_client import connect, lowest_z, pose_dict

KEYS = ("x", "y", "z", "o_x", "o_y", "o_z", "theta")


def dist(a: dict, b: dict) -> float:
    return math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))


async def main(a) -> int:
    from viam.components.arm import Arm
    from viam.proto.common import Pose, PoseInFrame
    from viam.services.motion import MotionClient

    robot = await connect(a)
    svc, arm = MotionClient.from_robot(robot, a.service), Arm.from_robot(robot, a.arm)
    base = f"{a.arm}_origin"
    floor = {"min_z_mm": a.min_z_mm, "tip_mm": a.tip_mm, "margin_mm": 15}
    r = await svc.do_command({"command": "servo_start", "component_name": a.arm, "frame": base, "mode": "pose",
                              "task": "pointing", "rate_hz": 50, "timeout_ms": 2000, "frames": True,
                              "limits": {"joint_deg_s": 60, "linear_mm_s": 70, "angular_deg_s": 30},
                              "target_filter": {"floor": floor}})
    sid, start = r["session_id"], pose_dict(r["pose"])
    fails = []

    def check(ok: bool, what: str):
        print(("PASS " if ok else "FAIL ") + what)
        if not ok:
            fails.append(what)

    async def in_cam(p: dict) -> dict:
        q = PoseInFrame(reference_frame=base, pose=Pose(**p))
        return pose_dict((await robot.transform_pose(q, a.camera)).pose)

    async def send(p: dict, frame: str, apply: bool = True) -> dict:
        cmd = {"command": "servo_target", "session_id": sid, "pose": p, "frame": frame}
        if not apply:
            cmd["apply"] = False
        return await svc.do_command(cmd)

    async def track(p: dict, frame: str, secs: float = 4) -> dict:
        for _ in range(int(secs * 20)):
            rep = await send(p, frame)
            await asyncio.sleep(0.05)
        return rep

    async def ee() -> dict:
        return pose_dict(await arm.get_end_position())

    try:
        rep = await send(await in_cam(start), a.camera)
        check(dist(pose_dict(rep["target_raw"]), start) <= 0.1,
              f"camera round trip: {dist(pose_dict(rep['target_raw']), start):.4f} mm")

        up = dict(start, z=start["z"] + 30)
        await track(await in_cam(up), a.camera)
        now = await ee()
        check(abs(now["z"] - start["z"] - 30) <= 2 and dist(now, up) <= 2,
              f"+30 mm via camera: dz {now['z'] - start['z']:.2f} mm, err {dist(now, up):.2f} mm")

        low = dict(start, z=start["z"] - 200)
        rep = await track(await in_cam(low), a.camera, 6)
        now = await ee()
        check(bool(rep["clamped"]), "below-floor target clamped")
        check(lowest_z(now, a.tip_mm) >= a.min_z_mm, f"arm above floor: lowest z {lowest_z(now, a.tip_mm):.1f} "
              f"(floor {a.min_z_mm})")

        before = await ee()
        for _ in range(20):
            await send(await in_cam(up), a.camera, apply=False)
            await asyncio.sleep(0.05)
        await track(before, base, 0.5)  # keep the dead-man fed with a no-op target
        after = await ee()
        check(dist(before, after) <= 0.5, f"apply:false does not move: {dist(before, after):.3f} mm")

        zero = dict.fromkeys(KEYS, 0.0) | {"o_z": 1.0}
        rep = await send(zero, a.arm, apply=False)
        cur = await ee()
        check(dist(pose_dict(rep["target_raw"]), cur) <= 1.0,
              f"zero pose in '{a.arm}' previews as the EE pose: {dist(pose_dict(rep['target_raw']), cur):.3f} mm")
    finally:
        await svc.do_command({"command": "servo_stop", "session_id": sid})
        await robot.close()
    return 1 if fails else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", required=True)
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--camera", default="scene-cam")
    ap.add_argument("--min-z-mm", type=float, default=-70)
    ap.add_argument("--tip-mm", type=float, default=105)
    sys.exit(asyncio.run(main(ap.parse_args())))
