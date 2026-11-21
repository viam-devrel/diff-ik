# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""Policy runner for the pick-to-bin cell. Sub-project 1 has only --once: one observation, one infer, one
servo_trajectory, to prove the layout and the command end to end.

  uv run clients/python/policy_run.py --address <addr> --once --lead-ms 200
"""
import argparse
import asyncio
import base64
import io
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_layout import chunk_to_poses, state_vector  # noqa: E402

CAMERAS = {"observation.images.scene_cam": "scene-cam", "observation.images.wrist_cam": "wrist-cam"}
MIME_ENC = {"image/jpeg": "jpeg", "image/png": "png"}


async def once(a):
    from viam.components.arm import Arm
    from viam.components.camera import Camera
    from viam.components.gripper import Gripper
    from viam.services.generic import Generic
    from viam.services.motion import MotionClient

    from bc_pick import connect

    (robot,) = await connect(a.address, a.insecure, 1)
    try:
        arm, grip = Arm.from_robot(robot, a.arm), Gripper.from_robot(robot, a.gripper)
        policy, svc = Generic.from_robot(robot, a.policy), MotionClient.from_robot(robot, a.service)
        specs = await policy.do_command({"command": "specs"})
        keys = specs["image_feature_keys"]
        if set(keys) != set(CAMERAS):
            raise SystemExit(f"policy wants image keys {keys}, cell has {list(CAMERAS)}")
        if specs.get("state_dim") != 10 or specs.get("action_dim") != 10:
            raise SystemExit(f"policy state/action dims {specs.get('state_dim')}/{specs.get('action_dim')}, want 10/10")
        from PIL import Image

        t_obs = time.monotonic()
        images = {}
        for key, cam in CAMERAS.items():
            imgs, _ = await Camera.from_robot(robot, cam).get_images()
            img = imgs[0]
            mime = getattr(img, "mime_type", "image/jpeg")
            if mime not in MIME_ENC:
                raise SystemExit(f"{cam}: unsupported image mime type {mime!r}")
            # the policy service does not resize; train frames were bicubic-resized to the input shape
            _, h, w = (int(v) for v in specs["input_features"][key])  # struct numbers arrive as floats
            buf = io.BytesIO()
            Image.open(io.BytesIO(img.data)).convert("RGB").resize((w, h), Image.Resampling.BICUBIC).save(
                buf, "JPEG", quality=95)
            images[key] = {"encoding": "jpeg", "data": base64.b64encode(buf.getvalue()).decode(),
                           "height": h, "width": w, "channels": 3}
        p = await arm.get_end_position()
        g = (await grip.do_command({"command": "position"}))["position"]
        pose = {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}
        state = state_vector(pose, g)
        r = await policy.do_command({"command": "infer", "images": images, "state": {"values": state},
                                     "task": "Pick up the red block and place it in the bin"})
        rows = r["actions"]["rows"]
        poses, gcol = chunk_to_poses(rows)
        sid = (await svc.do_command({"command": "servo_start", "component_name": a.arm, "frame": "arm_origin",
                                     "mode": "pose", "timeout_ms": 2000}))["session_id"]
        stop = {}
        try:
            res = await svc.do_command({"command": "servo_trajectory", "session_id": sid, "poses": poses,
                                        "dt_ms": 1000 / a.rate_hz, "age_ms": (time.monotonic() - t_obs) * 1000,
                                        "lead_ms": a.lead_ms})
            await asyncio.sleep(len(poses) / a.rate_hz + 0.5)
            st = await svc.do_command({"command": "servo_status", "session_id": sid})
        finally:
            stop = await svc.do_command({"command": "servo_stop", "session_id": sid})
        q = await arm.get_end_position()
        steps = [math.dist([a_["x"], a_["y"], a_["z"]], [b["x"], b["y"], b["z"]]) for a_, b in zip(poses, poses[1:])]
        xyz = lambda d: [d["x"], d["y"], d["z"]]  # noqa: E731
        print(json.dumps({
            "rows": len(rows), "row_width": len(rows[0]), "infer_s": r.get("server_s"),
            "first": poses[0], "last": poses[-1], "max_step_mm": max(steps) if steps else 0.0,
            "gripper_col": [round(v, 1) for v in gcol[:5]], "start_xyz": [p.x, p.y, p.z], "end_xyz": [q.x, q.y, q.z],
            "row0_to_state_mm": math.dist(xyz(poses[0]), [p.x, p.y, p.z]),
            "row_last_to_end_mm": math.dist(xyz(poses[-1]), [q.x, q.y, q.z]),
            "trajectory_reply_state": res.get("state"), "lead_ms": res.get("lead_ms"),
            "remaining_ms": res.get("remaining_ms"), "status_state": st.get("state"),
            "trajectory_ended": st.get("trajectory_ended"), "stop_state": stop.get("state"),
            "peak_accel_ratio": st.get("peak_accel_ratio")}))
    finally:
        await robot.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", required=True)
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--once", action="store_true", required=True)
    ap.add_argument("--rate-hz", type=float, default=10.0, help="the capture rate: dt_ms = 1000 / rate")
    ap.add_argument("--lead-ms", type=float, default=200.0)
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--gripper", default="gripper")
    ap.add_argument("--policy", default="vla-policy")
    ap.add_argument("--service", default="servo")
    asyncio.run(once(ap.parse_args()))


if __name__ == "__main__":
    main()
