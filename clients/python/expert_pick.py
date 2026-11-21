# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Scripted pick-to-bin expert for dev/bc-pick-mujoco.json (spec docs/superpowers/specs/2026-11-21-...-data-design.md).

  uv run clients/python/expert_pick.py run --address <addr> --seeds 1000-1019            # expert bar, no capture
  uv run clients/python/expert_pick.py run --address <addr> --seeds 1000-1049 --capture diff-ik-pick-v1
  uv run clients/python/expert_pick.py add --log .runs/policy-sim/expert-diff-ik-pick-v1.jsonl --dataset diff-ik-pick-v1
"""
import argparse
import asyncio
import json
import math
import os
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bc_pick as C  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.components.sensor import Sensor  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

INSTRUCTION = "put the red block in the bin"
TICK_HZ = 30
BASE_MM_S = 150.0  # expert speed at scale 1


class Discard(Exception):
    pass


class StartPoseError(Exception):
    pass


class Episode:
    def __init__(self, robot_conns, a):
        r0, r1 = robot_conns
        self.a = a
        self.svc = MotionClient.from_robot(r0, a.service)
        self.world = Generic.from_robot(r1, a.world)
        self.gripper = Gripper.from_robot(r1, a.gripper)
        self.capture = Sensor.from_robot(r1, a.capture_sensor)
        self.sid = None

    async def target(self, p):
        await self.svc.do_command({"command": "servo_target", "session_id": self.sid, "pose": C.pose_map(p)})

    async def move(self, cur, goal, speed, check_grasp=False):
        """Stream a min-jerk segment from cur to goal (flange, arm frame, (x, y, z, theta)); returns goal."""
        dist = math.dist(cur[:3], goal[:3])
        dur = max(0.6, dist / (BASE_MM_S * speed), abs(C.angle_diff(goal[3], cur[3])) / 60.0)
        for i, p in enumerate(C.minjerk_segment(cur, goal, dur, TICK_HZ)):
            t = time.monotonic()
            await self.target(p)
            if check_grasp and i % 6 == 0:
                hs = await self.gripper.is_holding_something()
                if not hs.meta.get("firm_grasp"):
                    raise Discard("lost grasp")
            await asyncio.sleep(max(0.0, 1 / TICK_HZ - (time.monotonic() - t)))
        return goal

    async def hold(self, p, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await self.target(p)
            await asyncio.sleep(0.1)

    async def hold_during(self, p, seconds, coro):
        """Run coro as a task while holding p, then await it; cancel it if hold() raises."""
        task = asyncio.create_task(coro)
        try:
            await self.hold(p, seconds)
        finally:
            if not task.done():
                task.cancel()
        await task

    async def block_world_mm(self):
        r = await self.world.do_command({"command": "get_prop_pose", "prop": "block"})
        pos, q = r["pose"]["position"], r["pose"]["orientation_wxyz"]
        return tuple(v * 1000 for v in pos), q

    async def run(self, seed, tags=None) -> dict:
        sc = C.sample_scene(seed)
        bx, by = sc["block_xy"]
        yaw = math.radians(sc["block_yaw_deg"])
        await self.world.do_command({"command": "reset"})
        await self.world.do_command({"command": "set_prop_pose", "prop": "block",
                                     "position": [bx / 1000, by / 1000, (C.TABLE_TOP + C.BLOCK_MM / 2) / 1000],
                                     "orientation_wxyz": [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]})
        await self.gripper.open()
        r = await self.svc.do_command({"command": "servo_start", "component_name": self.a.arm, "frame": "arm_origin",
                                       "mode": "pose", "gain": 8, "timeout_ms": 3000,
                                       "limits": {"linear_mm_s": 300, "angular_deg_s": 90}})
        self.sid = r["session_id"]
        p0 = r["pose"]
        cur = (p0["x"], p0["y"], p0["z"], p0["theta"])
        info = {"seed": seed, "scene": sc, "kept": False, "reason": ""}
        capturing = False
        try:
            if abs(p0["o_x"]) > 0.02 or abs(p0["o_y"]) > 0.02 or p0["o_z"] > -0.99:
                # theta is only comparable across poses when the orientation vector is (0, 0, -1)
                raise StartPoseError(f"start pose isn't tool-down: {p0}; fix home_joints (Task 2 Step 3)")
            n = sc["nudge_xyz"]
            start = (cur[0] + n[0], cur[1] + n[1], cur[2] + n[2], cur[3] + sc["nudge_yaw_deg"])
            cur = await self.move(cur, start, sc["speed"])
            await self.hold(cur, 0.5)
            if tags is not None:
                await self.capture.do_command({"start_capture": True, "frequency_hz": self.a.rate_hz, "tags": tags})
                capturing, info["t_start"] = True, datetime.now(timezone.utc).isoformat()
            (wx, wy, wz), q = await self.block_world_mm()
            theta = C.grasp_theta(C.yaw_from_wxyz(q), cur[3])
            grasp_tcp = C.world_to_arm((wx, wy, wz + C.GRASP_ABOVE_CENTER))
            pre = C.flange_from_tcp((grasp_tcp[0], grasp_tcp[1], grasp_tcp[2] + C.PREGRASP_ABOVE)) + (theta,)
            grasp = C.flange_from_tcp(grasp_tcp) + (theta,)
            cur = await self.move(cur, pre, sc["speed"])
            cur = await self.move(cur, grasp, sc["speed"] * 0.6)
            await self.hold(cur, 0.2)
            await self.hold_during(cur, 0.6, self.gripper.grab())
            deadline = time.monotonic() + 2.0
            while not (await self.gripper.is_holding_something()).meta.get("firm_grasp"):
                if time.monotonic() > deadline:
                    raise Discard("grasp timeout")
                await self.hold(cur, 0.1)
            lift = (cur[0], cur[1], cur[2] + C.LIFT, cur[3])
            cur = await self.move(cur, lift, sc["speed"] * 0.8, check_grasp=True)
            bin_tcp = C.world_to_arm((C.BIN_XY[0], C.BIN_XY[1], C.BIN_RIM_Z + C.PLACE_ABOVE_RIM))
            over_bin = C.flange_from_tcp(bin_tcp) + (cur[3],)
            cur = await self.move(cur, over_bin, sc["speed"], check_grasp=True)
            await self.hold_during(cur, 0.6, self.gripper.open())
            cur = await self.move(cur, (cur[0], cur[1], cur[2] + C.RETREAT, cur[3]), sc["speed"])
            await self.hold(cur, 0.4)
            b1, _ = await self.block_world_mm()
            await self.hold(cur, 0.3)
            b2, _ = await self.block_world_mm()
            if not (C.in_bin(b2) and math.dist(b1, b2) < 1.0):
                raise Discard(f"block not in bin at rest: {tuple(round(v) for v in b2)}")
            info["kept"] = True
        except Discard as e:
            info["reason"] = str(e)
        except StartPoseError:
            raise
        except Exception as e:
            info["reason"] = f"error: {e!r}"
        finally:
            try:
                await self.svc.do_command({"command": "servo_stop", "session_id": self.sid})
            except Exception as e:
                print(f"warning: servo_stop failed: {e!r}", file=sys.stderr)
            if capturing:
                try:
                    await self.capture.do_command({"stop_capture": True})
                    info["t_stop"] = datetime.now(timezone.utc).isoformat()
                except Exception as e:
                    print(f"warning: stop_capture failed: {e!r}", file=sys.stderr)
        return info


def seed_range(s: str) -> list:
    lo, _, hi = s.partition("-")
    return list(range(int(lo), int(hi or lo) + 1))


async def cmd_run(a):
    conns = await C.connect(a.address, a.insecure, 2)
    ep = Episode(conns, a)
    os.makedirs(".runs/policy-sim", exist_ok=True)
    log_path = f".runs/policy-sim/expert-{a.capture or 'bar'}.jsonl"
    kept = 0
    seeds = seed_range(a.seeds)
    try:
        with open(log_path, "a") as log:
            for seed in seeds:
                tags = None
                if a.capture:
                    tags = [f"cmd:{INSTRUCTION}", f"episode:{a.capture}-{seed}", "expert:v1"]
                info = await ep.run(seed, tags)
                if a.capture:
                    info["tags"] = tags
                kept += info["kept"]
                log.write(json.dumps(info) + "\n")
                log.flush()
                print(json.dumps({"seed": seed, "kept": info["kept"], "reason": info["reason"]}))
    finally:
        for c in conns:
            await c.close()
    n = len(seeds)
    print(json.dumps({"episodes": n, "kept": kept, "success": kept / n, "log": log_path}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run")
    run.add_argument("--address", required=True)
    run.add_argument("--insecure", action="store_true")
    run.add_argument("--seeds", required=True, help="e.g. 1000-1049")
    run.add_argument("--capture", default="", help="dataset name: capture each episode, tagged episode:<name>-<seed>")
    run.add_argument("--rate-hz", type=float, default=10.0)
    run.add_argument("--service", default="servo")
    run.add_argument("--arm", default="arm")
    run.add_argument("--gripper", default="gripper")
    run.add_argument("--world", default="sim-world")
    run.add_argument("--capture-sensor", default="demo-capture")
    a = ap.parse_args()
    if a.cmd == "run":
        asyncio.run(cmd_run(a))


if __name__ == "__main__":
    main()
