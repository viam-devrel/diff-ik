# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy"]
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

INSTRUCTION = "Pick up the red block and place it in the bin"
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
        """Stream a min-jerk segment from cur to goal (flange, arm frame, (x, y, z, theta, tx, ty)); returns goal."""
        dist = math.dist(cur[:3], goal[:3])
        # tilt term: run's tilts are <= 4 deg (<= 0.14 s (8° between two 4° tilts)), under the 0.6 s minimum, so run's timing is unchanged
        tilt = math.hypot(goal[4] - cur[4], goal[5] - cur[5]) / 60.0 if len(goal) == 6 and len(cur) == 6 else 0.0
        dur = max(0.6, dist / (BASE_MM_S * speed), abs(C.angle_diff(goal[3], cur[3])) / 60.0, tilt)
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

    async def place(self, seed):
        await self.world.do_command({"command": "reset"})
        pos, q = C.block_pose(seed)
        await self.world.do_command({"command": "set_prop_pose", "prop": "block", "position": pos,
                                     "orientation_wxyz": q})

    async def _start_session(self) -> dict:
        r = await self.svc.do_command({"command": "servo_start", "component_name": self.a.arm, "frame": "arm_origin",
                                       "mode": "pose", "gain": 8, "timeout_ms": 3000,
                                       "limits": {"linear_mm_s": 300, "angular_deg_s": 90}})
        self.sid = r["session_id"]
        return r["pose"]

    async def _pick(self, cur, sc):
        (wx, wy, wz), q = await self.block_world_mm()
        theta = C.grasp_theta(C.block_yaw_deg(q), cur[3])
        grasp_tcp = C.world_to_arm((wx, wy, wz + C.GRASP_ABOVE_CENTER))
        pre = C.flange_from_tcp((grasp_tcp[0], grasp_tcp[1], grasp_tcp[2] + C.PREGRASP_ABOVE)) + (theta, *sc["tilts"]["pregrasp"])
        grasp = C.flange_from_tcp(grasp_tcp) + (theta, 0.0, 0.0)
        cur = await self.move(cur, pre, sc["speed"])
        cur = await self.move(cur, grasp, sc["speed"] * 0.6)
        await self.hold(cur, 0.2)
        await self.hold_during(cur, 0.6, self.gripper.grab())
        deadline = time.monotonic() + 2.0
        while not (await self.gripper.is_holding_something()).meta.get("firm_grasp"):
            if time.monotonic() > deadline:
                raise Discard("grasp timeout")
            await self.hold(cur, 0.1)
        lift = (cur[0], cur[1], cur[2] + C.LIFT, cur[3], *sc["tilts"]["lift"])
        cur = await self.move(cur, lift, sc["speed"] * 0.8, check_grasp=True)
        return cur

    async def _place(self, cur, sc):
        bin_tcp = C.world_to_arm((C.BIN_XY[0], C.BIN_XY[1], C.BIN_RIM_Z + C.PLACE_ABOVE_RIM))
        over_bin = C.flange_from_tcp(bin_tcp) + (cur[3], 0.0, 0.0)
        cur = await self.move(cur, over_bin, sc["speed"], check_grasp=True)
        await self.hold_during(cur, 0.6, self.gripper.open())
        cur = await self.move(cur, (cur[0], cur[1], cur[2] + C.RETREAT, cur[3], 0.0, 0.0), sc["speed"])
        await self.hold(cur, 0.4)
        b1, _ = await self.block_world_mm()
        await self.hold(cur, 0.3)
        b2, _ = await self.block_world_mm()
        if not (C.in_bin(b2) and math.dist(b1, b2) < 1.0):
            raise Discard(f"block not in bin at rest: {tuple(round(v) for v in b2)}")

    async def _close(self, info, capturing):
        if self.sid is not None:
            try:
                await self.svc.do_command({"command": "servo_stop", "session_id": self.sid})
            except Exception as e:
                print(f"warning: servo_stop failed: {e!r}", file=sys.stderr)
        if capturing:
            try:
                await self.capture.do_command({"stop_capture": True})
                info["t_stop"] = datetime.now(timezone.utc).isoformat()
            except Exception as e:
                # capture may still run under this episode's tag: don't trust this episode or the next
                print(f"warning: stop_capture failed: {e!r}", file=sys.stderr)
                info["kept"] = False
                info["reason"] = (info["reason"] + "; " if info["reason"] else "") + f"stop_capture failed: {e!r}"
                info["abort"] = True

    async def run(self, seed, tags=None) -> dict:
        sc = C.sample_scene(seed)
        await self.place(seed)
        await self.gripper.open()
        p0 = await self._start_session()
        cur = (p0["x"], p0["y"], p0["z"], p0["theta"], 0.0, 0.0)
        info = {"seed": seed, "scene": sc, "kept": False, "reason": ""}
        capturing = False
        try:
            if abs(p0["o_x"]) > 0.02 or abs(p0["o_y"]) > 0.02 or p0["o_z"] > -0.99:
                # theta is only comparable across poses when the orientation vector is (0, 0, -1)
                raise StartPoseError(f"start pose isn't tool-down: {p0}; fix home_joints (Task 2 Step 3)")
            start = C.start_pose(p0, seed)
            cur = await self.move(cur, start, sc["speed"])
            await self.hold(cur, 0.5)
            if tags is not None:
                await self.capture.do_command({"start_capture": True, "frequency_hz": self.a.rate_hz, "tags": tags})
                capturing, info["t_start"] = True, datetime.now(timezone.utc).isoformat()
            cur = await self._pick(cur, sc)
            await self._place(cur, sc)
            info["kept"] = True
        except Discard as e:
            info["reason"] = str(e)
        except StartPoseError:
            raise
        except Exception as e:
            info["reason"] = f"error: {e!r}"
        finally:
            await self._close(info, capturing)
        return info

    async def takeover(self, seed, tags=None) -> dict:
        """Finish the pick from wherever the policy left the arm (DAgger spec §2-3). Capture (when tags are given)
        starts before the first expert move, so the episode's first frame is the takeover state."""
        sc = C.sample_scene(seed)
        info = {"seed": seed, "scene": sc, "kept": False, "reason": ""}
        capturing = False
        self.sid = None
        try:
            cur = C.six_tuple_from_pose(await self._start_session())
            holding = bool((await self.gripper.is_holding_something()).meta.get("firm_grasp"))
            info["holding"] = holding
            if tags is not None:
                await self.capture.do_command({"start_capture": True, "frequency_hz": self.a.rate_hz, "tags": tags})
                capturing, info["t_start"] = True, datetime.now(timezone.utc).isoformat()
                # capture's first frame lags start_capture: hold so the episode opens on the takeover state (e.g. the
                # gripper still closed on nothing) and records the expert's first action as a change from it
                await self.hold(cur, 0.5)
            if holding:
                info["block_at_takeover"] = [round(v, 1) for v in (await self.block_world_mm())[0]]
                over_z = C.flange_from_tcp(C.world_to_arm(C.RELEASE_TCP))[2]
                if cur[2] < over_z:  # rise straight up first, clear of the bin walls
                    cur = await self.move(cur, (cur[0], cur[1], over_z, cur[3], 0.0, 0.0), sc["speed"] * 0.8,
                                          check_grasp=True)
            else:
                await self.hold_during(cur, 0.6, self.gripper.open())
                deadline = time.monotonic() + 2.0
                while True:  # wait for the released block to rest
                    b1, _ = await self.block_world_mm()
                    await self.hold(cur, 0.3)
                    (wx, wy, wz), _ = await self.block_world_mm()
                    if math.dist(b1, (wx, wy, wz)) < 1.0:
                        break
                    if time.monotonic() > deadline:
                        raise Discard("block not at rest")
                info["block_at_takeover"] = [round(v, 1) for v in (wx, wy, wz)]
                if C.in_bin((wx, wy, wz)):
                    raise Discard("block already in bin")
                pre_tcp = (wx, wy, wz + C.GRASP_ABOVE_CENTER + C.PREGRASP_ABOVE)
                pre_z = C.flange_from_tcp(C.world_to_arm(pre_tcp))[2]
                if cur[2] < pre_z:  # rise straight up to pregrasp height, clear of the block
                    cur = await self.move(cur, (cur[0], cur[1], pre_z, *cur[3:]), sc["speed"])
                cur = await self._pick(cur, sc)
            await self._place(cur, sc)
            info["kept"] = True
        except Discard as e:
            info["reason"] = str(e)
        except Exception as e:
            info["reason"] = f"error: {e!r}"
        finally:
            await self._close(info, capturing)
        return info


def seed_range(s: str) -> list:
    lo, _, hi = s.partition("-")
    return list(range(int(lo), int(hi or lo) + 1))


async def cmd_run(a):
    conns = await C.connect(a.address, a.insecure, 2)
    ep = Episode(conns, a)
    os.makedirs(".runs/policy-sim", exist_ok=True)
    log_path = f".runs/policy-sim/expert-{a.capture or 'bar'}.jsonl"
    kept = n = 0
    seeds = seed_range(a.seeds)
    try:
        with open(log_path, "a") as log:
            for seed in seeds:
                tags = None
                if a.capture:
                    tags = [f"cmd:{INSTRUCTION}", f"episode:{a.capture}-{seed}", "expert:v2"]
                info = await ep.run(seed, tags)
                n += 1
                if a.capture:
                    info["tags"] = tags
                kept += info["kept"]
                log.write(json.dumps(info) + "\n")
                log.flush()
                print(json.dumps({"seed": seed, "kept": info["kept"], "reason": info["reason"]}))
                if info.get("abort"):
                    print("aborting batch: stop_capture failed", file=sys.stderr)
                    break
    finally:
        for c in conns:
            await c.close()
    print(json.dumps({"episodes": n, "kept": kept, "success": kept / max(n, 1), "log": log_path}))


async def cmd_add(a):
    """Add the log's kept episodes to a sequence dataset, creating it if needed; matched by episode tag."""
    from viam.app.viam_client import ViamClient
    from viam.proto.app.dataset import DatasetType
    from viam.rpc.dial import DialOptions

    # The log can hold several rows per seed after re-runs: the latest row decides.
    latest = {}
    for line in open(a.log):
        row = json.loads(line)
        if row.get("tags"):
            latest[next(t for t in row["tags"] if t.startswith("episode:"))] = row
    kept = {tag: row for tag, row in latest.items() if row.get("kept")}
    client = await ViamClient.create_from_dial_options(
        DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]))
    try:
        data = client.data_client
        # One sequence per episode tag: a crashed or re-run capture can leave several sequences under one tag,
        # so keep the one whose start is closest to the kept row's t_start.
        found, token = {}, None
        while True:
            seqs, token = await data.list_sequences(organization_id=a.org_id, page_token=token)
            for s in seqs:
                if s.part_id != a.part_id:
                    continue
                for t in s.sequence_tags:
                    if t in kept:
                        t0 = datetime.fromisoformat(kept[t]["t_start"])
                        gap = abs((s.start_time.ToDatetime(tzinfo=timezone.utc) - t0).total_seconds())
                        if t not in found or gap < found[t][1]:
                            found[t] = (s.id, gap)
            if not token:
                break
        found = {t: sid for t, (sid, gap) in found.items()}
        missing = sorted(set(kept) - set(found))
        ds = [d for d in await data.list_datasets_by_organization_id(a.org_id) if d.name == a.dataset]
        dataset_id = ds[0].id if ds else await data.create_dataset(
            name=a.dataset, organization_id=a.org_id, type=DatasetType.DATASET_TYPE_SEQUENCE_DATA)
        present, token = set(), None
        while True:
            seqs, token, _ = await data.sequences_by_dataset_id(dataset_id, page_token=token, page_size=100)
            present.update(s.id for s in seqs)
            if not token:
                break
        new = [sid for sid in found.values() if sid not in present]
        if new:
            await data.add_sequences_to_dataset(dataset_id=dataset_id, sequence_ids=new)
        print(json.dumps({"dataset_id": dataset_id, "added": len(new), "already": len(found) - len(new),
                          "missing": missing}))
    finally:
        client.close()


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
    add = sub.add_parser("add")
    add.add_argument("--log", required=True)
    add.add_argument("--dataset", required=True)
    add.add_argument("--org-id", default="16518049-9dd3-479f-9644-c5d112aa42d8")
    add.add_argument("--part-id", required=True)
    a = ap.parse_args()
    if a.cmd == "run":
        asyncio.run(cmd_run(a))
    elif a.cmd == "add":
        asyncio.run(cmd_add(a))


if __name__ == "__main__":
    main()
