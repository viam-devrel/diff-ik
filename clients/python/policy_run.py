# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""Closed-loop policy runner for the pick-to-bin cell: one JSONL row per episode.

  uv run clients/python/policy_run.py --address <addr> --mode module --cadence async --replan-every 10 \\
      --lead-ms auto --seeds 9100-9129 --out .runs/eval/module-async.jsonl
  uv run clients/python/policy_run.py --address <addr> --once     # one seed, one chunk, module/sync
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
import bc_pick  # noqa: E402
from policy_exec import ClientExec, Executor, ModuleExec, MoveExec, jerk_rms, sample_chunk, succeeded  # noqa: E402
from policy_layout import chunk_to_poses, state_vector  # noqa: E402

CAMERAS = {"observation.images.scene_cam": "scene-cam", "observation.images.wrist_cam": "wrist-cam"}
MIME_ENC = {"image/jpeg": "jpeg", "image/png": "png"}
TASK = "Pick up the red block and place it in the bin"
DT = 0.1
INFER_TIMEOUT_S = 5.0
OBSERVE_TIMEOUT_S = 5.0
PLAY_TIMEOUT_S = 5.0
CLOSE_TIMEOUT_S = 10.0
ABORT_AFTER = 3


# --- pure parts ---


def seed_range(s: str) -> list:
    lo, _, hi = s.partition("-")
    return list(range(int(lo), int(hi or lo) + 1))


def parse_lead(s: str, mode: str):
    """--lead-ms: 'auto' (module only) -> None, else a number of ms."""
    if s == "auto":
        if mode != "module":
            raise ValueError("--lead-ms auto is only valid with --mode module")
        return None
    return float(s)


def outcome_from(errors, close_state, success: bool) -> str:
    """setup > observe > infer > session (the state close() reports) > plan > gripper > success > timeout."""
    kinds = [k for k, _ in errors]
    for k in ("setup", "observe", "infer"):
        if k in kinds:
            return k
    if "session" in kinds:
        return str(close_state)
    for k in ("plan", "gripper"):
        if k in kinds:
            return k
    return "success" if success else "timeout"


def should_abort(outcomes, n: int = ABORT_AFTER) -> bool:
    """The last n episodes all failed systemically (setup, observe, or close() itself failed): the cell is broken."""
    return len(outcomes) >= n and all(o in ("setup", "observe") or o.startswith("close_failed") for o in outcomes[-n:])


def episode_row(*, seed, mode, cadence, replan_every, lead_ms, outcome, t_success_s, duration_s, close,
                jerk, row0_to_state_mm, seam_gap_mm, infer_s, chunks, checkpoint, errors) -> dict:
    return {
        "seed": seed, "mode": mode, "cadence": cadence, "replan_every": replan_every, "lead_ms": lead_ms,
        "success": outcome == "success", "outcome": outcome, "t_success_s": t_success_s,
        "msgs_per_s": close["msgs"] / duration_s if duration_s > 0 else None,
        "peak_accel_ratio": close.get("peak_accel_ratio"),
        "jerk_rms": None if jerk != jerk else jerk,  # NaN -> null: strict JSON
        "row0_to_state_mm": list(row0_to_state_mm), "seam_gap_mm": list(seam_gap_mm), "infer_s": list(infer_s),
        "chunks": chunks, "checkpoint": checkpoint, "errors": [list(e) for e in errors],
    }


# --- I/O ---


async def race(ev, timeout, coro=None):
    """Wait for ev, coro (re-raising its error) or the timeout, whichever is first."""
    tasks = {asyncio.ensure_future(ev.wait())}
    if coro is not None:
        tasks.add(asyncio.ensure_future(coro))
    try:
        done, _ = await asyncio.wait(tasks, timeout=max(0.0, timeout), return_when=asyncio.FIRST_COMPLETED)
    finally:  # also on an outer cancel: never leak a polling wait_done
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
    for d in done:
        if not d.cancelled():
            d.result()


async def observe(cams, arm, grip, specs):
    """(t_obs, images, pose, gripper input). The model was trained on the latched 0/1 command: `input`."""
    from PIL import Image

    t_obs = time.monotonic()
    images = {}
    for key, cam in cams.items():
        imgs, _ = await cam.get_images()
        img = imgs[0]
        mime = getattr(img, "mime_type", "image/jpeg")
        if mime not in MIME_ENC:
            raise RuntimeError(f"{cam.name}: unsupported image mime type {mime!r}")
        # the policy service does not resize; train frames were bicubic-resized to the input shape
        _, h, w = (int(v) for v in specs["input_features"][key])  # struct numbers arrive as floats
        buf = io.BytesIO()
        Image.open(io.BytesIO(img.data)).convert("RGB").resize((w, h), Image.Resampling.BICUBIC).save(
            buf, "JPEG", quality=95)
        images[key] = {"encoding": "jpeg", "data": base64.b64encode(buf.getvalue()).decode(),
                       "height": h, "width": w, "channels": 3}
    p = await arm.get_end_position()
    g = (await grip.do_command({"command": "position"}))["input"]
    pose = {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}
    return t_obs, images, pose, g


async def success_task(world, grip, t0, ev, out):
    prev = None
    while True:
        try:
            r = await world.do_command({"command": "get_prop_pose", "prop": "block"})
            xyz = tuple(v * 1000 for v in r["pose"]["position"])
            g = (await grip.do_command({"command": "position"}))["input"]
            if prev is not None and succeeded(prev, xyz, g):
                out["t_success_s"] = time.monotonic() - t0
                ev.set()
                return
            prev = xyz
        except Exception:  # noqa: BLE001 - a missed read is just a missed sample
            pass
        await asyncio.sleep(0.3)


async def jerk_task(arm, samples):
    while True:
        t = time.monotonic()
        try:
            p = await arm.get_end_position()
            samples.append(((t + time.monotonic()) / 2, p.x, p.y, p.z))  # midpoint of the call
        except Exception:  # noqa: BLE001
            pass
        await asyncio.sleep(max(0.0, 1 / 30 - (time.monotonic() - t)))


async def run_episode(a, c, seed, stop=None) -> dict:
    """t_success_s is the time of the confirming read (it lags the block settling by 0.3-0.6 s).
    stop (optional): begin(t0) / check() -> trigger or None / end(); end() may come without begin() (setup failed)."""
    ex = c["make_exec"]()
    errors_extra, infer_s, row0, seam, samples = [], [], [], [], []
    st = {"t_success_s": None}
    ev, chunks, old, trigger = asyncio.Event(), 0, None, None
    t0, duration, started, tasks = time.monotonic(), 0.0, False, []
    close = {"state": None, "peak_accel_ratio": None, "msgs": 0}
    try:
        try:
            async with asyncio.timeout(a.timeout_s + 30):  # bounds setup plus loop: nothing can hang the run
                await c["world"].do_command({"command": "reset"})
                pos, q = bc_pick.block_pose(seed)
                await c["world"].do_command({"command": "set_prop_pose", "prop": "block", "position": pos,
                                             "orientation_wxyz": q})
                await c["gripper"].open()
                await ex.start(seed)
                t0, started = time.monotonic(), True
                if stop is not None:
                    stop.begin(t0)
                tasks = [asyncio.create_task(success_task(c["world"], c["side_grip"], t0, ev, st)),
                         asyncio.create_task(jerk_task(c["side_arm"], samples))]
                while not ev.is_set() and time.monotonic() - t0 < a.timeout_s and not ex.errors:
                    if a.once and chunks >= 1:
                        break
                    try:
                        t_obs, images, pose, g = await asyncio.wait_for(
                            observe(c["cams"], c["arm"], c["grip"], c["specs"]), OBSERVE_TIMEOUT_S)
                    except Exception as e:  # noqa: BLE001 - timeouts included
                        errors_extra.append(("observe", repr(e)))
                        break
                    try:
                        t_inf = time.monotonic()
                        r = await asyncio.wait_for(c["policy"].do_command(
                            {"command": "infer", "images": images, "state": {"values": state_vector(pose, g)},
                             "task": TASK}), a.infer_timeout_s)
                        infer_s.append(time.monotonic() - t_inf)
                        poses, gcol = chunk_to_poses(r["actions"]["rows"])
                    except Exception as e:  # noqa: BLE001
                        errors_extra.append(("infer", repr(e)))
                        break
                    # Sync: the arm held still while the policy thought, so row 0 is due now, not at t_obs (an inference
                    # longer than the chunk would otherwise arrive already played out). Async: the arm kept moving, so
                    # the chunk keeps its real age.
                    chunk = {"t_obs": time.monotonic() if a.cadence == "sync" else t_obs, "dt": DT, "poses": poses,
                             "grip": gcol}
                    row0.append(math.dist([poses[0][k] for k in "xyz"], [pose[k] for k in "xyz"]))
                    if old is not None:
                        now = time.monotonic()
                        seam.append(math.dist([sample_chunk(old, now)[k] for k in "xyz"],
                                              [sample_chunk(chunk, now)[k] for k in "xyz"]))
                    if ev.is_set():
                        break
                    await asyncio.wait_for(ex.play(chunk), PLAY_TIMEOUT_S)
                    old = chunk
                    chunks += 1
                    left = a.timeout_s - (time.monotonic() - t0)
                    if a.cadence == "sync":
                        await race(ev, left, ex.wait_done())
                    else:
                        await race(ev, min(left, t_obs + a.replan_every * DT - time.monotonic()))
                    if stop is not None and not ev.is_set():
                        trigger = stop.check()
                        if trigger:
                            break
                duration = time.monotonic() - t0
        except Exception as e:  # noqa: BLE001 - this episode only; TimeoutError included
            duration = time.monotonic() - t0
            if not started:
                errors_extra.append(("setup", repr(e)))
            elif not any(k == "session" for k, _ in ex.errors):
                ex.errors.append(("session", repr(e)))
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        try:
            close = await asyncio.wait_for(ex.close(), CLOSE_TIMEOUT_S)
        except Exception as e:  # noqa: BLE001
            close = {"state": f"close_failed: {e!r}", "peak_accel_ratio": None, "msgs": ex.msgs}
        if stop is not None:
            try:
                stop.end()
            except Exception:  # noqa: BLE001 - a hook bug must not skip teardown or abort the run
                pass
    errors = list(ex.errors) + errors_extra
    outcome = outcome_from(errors, close["state"], ev.is_set())
    if a.once and outcome == "timeout":
        outcome = "once"
    if trigger:
        outcome = "trigger"
    row = episode_row(
        seed=seed, mode=a.mode, cadence=a.cadence, replan_every=a.replan_every, lead_ms=a.lead_ms, outcome=outcome,
        t_success_s=st["t_success_s"], duration_s=duration, close=close, jerk=jerk_rms(samples),
        row0_to_state_mm=row0, seam_gap_mm=seam, infer_s=infer_s, chunks=chunks, checkpoint=a.checkpoint_label,
        errors=errors)
    row["trigger"] = trigger
    return row


async def build_context(a, servo_r, obs_r, side_r) -> dict:
    from viam.components.arm import Arm
    from viam.components.camera import Camera
    from viam.components.gripper import Gripper
    from viam.components.generic import Generic as GenericComponent
    from viam.services.generic import Generic
    from viam.services.motion import MotionClient

    policy, world = Generic.from_robot(obs_r, a.policy), GenericComponent.from_robot(side_r, a.world)  # the sim world is a component
    specs = await policy.do_command({"command": "specs"})
    keys = specs["image_feature_keys"]
    if set(keys) != set(CAMERAS):
        raise SystemExit(f"policy wants image keys {keys}, cell has {list(CAMERAS)}")
    if specs.get("state_dim") != 10 or specs.get("action_dim") != 10:
        raise SystemExit(f"policy state/action dims {specs.get('state_dim')}/{specs.get('action_dim')}, want 10/10")
    svc, gripper = MotionClient.from_robot(servo_r, a.service), Gripper.from_robot(servo_r, a.gripper)
    motion = MotionClient.from_robot(servo_r, "builtin")
    Executor.grip_early = a.grip_early
    Executor.session_timeout_ms = a.session_timeout_ms
    makers = {"module": lambda: ModuleExec(svc, gripper, a.arm, a.lead_ms),
              "client": lambda: ClientExec(svc, gripper, a.arm, a.lead_ms),
              "move": lambda: MoveExec(motion, gripper, a.arm, svc)}
    c = {"make_exec": makers[a.mode], "world": world, "gripper": gripper, "policy": policy, "specs": specs,
         "cams": {k: Camera.from_robot(obs_r, cam) for k, cam in CAMERAS.items()},
         "arm": Arm.from_robot(obs_r, a.arm), "grip": Gripper.from_robot(obs_r, a.gripper),
         "side_arm": Arm.from_robot(side_r, a.arm), "side_grip": Gripper.from_robot(side_r, a.gripper)}
    return c


async def main_async(a):
    servo_r, obs_r, side_r = await bc_pick.connect(a.address, a.insecure, 3)
    try:
        c = await build_context(a, servo_r, obs_r, side_r)
        seeds, wins, outcomes = seed_range(a.seeds), 0, []
        if a.out:
            os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        for seed in seeds:
            row = await run_episode(a, c, seed)
            wins += row["success"]
            outcomes.append(row["outcome"])
            if a.out:
                with open(a.out, "a") as f:
                    f.write(json.dumps(row) + "\n")
            print(f"seed {seed} {row['outcome']} t_success={row['t_success_s']} chunks={row['chunks']} "
                  f"msgs/s={row['msgs_per_s']} jerk={row['jerk_rms']} errors={len(row['errors'])}", flush=True)
            if a.once:
                print(json.dumps(row))
            if should_abort(outcomes):
                print(f"aborting: {ABORT_AFTER} consecutive setup/close failures ({row['errors'][-1:]}); "
                      "the machine or world is not usable", flush=True)
                break
        print(f"{wins}/{len(outcomes)} succeeded")
    finally:
        for r in (servo_r, obs_r, side_r):
            await r.close()


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", required=True)
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--once", action="store_true", help="one seed, module/sync, a single chunk")
    ap.add_argument("--mode", choices=["module", "client", "move"], default="module")
    ap.add_argument("--cadence", choices=["sync", "async"], default="sync")
    ap.add_argument("--replan-every", type=int, default=10, help="async: rows played before the next observation")
    ap.add_argument("--lead-ms", default="100", help="a number, or auto (module mode only)")
    ap.add_argument("--seeds", default="9100", help="N or LO-HI")
    ap.add_argument("--out", help="JSONL file, appended (required unless --once)")
    ap.add_argument("--timeout-s", type=float, default=30.0)
    ap.add_argument("--infer-timeout-s", type=float, default=INFER_TIMEOUT_S, help="raise it for slow policies")
    ap.add_argument("--session-timeout-ms", type=float, default=3000, help="servo dead-man; raise it for slow policies")
    ap.add_argument("--grip-early", action="store_true", help="fire the gripper lead early (old behaviour; diagnosis only)")
    ap.add_argument("--checkpoint-label", default="")
    ap.add_argument("--arm", default="arm")
    ap.add_argument("--gripper", default="gripper")
    ap.add_argument("--policy", default="vla-policy")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--world", default="sim-world")
    return ap


def parse_args(argv=None, parser=None):
    ap = parser or build_parser()
    a = ap.parse_args(argv)
    if a.once:
        a.mode, a.cadence = "module", "sync"
        a.seeds = str(seed_range(a.seeds)[0])
    elif not a.out:
        ap.error("--out is required unless --once")
    try:
        a.lead_ms = parse_lead(a.lead_ms, a.mode)
    except ValueError as e:
        ap.error(str(e))
    return a


if __name__ == "__main__":
    asyncio.run(main_async(parse_args()))
