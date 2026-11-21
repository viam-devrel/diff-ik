# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Learned-policy streaming, emulated: a policy that emits action chunks (H future tool poses at --policy-hz) after a
random inference latency, with chunk-to-chunk noise and occasional stalls, drives a servo session through
servo_target. The policy follows a known reference path (min-jerk moves between waypoints), so tracking error is
measured against ground truth.

Execution strategies (how the client turns chunks into targets at --policy-hz):
  naive    play each new chunk from its first action, ignoring its age
  aligned  skip actions older than now: index = (now - observation time) * policy_hz
  blend    aligned, plus a linear crossfade from the previous chunk over --blend-ms
  module   one servo_trajectory per chunk (its age, --blend-ms): the module aligns, leads and blends

  uv run clients/python/policy_stream.py --insecure --strategy aligned --csv .runs/policy-aligned
Cell: dev/ft-mujoco.json (free space above the table).
"""
import argparse
import math
import random

WAYPOINTS = [(450, -150, 520), (600, -50, 470), (520, 150, 560), (400, 100, 480), (620, 120, 540), (450, -150, 520)]


def minjerk(s: float) -> float:
    """Min-jerk progress for s in [0, 1]."""
    s = min(1.0, max(0.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


def reference(t: float, seg_s: float, wps=WAYPOINTS) -> tuple:
    """The reference flange position (world mm) t s into the episode: a min-jerk move per waypoint pair."""
    i = int(t // seg_s)
    if i >= len(wps) - 1:
        return wps[-1]
    a, b, s = wps[i], wps[i + 1], minjerk((t - i * seg_s) / seg_s)
    return tuple(p + (q - p) * s for p, q in zip(a, b))


def latency_s(rng: random.Random, median_ms: float, sigma: float, stall_p: float, stall_ms: float) -> float:
    """Inference latency: lognormal around median_ms, plus a stall_ms stall with probability stall_p."""
    lat = median_ms * math.exp(rng.gauss(0, sigma))
    if rng.random() < stall_p:
        lat += stall_ms
    return lat / 1000


def make_chunk(rng: random.Random, t_obs: float, horizon: int, hz: float, seg_s: float, bias_mm: float,
               jitter_mm: float) -> list:
    """H actions from t_obs: the reference plus a per-chunk bias (chunks disagree) and per-action jitter."""
    bias = [rng.gauss(0, bias_mm) for _ in range(3)]
    return [tuple(r + b + rng.gauss(0, jitter_mm) for r, b in zip(reference(t_obs + k / hz, seg_s), bias))
            for k in range(horizon)]


def chunk_action(chunk: dict, t: float, hz: float, aligned: bool, t_arrive: float):
    """The chunk's action for time t, or None when it's used up. chunk = {"t_obs", "actions"}."""
    k = int((t - (chunk["t_obs"] if aligned else t_arrive)) * hz + 1e-9)  # epsilon: 0.1 s × 30 is 2.999…
    if k < 0:
        k = 0
    return chunk["actions"][k] if k < len(chunk["actions"]) else None


def blend(a: tuple, b: tuple, w: float) -> tuple:
    """(1 − w)·a + w·b."""
    return tuple(p + (q - p) * w for p, q in zip(a, b))


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

    conns = await connect(a, 3)
    svc, svc2, arm = (MotionClient.from_robot(conns[0], a.service), MotionClient.from_robot(conns[1], a.service),
                      Arm.from_robot(conns[2], a.arm))
    rng = random.Random(a.seed)
    seg_s = a.seg_s
    episode_s = seg_s * (len(WAYPOINTS) - 1)

    def pose(p):
        return {"x": p[0], "y": p[1], "z": p[2], "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}

    await goto(svc, a.arm, pose(WAYPOINTS[0]))
    start = {"command": "servo_start", "component_name": a.arm, "frame": "world", "mode": "pose",
             "limits": {"linear_mm_s": a.linear_mm_s, "joint_deg_s": 120}, "timeout_ms": a.timeout_ms}
    if a.gain:
        start["gain"] = a.gain
    sid = (await svc.do_command(start))["session_id"]
    t0 = time.monotonic()
    now = lambda: time.monotonic() - t0  # noqa: E731
    state = {"cur": None, "prev": None, "switch_t": 0.0, "done": False, "chunks": 0, "stalls": 0}
    net_q: asyncio.Queue = asyncio.Queue()
    net_rng = random.Random(a.seed + 1000)

    async def send(cmd):
        """Through the emulated link: one-way delay --net-ms ± --net-jitter-ms, delivered in order, fire and forget."""
        if a.net_ms <= 0 and a.net_jitter_ms <= 0:
            return await svc.do_command(cmd)
        await net_q.put((now() + max(0.0, net_rng.gauss(a.net_ms, a.net_jitter_ms)) / 1000, cmd))

    async def link():
        last = 0.0
        while True:
            due, cmd = await net_q.get()
            due = max(due, last)  # in order: a message can't overtake the one before it
            await asyncio.sleep(max(0.0, due - now()))
            last = due
            try:
                await svc.do_command(cmd)
            except Exception as e:
                state["error"] = str(e)[:120]
    log, jumps, ends = [], [], []

    async def policy():
        """Observe, infer (sleep the latency), deliver; repeat. The reference stands in for the observation."""
        while not state["done"]:
            t_obs = now()
            lat = latency_s(rng, a.latency_ms, a.latency_sigma, a.stall_p, a.stall_ms)
            state["stalls"] += lat * 1000 > a.latency_ms * 3
            chunk = {"t_obs": t_obs, "actions": make_chunk(rng, t_obs, a.horizon, a.policy_hz, seg_s, a.bias_mm,
                                                           a.jitter_mm)}
            await asyncio.sleep(lat)
            chunk["t_arrive"] = now()
            state["prev"], state["cur"], state["switch_t"] = state["cur"], chunk, now()
            state["chunks"] += 1
            if a.strategy == "module":
                cmd = {"command": "servo_trajectory", "session_id": sid, "poses": [pose(p) for p in chunk["actions"]],
                       "dt_ms": 1000 / a.policy_hz, "age_ms": (now() - t_obs) * 1000, "blend_ms": a.blend_ms}
                if a.module_lead_ms >= 0:
                    cmd["lead_ms"] = a.module_lead_ms
                try:
                    await send(cmd)
                except Exception as e:  # the session ended (dead-man): record it and stop
                    state["error"] = str(e)[:120]
                    return

    async def watch():
        while not state["done"]:
            p = await arm.get_end_position()
            log.append((now(), p.x, p.y, p.z + a.base_z))
            await asyncio.sleep(0.01)

    tasks = [asyncio.create_task(policy()), asyncio.create_task(watch()), asyncio.create_task(link())]
    last_target, period, aligned = None, 1 / a.policy_hz, a.strategy != "naive"
    try:
        while a.strategy == "module" and now() < episode_s + 1.0 and "error" not in state:
            await asyncio.sleep(0.05)
        while a.strategy != "module" and now() < episode_s + 1.0:
            tick = now()
            ta = tick + a.lead_ms / 1000  # play ahead by the servo's own lag (aligned/blend only)
            cur = state["cur"]
            target = None
            if cur is not None:
                target = chunk_action(cur, ta if aligned else tick, a.policy_hz, aligned, cur["t_arrive"])
                prev = state["prev"]
                if a.strategy == "blend" and prev is not None and target is not None:
                    w = min(1.0, (tick - state["switch_t"]) * 1000 / a.blend_ms)
                    old = chunk_action(prev, ta, a.policy_hz, True, prev["t_arrive"])
                    if old is not None and w < 1:
                        target = blend(old, target, w)
            if target is None and last_target is not None:
                ends.append(tick)  # chunk used up and no new one: hold the last action
                target = last_target
            if target is not None:
                if last_target is not None:
                    jumps.append(math.dist(target, last_target))
                await send({"command": "servo_target", "session_id": sid, "pose": pose(target)})
                last_target = target
            await asyncio.sleep(max(0.0, period - (now() - tick)))
        try:
            st = await svc2.do_command({"command": "servo_status", "session_id": sid})
        except Exception:
            st = {}
    finally:
        state["done"] = True
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        try:
            final = await svc.do_command({"command": "servo_stop", "session_id": sid})
        except Exception as e:
            final = {"state": str(e)[:80]}
    errs = [math.dist((x, y, z), reference(t, seg_s)) for t, x, y, z in log if 0.5 < t < episode_s]
    errs.sort()
    steps = sorted(jumps) or [float("nan")]  # the module strategy sends no per-tick targets
    m = {"strategy": a.strategy, "latency_ms": a.latency_ms, "seed": a.seed, "net_ms": a.net_ms,
         "net_jitter_ms": a.net_jitter_ms,
         "err_rms_mm": math.sqrt(sum(e * e for e in errs) / len(errs)), "err_p95_mm": errs[int(len(errs) * 0.95)],
         "err_max_mm": errs[-1], "step_p50_mm": steps[len(steps) // 2], "step_p99_mm": steps[int(len(steps) * 0.99)],
         "step_max_mm": steps[-1], "held_ticks": len(ends), "chunks": state["chunks"], "stalls": state["stalls"],
         "lead_ms": a.lead_ms if a.strategy != "module" else st.get("trajectory_lead_ms", a.module_lead_ms),
         "lag_ms": st.get("lag_ms"), "error": state.get("error", ""),
         "state": final.get("state"), "peak_accel_ratio": st.get("peak_accel_ratio"), "resyncs": st.get("resyncs"),
         "clamps": st.get("clamps")}
    print(json.dumps({q: round(x, 2) if isinstance(x, float) else x for q, x in m.items()}))
    if a.csv:
        with open(f"{a.csv}-track.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "rx", "ry", "rz"])
            w.writerows([f"{t:.3f}", f"{x:.2f}", f"{y:.2f}", f"{z:.2f}",
                         *[f"{v:.2f}" for v in reference(t, seg_s)]] for t, x, y, z in log)
        new = not os.path.exists(f"{a.csv}-runs.csv")
        with open(f"{a.csv}-runs.csv", "a", newline="") as f:
            w = csv.DictWriter(f, list(m))
            if new:
                w.writeheader()
            w.writerow(m)
    await goto(svc, a.arm, pose(WAYPOINTS[0]))
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
    ap.add_argument("--base-z", type=float, default=400, help="arm base height: get_end_position is in its frame")
    ap.add_argument("--strategy", choices=["naive", "aligned", "blend", "module"], default="aligned")
    ap.add_argument("--module-lead-ms", type=float, default=-1, help="module: lead_ms to send (-1 = auto, the lag)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--policy-hz", type=float, default=30)
    ap.add_argument("--horizon", type=int, default=50, help="actions per chunk")
    ap.add_argument("--latency-ms", type=float, default=150, help="median inference latency")
    ap.add_argument("--latency-sigma", type=float, default=0.4, help="lognormal spread of the latency")
    ap.add_argument("--stall-p", type=float, default=0.05, help="chance an inference stalls")
    ap.add_argument("--stall-ms", type=float, default=1200)
    ap.add_argument("--bias-mm", type=float, default=3.0, help="per-chunk offset: how much chunks disagree")
    ap.add_argument("--jitter-mm", type=float, default=0.5, help="per-action noise")
    ap.add_argument("--blend-ms", type=float, default=200)
    ap.add_argument("--lead-ms", type=float, default=0, help="aligned/blend: play the chunk this far ahead of now")
    ap.add_argument("--seg-s", type=float, default=2.5, help="seconds per waypoint move")
    ap.add_argument("--linear-mm-s", type=float, default=400)
    ap.add_argument("--timeout-ms", type=float, default=500)
    ap.add_argument("--gain", type=float, default=0, help="servo gain (0 = the module's default)")
    ap.add_argument("--net-ms", type=float, default=0, help="emulated one-way client→module delay")
    ap.add_argument("--net-jitter-ms", type=float, default=0, help="its spread (σ); messages stay in order")
    ap.add_argument("--csv", help="prefix: PREFIX-track.csv (overwritten) and PREFIX-runs.csv (appended)")
    a = ap.parse_args()
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
    asyncio.run(run(a))


if __name__ == "__main__":
    main()
