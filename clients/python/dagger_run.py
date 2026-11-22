# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""Expert-gated DAgger rollouts (spec docs/superpowers/specs/2026-11-22-diff-ik-policy-dagger-design.md): the policy
runs; on a trigger the scripted expert finishes from that state, and only the expert's segment is captured.

  uv run clients/python/dagger_run.py --address <addr> --seeds 3000-3299 --max-kept 80
  uv run clients/python/expert_pick.py add --log .runs/policy-dagger/dagger-r1.jsonl --dataset diff-ik-pick-dagger-r1 \\
      --part-id <part>
"""
import asyncio
import collections
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bc_pick  # noqa: E402
import dagger_trigger  # noqa: E402
import policy_run  # noqa: E402
from expert_pick import INSTRUCTION, Episode  # noqa: E402

LOG_KEYS = ("seed", "outcome", "trigger", "errors", "chunks", "checkpoint")


def takeover_tags(name, rnd, seed, kind) -> list:
    return [f"cmd:{INSTRUCTION}", f"episode:{name}-{seed}", f"dagger:{rnd}", f"trigger:{kind}", "expert:v2-takeover"]


def log_row(row) -> dict:
    return {k: row.get(k) for k in LOG_KEYS}


def cfg_from_args(a) -> dagger_trigger.Cfg:
    return dagger_trigger.Cfg(stall_s=a.stall_s, stall_mm=a.stall_mm, near_mm=a.near_mm,
                              release_xy_mm=a.release_xy_mm, empty_s=a.empty_s, timeout_s=a.trigger_timeout_s,
                              rest_mm=a.rest_mm)


class Watch:
    """run_episode's stop hook: samples the trigger state at 10 Hz from the policy's t0."""

    def __init__(self, arm, grip, world, cfg):
        self.arm, self.grip, self.world, self.cfg = arm, grip, world, cfg
        self.hist, self.task, self.t0, self.misses = [], None, None, 0

    def begin(self, t0):
        self.t0, self.hist, self.misses = t0, [], 0
        self.task = asyncio.create_task(self._sample())

    async def _read(self) -> dict:
        t = time.monotonic() - self.t0
        p, r, g, hs = await asyncio.gather(
            self.arm.get_end_position(), self.world.do_command({"command": "get_prop_pose", "prop": "block"}),
            self.grip.do_command({"command": "position"}), self.grip.is_holding_something())
        pose = {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}
        return {"t": t, "tcp": bc_pick.tcp_world(pose),
                "block": tuple(v * 1000 for v in r["pose"]["position"]), "grip_cmd": int(g["input"] > 0.5),
                "holding": bool(hs.meta.get("firm_grasp"))}

    async def _sample(self):
        while True:
            t = time.monotonic()
            try:
                self.hist.append(await self._read())
            except Exception:  # noqa: BLE001 - a missed read is a missed sample
                self.misses += 1
            await asyncio.sleep(max(0.0, 0.1 - (time.monotonic() - t)))

    def check(self):
        if not self.hist or time.monotonic() - self.t0 - self.hist[-1]["t"] > 0.5:  # never decide on a stale state
            return None
        return dagger_trigger.check(self.hist[-1], self.hist[:-1], self.cfg)

    def end(self):
        if self.task is not None:
            self.task.cancel()
            self.task = None


def parse_args(argv=None):
    ap = policy_run.build_parser()
    ap.add_argument("--name", help="episode tag prefix: episode:<name>-<seed> (default dagger-<round>)")
    ap.add_argument("--round", default="r1", help="dagger:<round> tag")
    ap.add_argument("--takeover-timeout-s", type=float, default=60.0)
    ap.add_argument("--max-kept", type=int, default=80)
    ap.add_argument("--capture-sensor", default="demo-capture")
    ap.add_argument("--rate-hz", type=float, default=10.0)
    d = dagger_trigger.Cfg()
    ap.add_argument("--stall-s", type=float, default=d.stall_s)
    ap.add_argument("--stall-mm", type=float, default=d.stall_mm)
    ap.add_argument("--near-mm", type=float, default=d.near_mm)
    ap.add_argument("--release-xy-mm", type=float, default=d.release_xy_mm)
    ap.add_argument("--empty-s", type=float, default=d.empty_s)
    ap.add_argument("--trigger-timeout-s", type=float, default=d.timeout_s)
    ap.add_argument("--rest-mm", type=float, default=d.rest_mm)
    rnd = ap.parse_known_args(argv)[0].round
    ap.set_defaults(seeds="3000-3299", timeout_s=40.0, out=f".runs/policy-dagger/dagger-{rnd}.jsonl")
    a = policy_run.parse_args(argv, parser=ap)
    a.name = a.name or f"dagger-{a.round}"
    return a


async def main_async(a):
    servo_r, obs_r, side_r = await bc_pick.connect(a.address, a.insecure, 3)
    try:
        c = await policy_run.build_context(a, servo_r, obs_r, side_r)
        ep = Episode((servo_r, side_r), a)
        watch = Watch(c["side_arm"], c["side_grip"], c["world"], cfg_from_args(a))
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        n = kept = err_run = 0
        kinds, outcomes = collections.Counter(), []
        for seed in policy_run.seed_range(a.seeds):
            row = await policy_run.run_episode(a, c, seed, stop=watch)
            rec = log_row(row)
            rec.update(samples=len(watch.hist), sampler_misses=watch.misses)
            if row["outcome"] == "trigger":  # a trigger with a failed close leaves the session live: no takeover
                kinds[row["trigger"]] += 1
                tags = takeover_tags(a.name, a.round, seed, row["trigger"])
                try:
                    async with asyncio.timeout(a.takeover_timeout_s):
                        info = await ep.takeover(seed, tags)
                    rec.update(info, tags=tags)
                except TimeoutError:  # capture may still be running: abort
                    rec.update(kept=False, reason="takeover timeout", tags=tags, abort=True)
                err_run = err_run + 1 if str(rec.get("reason", "")).startswith("error:") else 0
            n += 1
            kept += bool(rec.get("kept"))
            outcomes.append(row["outcome"])
            with open(a.out, "a") as f:
                f.write(json.dumps(rec) + "\n")
            print(f"seed {seed} {rec['outcome']} trigger={rec['trigger']} kept={rec.get('kept')} "
                  f"samples={rec['samples']} misses={rec['sampler_misses']} reason={rec.get('reason', '')!r} [{kept} kept / {n}]", flush=True)
            if rec.get("abort"):
                print(f"aborting: {rec.get('reason') or 'stop_capture failed'}", flush=True)
                break
            if err_run >= 3:
                print("aborting: 3 consecutive takeover errors", flush=True)
                break
            if kept >= a.max_kept:
                break
            if policy_run.should_abort(outcomes):
                print("aborting: the cell is not usable (repeated setup/close failures)", flush=True)
                break
        print(json.dumps({"rollouts": n, "kept": kept, "triggers": dict(kinds), "log": a.out}))
    finally:
        for r in (servo_r, obs_r, side_r):
            await r.close()


if __name__ == "__main__":
    asyncio.run(main_async(parse_args()))
