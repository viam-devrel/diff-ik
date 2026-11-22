# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""run_episode with fakes: the stop hook ends the loop and the row records the trigger.
Run: uv run clients/python/test_policy_run_stop.py"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import policy_run  # noqa: E402

DOWN = {"x": 400.0, "y": 0.0, "z": 300.0, "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}
ROW = [400.0, 0.0, 300.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0]


async def fake_observe(cams, arm, grip, specs):
    import time
    return time.monotonic(), {}, dict(DOWN), 0.0


async def idle(*_a, **_k):
    await asyncio.sleep(3600)


class World:
    async def do_command(self, cmd):
        return {}


class Grip:
    async def open(self):
        pass


class Policy:
    async def do_command(self, cmd):
        return {"actions": {"rows": [ROW] * 20}}


class Exec:
    def __init__(self):
        self.errors, self.msgs = [], 0

    async def start(self, seed):
        pass

    async def play(self, chunk):
        pass

    async def wait_done(self):
        pass

    async def close(self):
        return {"state": "stopped", "peak_accel_ratio": None, "msgs": 0}


class Stop:
    def __init__(self, results):
        self.results, self.t0, self.ended = list(results), None, False

    def begin(self, t0):
        self.t0 = t0

    def check(self):
        return self.results.pop(0) if self.results else None

    def end(self):
        self.ended = True


policy_run.observe, policy_run.success_task, policy_run.jerk_task = fake_observe, idle, idle
c = {"make_exec": Exec, "world": World(), "gripper": Grip(), "policy": Policy(), "specs": {}, "cams": {},
     "arm": None, "grip": None, "side_arm": None, "side_grip": None}

a = policy_run.parse_args(["--address", "x", "--out", os.devnull, "--timeout-s", "5"])
stop = Stop([None, "approach_stall"])
row = asyncio.run(policy_run.run_episode(a, c, 9100, stop=stop))
assert row["trigger"] == "approach_stall" and row["chunks"] == 2, row
assert stop.t0 is not None and stop.ended

a = policy_run.parse_args(["--address", "x", "--out", os.devnull, "--timeout-s", "0.3"])
row = asyncio.run(policy_run.run_episode(a, c, 9100))
assert row["trigger"] is None and row["outcome"] == "timeout", row

ap = policy_run.build_parser()
ap.add_argument("--extra-flag", default="x")
a = policy_run.parse_args(["--address", "x", "--out", os.devnull], parser=ap)
assert a.extra_flag == "x" and a.lead_ms == 100.0
print("ok")
