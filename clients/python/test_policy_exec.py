# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy"]
# ///
"""Pure-path checks for policy_exec.py. Run: uv run clients/python/test_policy_exec.py"""
import asyncio, math, os, random, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bc_pick  # noqa: E402
from policy_exec import ClientExec, ModuleExec, MoveExec, first_live_row, gripper_events, jerk_rms, mix_pose, sample_chunk, succeeded  # noqa: E402

# rows still ahead of now: row k is due at t_obs + k*dt
assert first_live_row(t_obs=10.0, dt=0.1, now=10.0) == 0
assert first_live_row(10.0, 0.1, 10.25) == 3
assert first_live_row(10.0, 0.1, 10.3) == 3   # exactly due counts as live (epsilon)

# gripper: an event at each 0.5 crossing, from the last commanded state, fired lead earlier
ev = gripper_events([0.0, 0.1, 0.9, 1.0, 0.2], t_obs=5.0, dt=0.1, closed=False, lead=0.05)
assert [(round(t, 3), c) for t, c in ev] == [(5.15, "grab"), (5.35, "open")]
assert gripper_events([0.9, 0.9], 0.0, 0.1, closed=True, lead=0.0) == []
assert gripper_events([0.9], 0.0, 0.1, closed=False, lead=0.0) == [(0.0, "grab")]

# jerk: a straight line at constant speed has none; a cubic x = t^3 (mm, s) has 6 mm/s^3
line = [(i / 30, i * 2.0, 0, 0) for i in range(60)]
assert jerk_rms(line) < 1e-6
cubic = [(i / 30, (i / 30) ** 3, 0, 0) for i in range(60)]
assert abs(jerk_rms(cubic) - 6.0) < 1e-6
assert math.isnan(jerk_rms(line[:3]))
assert abs(jerk_rms(line[:30] + line[29:31] + line[31:]) - 0) < 1e-6  # a duplicated timestamp does not raise
assert math.isnan(jerk_rms([line[0]] * 5))

# jitter (1/30 s plus up to 8 ms) must not blow it up
rng = random.Random(0)
ts = [i / 30 + rng.uniform(0, 0.008) for i in range(60)]
assert jerk_rms([(t, t * 60.0, 0, 0) for t in ts]) < 1e-6
assert abs(jerk_rms([(t, t**3, 0, 0) for t in ts]) / 6.0 - 1) < 0.05

# success: in the bin, at rest, gripper open
from bc_pick import BIN_XY  # noqa: E402
inside = (BIN_XY[0], BIN_XY[1], 320.0)
assert succeeded(inside, (inside[0] + 0.5, inside[1], 320.0), gripper_input=0.0)
assert not succeeded(inside, inside, gripper_input=1.0)                      # still holding
assert not succeeded(inside, (inside[0] + 5, inside[1], 320.0), 0.0)         # still moving
assert not succeeded((0, 0, 320.0), (0, 0, 320.0), 0.0)                      # not in the bin

# chunk sampling: linear position, slerp orientation, clamped at the ends
A = {"x": 0, "y": 0, "z": 0, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 0}
B = {"x": 10, "y": 0, "z": 0, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 90}
ch = {"t_obs": 0.0, "dt": 0.1, "poses": [A, B]}
p = sample_chunk(ch, 0.05)
assert abs(p["x"] - 5) < 1e-9 and abs(p["o_z"] + 1) < 1e-6 and abs(abs(p["theta"]) - 45) < 1e-4
assert sample_chunk(ch, -1)["x"] == 0 and sample_chunk(ch, 5)["x"] == 10

# mix_pose: w=0 is a, w=1 is b, 0.5 is halfway
m = mix_pose(A, B, 0.5)
assert abs(m["x"] - 5) < 1e-9 and abs(abs(m["theta"]) - 45) < 1e-4
assert mix_pose(A, B, 0)["x"] == 0 and mix_pose(A, B, 1)["x"] == 10

# --- executors, against fakes (no network) ---
TS = 0.05  # start() time_scale: test-only, shortens the start motion


class FakeSvc:
    def __init__(self):
        self.cmds = []

    async def do_command(self, c):
        self.cmds.append(c)
        k = c["command"]
        if k == "servo_start":
            return {"session_id": "s", "pose": {"x": 500, "y": 0, "z": 300, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}}
        if k == "servo_trajectory":
            return {"lead_ms": 80, "remaining_ms": 1000, "state": "running"}
        return {"state": "running" if k == "servo_status" else "stopped", "peak_accel_ratio": 0.5}

    def of(self, k):
        return [c for c in self.cmds if c["command"] == k]


class FakeGripper:
    def __init__(self):
        self.calls = []

    async def grab(self):
        self.calls.append("grab")

    async def open(self):
        self.calls.append("open")


def mk_chunk(grip=None, ahead=0.0):
    pose = lambda x: {"x": x, "y": 0, "z": 300, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}
    return {"t_obs": time.monotonic() + ahead, "dt": 0.1, "poses": [pose(500 + 10 * k) for k in range(20)],
            "grip": grip or [0.0] * 20}


async def check_start(ex, svc, seed=3):
    await ex.start(seed, time_scale=TS)
    assert len(svc.of("servo_start")) == 1 and ex.msgs == 0
    goal = bc_pick.start_pose({"x": 500, "y": 0, "z": 300, "theta": 180}, seed)
    assert svc.of("servo_target")[-1]["pose"] == bc_pick.pose_map(goal)


async def main():
    svc, g = FakeSvc(), FakeGripper()
    ex = ModuleExec(svc, g, "arm", lead_ms=None)
    await check_start(ex, svc)
    await ex.play(mk_chunk([0.0] * 5 + [1.0] * 15))
    tr = svc.of("servo_trajectory")
    assert len(tr) == 1 and tr[0]["dt_ms"] == 100 and tr[0]["age_ms"] >= 0 and "lead_ms" not in tr[0]
    await asyncio.sleep(0.6)  # event at 0.5 - 0.08 s
    assert g.calls == ["grab"] and ex.msgs == 1
    r = await ex.close()
    assert r["state"] == "stopped" and r["peak_accel_ratio"] == 0.5

    svc, g = FakeSvc(), FakeGripper()
    ex = ClientExec(svc, g, "arm", lead_ms=0)
    await check_start(ex, svc)
    n0 = len(svc.of("servo_target"))
    await ex.play(mk_chunk())
    await asyncio.sleep(0.3)
    await ex.close()
    xs = [c["pose"]["x"] for c in svc.of("servo_target")[n0:]]
    assert 6 <= len(xs) <= 12, len(xs)
    assert xs == sorted(xs) and 500 <= xs[0] < xs[-1] < 540, xs

    svc, g, moved = FakeSvc(), FakeGripper(), []

    class FakeMotion:
        async def move(self, component_name, destination):
            moved.append(destination.pose.x)
            await asyncio.sleep(0.01)

    ex = MoveExec(FakeMotion(), g, "arm", svc)
    await check_start(ex, svc)
    assert svc.of("servo_stop")
    ch = mk_chunk([0.0, 0.0] + [1.0] * 18, ahead=0.05)  # all rows live
    await ex.play(ch)
    await ex.wait_done()
    assert moved == [500 + 10 * k for k in range(20)] and g.calls == ["grab"] and ex.msgs == 20
    assert (await ex.close())["state"] == "done"

asyncio.run(main())
print("ok")
