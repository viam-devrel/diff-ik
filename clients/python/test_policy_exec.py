# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy"]
# ///
"""Pure-path checks for policy_exec.py. Run: uv run clients/python/test_policy_exec.py"""
import math, os, random, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_exec import first_live_row, gripper_events, jerk_rms, mix_pose, sample_chunk, succeeded  # noqa: E402

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
print("ok")
