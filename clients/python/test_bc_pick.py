# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy"]
# ///
"""Pure-path checks for bc_pick.py. Run: uv run clients/python/test_bc_pick.py"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bc_pick import (  # noqa: E402
    block_pose, start_pose,
    BIN_XY, BLOCK_AREA, TCP_MM, TILT_MAX_DEG, TILT_MIN_DEG, angle_diff, flange_from_tcp, grasp_theta, in_bin, minjerk_segment, pose_map,
    sample_scene, tilt_vector, world_to_arm, yaw_from_wxyz,
)

# scenes: deterministic, in range
a, b = sample_scene(1003), sample_scene(1003)
assert a == b and sample_scene(1004) != a
for s in range(200):
    sc = sample_scene(s)
    (x0, x1), (y0, y1) = BLOCK_AREA
    assert x0 <= sc["block_xy"][0] <= x1 and y0 <= sc["block_xy"][1] <= y1
    assert -45 <= sc["block_yaw_deg"] <= 45 and 0.8 <= sc["speed"] <= 1.2
    assert all(abs(v) <= 30 for v in sc["nudge_xyz"]) and abs(sc["nudge_yaw_deg"]) <= 10

# angles
assert angle_diff(350, 10) == -20 and angle_diff(10, 350) == 20
assert abs(yaw_from_wxyz([math.cos(math.radians(15)), 0, 0, math.sin(math.radians(15))]) - 30) < 1e-9
# grasp yaw never turns more than 45 degrees from the current theta, and stays a cube-face alignment
for cur in (180.0, 170.0, -170.0, 95.0):
    for yaw in range(-45, 46, 5):
        t = grasp_theta(yaw, cur)
        assert abs(angle_diff(t, cur)) <= 45 + 1e-9, (yaw, cur, t)
        assert abs(((180 - yaw - t) % 90 + 45) % 90 - 45) < 1e-9  # t = 180 - yaw (mod 90)

# frames
assert world_to_arm((550, 0, 320)) == (550, 0, -80)
assert flange_from_tcp((550, 0, 328)) == (550, 0, 328 + TCP_MM)

# min-jerk: exact ends, monotonic progress, count
seg = minjerk_segment((0, 0, 0, 180), (100, 0, 0, 200), 1.0, 30)
assert seg[0] == (0, 0, 0, 180) and seg[-1][:3] == (100, 0, 0) and angle_diff(seg[-1][3], 200) == 0 and len(seg) == 31
assert all(p[0] <= q[0] for p, q in zip(seg, seg[1:]))
wrap = minjerk_segment((0, 0, 0, 170), (0, 0, 0, -170), 1.0, 10)  # crosses 180: short way, +20 degrees
assert abs(angle_diff(wrap[5][3], 180)) < 1e-6
assert abs(wrap[-1][3] - wrap[-2][3]) < 5 and angle_diff(wrap[-1][3], -170) == 0  # no 360 jump at the end
assert -180 <= yaw_from_wxyz([0, 0, 0, 1]) < 180 and yaw_from_wxyz([0, 0, 0, 1]) == -180

# in the bin: inside the inner footprint and below the rim
bx, by = BIN_XY
assert in_bin((bx, by, 320), BIN_XY) and in_bin((bx + 60, by - 60, 330), BIN_XY)
assert not in_bin((bx + 80, by, 320), BIN_XY) and not in_bin((bx, by, 400), BIN_XY)

assert pose_map((1, 2, 3, 45)) == {"x": 1, "y": 2, "z": 3, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 45}

# tilts: drawn after the v1 draws, so old seeds keep their scene
r = random.Random(1003)
(x0, x1), (y0, y1) = BLOCK_AREA
v1 = {"block_xy": (r.uniform(x0, x1), r.uniform(y0, y1)), "block_yaw_deg": r.uniform(-45, 45),
      "nudge_xyz": tuple(r.uniform(-30, 30) for _ in range(3)), "nudge_yaw_deg": r.uniform(-10, 10),
      "speed": r.uniform(0.8, 1.2)}
sc = sample_scene(1003)
assert {k: sc[k] for k in v1} == v1
assert set(sc["tilts"]) == {"start", "pregrasp", "lift"}
for s in range(200):
    for tx, ty in sample_scene(s)["tilts"].values():
        assert TILT_MIN_DEG - 1e-9 <= math.hypot(tx, ty) <= TILT_MAX_DEG + 1e-9
assert sample_scene(7)["tilts"] == sample_scene(7)["tilts"]

# 6-tuple poses: (x, y, z, theta, tx, ty); untilted matches the old tool-down map exactly
assert pose_map((1, 2, 3, 45)) == {"x": 1, "y": 2, "z": 3, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 45}
p = pose_map((1, 2, 3, 45, 0.0, 0.0))
assert abs(p["o_z"] + 1) < 1e-9 and abs(p["theta"] - 45) < 1e-6
p = pose_map((0, 0, 0, 180, 4.0, 0.0))  # 4 degrees about world x
assert abs(math.degrees(math.acos(-p["o_z"])) - 4.0) < 1e-6
assert pose_map((0, 0, 0, 180, 0.5, 0.0)) == pose_map((0, 0, 0, 180))  # inside the pole band: untilted
from policy_layout import rotation  # noqa: E402
ax = rotation(pose_map((0, 0, 0, 180, 1.2, 1.6))).apply([0, 0, 1])  # 2 degrees off vertical
assert abs(math.degrees(math.acos(-ax[2])) - 2.0) < 1e-6
assert max(abs(a - b) for a, b in zip(tilt_vector(3.0, 90.0), (0.0, 3.0))) < 1e-9

# segments carry tilt: exact ends, min-jerk between
seg = minjerk_segment((0, 0, 0, 180, 0.0, 0.0), (100, 0, 0, 180, 4.0, -2.0), 1.0, 30)
assert seg[0] == (0, 0, 0, 180, 0.0, 0.0) and seg[-1] == (100, 0, 0, 180, 4.0, -2.0) and len(seg) == 31
assert abs(seg[15][4] - 2.0) < 0.2  # halfway through min-jerk is halfway
assert minjerk_segment((0, 0, 0, 180), (1, 0, 0, 180), 1.0, 10)[-1] == (1, 0, 0, 180)  # 4-tuples still work
# mixed lengths never drop a tilt
assert minjerk_segment((0, 0, 0, 180), (0, 0, 0, 180, 4.0, 0.0), 1.0, 10)[-1] == (0, 0, 0, 180, 4.0, 0.0)
assert len(minjerk_segment((0, 0, 0, 180, 4.0, 0.0), (0, 0, 0, 180), 1.0, 10)[0]) == 6

# shared block layout and start pose
pos, q = block_pose(1003)
bx, by = sc["block_xy"]
yaw = math.radians(sc["block_yaw_deg"])
assert pos == [bx / 1000, by / 1000, 0.32] and q == [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
sp = start_pose({"x": 0, "y": 0, "z": 0, "theta": 180}, 1003)
assert sp[4:] == tuple(sc["tilts"]["start"])
assert sp[:4] == (*sc["nudge_xyz"], 180 + sc["nudge_yaw_deg"])

print("ok")
