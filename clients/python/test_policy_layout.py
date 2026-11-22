# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "viam-sdk"]
# ///
"""Pure-path checks for policy_layout.py, including against the converter's own pose code.
Run: uv run clients/python/test_policy_layout.py"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_layout import chunk_to_poses, state_vector  # noqa: E402

pose = {"x": 512.3, "y": -40.1, "z": 95.0, "o_x": 0.02, "o_y": -0.03, "o_z": -0.999, "theta": 137.0}
s = state_vector(pose, 63.0)
assert len(s) == 10 and s[9] == 63.0 and s[:3] == [512.3, -40.1, 95.0]

# round trip: state -> pose -> state
poses, grip = chunk_to_poses([s])
s2 = state_vector(poses[0], grip[0])
np.testing.assert_allclose(s2, s, atol=1e-6)

# exactly at the tool-down pole (every pose in the dataset sits near it)
pole = {"x": 550.0, "y": 0.0, "z": 85.0, "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 137.0}
sp = state_vector(pole, 44.0)
pp, gp = chunk_to_poses([sp])
np.testing.assert_allclose(state_vector(pp[0], gp[0]), sp, atol=1e-6)

# noisy rotation rows (a policy's raw output) still give a valid pose
noisy = list(s)
noisy[3:9] = [v * 1.07 + 0.01 for v in noisy[3:9]]
p, _ = chunk_to_poses([noisy])
n = np.linalg.norm([p[0]["o_x"], p[0]["o_y"], p[0]["o_z"]])
assert abs(n - 1) < 1e-6

# pole band: a sub-TILT_MIN_DEG tilt snaps to tool-down at the same yaw; a bigger one passes through
from scipy.spatial.transform import Rotation  # noqa: E402
from policy_layout import pose_map_from, rotation  # noqa: E402

down = rotation({"o_x": 0, "o_y": 0, "o_z": -1, "theta": 33})
want = pose_map_from([1, 2, 3], down)
snap = pose_map_from([1, 2, 3], Rotation.from_rotvec(np.radians([0.5, 0.0, 0.0])) * down)
assert max(abs(snap[k] - want[k]) for k in want) < 1e-6
assert abs(snap["theta"] - want["theta"]) < 1e-6 and abs(snap["o_z"] + 1) < 1e-9
tilt = pose_map_from([1, 2, 3], Rotation.from_rotvec(np.radians([2.0, 0.0, 0.0])) * down)
assert abs(tilt["o_z"] + 1) > 1e-5 and abs(abs(np.degrees(np.arccos(-tilt["o_z"]))) - 2) < 1e-3

# the same layout as the converter's pose_state (the dataset's source of truth)
conv = os.path.expanduser("~/src/viam-sequence-to-lerobot/src")
if os.path.isdir(conv):
    sys.path.insert(0, conv)
    from viam_sequence_to_lerobot.pose import pose_state  # noqa: E402

    np.testing.assert_allclose(pose_state({"pose": pose}), s[:9], atol=1e-9)
else:
    print("skipped the converter cross-check: ~/src/viam-sequence-to-lerobot not found")
print("ok")
