"""Sanity checks for a converted policy-sim LeRobot dataset.

Run with the converter's env:
  cd ~/src/viam-sequence-to-lerobot && uv run python ~/src/diff-ik/dev/check_policy_dataset.py <dataset root> [repo-id]

repo-id defaults to local/<root dirname> with a leading "lerobot-" stripped.
State/action are 10-d: x, y, z, r00..r12, gripper. The 2F-85 reads ~100 open and ~44
holding the 40 mm block (never near 0).
"""
import os, sys
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = sys.argv[1].rstrip("/")
rid = sys.argv[2] if len(sys.argv) > 2 else "local/" + os.path.basename(root).removeprefix("lerobot-")
ds = LeRobotDataset(rid, root=root)
print("episodes", ds.num_episodes, "frames", ds.num_frames, "fps", ds.fps)
eps = np.array(ds.hf_dataset["episode_index"])
S = np.array(ds.hf_dataset["observation.state"])
A = np.array(ds.hf_dataset["action"])
lens = np.bincount(eps)
print("len min/median/max", lens.min(), int(np.median(lens)), lens.max())
print("NaN state/action", int(np.isnan(S).sum()), int(np.isnan(A).sum()))
g = S[:, 9]
print("gripper min/max %.1f %.1f (action %.1f %.1f)" % (g.min(), g.max(), A[:, 9].min(), A[:, 9].max()))
print("gripper open(>=90) %.3f holding(40-50) %.3f bimodal(within 10 of 44 or 100) %.3f"
      % ((g >= 90).mean(), ((g >= 40) & (g <= 50)).mean(), ((abs(g - 44) <= 10) | (abs(g - 100) <= 10)).mean()))
print("xyz min", S[:, :3].min(0).round(1), "max", S[:, :3].max(0).round(1))
grasp, last, worst = [], [], 0
for e in range(ds.num_episodes):
    idx = np.where(eps == e)[0]
    s = S[idx]
    grasp.append(s[np.argmax(s[:, 9] < 60), :3])  # first frame gripper < 60
    last.append(s[-1, :3])
    worst = max(worst, np.abs(A[idx[:-1], :3] - S[idx[1:], :3]).max())
grasp, last = np.array(grasp), np.array(last)
print("grasp xyz min", grasp.min(0).round(1), "max", grasp.max(0).round(1))
print("last-frame (release) xyz min", last.min(0).round(1), "max", last.max(0).round(1))
print("max |action[t,:3]-state[t+1,:3]| (mm)", worst)
f = ds[0]
for k in ds.meta.camera_keys:
    print(k, tuple(f[k].shape))
print("task", f["task"])
