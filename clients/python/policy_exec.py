"""Policy chunk execution: pure helpers (top), executors below."""
import math

import numpy as np
from scipy.spatial.transform import Slerp, Rotation

from bc_pick import in_bin
from policy_layout import pose_map_from, rotation

# --- helpers ---


def first_live_row(t_obs: float, dt: float, now: float) -> int:
    """Index of the first chunk row not yet due at `now` (a row due exactly now counts as live)."""
    return max(0, math.ceil((now - t_obs) / dt - 1e-9))


def gripper_events(gcol, t_obs: float, dt: float, closed: bool, lead: float) -> list:
    """(time, "grab"|"open") at each 0.5 crossing from the last commanded state, fired `lead` s early for actuation latency."""
    out = []
    for k, v in enumerate(gcol):
        want = v > 0.5
        if want != closed:
            out.append((t_obs + k * dt - lead, "grab" if want else "open"))
            closed = want
    return out


def jerk_rms(samples) -> float:
    """RMS of the 3-D jerk (mm/s^3) from (t, x, y, z) samples; NaN under 4 samples."""
    if len(samples) < 4:
        return float("nan")
    a = np.asarray(samples, dtype=float)
    dt = np.diff(a[:, 0]).mean()
    j = np.diff(a[:, 1:], n=3, axis=0) / dt**3
    return float(np.sqrt((np.linalg.norm(j, axis=1) ** 2).mean()))


def succeeded(b1, b2, gripper_input: float) -> bool:
    """Block in the bin, at rest between two readings, and the gripper released."""
    return in_bin(b2) and math.dist(b1, b2) < 1.0 and gripper_input < 0.5


def _blend(a: dict, b: dict, f: float) -> dict:
    ra, rb = rotation(a), rotation(b)
    R = Slerp([0.0, 1.0], Rotation.concatenate([ra, rb]))([f])[0]
    return pose_map_from([a[k] + f * (b[k] - a[k]) for k in "xyz"], R)


def sample_chunk(chunk: dict, t: float) -> dict:
    """Pose at time t along a chunk: linear position, slerped orientation, clamped to the ends."""
    p, n = chunk["poses"], len(chunk["poses"])
    s = min(max((t - chunk["t_obs"]) / chunk["dt"], 0.0), n - 1)
    k = min(int(math.floor(s)), max(n - 2, 0))
    return _blend(p[k], p[min(k + 1, n - 1)], s - k)


def mix_pose(a: dict, b: dict, w: float) -> dict:
    """Pose w of the way from a to b (w=0 is a, w=1 is b), for crossfading between chunks."""
    return _blend(a, b, w)


# --- executors ---
