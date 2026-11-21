"""The pick-to-bin cell's constants and pure geometry, shared by expert_pick.py and policy_run.py.
World frame in mm; the arm stands at world (0, 0, 400) with no rotation (dev/bc-pick-mujoco.json)."""
import math
import random

ARM_BASE = (0.0, 0.0, 400.0)  # arm_origin in world, mm
TABLE_TOP = 300.0
BLOCK_MM = 40.0
BIN_XY = (550.0, -200.0)  # the bin's frame: the centre of its underside, on the table
BIN_INNER_HALF = 70.0  # 150 mm outside, 5 mm walls
BIN_RIM_Z = TABLE_TOP + 80.0
TCP_MM = 155.8  # 2F-85 tool point along the tool axis
BLOCK_AREA = ((425.0, 675.0), (-60.0, 140.0))  # world x and y ranges for the block's centre
GRASP_ABOVE_CENTER = 8.0  # TCP this far above the block's centre: closed fingertips clear the table
PREGRASP_ABOVE = 100.0
LIFT = 100.0
PLACE_ABOVE_RIM = 120.0
RETREAT = 50.0
TILT_MAX_DEG = 4.0
TILTED = ("start", "pregrasp", "lift")  # grasp, above-bin (release) and retreat stay vertical


def tilt_vector(mag_deg: float, azimuth_deg: float) -> tuple:
    a = math.radians(azimuth_deg)
    return (mag_deg * math.cos(a), mag_deg * math.sin(a))


def sample_scene(seed: int) -> dict:
    """The episode's randomization (spec "Randomization"); the same seed gives the same scene."""
    r = random.Random(seed)
    (x0, x1), (y0, y1) = BLOCK_AREA
    d = {"block_xy": (r.uniform(x0, x1), r.uniform(y0, y1)), "block_yaw_deg": r.uniform(-45, 45),
         "nudge_xyz": tuple(r.uniform(-30, 30) for _ in range(3)), "nudge_yaw_deg": r.uniform(-10, 10),
         "speed": r.uniform(0.8, 1.2)}
    d["tilts"] = {k: tilt_vector(r.uniform(0, TILT_MAX_DEG), r.uniform(0, 360)) for k in TILTED}
    return d


def angle_diff(a: float, b: float) -> float:
    """a − b wrapped to [−180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def yaw_from_wxyz(q) -> float:
    """Yaw (degrees about world z) of a quaternion [w, x, y, z] that rotates about z only."""
    return angle_diff(math.degrees(2 * math.atan2(q[3], q[0])), 0)


def grasp_theta(block_yaw_deg: float, current_theta_deg: float) -> float:
    """The tool's orientation-vector theta (tool pointing down) that lines the jaws up with a block face, choosing
    the face nearest the current theta. For o = (0, 0, −1), the tool's x axis points at 180° − theta in the world,
    and a cube repeats every 90°, so the candidates are 180 − yaw + 90k."""
    base = 180.0 - block_yaw_deg
    k = round(angle_diff(current_theta_deg, base) / 90.0)
    return current_theta_deg - angle_diff(current_theta_deg, base + 90.0 * k)


def world_to_arm(p) -> tuple:
    return tuple(v - b for v, b in zip(p, ARM_BASE))


def flange_from_tcp(tcp) -> tuple:
    """Tool pointing down: the flange is TCP_MM above the tool point."""
    return (tcp[0], tcp[1], tcp[2] + TCP_MM)


def minjerk_segment(a, b, duration_s: float, hz: float) -> list:
    """Poses from a to b along a min-jerk profile, both ends included; theta turns the short way. Ends are
    (x, y, z, theta) or (x, y, z, theta, tx, ty); the output is 6-long if either end is, else 4."""
    width = 6 if max(len(a), len(b)) == 6 else 4
    a, b = ((*p, 0.0, 0.0)[:6] for p in (a, b))  # pad 4-tuples with zero tilt
    n = max(1, round(duration_s * hz))
    dth = angle_diff(b[3], a[3])
    out = []
    for i in range(n + 1):
        s = i / n
        m = s * s * s * (10 - 15 * s + 6 * s * s)
        q = [a[k] + (b[k] - a[k]) * m for k in range(6)]
        q[3] = a[3] + dth * m
        out.append(tuple(q[:width]))
    out[-1] = (b[0], b[1], b[2], a[3] + dth, b[4], b[5])[:width]  # same orientation as b, no 360 jump
    return out


def in_bin(block_world_mm, bin_xy=BIN_XY) -> bool:
    """The block's centre is inside the bin's inner footprint and below its rim."""
    x, y, z = block_world_mm
    return abs(x - bin_xy[0]) <= BIN_INNER_HALF and abs(y - bin_xy[1]) <= BIN_INNER_HALF and z < BIN_RIM_Z


def pose_map(p) -> dict:
    """A servo pose map for (x, y, z, theta[, tx, ty]): tool down, then tilted by the world rotation vector
    (tx, ty) degrees."""
    if len(p) < 6 or not (p[4] or p[5]):
        return {"x": p[0], "y": p[1], "z": p[2], "o_x": 0, "o_y": 0, "o_z": -1, "theta": p[3]}
    import numpy as np  # here so bc_pick imports without them for callers that never tilt
    from scipy.spatial.transform import Rotation
    from viam.spatialmath import RotationMatrix

    from policy_layout import rotation

    down = rotation({"o_x": 0, "o_y": 0, "o_z": -1, "theta": p[3]})
    R = Rotation.from_rotvec(np.radians([p[4], p[5], 0.0])) * down
    v = RotationMatrix(R.as_matrix().reshape(9).tolist()).to_quaternion().to_orientation_vector()
    return {"x": p[0], "y": p[1], "z": p[2], "o_x": float(v.o_x), "o_y": float(v.o_y), "o_z": float(v.o_z),
            "theta": float(np.degrees(v.theta))}


async def connect(address: str, insecure: bool, n: int = 1) -> list:
    """n robot connections (so streams don't queue behind each other): insecure for a local viam-server, else the
    API key in VIAM_API_KEY / VIAM_API_KEY_ID."""
    import os

    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions

    dial = (DialOptions(insecure=True, disable_webrtc=True) if insecure
            else DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]))
    return [await RobotClient.at_address(address, RobotClient.Options(dial_options=dial)) for _ in range(n)]
