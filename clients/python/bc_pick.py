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


def sample_scene(seed: int) -> dict:
    """The episode's randomization (spec "Randomization"); the same seed gives the same scene."""
    r = random.Random(seed)
    (x0, x1), (y0, y1) = BLOCK_AREA
    return {"block_xy": (r.uniform(x0, x1), r.uniform(y0, y1)), "block_yaw_deg": r.uniform(-45, 45),
            "nudge_xyz": tuple(r.uniform(-30, 30) for _ in range(3)), "nudge_yaw_deg": r.uniform(-10, 10),
            "speed": r.uniform(0.8, 1.2)}


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
    """Poses (x, y, z, theta) from a to b along a min-jerk profile, both ends included; theta turns the short way."""
    n = max(1, round(duration_s * hz))
    dth = angle_diff(b[3], a[3])
    out = []
    for i in range(n + 1):
        s = i / n
        m = s * s * s * (10 - 15 * s + 6 * s * s)
        out.append((a[0] + (b[0] - a[0]) * m, a[1] + (b[1] - a[1]) * m, a[2] + (b[2] - a[2]) * m, a[3] + dth * m))
    out[-1] = (b[0], b[1], b[2], a[3] + dth)  # same orientation as b, no 360 jump
    return out


def in_bin(block_world_mm, bin_xy=BIN_XY) -> bool:
    """The block's centre is inside the bin's inner footprint and below its rim."""
    x, y, z = block_world_mm
    return abs(x - bin_xy[0]) <= BIN_INNER_HALF and abs(y - bin_xy[1]) <= BIN_INNER_HALF and z < BIN_RIM_Z


def pose_map(p) -> dict:
    """A servo pose map for (x, y, z, theta) with the tool pointing down."""
    return {"x": p[0], "y": p[1], "z": p[2], "o_x": 0, "o_y": 0, "o_z": -1, "theta": p[3]}


async def connect(address: str, insecure: bool, n: int = 1) -> list:
    """n robot connections (so streams don't queue behind each other): insecure for a local viam-server, else the
    API key in VIAM_API_KEY / VIAM_API_KEY_ID."""
    import os

    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions

    dial = (DialOptions(insecure=True, disable_webrtc=True) if insecure
            else DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]))
    return [await RobotClient.at_address(address, RobotClient.Options(dial_options=dial)) for _ in range(n)]
