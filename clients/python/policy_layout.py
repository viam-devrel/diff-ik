"""The ee dataset layout <-> servo poses: [x, y, z (mm), r00 r01 r02 r10 r11 r12, gripper 0-100], flange in the arm's
base frame (viam-sequence-to-lerobot --action-space ee)."""
import numpy as np
from scipy.spatial.transform import Rotation
from viam.spatialmath import OrientationVector, RotationMatrix


def rotation(pose: dict) -> Rotation:
    q = OrientationVector(pose["o_x"], pose["o_y"], pose["o_z"], np.deg2rad(pose["theta"])).to_quaternion()
    return Rotation.from_matrix(np.asarray(q.to_rotation_matrix().elements, dtype=np.float64).reshape(3, 3))


_rotation = rotation  # old name, still imported elsewhere


def state_vector(pose: dict, gripper: float) -> list:
    """A pose map (x, y, z mm; o_x, o_y, o_z, theta degrees) and the gripper opening as the 10-number state."""
    rows = rotation(pose).as_matrix()[:2, :].reshape(6)
    return [float(pose["x"]), float(pose["y"]), float(pose["z"]), *map(float, rows), float(gripper)]


def _rows_rotation(r) -> Rotation:
    a1, a2 = np.asarray(r[0:3], dtype=np.float64), np.asarray(r[3:6], dtype=np.float64)
    b1 = a1 / np.linalg.norm(a1)
    b2 = a2 - (b1 @ a2) * b1
    b2 /= np.linalg.norm(b2)
    return Rotation.from_matrix(np.stack([b1, b2, np.cross(b1, b2)]))


def chunk_to_poses(rows) -> tuple:
    """Policy action rows (10 numbers each) as servo pose maps, plus the gripper column."""
    poses, grip = [], []
    for r in rows:
        m = RotationMatrix(_rows_rotation(r[3:9]).as_matrix().reshape(9).tolist())
        v = m.to_quaternion().to_orientation_vector()
        poses.append({"x": float(r[0]), "y": float(r[1]), "z": float(r[2]), "o_x": float(v.o_x),
                      "o_y": float(v.o_y), "o_z": float(v.o_z), "theta": float(np.degrees(v.theta))})
        grip.append(float(r[9]) if len(r) > 9 else float("nan"))
    return poses, grip
