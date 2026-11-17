# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python", "numpy", "viam-sdk"]
# ///
"""ChArUco intrinsic calibration for a Viam webcam.

uv run clients/python/intrinsics.py --address HOST:8080 --camera NAME \
    --dict DICT_5X5_100 --squares 7x5 --square-mm 40 --marker-mm 30 --views 20

Prints the JSON to paste into the builtin webcam's attributes. Note: viam:apriltag uses only
fx, fy, ppx, ppy and never applies the distortion terms, so residual distortion costs accuracy.
"""
import argparse
import asyncio
import json

import cv2
import numpy as np

MIN_CORNERS = 8


def make_board(dict_name, squares, square_mm, marker_mm):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    return cv2.aruco.CharucoBoard(squares, square_mm / 1000, marker_mm / 1000, d)


def detect(img, detector):
    """-> (corners, ids) of the ChArUco corners, or (None, None)."""
    corners, ids, _, _ = detector.detectBoard(img)
    if ids is None or len(ids) < MIN_CORNERS:
        return None, None
    return corners, ids


def calibrate(frames, board):
    """-> (K, dist, rms, n_views). OpenCV 5 has no calibrateCameraCharuco."""
    det = cv2.aruco.CharucoDetector(board)
    obj, img = [], []
    for f in frames:
        corners, ids = detect(f, det)
        if ids is None:
            continue
        o, i = board.matchImagePoints(corners, ids)
        if len(o) >= MIN_CORNERS:
            obj.append(o.reshape(-1, 3).astype(np.float32))
            img.append(i.reshape(-1, 2).astype(np.float32))
    if len(obj) < 3:
        raise RuntimeError(f"only {len(obj)} usable views")
    size = (frames[0].shape[1], frames[0].shape[0])
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None)
    return K, dist.ravel(), float(rms), len(obj)


def hull_stats(corners):
    h = cv2.convexHull(corners.reshape(-1, 2).astype(np.float32))
    return h.reshape(-1, 2).mean(axis=0), cv2.contourArea(h)


def is_new_view(corners, accepted, size):
    """Accept if the hull centre moved > 5 % of the image, or the hull area changed > 15 %."""
    c, a = hull_stats(corners)
    diag = float(np.hypot(*size))
    return all(np.hypot(*(c - c2)) > 0.05 * diag or abs(a - a2) > 0.15 * a2 for c2, a2 in accepted)


def webcam_json(K, dist, size):
    k1, k2, p1, p2, k3 = (list(dist) + [0.0] * 5)[:5]
    return {
        "intrinsic_parameters": {"width_px": size[0], "height_px": size[1], "fx": float(K[0, 0]),
                                 "fy": float(K[1, 1]), "ppx": float(K[0, 2]), "ppy": float(K[1, 2])},
        "distortion_parameters": {"rk1": float(k1), "rk2": float(k2), "rk3": float(k3),
                                  "tp1": float(p1), "tp2": float(p2)},
    }


async def grab(cam):
    imgs, _ = await cam.get_images()
    img = cv2.imdecode(np.frombuffer(imgs[0].data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"cannot decode image of type {imgs[0].mime_type}")
    return img


async def main():
    from viam.components.camera import Camera
    from servo_client import connect

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8080")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--camera", required=True)
    ap.add_argument("--dict", default="DICT_5X5_100")
    ap.add_argument("--squares", default="7x5")
    ap.add_argument("--square-mm", type=float, default=40)
    ap.add_argument("--marker-mm", type=float, default=30)
    ap.add_argument("--views", type=int, default=20)
    args = ap.parse_args()
    board = make_board(args.dict, tuple(map(int, args.squares.split("x"))), args.square_mm, args.marker_mm)
    det = cv2.aruco.CharucoDetector(board)
    robot = await connect(args)
    try:
        cam = Camera.from_robot(robot, args.camera)
        frames, accepted = [], []
        while len(frames) < args.views:
            img = await grab(cam)
            corners, ids = detect(img, det)
            if ids is not None and is_new_view(corners, accepted, img.shape[::-1]):
                accepted.append(hull_stats(corners))
                frames.append(img)
                print(f"view {len(frames)}/{args.views} accepted ({len(ids)} corners); move the board")
            await asyncio.sleep(0.3)
    finally:
        await robot.close()
    K, dist, rms, n = calibrate(frames, board)
    print(f"\nRMS reprojection error {rms:.3f} px over {n} views (want < 1.0)")
    print(json.dumps(webcam_json(K, dist, frames[0].shape[::-1]), indent=2))


if __name__ == "__main__":
    asyncio.run(main())
