# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python", "numpy"]
# ///
"""Synthetic check for intrinsics.calibrate. Run: uv run clients/python/test_intrinsics.py"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from intrinsics import calibrate  # noqa: E402

W, H = 1280, 720
K_TRUE = np.array([[800.0, 0, 640], [0, 800.0, 360], [0, 0, 1]])


def make_board():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    return cv2.aruco.CharucoBoard((7, 5), 0.04, 0.03, d)


def render(board, rng, n=15):
    m = 40
    bw, bh = 7 * 100 + 2 * m, 5 * 100 + 2 * m  # 100 px per square
    img = board.generateImage((bw, bh), marginSize=m)
    # board-image px -> metric board plane, centred on the origin (0.28 x 0.20 m)
    src = np.float32([[m, m], [bw - m, m], [bw - m, bh - m], [m, bh - m]])
    plane = np.float32([[-.14, -.10, 0], [.14, -.10, 0], [.14, .10, 0], [-.14, .10, 0]])
    frames = []
    while len(frames) < n:
        rvec = rng.uniform(-0.5, 0.5, 3)
        tvec = np.array([rng.uniform(-.08, .08), rng.uniform(-.05, .05), rng.uniform(.45, .7)])
        # the margin corners map via the homography of the 4 board corners
        dst, _ = cv2.projectPoints(plane, rvec, tvec, K_TRUE, None)
        Hm = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
        frames.append(cv2.warpPerspective(img, Hm, (W, H), borderValue=255))
    return frames


def test_calibrate():
    board = make_board()
    frames = render(board, np.random.default_rng(1))
    K, dist, rms, n = calibrate(frames, board)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"views={n} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} rms={rms:.3f}")
    assert n >= 10
    assert abs(fx - 800) / 800 < 0.02 and abs(fy - 800) / 800 < 0.02
    assert abs(cx - 640) < 5 and abs(cy - 360) < 5
    assert rms < 1.0


if __name__ == "__main__":
    test_calibrate()
    print("ok")
