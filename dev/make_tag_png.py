# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python", "numpy"]
# ///
"""Phase 4 tag texture: tag36h11 ID 0 as a 10x10-cell PNG (the 8-cell black square plus a 1-cell white
border). On the 75 mm sim prop the black square is 60 mm, so tag_width_mm is 60.
Run: uv run dev/make_tag_png.py"""
import os

import cv2
import numpy as np

CELL = 80  # px per cell
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "tag36h11_0.png")

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
black = cv2.aruco.generateImageMarker(d, 0, 8 * CELL, borderBits=1)  # 6x6 data + 1-cell black border
img = np.full((10 * CELL, 10 * CELL), 255, np.uint8)
img[CELL:9 * CELL, CELL:9 * CELL] = black
os.makedirs(os.path.dirname(OUT), exist_ok=True)
cv2.imwrite(OUT, img)

# self-check: it decodes as ID 0, and the black square is 80 % of the image
corners, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(img)
assert ids is not None and ids.flatten().tolist() == [0], ids
side = np.linalg.norm(corners[0][0][0] - corners[0][0][1])
assert abs(side / img.shape[0] - 0.8) < 0.01, side
print("wrote", OUT, img.shape)
