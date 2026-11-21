# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "pillow"]
# ///
"""Write dev/assets/wood.png: a procedural light-oak grain, so the cell needs no downloaded texture.
Run: uv run dev/make_wood_texture.py"""
import os

import numpy as np
from PIL import Image

rng = np.random.default_rng(7)
n = 1024
y, x = np.mgrid[0:n, 0:n] / n
warp = 0.04 * np.sin(2 * np.pi * (3 * x + rng.random())) + 0.02 * np.sin(2 * np.pi * (11 * x + rng.random()))
rings = np.sin(2 * np.pi * (38 * (y + warp)))
grain = 0.5 + 0.35 * rings + 0.15 * rng.standard_normal((n, n)).cumsum(axis=1) / np.sqrt(np.arange(1, n + 1))
grain = (grain - grain.min()) / (grain.max() - grain.min())
light, dark = np.array([214, 178, 128]), np.array([150, 105, 62])
img = (dark + (light - dark) * grain[..., None]).astype(np.uint8)
os.makedirs(os.path.join(os.path.dirname(__file__), "assets"), exist_ok=True)
Image.fromarray(img).save(os.path.join(os.path.dirname(__file__), "assets", "wood.png"))
print("wrote dev/assets/wood.png")
