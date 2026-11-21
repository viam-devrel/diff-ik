# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "pillow"]
# ///
"""Write dev/assets/wood-v2.png: procedural dark, desaturated, irregular wood grain with knots.
Run: uv run dev/make_wood_texture.py"""
import os

import numpy as np
from PIL import Image

rng = np.random.default_rng(11)
n = 1024
y, x = np.mgrid[0:n, 0:n] / n


def noise(fx, fy, octaves=5):
    """Multi-octave value noise (not tileable), stretched by (fx, fy) base frequencies."""
    out = np.zeros((n, n))
    for o in range(octaves):
        gx, gy = max(2, int(fx * 2**o)), max(2, int(fy * 2**o))
        g = rng.random((gy + 2, gx + 2))
        u, v = x * gx, y * gy
        iu, iv = u.astype(int), v.astype(int)
        fu, fv = u - iu, v - iv
        fu, fv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
        a = g[iv, iu] * (1 - fu) + g[iv, iu + 1] * fu
        b = g[iv + 1, iu] * (1 - fu) + g[iv + 1, iu + 1] * fu
        out += (a * (1 - fv) + b * fv) / 2**o
    return out / out.max()


# grain runs along x: low frequency in x, high in y
warp = 0.12 * (noise(2, 3, 3) - 0.5)
yy = y + warp
# knots: elliptical distortion of ring coordinate
for _ in range(5):
    kx, ky, r = rng.random(), rng.random(), rng.uniform(0.03, 0.06)
    d = np.sqrt(((x - kx) / 2.5) ** 2 + (yy - ky) ** 2)
    yy = yy + 0.05 * np.exp(-(d / (r * 1.5)) ** 2) * np.sign(yy - ky)
ring_pos = 22 * yy + 3 * noise(3, 6, 4)  # irregular ring spacing
rings = np.abs(ring_pos - np.round(ring_pos))  # sharp-ish ring lines
fine = noise(6, 160, 4)  # long thin fibres
grain = 0.45 * (1 - rings * 2) ** 2 + 0.35 * fine + 0.2 * noise(2, 5, 3)
grain = (grain - grain.min()) / (grain.max() - grain.min())
light, dark = np.array([170, 135, 95]), np.array([105, 75, 48])
img = dark + (light - dark) * grain[..., None]
os.makedirs(os.path.join(os.path.dirname(__file__), "assets"), exist_ok=True)
Image.fromarray(img.astype(np.uint8)).save(os.path.join(os.path.dirname(__file__), "assets", "wood-v2.png"))
print("wrote dev/assets/wood-v2.png")
