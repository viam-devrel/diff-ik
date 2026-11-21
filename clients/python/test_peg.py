# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for peg.py. Run: uv run clients/python/test_peg.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from peg import offsets, spiral_point, spiral_step  # noqa: E402

x, y = spiral_point(2 * math.pi, 1.5)
assert abs(x - 1.5) < 1e-9 and abs(y) < 1e-9, (x, y)  # one turn out: radius = pitch
assert abs(spiral_step(0.0, 1.5, 0.3) - 0.2) < 1e-9  # near the centre the step is floored at r = pitch
assert abs(spiral_step(4 * math.pi, 1.5, 0.3) - 0.1) < 1e-9  # r = 3 mm: dθ = ds / r
os_ = offsets(200, 5, 1)
assert len(os_) == 200 and all(math.hypot(*o) <= 5 for o in os_) and offsets(3, 5, 1) == offsets(3, 5, 1)
assert max(math.hypot(*o) for o in os_) > 4  # spread over the disc, not bunched at the centre
print("ok")
