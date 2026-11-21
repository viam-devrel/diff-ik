# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for sand.py. Run: uv run clients/python/test_sand.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sand import pass_stats, raster  # noqa: E402

assert raster(0, 10, [1, 2, 3]) == [(10, 1, 0), (10, 2, None), (0, 2, 1), (0, 3, None), (10, 3, 2)]
m = pass_stats([8.0, 10.0, 12.0, 13.0], target=10, band=0.2)
assert m["mean_n"] == 10.75 and m["min_n"] == 8.0 and m["max_n"] == 13.0 and m["in_band"] == 0.75, m
print("ok")
