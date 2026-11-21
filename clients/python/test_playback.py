# /// script
# requires-python = ">=3.11"
# ///
"""Checks for playback.py. Run: uv run clients/python/test_playback.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from playback import blended  # noqa: E402

at = lambda c, t: c.get(t)  # noqa: E731  a chunk is {time: action}
mix = lambda a, b, w: a + (b - a) * w  # noqa: E731
cur, prev = {1: 10.0}, {1: 0.0}
assert blended(cur, None, 1, 0.5, at, mix) == 10.0          # no previous chunk
assert blended(cur, prev, 1, 0.25, at, mix) == 2.5           # crossfade
assert blended(cur, prev, 1, 1.0, at, mix) == 10.0           # fade done
assert blended(cur, {}, 1, 0.25, at, mix) == 10.0            # previous chunk used up
assert blended({}, prev, 1, 0.25, at, mix) is None           # current chunk used up
print("ok")
