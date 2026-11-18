# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for prop_mover.py. Run: uv run clients/python/test_prop_mover.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prop_mover import path_state, vec3  # noqa: E402

A, B = (0.0, 0.0, 100.0), (300.0, 400.0, 100.0)  # 500 mm apart: 10 s at 50 mm/s
assert path_state(-1, A, B, 50, 2, True) == (A, False)
assert path_state(0, A, B, 50, 2, True) == (A, False)
pos, done = path_state(5, A, B, 50, 2, True)
assert pos == (150.0, 200.0, 100.0) and not done, pos
assert path_state(10, A, B, 50, 2, True) == (B, False)
assert path_state(11, A, B, 50, 2, True) == (B, False)  # holding
pos, done = path_state(17, A, B, 50, 2, True)  # 5 s into the way back
assert all(abs(p - q) < 1e-9 for p, q in zip(pos, (150.0, 200.0, 100.0))) and not done, pos
assert path_state(22, A, B, 50, 2, True) == (A, True)
assert path_state(99, A, B, 50, 2, True) == (A, True)
assert path_state(11.9, A, B, 50, 2, False) == (B, False)
assert path_state(12, A, B, 50, 2, False) == (B, True)
assert path_state(-1, A, A, 50, 2, True) == (A, False)  # from == to: the delay and the hold still apply
assert path_state(1.9, A, A, 50, 2, True) == (A, False)
assert path_state(2, A, A, 50, 2, True) == (A, True)
assert vec3("1, 2,3.5") == (1.0, 2.0, 3.5)
for bad in ("1,2", "1,2,x", "1,2,nan"):
    try:
        vec3(bad)
    except Exception:
        continue
    raise AssertionError(bad)
print("ok")
