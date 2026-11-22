# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy"]
# ///
"""Table checks for dagger_trigger.py. Run: uv run clients/python/test_dagger_trigger.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bc_pick import BIN_XY, GRASP_ABOVE_CENTER, RELEASE_TCP  # noqa: E402
from dagger_trigger import Cfg, check  # noqa: E402

BLOCK = (500.0, 50.0, 320.0)
GRASP = (BLOCK[0], BLOCK[1], BLOCK[2] + GRASP_ABOVE_CENTER)
CFG = Cfg()


def st(t, tcp, block=BLOCK, grip=0, holding=False):
    return {"t": round(t, 3), "tcp": tcp, "block": block, "grip_cmd": grip, "holding": holding}


def run(states):
    """check() at every state with all earlier states as history; returns the list of results."""
    return [check(s, states[:i], CFG) for i, s in enumerate(states)]


def ticks(t0, t1):
    n = round((t1 - t0) * 10)
    return [t0 + i / 10 for i in range(n + 1)]


# approach_stall: static far from the block; nothing until the 2 s window fills
far = (GRASP[0] - 100, GRASP[1], GRASP[2] + 100)
out = run([st(t, far) for t in ticks(0.0, 2.0)])
assert out[:-1] == [None] * (len(out) - 1) and out[-1] == "approach_stall", out

# approaching at 50 mm/s: no stall
out = run([st(t, (far[0] + 50 * t, far[1], far[2])) for t in ticks(0.0, 1.9)] +
          [st(t, (far[0] + 50 * t, far[1], far[2])) for t in ticks(2.0, 2.0)])
assert out[-1] is None, out

# parked within 15 mm of the grasp point (closing on it): no stall
near = (GRASP[0] + 5, GRASP[1], GRASP[2])
assert run([st(t, near) for t in ticks(0.0, 3.0)])[-1] is None

# empty_grasp: closed at 5.0 with no firm grasp; fires at 6.0, not at 5.9
s = [st(t, near) for t in ticks(4.0, 4.9)] + [st(t, near, grip=1) for t in ticks(5.0, 6.0)]
out = run(s)
assert out[-2] is None and out[-1] == "empty_grasp", out

# grasped, then dropped: no empty_grasp while it falls, drop once it rests outside the bin
s = [st(t, near, grip=1) for t in ticks(5.0, 5.2)]
s += [st(t, (near[0], near[1], near[2] + 20 * (t - 5.3)), grip=1, holding=True) for t in ticks(5.3, 7.0)]
s += [st(7.1, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 360.0), grip=1),
      st(7.2, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 330.0), grip=1),
      st(7.3, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 320.0), grip=1),
      st(7.4, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 320.0), grip=1)]
out = run(s)
assert out[-4] is None and out[-3] is None and out[-2] is None and out[-1] == "drop", out[-5:]

# released over the bin: falling, then at rest in the bin; never a trigger
hold = (RELEASE_TCP[0], RELEASE_TCP[1], RELEASE_TCP[2])
inside = (BIN_XY[0], BIN_XY[1])
s = [st(t, hold, block=(*inside, 480.0), grip=1, holding=True) for t in ticks(10.0, 12.0)]
s += [st(12.1, hold, block=(*inside, 450.0)), st(12.2, hold, block=(*inside, 400.0)),
      st(12.3, hold, block=(*inside, 340.0)), st(12.4, hold, block=(*inside, 325.0)),
      st(12.5, hold, block=(*inside, 325.0))]
assert run(s)[-5:] == [None] * 5, run(s)[-5:]

# carry_stall: holding, parked away from the bin; a vertical lift is progress; hovering over the bin is not a stall
park = (400.0, 100.0, 450.0)
assert run([st(t, park, grip=1, holding=True) for t in ticks(0.0, 2.0)])[-1] == "carry_stall"
lift = [st(t, (400.0, 100.0, 330.0 + 50 * t), grip=1, holding=True) for t in ticks(0.0, 2.0)]
assert run(lift)[-1] is None
over = (RELEASE_TCP[0] + 30, RELEASE_TCP[1], RELEASE_TCP[2] - 40)
assert run([st(t, over, grip=1, holding=True) for t in ticks(0.0, 3.0)])[-1] is None

# timeout, and precedence (drop beats timeout)
s = [st(t, (far[0] + 50 * (t - 23.0), far[1], far[2])) for t in ticks(23.0, 25.0)]
assert run(s)[-1] == "timeout"
s = [st(24.8, near, holding=True, grip=1), st(24.9, near, block=(450.0, 0.0, 320.0), grip=1),
     st(25.0, near, block=(450.0, 0.0, 320.0), grip=1)]
assert run(s)[-1] == "drop"
print("ok")
