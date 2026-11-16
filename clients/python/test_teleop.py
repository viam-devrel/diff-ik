# /// script
# requires-python = ">=3.11"
# ///
"""Checks for teleop.py's pure helpers. Run: uv run clients/python/test_teleop.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop import Hold, Runaway, floor_lift, pose_delta  # noqa: E402


def P(x=200.0, y=0.0, z=150.0, ox=0.0, oy=0.0, oz=-1.0, th=0.0):
    return {"x": x, "y": y, "z": z, "o_x": ox, "o_y": oy, "o_z": oz, "theta": th}


def test_pose_delta():
    mm, deg = pose_delta(P(), P(x=203.0, y=4.0))
    assert abs(mm - 5.0) < 1e-9 and deg < 1e-9
    # axis tilted 10° in the xz plane
    t = math.radians(10)
    _, deg = pose_delta(P(), P(ox=math.sin(t), oz=-math.cos(t)))
    assert abs(deg - 10) < 1e-6, deg
    # theta wraps: 179 vs -179 is 2°, not 358°
    _, deg = pose_delta(P(th=179), P(th=-179))
    assert abs(deg - 2) < 1e-9, deg


def test_floor_lift():
    # tool straight down, end effector at z 150, tip 105 below: tip at 45 ≥ 30+15
    assert floor_lift(P(z=150), 30, 105) == 0
    # end effector at z 140: tip at 35 < 45, lift by 10
    assert abs(floor_lift(P(z=140), 30, 105) - 10) < 1e-9
    # tool pointing up: the end effector is the lowest point
    assert abs(floor_lift(P(z=40, oz=1.0), 30, 105) - 5) < 1e-9
    # unnormalised orientation vector is normalised
    assert floor_lift(P(z=150, oz=-2.0), 30, 105) == 0


def test_hold_sends_raw_while_moving_and_holds_after_dwell():
    h = Hold(band_mm=1, band_deg=0.5, dwell_s=0.2)
    t = 0.0
    # slow sweep 0.4 mm per 20 ms tick: always raw, never a held step
    for k in range(20):
        p = P(x=200 + 0.4 * k)
        assert h.update(p, t) == p, k
        t += 0.02
    # now flicker within ±0.3 mm around a point for 0.4 s
    base = P(x=210)
    sent = []
    for k in range(20):
        sent.append(h.update(P(x=210 + (0.3 if k % 2 else -0.3)), t))
        t += 0.02
    held = sent[-1]
    # after the dwell the same anchor is resent every tick
    assert all(s == held for s in sent[-5:]), sent[-5:]
    assert abs(held["x"] - 210) <= 1.0
    # leaving the band releases immediately and sends the raw pose
    p = P(x=213)
    assert h.update(p, t) == p
    del base


def test_hold_never_steps_backwards_on_slow_drift():
    h = Hold(band_mm=1, band_deg=0.5, dwell_s=0.2)
    xs = []
    for k in range(100):  # 3 mm/s drift for 2 s
        xs.append(h.update(P(x=200 + 0.06 * k), k * 0.02)["x"])
    assert all(b >= a for a, b in zip(xs, xs[1:])), "target stepped backwards"


def test_runaway():
    r = Runaway(limit_mm=250, dur_s=1.0)
    assert not r.update(300, 0.0)
    assert not r.update(300, 0.5)
    assert not r.update(100, 0.9)  # dips below: timer resets
    assert not r.update(300, 1.0)
    assert r.update(300, 2.01)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
