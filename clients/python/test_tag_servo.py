# /// script
# requires-python = ">=3.11"
# ///
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asyncio
from types import SimpleNamespace
from tag_servo import (hover_target, normal_jump_deg, lower_along_axis, down_target, grazing, Gate,  # noqa: E402
                       keepalive_due, TagServo)


def T(x, y, z, ox, oy, oz, th=0.0):
    return {"x": x, "y": y, "z": z, "o_x": ox, "o_y": oy, "o_z": oz, "theta": th}


def test_hover_flips_normal_toward_camera():
    # tag 500 mm in front of the camera (camera looks along +z); tag normal +z (away from camera) or -z
    for oz in (1.0, -1.0):
        t = hover_target(T(0, 0, 500, 0, 0, oz), hover_mm=150, tip_mm=105)
        assert abs(t["z"] - (500 - 255)) < 1e-9, t       # moved toward the camera
        assert abs(t["o_z"] - 1.0) < 1e-9, t              # tool axis points at the tag (away from camera)


def test_hover_tilted_tag():
    s = math.sin(math.radians(30)); c = math.cos(math.radians(30))
    t = hover_target(T(100, 0, 400, s, 0, -c), hover_mm=20, tip_mm=105)
    d = math.dist((t["x"], t["y"], t["z"]), (100, 0, 400))
    assert abs(d - 125) < 1e-9
    assert t["o_x"] * s + t["o_z"] * -c < 0  # axis opposite the camera-facing normal


def test_normal_jump():
    assert abs(normal_jump_deg(T(0, 0, 1, 0, 0, 1), T(0, 0, 1, math.sin(0.2), 0, math.cos(0.2))) - math.degrees(0.2)) < 1e-6


def test_lower_along_axis():
    p = lower_along_axis(T(200, 0, 255, 0, 0, -1), 130)  # tool down, descend 130 mm
    assert abs(p["z"] - 125) < 1e-9 and p["x"] == 200


def test_down_target_ignores_tag_normal():
    # arm-frame tag position (previewed with apply:false); the hover is vertical, whatever the normal says
    t = down_target({"x": 210, "y": -5, "z": 3, "o_x": 0.05, "o_y": 0, "o_z": 0.99, "theta": 0}, hover_mm=20, tip_mm=105)
    assert (t["x"], t["y"], t["z"]) == (210, -5, 128) and (t["o_x"], t["o_y"], t["o_z"]) == (0, 0, -1), t


def test_grazing():
    assert not grazing(T(0, 0, 500, 0, 0, 1))  # tag faces the camera: not grazing
    assert grazing(T(0, 0, 500, 1, 0, 0.1))    # tag seen nearly edge-on: grazing


def test_gate_never_yields_flipped_first_detection():
    flip = T(0, 0, 500, 0.5, 0, -0.866)   # PnP flip: 30 deg off, not grazing
    good = T(0, 0, 500, 0, 0, 1)
    g, out = Gate(20), []
    for i, tag in enumerate([flip, good, good, good]):
        if g.update(tag, i * 0.05) is None:
            out.append(tag)
    assert out == [good, good], out  # the flip only seeds a candidate; two agreeing detections pass
    assert g.update(flip, 0.2) == "jump"  # fresh anchor: a later flip is a logged drop
    assert g.update(T(0, 0, 500, 1, 0, 0.1), 0.25) == "grazing"
    # stale anchor: no direct accept, needs an agreeing pair again
    assert g.update(flip, 5.0) == "candidate" and g.update(flip, 5.05) is None


def test_resend_carries_applied_arm_frame_target():
    calls = []

    async def call(pose, frame=None, apply=True):
        calls.append((pose, frame, apply))
        return {"target": pose, "target_raw": pose, "state": "running", "err_mm": 0, "err_deg": 0,
                "reach_deg": 0, "pointing_residual_deg": 0, "clamped": False}

    c = TagServo(SimpleNamespace(rate=50, max_normal_jump_deg=20, gate_stale_s=0.2))
    c.call = call
    applied = T(1, 2, 3, 0, 0, -1)
    assert keepalive_due(None, 0, 9) is False and keepalive_due(applied, 0, 0.05) is False
    assert keepalive_due(applied, 0, 0.1) is True
    asyncio.run(c.send("resend", pose=applied))
    assert calls == [(applied, None, True)], calls
    try:
        asyncio.run(c.send("resend"))
        raise SystemExit("send without a pose must assert")
    except AssertionError:
        pass


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
