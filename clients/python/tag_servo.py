# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 3c stage 3: servo the follower SO-101 to an AprilTag seen by a camera (wrist or scene), using
camera-frame servo_targets that the diff-ik module transforms with its measured joints.

Usage (local plaintext listener):
  uv run clients/python/tag_servo.py --address 127.0.0.1:<port> --insecure --camera wrist-cam --tag-id 0 \\
      --hover-mm 150 --csv .runs/3c3-wrist
  # static accuracy, wrist camera: converge at 150 mm, then descend blind to 20 mm
  uv run clients/python/tag_servo.py ... --axis down --hover-mm 150 --final-hover-mm 20
Spec: docs/superpowers/specs/2026-11-16-diff-ik-phase3c3-design.md
"""

import argparse
import asyncio
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_client import POSE_KEYS, ServoClient, base_parser, pose_dict, run_client, lowest_z  # noqa: E402

def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def hover_target(tag: dict, hover_mm: float, tip_mm: float) -> dict:
    """Flange pose (camera frame) hovering hover_mm (jaw tip) above the tag along its normal on the camera's
    side, tool axis pointing at the tag. Robust to the tag's z sign: the normal is flipped toward the camera."""
    p = (tag["x"], tag["y"], tag["z"])
    n = _unit((tag["o_x"], tag["o_y"], tag["o_z"]))
    if sum(a * -b for a, b in zip(n, p)) < 0:  # want n . (camera - tag) > 0, camera at the origin
        n = tuple(-c for c in n)
    d = hover_mm + tip_mm
    return {"x": p[0] + d * n[0], "y": p[1] + d * n[1], "z": p[2] + d * n[2],
            "o_x": -n[0], "o_y": -n[1], "o_z": -n[2], "theta": 0.0}


def normal_jump_deg(a: dict, b: dict) -> float:
    na, nb = _unit((a["o_x"], a["o_y"], a["o_z"])), _unit((b["o_x"], b["o_y"], b["o_z"]))
    dot = abs(sum(x * y for x, y in zip(na, nb)))  # sign-free: the flip handles orientation
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def down_target(tag_arm: dict, hover_mm: float, tip_mm: float) -> dict:
    """--axis down: a vertical hover over the tag's arm-frame position. It ignores the tag's normal, so PnP normal
    noise (about 2 mm per degree at a 125 mm lever) doesn't move the target sideways (static accuracy)."""
    return {"x": tag_arm["x"], "y": tag_arm["y"], "z": tag_arm["z"] + hover_mm + tip_mm,
            "o_x": 0, "o_y": 0, "o_z": -1, "theta": 0}


def grazing(tag: dict, max_incidence_deg: float = 70.0) -> bool:
    """True when the camera sees the tag nearly edge-on (|n . p| < cos 70°): the normal is unreliable there."""
    p, n = _unit((tag["x"], tag["y"], tag["z"])), _unit((tag["o_x"], tag["o_y"], tag["o_z"]))
    return abs(sum(a * b for a, b in zip(n, p))) < math.cos(math.radians(max_incidence_deg))


def lower_along_axis(p: dict, mm: float) -> dict:
    """Move the flange mm along its tool axis (toward what it points at)."""
    a = _unit((p["o_x"], p["o_y"], p["o_z"]))
    return dict(p, x=p["x"] + mm * a[0], y=p["y"] + mm * a[1], z=p["z"] + mm * a[2])


class Gate:
    """Detection gate. Grazing views are dropped (< 20 degrees above the tag plane: the normal is unreliable).
    With a fresh anchor (accepted < stale_s ago) a detection passes if its normal is within max_jump_deg of it;
    3 consecutive jump rejections drop the anchor. With no anchor, a detection only seeds a candidate and
    is accepted when the next one agrees with it: a flipped PnP solution is never applied on its own.
    update returns None (accepted), "grazing", "jump" (both logged as drops) or "candidate" (not logged)."""

    def __init__(self, max_jump_deg: float, stale_s: float = 0.2, cand_s: float = 0.5):
        self.max_jump_deg, self.stale_s, self.cand_s = max_jump_deg, stale_s, cand_s
        self.anchor: dict | None = None
        self.anchor_t = 0.0
        self.cand: dict | None = None
        self.cand_t = 0.0
        self.rejects = 0

    def update(self, tag: dict, t: float) -> str | None:
        if grazing(tag):
            return "grazing"
        if self.anchor is not None and (t - self.anchor_t > self.stale_s or self.rejects >= 3):
            self.anchor = None
        if self.anchor is not None:
            if normal_jump_deg(tag, self.anchor) > self.max_jump_deg:
                self.rejects += 1
                return "jump"
        elif not (self.cand is not None and t - self.cand_t <= self.cand_s
                  and normal_jump_deg(tag, self.cand) <= self.max_jump_deg):
            self.cand, self.cand_t, self.rejects = tag, t, 0
            return "candidate"
        self.anchor, self.anchor_t, self.cand, self.rejects = tag, t, None, 0
        return None


def keepalive_due(applied, last_send: float, now: float, period: float = 0.1) -> bool:
    """Re-send the last APPLIED arm-frame target when nothing was sent for period s."""
    return applied is not None and now - last_send >= period


class TagLost(Exception):
    pass


def _cells(p) -> list:
    return [p[k] for k in POSE_KEYS] if p else [""] * len(POSE_KEYS)


class TagServo(ServoClient):
    def __init__(self, args):
        super().__init__(args)
        self.tracker = self.det_task = None
        self.det: tuple[float, dict] | None = None  # (t, pose) of the latest accepted detection
        self.seq = 0  # bumped per accepted detection
        self.sent_seq = -1
        self.gate = Gate(args.max_normal_jump_deg, args.gate_stale_s)
        self.first_det_t: float | None = None
        self.phase = "engage"  # drops before the session starts are logged as engage
        self.applied: dict | None = None  # arm-frame target of the last APPLIED reply (never a preview's)
        self.last_send = 0.0

    # ---- setup -------------------------------------------------------------------------------
    async def open(self):
        # the robot client only builds clients for APIs imported before it connects
        import viam.components.pose_tracker  # noqa: F401

        await super().open()

    async def open_extras(self):
        if not self.args.fake_tag_base:
            from viam.components.pose_tracker import PoseTracker

            self.tracker = PoseTracker.from_robot(self.robot2, self.args.tracker)

    def csv_columns(self):
        # drop rows: kind "drop", state = the reason (jump|grazing), tag_* = the rejected pose
        return ["t", "kind", "phase", *[f"tag_{k}" for k in POSE_KEYS],
                *[f"target_{k}" for k in POSE_KEYS], *[f"raw_{k}" for k in POSE_KEYS],
                *[f"tagarm_{k}" for k in POSE_KEYS], "state", "err_mm", "err_deg", "reach_deg", "residual_deg",
                "clamped", "rtt_ms"]

    def target_filter(self) -> dict:
        a = self.args
        return {"ema_alpha": a.ema_alpha, "hold_mm": a.hold_mm, "hold_deg": a.hold_deg, "dwell_ms": a.dwell_ms,
                "floor": {"min_z_mm": a.min_z_mm, "tip_mm": a.tip_mm, "margin_mm": 15},
                "feedforward": a.feedforward == "on"}

    # ---- detections ---------------------------------------------------------------------------
    def drop(self, tag: dict, reason: str):
        if self.ticks_csv:
            self.ticks_csv.writerow([f"{self.now():.4f}", "drop", self.phase, *_cells(tag), *[""] * (3 * len(POSE_KEYS)), reason,
                                     "", "", "", "", "", ""])

    def accept(self, tag: dict):
        reason = self.gate.update(tag, self.now())
        if reason is None:
            self.det = (self.now(), tag)
            self.seq += 1
            if self.first_det_t is None:
                self.first_det_t = self.now()
        elif reason != "candidate":
            self.drop(tag, reason)

    async def detect(self):
        a = self.args
        try:
            if a.fake_tag_base:
                await self.detect_fake()
                return
            err = None
            while True:
                try:
                    poses = await self.tracker.get_poses(body_names=[str(a.tag_id)])
                except Exception as e:
                    if str(e) != err:  # print a repeating error once
                        print(f"tracker: {e}")
                        err = str(e)
                    await asyncio.sleep(0.05)
                    continue
                pif = poses.get(str(a.tag_id))
                if pif is None:
                    await asyncio.sleep(0.01)
                    continue
                if pif.reference_frame != a.camera:
                    self.fault = f"tracker frame {pif.reference_frame!r} != --camera {a.camera!r}"
                    return
                self.accept(pose_dict(pif.pose))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.fault = f"detect: {e}"

    async def detect_fake(self):
        """Rehearsal: a fixed base-frame tag (normal up) seen from the camera, at 20 Hz."""
        from viam.proto.common import Pose, PoseInFrame

        a = self.args
        x, y, z = (float(v) for v in a.fake_tag_base.split(","))
        q = PoseInFrame(reference_frame=f"{a.follower}_origin", pose=Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=1, theta=0))
        while True:
            if not (a.fake_tag_drop_after and self.first_det_t is not None
                    and self.now() - self.first_det_t > a.fake_tag_drop_after):
                self.accept(pose_dict((await self.robot.transform_pose(q, a.camera)).pose))
            await asyncio.sleep(0.05)

    # ---- sending ------------------------------------------------------------------------------
    async def call(self, pose: dict, frame: str | None = None, apply: bool = True) -> dict:
        cmd = {"command": "servo_target", "session_id": self.sid, "pose": pose}
        if frame:
            cmd["frame"] = frame
        if not apply:
            cmd["apply"] = False
        return await asyncio.wait_for(self.svc.do_command(cmd), 0.5)

    async def send(self, kind: str, tag: dict | None = None, pose: dict | None = None) -> dict:
        """det: the tag becomes a target (camera frame); resend/frozen: an arm-frame pose, no frame."""
        a = self.args
        assert (tag if kind == "det" else pose) is not None, f"send({kind}) needs a pose"
        t0 = time.monotonic()
        tagarm = None
        if kind == "det" and a.axis == "normal":
            rep = await self.call(hover_target(tag, a.hover_mm, a.tip_mm), a.camera)
        elif kind == "det":
            # the tag's own arm-frame pose, previewed without applying; the hover is built from its position only
            pre = await self.call(tag, a.camera, apply=False)
            tagarm = pre["target_raw"]
            rep = await self.call(down_target(pre["target_raw"], a.hover_mm, a.tip_mm))
        else:
            rep = await self.call(pose)
        rtt = (time.monotonic() - t0) * 1000
        self.last_send = t0
        self.applied = rep["target"]  # only applied replies reach here: previews are consumed above
        if self.ticks_csv:
            self.ticks_csv.writerow([f"{self.now():.4f}", kind, self.phase, *_cells(tag), *_cells(rep["target"]),
                                     *_cells(rep["target_raw"]), *_cells(tagarm), rep["state"], rep["err_mm"], rep["err_deg"],
                                     rep["reach_deg"], rep["pointing_residual_deg"], int(rep["clamped"]), f"{rtt:.2f}"])
        # live states: idle, running, converged, stalled; ended ones raise
        if rep["state"] in ("stopped", "stopped_timeout", "faulted"):
            raise RuntimeError(f"servo session ended: {rep['state']}")
        return rep

    async def stream(self, phase: str, until, frozen: dict | None = None) -> None:
        """Send at --rate until until(reply, t) or a fault. Tracking: one target per accepted detection,
        a re-send of the last applied target after 100 ms without one, "tag lost" after 0.5 s. With frozen:
        that arm-frame pose every 100 ms, no detections and no loss timer (the blind descent)."""
        self.phase = phase
        n, start = 0, time.monotonic()
        self.sent_seq = -1  # a new session gets the latest detection at once
        while self.fault is None:
            now = self.now()
            rep = None
            if frozen is not None:
                if time.monotonic() - self.last_send >= 0.1:
                    rep = await self.send("frozen", pose=frozen)
            else:
                if self.det is not None and now - self.det[0] > 0.5:
                    raise TagLost()
                if self.det is not None and self.seq != self.sent_seq:
                    self.sent_seq = self.seq
                    rep = await self.send("det", tag=self.det[1])
                elif keepalive_due(self.applied, self.last_send, time.monotonic()):
                    rep = await self.send("resend", pose=self.applied)
            if rep is not None and until(rep, self.now()):
                return
            n += 1
            if time.monotonic() - (start + n * self.period) > self.period:
                start, n = time.monotonic(), 0  # behind after a stall: re-anchor, don't burst
            await asyncio.sleep(max(0.0, start + n * self.period - time.monotonic()))
        raise RuntimeError(self.fault)

    def settle(self, mm: float, deg: float, dur: float, timeout: float, what: str):
        """until-callable: err_mm <= mm and reach_deg <= deg for dur s (not err_deg: on 5 DOF a tilted
        tag's normal can be out of reach). Smooth gains droop more, so --tol-* widen the thresholds."""
        a = self.args
        mm, deg = max(mm, a.tol_mm or 0), max(deg, a.tol_deg or 0)
        since: list[float | None] = [None]
        deadline = self.now() + timeout

        def until(rep, t):
            if rep["state"] != "idle" and rep["err_mm"] <= mm and rep["reach_deg"] <= deg:
                since[0] = since[0] if since[0] is not None else t
                return t - since[0] >= dur
            since[0] = None
            if t > deadline:
                raise RuntimeError(f"{what} did not reach {mm:.0f} mm / {deg:.0f} deg in {timeout:.0f} s")
            return False

        return until

    # ---- run ----------------------------------------------------------------------------------
    async def go(self):
        try:
            await self.run()
        except TagLost:
            print("tag lost: stopped")  # exit 0; close() stops the session

    async def start_tracking(self):
        """Floor check, the detection and watchdog tasks, the first tag, the half-limit engage session until
        settled, then the full-limit track session. Keep the tag still until this returns."""
        a = self.args
        fp = pose_dict(await self.follower2.get_end_position())  # the watchdog's first read may lag
        if lowest_z(fp, a.tip_mm) < a.min_z_mm:
            raise RuntimeError(f"follower is below the floor (lowest z {lowest_z(fp, a.tip_mm):.1f} mm); "
                               "park it at q_start first (runbook)")
        self.det_task = asyncio.create_task(self.detect())
        self.tasks = [asyncio.create_task(self.watchdog()), self.det_task]
        # the dead-man starts at servo_start and only applied targets refresh it: see the tag first
        t_end = time.monotonic() + 10
        while self.det is None and self.fault is None:
            if time.monotonic() > t_end:
                raise RuntimeError("no tag seen in 10 s")
            await asyncio.sleep(0.02)
        if self.fault:
            raise RuntimeError(self.fault)
        kw = {"frames": True, "target_filter": self.target_filter()}

        print("engaging at half the L2 limits…")
        await self.start_session("pointing", half=True, **kw)
        await self.stream("engage", self.settle(10, 5, 0.2, 30, "engage"))
        await self.stop_session()
        print("tracking. Ctrl-C to stop.")
        await self.start_session("pointing", half=False, **kw)

    async def run(self):
        a = self.args
        if a.final_hover_mm is not None and a.final_hover_mm >= a.hover_mm:
            raise RuntimeError("--final-hover-mm must be below --hover-mm")
        await self.start_tracking()
        end = math.inf
        if a.final_hover_mm is None:
            if a.duration > 0:
                end = self.now() + a.duration
            await self.stream("track", lambda rep, t: t >= end)
            return
        await self.stream("track", self.settle(2, 2, 1, 60, "converge"))
        self.det_task.cancel()  # the descent deliberately stops using detections
        frozen = lower_along_axis(self.applied, a.hover_mm - a.final_hover_mm)
        print(f"converged at {a.hover_mm:.0f} mm; descending to {a.final_hover_mm:.0f} mm")
        await self.stream("descent", self.settle(2, 2, 1, 30, "descent"), frozen=frozen)
        print("measure now")
        if a.duration > 0:
            end = self.now() + a.duration
        await self.stream("hold", lambda rep, t: t >= end, frozen=frozen)


def tag_parser(doc: str) -> argparse.ArgumentParser:
    ap = base_parser(doc)
    ap.set_defaults(min_z_mm=-10)
    ap.add_argument("--camera", default="wrist-cam")
    ap.add_argument("--tracker", default="tag-tracker")
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--hover-mm", type=float, default=150)
    ap.add_argument("--final-hover-mm", type=float, default=None, help="look-then-move: descend blind to this")
    ap.add_argument("--axis", choices=("normal", "down"), default="normal")
    ap.add_argument("--ema-alpha", type=float, default=0.3)
    ap.add_argument("--hold-mm", type=float, default=1.0)
    ap.add_argument("--hold-deg", type=float, default=0.5)
    ap.add_argument("--dwell-ms", type=float, default=200)
    ap.add_argument("--feedforward", choices=("on", "off"), default="on")
    ap.add_argument("--max-normal-jump-deg", type=float, default=20)
    ap.add_argument("--gate-stale-s", type=float, default=0.2)
    ap.add_argument("--fake-tag-base", default=None, help=argparse.SUPPRESS)  # rehearsal: x,y,z mm in <follower>_origin
    ap.add_argument("--fake-tag-drop-after", type=float, default=0, help=argparse.SUPPRESS)  # rehearsal: tag loss
    return ap


def main() -> int:
    return run_client(TagServo, tag_parser(__doc__))


if __name__ == "__main__":
    sys.exit(main())
