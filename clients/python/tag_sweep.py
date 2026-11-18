# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 4: follow an AprilTag that the MuJoCo sim slides around a circle at increasing speeds.

  uv run clients/python/tag_sweep.py --address localhost:8082 --insecure --camera wrist-cam --tracker wrist-tags \\
      --csv .runs/p4-wrist
Spec: docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md. Runbook: dev/so101-mujoco.md.
"""
import math


def total_time(speeds: list, laps: int, radius_mm: float, ramp_s: float) -> float:
    c = 2 * math.pi * radius_mm
    return sum(ramp_s + laps * c / v for v in speeds)


def circle_state(t: float, speeds: list, laps: int, radius_mm: float, ramp_s: float) -> dict:
    """Where the tag is t s after the mover starts. Each speed: a ramp_s linear speed ramp from the previous
    speed (0 for the first), then laps full circles at that speed. Arc length s (mm) is integrated, so
    position and speed are continuous. lap is -1 in a ramp, else the completed laps at this speed."""
    c = 2 * math.pi * radius_mm
    s, prev, t0 = 0.0, 0.0, 0.0
    for i, v in enumerate(speeds):
        if t < t0 + ramp_s:
            d = t - t0
            return {"s": s + prev * d + (v - prev) * d * d / (2 * ramp_s), "speed": prev + (v - prev) * d / ramp_s,
                    "seg": i, "seg_speed": v, "lap": -1, "done": False}
        s += (prev + v) / 2 * ramp_s
        t0 += ramp_s
        hold = laps * c / v
        if t < t0 + hold:
            return {"s": s + v * (t - t0), "speed": v, "seg": i, "seg_speed": v,
                    "lap": int(v * (t - t0) // c), "done": False}
        s += laps * c
        t0 += hold
        prev = v
    return {"s": s, "speed": 0.0, "seg": len(speeds) - 1, "seg_speed": speeds[-1], "lap": laps, "done": True}


def circle_xy(s: float, cx: float, cy: float, radius_mm: float) -> tuple:
    """Arc length s (mm) from angle 0 to a point on the circle, counter-clockwise."""
    a = s / radius_mm
    return cx + radius_mm * math.cos(a), cy + radius_mm * math.sin(a)


def speeds_arg(s: str) -> str:
    """argparse type: a non-empty comma list of speeds, all > 0 (ValueError makes argparse exit with usage)."""
    if not s.strip() or not all(float(v) > 0 for v in s.split(",")):
        raise ValueError("--speeds needs a comma list of values > 0")
    return s


def laps_arg(s: str) -> int:
    if int(s) < 0:
        raise ValueError("--laps must be >= 0")
    return int(s)


def radius_arg(s: str) -> float:
    if not float(s) > 0:
        raise ValueError("--radius-mm must be > 0")
    return float(s)


import asyncio  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_client import POSE_KEYS, pose_dict, run_client  # noqa: E402
from tag_servo import TagLost, TagServo, tag_parser  # noqa: E402

MOVER_HZ = 50
ARM_HZ = 50


class TagSweep(TagServo):
    def __init__(self, args):
        super().__init__(args)
        self.speeds = [float(v) for v in args.speeds.split(",")]
        self.cx, self.cy = (float(v) for v in args.centre.split(","))
        self.state: dict | None = None  # the mover's latest circle_state
        self.origin: tuple | None = None  # arm origin in world, mm
        self.mover_calls, self.mover_t0 = 0, None
        self._tagf = self._armf = None
        self.tag_csv = self.arm_csv = None
        self.world = None

    async def open(self):
        # the robot client only builds clients for APIs imported before it connects
        import viam.components.generic  # noqa: F401

        await super().open()

    async def open_extras(self):
        await super().open_extras()
        from viam.components.generic import Generic
        from viam.proto.common import Pose, PoseInFrame

        a = self.args
        self.world = Generic.from_robot(self.robot2, a.world)
        o = (await self.robot.transform_pose(PoseInFrame(reference_frame=f"{a.follower}_origin", pose=Pose(o_z=1)),
                                             "world")).pose
        if abs(o.o_z - 1) > 1e-6 or abs(o.theta) > 1e-6:
            raise RuntimeError("the arm origin must be unrotated in world (only a translation is supported)")
        self.origin = (o.x, o.y, o.z)
        if a.csv:
            os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
            self._tagf = open(f"{a.csv}-tag.csv", "w", newline="")
            self._armf = open(f"{a.csv}-joints.csv", "w", newline="")
            self.tag_csv, self.arm_csv = csv.writer(self._tagf), csv.writer(self._armf)
            self.tag_csv.writerow(["t", "x", "y", "z", "speed", "seg", "seg_speed", "lap"])

    async def release_extras(self, step):
        for f in (self._tagf, self._armf):
            if f:
                f.close()

    # ---- the tag ------------------------------------------------------------------------------
    async def place(self, x: float, y: float):
        """Put the prop's centre at arm-frame (x, y) mm, resting flat; set_prop_pose wants world metres."""
        ox, oy, _ = self.origin
        await self.world.do_command({"command": "set_prop_pose", "prop": self.args.prop,
                                     "position": [(x + ox) / 1000, (y + oy) / 1000, self.args.prop_z_mm / 1000],
                                     "orientation_wxyz": [1, 0, 0, 0]})

    def face_z(self) -> float:
        """The tag face's z in the arm frame (mm): prop centre + half the 2 mm thickness, minus the origin."""
        return self.args.prop_z_mm + 1.0 - self.origin[2]

    async def mover(self):
        a = self.args
        start, n = time.monotonic(), 0
        self.mover_t0 = self.now()
        try:
            while True:
                st = circle_state(time.monotonic() - start, self.speeds, a.laps, a.radius_mm, a.ramp_s)
                x, y = circle_xy(st["s"], self.cx, self.cy, a.radius_mm)
                await self.place(x, y)
                self.state = st
                self.mover_calls += 1
                if self.tag_csv:
                    self.tag_csv.writerow([f"{self.now():.4f}", f"{x:.3f}", f"{y:.3f}", f"{self.face_z():.3f}",
                                           f"{st['speed']:.3f}", st["seg"], st["seg_speed"], st["lap"]])
                if st["done"]:
                    return
                n += 1
                await asyncio.sleep(max(0.0, start + n / MOVER_HZ - time.monotonic()))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.fault = f"mover: {e}"

    async def arm_logger(self):
        """Joints (deg) and end-effector pose at 50 Hz: the report's jaw-tip path and joint accelerations."""
        start, n, header = time.monotonic(), 0, False
        try:
            while True:
                q = (await self.follower2.get_joint_positions()).values
                t = self.now()  # stamp the joints as they return; jitter here inflates the acceleration estimate
                p = pose_dict(await self.follower2.get_end_position())
                if self.arm_csv:
                    if not header:
                        self.arm_csv.writerow(["t", *[f"q{i}" for i in range(len(q))], *POSE_KEYS])
                        header = True
                    self.arm_csv.writerow([f"{t:.4f}", *q, *[p[k] for k in POSE_KEYS]])
                n += 1
                await asyncio.sleep(max(0.0, start + n / ARM_HZ - time.monotonic()))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.fault = f"arm logger: {e}"

    # ---- run ----------------------------------------------------------------------------------
    async def run(self):
        a = self.args
        await self.place(*circle_xy(0.0, self.cx, self.cy, a.radius_mm))
        await asyncio.sleep(0.5)  # let the prop settle and the trackers see it
        await self.start_tracking()
        self.tasks.append(asyncio.create_task(self.arm_logger()))
        await self.stream("track", self.settle(2, 2, 1, 30, "converge"))
        mv = asyncio.create_task(self.mover())
        self.tasks.append(mv)
        print(f"sweeping {a.speeds} mm/s, {a.laps} laps each, "
              f"{total_time(self.speeds, a.laps, a.radius_mm, a.ramp_s):.0f} s")
        await self.stream("sweep", lambda rep, t: mv.done())
        if self.fault:
            raise RuntimeError(self.fault)

    async def go(self):
        lost = None
        try:
            await self.run()
        except TagLost:
            lost = self.state["seg_speed"] if self.state else None  # None: lost before the sweep started
            print(f"tag lost in phase {self.phase}" + (f" at {lost} mm/s" if lost is not None else ""))
        finally:
            try:
                self.write_summary(lost)
            except Exception as e:  # never mask the run's own error
                print(f"summary: {e}")

    def write_summary(self, lost):
        a = self.args
        if not a.csv:
            return
        dur = self.now() - self.mover_t0 if self.mover_t0 is not None else 0.0
        det_dur = self.now() - self.first_det_t if self.first_det_t is not None else 0.0
        out = {"args": vars(a), "arm_origin_mm": self.origin, "breaking_speed_mm_s": lost,
               "completed": bool(self.state and self.state["done"]), "fault": self.fault, "end_phase": self.phase,
               "mover_hz": self.mover_calls / dur if dur > 0 else None,
               "accepted_det_hz": self.seq / det_dur if det_dur > 0 else None}
        with open(f"{a.csv}-summary.json", "w") as f:
            json.dump(out, f, indent=2)


def main() -> int:
    ap = tag_parser(__doc__)
    ap.set_defaults(address="localhost:8082", follower="arm", camera="wrist-cam", tracker="wrist-tags",
                    axis="down", hover_mm=150, tip_mm=98, joint_deg_s=90, linear_mm_s=120)
    ap.add_argument("--speeds", type=speeds_arg, default="10,20,40,80", help="mm/s, in order")
    ap.add_argument("--laps", type=laps_arg, default=3)
    ap.add_argument("--radius-mm", type=radius_arg, default=60)
    ap.add_argument("--centre", default="220,0", help="x,y mm in the arm frame")
    ap.add_argument("--ramp-s", type=float, default=1.0)
    ap.add_argument("--world", default="sim-world")
    ap.add_argument("--prop", default="tag")
    ap.add_argument("--prop-z-mm", type=float, default=1.0, help="world z of the prop centre")
    return run_client(TagSweep, ap)


if __name__ == "__main__":
    sys.exit(main())
