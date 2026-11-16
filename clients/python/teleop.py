# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 3c stage 2: teleoperate a follower SO-101 from a leader SO-101 through the diff-ik servo.

Usage (laptop-main, local plaintext listener):
  uv run clients/python/teleop.py --address 127.0.0.1:<port> --insecure --dry-run
  uv run clients/python/teleop.py --address 127.0.0.1:<port> --insecure --csv .runs/3c2-tuned
Spec: docs/superpowers/specs/2026-11-15-diff-ik-phase3c2-design.md
"""

import math

POSE_KEYS = ("x", "y", "z", "o_x", "o_y", "o_z", "theta")


def pose_dict(p) -> dict:
    """A viam Pose (or dict) as the servo's pose map."""
    if isinstance(p, dict):
        return {k: float(p[k]) for k in POSE_KEYS}
    return {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}


def tool_axis(p: dict) -> tuple[float, float, float]:
    """Viam's orientation vector is the tool z axis in the base frame."""
    n = math.sqrt(p["o_x"] ** 2 + p["o_y"] ** 2 + p["o_z"] ** 2)
    return p["o_x"] / n, p["o_y"] / n, p["o_z"] / n


def pose_delta(a: dict, b: dict) -> tuple[float, float]:
    """(position distance mm, orientation distance deg): the larger of the tool-axis angle and
    the wrapped theta difference."""
    mm = math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))
    dot = sum(x * y for x, y in zip(tool_axis(a), tool_axis(b)))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    dth = abs((a["theta"] - b["theta"] + 180.0) % 360.0 - 180.0)
    return mm, max(ang, dth)


def lowest_z(p: dict, tip_mm: float) -> float:
    """Lowest of the end effector and the jaw tip (end effector + tip_mm along the tool axis)."""
    return min(p["z"], p["z"] + tip_mm * tool_axis(p)[2])


def floor_lift(p: dict, min_z: float, tip_mm: float, margin: float = 15.0) -> float:
    """How far to raise p so the end effector and jaw tip clear min_z + margin (0 if they do).
    The margin stands in for the CLI guard's ±15 mm jaw corners."""
    return max(0.0, min_z + margin - lowest_z(p, tip_mm))


class Hold:
    """Hold with hysteresis. Send the raw leader pose while it moves; once it stays within
    band_mm/band_deg of an anchor for dwell_s, resend the anchor unchanged until it leaves the band.
    A one-count encoder flicker would otherwise read as target motion and keep the servo from
    converging; a plain deadband would turn slow sweeps into steps (spec, Tracking)."""

    def __init__(self, band_mm: float = 1.0, band_deg: float = 0.5, dwell_s: float = 0.2):
        self.band_mm, self.band_deg, self.dwell_s = band_mm, band_deg, dwell_s
        self.anchor: dict | None = None
        self.since = 0.0
        self.holding = False

    def update(self, pose: dict, t: float) -> dict:
        if self.anchor is None:
            self.anchor, self.since = pose, t
            return pose
        mm, deg = pose_delta(pose, self.anchor)
        if mm > self.band_mm or deg > self.band_deg:
            self.anchor, self.since, self.holding = pose, t, False
            return pose
        if not self.holding and t - self.since >= self.dwell_s:
            # hold the LATEST pose: resending the older anchor would step the target backwards
            self.anchor, self.holding = pose, True
        return self.anchor if self.holding else pose


class Runaway:
    """True once err_mm has stayed above limit_mm for dur_s (tracking session only)."""

    def __init__(self, limit_mm: float = 250.0, dur_s: float = 1.0):
        self.limit_mm, self.dur_s = limit_mm, dur_s
        self.since: float | None = None

    def update(self, err_mm: float, t: float) -> bool:
        if err_mm <= self.limit_mm:
            self.since = None
            return False
        if self.since is None:
            self.since = t
        return t - self.since >= self.dur_s

import argparse
import asyncio
import csv
import os
import sys
import time


def level_limits(args, half: bool = False) -> dict:
    k = 0.5 if half else 1.0
    lim = {"joint_deg_s": args.joint_deg_s * k, "linear_mm_s": args.linear_mm_s * k,
           "angular_deg_s": args.angular_deg_s * k, "resync_deg": args.resync_deg,
           "singular_eps": args.singular_eps, "max_damping": args.max_damping}
    if args.tol_mm:
        lim["tol_mm"] = args.tol_mm
    if args.tol_deg:
        lim["tol_deg"] = args.tol_deg
    return lim


async def connect(args):
    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions

    if args.insecure:
        dial = DialOptions(insecure=True, disable_webrtc=True)
    else:
        dial = DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    return await RobotClient.at_address(args.address, RobotClient.Options(dial_options=dial))


class Teleop:
    def __init__(self, args):
        self.args = args
        self.period = 1.0 / args.rate
        self.fault: str | None = None  # set by the watchdog or the runaway check; ends the session
        self.sid: str | None = None
        self.t0 = time.monotonic()
        self.ticks_csv = self.follow_csv = None
        # set before anything can fail, so close() never trips on a missing attribute
        self.robot = self.robot2 = self.leader = self.lgrip = self.fgrip = None
        self._tf = self._ff = None
        self.tasks: list = []
        self.released = False

    def now(self) -> float:
        return time.monotonic() - self.t0

    @staticmethod
    async def ask(prompt: str) -> str:
        """input() on a daemon thread: Ctrl-C at the prompt exits at once instead of hanging on the
        executor thread (asyncio.to_thread) until Enter."""
        import threading

        loop = asyncio.get_running_loop()
        fut = loop.create_future()

        def read():
            try:
                v = input(prompt)
            except BaseException as e:  # EOF etc.
                loop.call_soon_threadsafe(lambda: fut.done() or fut.set_exception(e))
                return
            loop.call_soon_threadsafe(lambda: fut.done() or fut.set_result(v))

        threading.Thread(target=read, daemon=True).start()
        return await fut

    # ---- setup -------------------------------------------------------------------------------
    async def open(self):
        from viam.components.arm import Arm
        from viam.components.gripper import Gripper
        from viam.services.motion import MotionClient

        a = self.args
        self.robot, self.robot2 = await connect(a), await connect(a)
        self.svc = MotionClient.from_robot(self.robot, a.service)
        self.leader = Arm.from_robot(self.robot, a.leader)
        self.follower2 = Arm.from_robot(self.robot2, a.follower)
        self.svc2 = MotionClient.from_robot(self.robot2, a.service)
        if a.leader != a.follower and not a.no_release:
            # needed on exit to free the leader trigger after re-torque, mirroring or not
            self.lgrip = Gripper.from_robot(self.robot2, a.leader_gripper)
        if not a.no_gripper:
            self.fgrip = Gripper.from_robot(self.robot2, a.follower_gripper)
        if a.csv:
            os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
            self._tf = open(f"{a.csv}-ticks.csv", "w", newline="")
            self._ff = open(f"{a.csv}-follower.csv", "w", newline="")
            self.ticks_csv, self.follow_csv = csv.writer(self._tf), csv.writer(self._ff)
            self.ticks_csv.writerow(["t", "phase", *[f"l_{k}" for k in POSE_KEYS], *[f"s_{k}" for k in POSE_KEYS],
                                     "held", "clamped", "state", "err_mm", "err_deg", "tick_ms"])
            self.follow_csv.writerow(["t", "kind", *[f"f_{k}" for k in POSE_KEYS],
                                      "rate_hz", "tick_p99_ms", "overruns", "resyncs"])

    async def leader_pose(self) -> dict:
        return pose_dict(await self.leader.get_end_position())

    async def dry_run(self):
        reads = []
        end = time.monotonic() + 10
        while time.monotonic() < end:
            t = time.monotonic()
            await self.leader.get_end_position()
            reads.append(time.monotonic() - t)
            await asyncio.sleep(max(0.0, self.period - (time.monotonic() - t)))
        reads.sort()
        p50, p99 = reads[len(reads) // 2] * 1000, reads[int(len(reads) * 0.99)] * 1000
        budget = self.period * 1000
        print(f"leader read: n {len(reads)}  p50 {p50:.1f} ms  p99 {p99:.1f} ms; "
              f"read + ~1 ms servo_target vs {budget:.0f} ms period: {'fits' if p99 + 1 < budget else 'DOES NOT FIT'}")

    # ---- the two sessions ---------------------------------------------------------------------
    async def start_session(self, half: bool) -> dict:
        a = self.args
        r = await self.svc.do_command({
            "command": "servo_start", "component_name": a.follower, "frame": f"{a.follower}_origin",
            "mode": "pose", "task": "pose", "rate_hz": a.rate, "timeout_ms": a.timeout_ms,
            "limits": level_limits(a, half=half)})
        self.sid = r["session_id"]
        return r["pose"]

    async def stop_session(self):
        sid, self.sid = self.sid, None
        if sid:
            try:
                await self.svc.do_command({"command": "servo_stop", "session_id": sid})
            except Exception as e:  # an ended session (dead-man, fault) errors: log and carry on
                print(f"servo_stop: {e}")

    async def stream(self, phase: str, until) -> None:
        """Stream the (held, clamped) leader pose at --rate until until(reply, t) is true or a fault."""
        a = self.args
        hold, runaway = Hold(), Runaway()
        n, start = 0, time.monotonic()
        while self.fault is None:
            tick = time.monotonic()
            lp = await asyncio.wait_for(self.leader_pose(), 0.5)  # a hung RPC is an error, not a limp wait
            sp = dict(hold.update(lp, self.now()))
            lift = floor_lift(sp, a.min_z_mm, a.tip_mm)
            sp["z"] += lift
            rep = await asyncio.wait_for(
                self.svc.do_command({"command": "servo_target", "session_id": self.sid, "pose": sp}), 0.5)
            t = self.now()
            if self.ticks_csv:
                self.ticks_csv.writerow([f"{t:.4f}", phase, *[lp[k] for k in POSE_KEYS], *[sp[k] for k in POSE_KEYS],
                                         int(hold.holding), int(lift > 0), rep["state"],
                                         rep["err_mm"], rep["err_deg"], f"{(time.monotonic() - tick) * 1000:.2f}"])
            # live states: idle (no tick yet), running, converged, stalled; ended ones raise
            if rep["state"] in ("stopped", "stopped_timeout", "faulted"):
                raise RuntimeError(f"servo session ended: {rep['state']}")
            if phase == "track" and runaway.update(rep["err_mm"], t):
                raise RuntimeError(f"runaway: err {rep['err_mm']:.0f} mm for {runaway.dur_s} s")
            if until(rep, t):
                return
            n += 1
            if time.monotonic() - (start + n * self.period) > self.period:
                start, n = time.monotonic(), 0  # behind after a stall: re-anchor, don't burst to catch up
            await asyncio.sleep(max(0.0, start + n * self.period - time.monotonic()))
        raise RuntimeError(self.fault)

    # ---- side tasks ---------------------------------------------------------------------------
    async def watchdog(self):
        """Follower pose at 10 Hz (floor guard + lag data) and servo_status at 5 Hz. Any failure here
        is a fault: losing the floor guard silently is not allowed."""
        a = self.args
        k = 0
        try:
            while True:
                fp = pose_dict(await self.follower2.get_end_position())
                tp = self.now()  # stamp the pose read, not the later status RPC
                row = [f"{tp:.4f}", "pose", *[fp[q] for q in POSE_KEYS], "", "", "", ""]
                if lowest_z(fp, a.tip_mm) < a.min_z_mm:
                    self.fault = f"follower below the floor: lowest z {lowest_z(fp, a.tip_mm):.1f} mm"
                if k % 2 == 0 and self.sid:
                    try:
                        st = await self.svc2.do_command({"command": "servo_status", "session_id": self.sid})
                        row = [f"{tp:.4f}", "status", *[fp[q] for q in POSE_KEYS],
                               st.get("rate_hz"), st.get("tick_p99_ms"), st.get("overruns"), st.get("resyncs")]
                    except Exception:
                        pass  # the session may be between engage and track
                if self.follow_csv:
                    self.follow_csv.writerow(row)
                if a.fault_after and self.now() > a.fault_after:
                    self.fault = "injected fault (--fault-after)"
                k += 1
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.fault = f"watchdog: {e}"

    async def gripper(self):
        last, last_err = None, None
        while True:
            try:
                pct = (await self.lgrip.do_command({"command": "get_position"}))["position_percentage"]
                if last is None or abs(pct - last) >= 1.0:
                    await self.fgrip.do_command({"command": "set_position", "percentage": pct, "wait": False})
                    last = pct
            except Exception as e:
                if str(e) != last_err:  # print a repeating error once
                    print(f"gripper: {e}")
                    last_err = str(e)
            await asyncio.sleep(1 / 15)

    # ---- run and exit -------------------------------------------------------------------------
    async def run(self):
        a = self.args
        release = not a.no_release and a.leader != a.follower
        await self.ask("Hold the leader arm (or rest it on a support), then press Enter. ")
        if release:
            self.released = True  # set first: a failure mid-call must still re-torque on exit
            await self.leader.do_command({"command": "set_torque", "enable": False})
        while True:
            lp = await self.leader_pose()
            lift = floor_lift(lp, a.min_z_mm, a.tip_mm)
            if lift == 0:
                break
            await self.ask(f"Leader is {lift:.0f} mm too low; lift it, then press Enter. ")
        fp = pose_dict(await self.follower2.get_end_position())  # the watchdog's first read may lag
        if lowest_z(fp, a.tip_mm) < a.min_z_mm:
            raise RuntimeError(f"follower is below the floor (lowest z {lowest_z(fp, a.tip_mm):.1f} mm); "
                               "park it at q_start first (runbook)")
        self.tasks = [asyncio.create_task(self.watchdog())]
        if self.lgrip and self.fgrip:
            self.tasks.append(asyncio.create_task(self.gripper()))

        print("engaging at half the L2 limits…")
        await self.start_session(half=True)
        close_since: list[float | None] = [None]
        deadline = self.now() + 30

        def engaged(rep, t):
            # close enough, not converged: a hand-held leader is never still for long (hardware run 1).
            # The tracking session closes the rest of the gap at full limits.
            near_mm, near_deg = max(10.0, a.tol_mm or 0), max(5.0, a.tol_deg or 0)  # smooth gains droop more
            if rep["state"] != "idle" and rep["err_mm"] <= near_mm and rep["err_deg"] <= near_deg:
                close_since[0] = close_since[0] if close_since[0] is not None else t
                return t - close_since[0] >= 0.2
            close_since[0] = None
            if t > deadline:
                raise RuntimeError("engage did not get within 10 mm / 5° of the leader in 30 s")
            return False

        await self.stream("engage", engaged)
        await self.stop_session()
        print("tracking. Ctrl-C to stop.")
        await self.start_session(half=False)
        end = self.now() + a.duration if a.duration > 0 else math.inf
        await self.stream("track", lambda rep, t: t >= end)

    async def close(self):
        """Ordered, independent exit steps (spec, Exit): one failing or hanging never skips the rest.
        Each step has a 3 s timeout, and a second Ctrl-C is ignored until the leader is re-torqued."""
        import signal

        print("stopping, wait…")
        prev = signal.signal(signal.SIGINT, signal.SIG_IGN)
        a = self.args

        async def step(name, make):
            try:
                await asyncio.wait_for(make(), 3)
                return True
            except Exception as e:
                print(f"exit: {name}: {e}")
                return False

        try:
            await step("servo_stop", self.stop_session)
            for t in self.tasks:
                t.cancel()
            if self.released:
                ok = await step("leader torque on",
                                lambda: self.leader.do_command({"command": "set_torque", "enable": True}))
                if not ok and self.robot2:
                    from viam.components.arm import Arm

                    await step("leader torque on (2nd connection)", lambda: Arm.from_robot(
                        self.robot2, a.leader).do_command({"command": "set_torque", "enable": True}))
                if self.lgrip:
                    await step("leader gripper free",
                               lambda: self.lgrip.do_command({"command": "set_torque", "enable": False}))
            for f in (self._tf, self._ff):
                if f:
                    f.close()
            for r in (self.robot, self.robot2):
                if r:
                    await step("close", r.close)
        finally:
            signal.signal(signal.SIGINT, prev)


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8080")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--leader", default="leader-arm")
    ap.add_argument("--follower", default="follower-arm")
    ap.add_argument("--leader-gripper", default="leader-gripper")
    ap.add_argument("--follower-gripper", default="follower-gripper")
    ap.add_argument("--no-gripper", action="store_true")
    ap.add_argument("--no-release", action="store_true", help="don't release the leader's torque (rehearsal)")
    ap.add_argument("--joint-deg-s", type=float, default=60)
    ap.add_argument("--linear-mm-s", type=float, default=70)
    ap.add_argument("--angular-deg-s", type=float, default=30)
    ap.add_argument("--resync-deg", type=float, default=9.6)
    ap.add_argument("--singular-eps", type=float, default=0.03)
    ap.add_argument("--max-damping", type=float, default=0.05)
    ap.add_argument("--tol-mm", type=float, default=None)
    ap.add_argument("--tol-deg", type=float, default=None)
    ap.add_argument("--rate", type=float, default=50)
    ap.add_argument("--timeout-ms", type=float, default=300)
    ap.add_argument("--min-z-mm", type=float, default=30)
    ap.add_argument("--tip-mm", type=float, default=105)
    ap.add_argument("--duration", type=float, default=0, help="seconds of tracking; 0 = until Ctrl-C")
    ap.add_argument("--csv", default=None, help="prefix: writes PREFIX-ticks.csv and PREFIX-follower.csv")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--fault-after", type=float, default=0, help=argparse.SUPPRESS)  # rehearsal: inject a fault
    args = ap.parse_args()
    if args.leader == args.follower:
        args.no_gripper = True  # rehearsal: the follower tracks itself

    tp = Teleop(args)
    # SIGTERM (a plain kill, a closing terminal) takes the same exit path as Ctrl-C: cancel main,
    # so close() stops the session and re-torques the leader.
    import signal

    asyncio.get_running_loop().add_signal_handler(signal.SIGTERM, asyncio.current_task().cancel)
    try:
        await tp.open()
        if args.dry_run:
            await tp.dry_run()
            return 0
        await tp.run()
        return 0
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("stopping")
        return 0
    except Exception as e:
        print(f"error: {e}")
        return 1
    finally:
        await tp.close()


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(130)
