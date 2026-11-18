# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Shared plumbing for servo-session clients (teleop.py, tag_servo.py): connections, the servo
session start/stop, the follower watchdog, ordered exit, and the main scaffolding. Every viam import
is inside a function, so the pure-helper tests run without viam-sdk."""

import argparse
import asyncio
import csv
import math
import os
import time

POSE_KEYS = ("x", "y", "z", "o_x", "o_y", "o_z", "theta")
FOLLOW_COLS = ["t", "kind", *[f"f_{k}" for k in POSE_KEYS], "rate_hz", "tick_p99_ms", "overruns", "resyncs", "lag_ms", "peak_accel_ratio"]


def follow_row(t: float, fp: dict, st: dict | None = None) -> list:
    """One follower-CSV row: a "pose" row, or a "status" row from a servo_status reply st."""
    if st is None:
        return [f"{t:.4f}", "pose", *[fp[q] for q in POSE_KEYS], "", "", "", "", "", ""]
    return [f"{t:.4f}", "status", *[fp[q] for q in POSE_KEYS], st.get("rate_hz"), st.get("tick_p99_ms"),
            st.get("overruns"), st.get("resyncs"), st.get("lag_ms"), st.get("peak_accel_ratio")]


def pose_dict(p) -> dict:
    """A viam Pose (or dict) as the servo's pose map."""
    if isinstance(p, dict):
        return {k: float(p[k]) for k in POSE_KEYS}
    return {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}


def tool_axis(p: dict) -> tuple[float, float, float]:
    """Viam's orientation vector is the tool z axis in the base frame."""
    n = math.sqrt(p["o_x"] ** 2 + p["o_y"] ** 2 + p["o_z"] ** 2)
    return p["o_x"] / n, p["o_y"] / n, p["o_z"] / n


def lowest_z(p: dict, tip_mm: float) -> float:
    """Lowest of the end effector and the jaw tip (end effector + tip_mm along the tool axis)."""
    return min(p["z"], p["z"] + tip_mm * tool_axis(p)[2])


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



def base_parser(doc: str) -> argparse.ArgumentParser:
    """Every flag ServoClient reads; subclasses add their own."""
    ap = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8080")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--follower", default="follower-arm")
    ap.add_argument("--joint-deg-s", type=float, default=60)
    ap.add_argument("--solver", choices=["dls", "qp"], default=None, help="servo_start solver (default: the server's, dls)")
    ap.add_argument("--collision", action="store_true", help="servo_start collision: true (needs --solver qp; frame-system obstacles)")
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
    ap.add_argument("--fault-after", type=float, default=0, help=argparse.SUPPRESS)  # rehearsal: inject a fault
    return ap


class ServoClient:
    """Subclasses define: csv_columns(), open_extras(), run(), release_extras(); optionally go()."""

    def __init__(self, args):
        self.args = args
        self.period = 1.0 / args.rate
        self.fault: str | None = None  # set by the watchdog or a subclass check; ends the session
        self.sid: str | None = None
        self.t0 = time.monotonic()
        self.ticks_csv = self.follow_csv = None
        # set before anything can fail, so close() never trips on a missing attribute
        self.robot = self.robot2 = None
        self._tf = self._ff = None
        self.tasks: list = []

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
                loop.call_soon_threadsafe(lambda e=e: fut.done() or fut.set_exception(e))
                return
            loop.call_soon_threadsafe(lambda: fut.done() or fut.set_result(v))

        threading.Thread(target=read, daemon=True).start()
        return await fut

    # ---- setup -------------------------------------------------------------------------------
    def csv_columns(self) -> list:
        """The ticks CSV header (the follower CSV's is FOLLOW_COLS)."""
        raise NotImplementedError

    async def open_extras(self):
        """Subclass handles (leader arm, grippers, trackers)."""

    async def open(self):
        from viam.components.arm import Arm
        from viam.services.motion import MotionClient

        a = self.args
        self.robot, self.robot2 = await connect(a), await connect(a)
        self.svc = MotionClient.from_robot(self.robot, a.service)
        self.follower2 = Arm.from_robot(self.robot2, a.follower)
        self.svc2 = MotionClient.from_robot(self.robot2, a.service)
        await self.open_extras()
        if a.csv:
            os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
            self._tf = open(f"{a.csv}-ticks.csv", "w", newline="")
            self._ff = open(f"{a.csv}-follower.csv", "w", newline="")
            self.ticks_csv, self.follow_csv = csv.writer(self._tf), csv.writer(self._ff)
            self.ticks_csv.writerow(self.csv_columns())
            self.follow_csv.writerow(FOLLOW_COLS)

    # ---- the session --------------------------------------------------------------------------
    async def start_session(self, task: str, half: bool, **extra) -> dict:
        """extra keys (frames, target_filter, ...) are merged into servo_start."""
        a = self.args
        r = await self.svc.do_command({
            "command": "servo_start", "component_name": a.follower, "frame": f"{a.follower}_origin",
            "mode": "pose", "task": task, "rate_hz": a.rate, "timeout_ms": a.timeout_ms,
            "limits": level_limits(a, half=half), **({"solver": a.solver} if a.solver else {}),
            **({"collision": True} if getattr(a, "collision", False) else {}), **extra})
        self.sid = r["session_id"]
        return r["pose"]

    async def stop_session(self):
        sid, self.sid = self.sid, None
        if sid:
            try:
                await self.svc.do_command({"command": "servo_stop", "session_id": sid})
            except Exception as e:  # an ended session (dead-man, fault) errors: log and carry on
                print(f"servo_stop: {e}")

    async def watchdog(self):
        """Follower pose at 10 Hz (floor guard + lag data) and servo_status at 5 Hz. Any failure here
        is a fault: losing the floor guard silently is not allowed."""
        a = self.args
        k = 0
        try:
            while True:
                fp = pose_dict(await self.follower2.get_end_position())
                tp = self.now()  # stamp the pose read, not the later status RPC
                row = follow_row(tp, fp)
                if lowest_z(fp, a.tip_mm) < a.min_z_mm:
                    self.fault = f"follower below the floor: lowest z {lowest_z(fp, a.tip_mm):.1f} mm"
                if k % 2 == 0 and self.sid:
                    try:
                        st = await self.svc2.do_command({"command": "servo_status", "session_id": self.sid})
                        row = follow_row(tp, fp, st)
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

    # ---- run and exit -------------------------------------------------------------------------
    async def run(self):
        raise NotImplementedError

    async def go(self):
        """What main runs after open(); teleop adds --dry-run."""
        await self.run()

    async def release_extras(self, step):
        """Subclass exit steps (run after servo_stop, before the connections close); step(name, make)
        is the timed, independent step runner."""

    async def close(self):
        """Ordered, independent exit steps (spec, Exit): one failing or hanging never skips the rest.
        Each step has a 3 s timeout, and a second Ctrl-C is ignored until release_extras is done."""
        import signal

        print("stopping, wait…")
        prev = signal.signal(signal.SIGINT, signal.SIG_IGN)

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
            await self.release_extras(step)
            for f in (self._tf, self._ff):
                if f:
                    f.close()
            for r in (self.robot, self.robot2):
                if r:
                    await step("close", r.close)
        finally:
            signal.signal(signal.SIGINT, prev)


def run_client(cls, ap: argparse.ArgumentParser) -> int:
    """Parse, run cls(args), exit code. SIGTERM (a plain kill, a closing terminal) takes the same exit
    path as Ctrl-C: cancel main, so close() stops the session and releases whatever the client holds."""

    async def main() -> int:
        import signal

        args = ap.parse_args()
        client = cls(args)
        asyncio.get_running_loop().add_signal_handler(signal.SIGTERM, asyncio.current_task().cancel)
        try:
            await client.open()
            await client.go()
            return 0
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("stopping")
            return 0
        except Exception as e:
            print(f"error: {e}")
            return 1
        finally:
            await client.close()

    try:
        return asyncio.run(main())
    except KeyboardInterrupt:
        return 130
