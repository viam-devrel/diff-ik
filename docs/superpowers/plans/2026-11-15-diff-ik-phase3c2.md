# Diff-IK Phase 3c stage 2 (leader-arm teleop) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Python client that teleoperates the real SO-101 follower from the SO-101 leader through the diff-ik `servo` DoCommand API, with the follower in the pose task. The plan also covers an analysis tool, a rehearsal on the sim, and hardware runs under the `tuned` and `smooth` gain profiles.

**Architecture:** `clients/python/teleop.py` has three parts:
- pure helpers (pose delta, floor lift, hold-with-hysteresis, runaway check), unit-tested in `test_teleop.py`;
- an async client: an engage session at half the L2 limits, then a tracking session at L2;
- a watchdog task and a gripper task, writing two CSVs.

`clients/python/teleop_lag.py` analyses the CSVs. No Go changes.

**Tech Stack:** Python ≥ 3.11 run with `uv run` (PEP 723 script headers, `viam-sdk`); the existing diff-ik `servo` service; the so-101 driver (leader `set_torque`, gripper `get_position` / `set_position`).

**Spec:** `docs/superpowers/specs/2026-11-15-diff-ik-phase3c2-design.md`. Read its Decisions, Client and Safety sections first.

**Branch:** `diff-ik-phase3c2` (checked out in `/Users/nick.hehr/src/diff-ik`).

**Commit trailer** (every commit):
```
Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY
```

**Facts the code relies on** (verified in `module.go` / `session.go`):
- Pose maps are `{x, y, z (mm), o_x, o_y, o_z, theta (deg)}`. The orientation vector (o_x, o_y, o_z) **is the tool's z axis in the base frame**, normalised by the server.
- The `servo_start` reply is `{session_id, pose}` (the follower's pose in the session frame). With `frame: "<arm>_origin"`, the frame is the arm base.
- The `servo_target` reply is `{state, err_mm, err_deg}`. `state` is one of `running`, `converged`, `stopped`, `stopped_timeout` and so on.
- `servo_status` includes `rate_hz`, `tick_p99_ms`, `overruns`, `resyncs`, `state`, `err_mm` and `err_deg`.
- `servo_stop` on an ended session returns an error.
- Driver: `arm.do_command({"command": "set_torque", "enable": bool})` acts on the arm's whole bus (servos 1–6). The gripper accepts `{"command": "get_position"}` → `{"position_percentage": …}`, `{"command": "set_position", "percentage": p, "wait": False}` and `{"command": "set_torque", "enable": bool}`.

---

## File structure

| File | Change | Responsibility |
|---|---|---|
| `clients/python/teleop.py` | create | helpers + teleop client (dry run, engage, tracking, watchdog, gripper, exit, CSV) |
| `clients/python/test_teleop.py` | create | `assert`-based tests of the helpers (runs with `uv run`) |
| `clients/python/teleop_lag.py` | create | CSV analysis: lag, tracking error, rest convergence, bars |
| `clients/python/test_teleop_lag.py` | create | synthetic-CSV test of the analysis |
| `dev/so101-hw.md` | modify | stage-2 section: setup, commands, safety, results log |
| `docs/results/2026-11-phase3c2.md` | create (Part C) | results |

---

## Part A — code (subagents)

### Task 1: teleop helpers

**Files:** create `clients/python/teleop.py` (helpers only for now) and `clients/python/test_teleop.py`.

- [ ] **Step 1: Write the failing tests.** `clients/python/test_teleop.py`:

```python
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
```

- [ ] **Step 2: Run.** `uv run clients/python/test_teleop.py` → FAILS with `ModuleNotFoundError: teleop` (or an ImportError).

- [ ] **Step 3: Implement the helpers.** Create `clients/python/teleop.py` with this header and helpers. The client comes in Task 2; keep every `viam` import inside functions, so the tests import without the SDK.

```python
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
```

- [ ] **Step 4: Run.** `uv run clients/python/test_teleop.py` → four `ok` lines.
- [ ] **Step 5: Commit.** `git add clients/python/teleop.py clients/python/test_teleop.py && git commit -m "teleop: pose delta, floor lift, hold-with-hysteresis and runaway helpers"`.

### Task 2: the teleop client

**Files:** modify `clients/python/teleop.py` (append the client).

There are no unit tests for the I/O. Verification is `--help`, `py_compile`, Task 1's tests still passing, and Part B's rehearsal.

- [ ] **Step 1: Append the client** to `clients/python/teleop.py`:

```python
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
            lp = await self.leader_pose()
            sp = dict(hold.update(lp, self.now()))
            lift = floor_lift(sp, a.min_z_mm, a.tip_mm)
            sp["z"] += lift
            rep = await self.svc.do_command({"command": "servo_target", "session_id": self.sid, "pose": sp})
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
        last = None
        while True:
            try:
                pct = (await self.lgrip.do_command({"command": "get_position"}))["position_percentage"]
                if last is None or abs(pct - last) >= 1.0:
                    await self.fgrip.do_command({"command": "set_position", "percentage": pct, "wait": False})
                    last = pct
            except Exception as e:
                print(f"gripper: {e}")
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
        self.tasks = [asyncio.create_task(self.watchdog())]
        if self.lgrip and self.fgrip:
            self.tasks.append(asyncio.create_task(self.gripper()))

        print("engaging at half the L2 limits…")
        await self.start_session(half=True)
        conv_since: list[float | None] = [None]
        deadline = self.now() + 15

        def engaged(rep, t):
            if rep["state"] == "converged":
                conv_since[0] = conv_since[0] if conv_since[0] is not None else t
                return t - conv_since[0] >= 0.5
            conv_since[0] = None
            if t > deadline:
                raise RuntimeError("engage did not converge within 15 s")
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
```

  **The implementer must check these points against the SDK and the code:**
  - **Engage closure:** `engaged()` closes over `conv_since` and `deadline`. The one-element list is deliberate. Keep it, or switch to `nonlocal`.
  - **Ctrl-C:** under `asyncio.run`, Ctrl-C cancels `main()`, and the `finally` still awaits `tp.close()`. `close()` ignores further SIGINTs until done, and each step has a 3 s timeout. Confirm with the rehearsal that Ctrl-C mid-stream reaches `servo_stop` and closes the connections.
  - **Watchdog fault:** when the watchdog sets `self.fault` (a floor breach, its own failure, or `--fault-after`), `stream` raises on its next tick, and `close()` runs. That's at most one tick plus one RPC late.
  - **Imports:** if any `viam` import path differs in the installed SDK (`viam.services.motion.MotionClient`, `viam.components.arm.Arm`, `viam.components.gripper.Gripper`), use the bench's imports as the reference.

- [ ] **Step 2: Verify.** `python3 -m py_compile clients/python/teleop.py`; `uv run clients/python/teleop.py --help` (lists the flags); `uv run clients/python/test_teleop.py` (still four `ok`).
- [ ] **Step 3: Commit.** `git commit -am "teleop: leader→follower client (engage at half limits, tracking, watchdog, gripper mirror, ordered exit, CSV)"`.

### Task 3: analysis tool

**Files:** create `clients/python/teleop_lag.py` and `clients/python/test_teleop_lag.py`.

- [ ] **Step 1: Write the failing test.** `clients/python/test_teleop_lag.py`:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Synthetic-CSV check for teleop_lag.py. Run: uv run clients/python/test_teleop_lag.py"""
import csv
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_lag import analyze  # noqa: E402

KEYS = ("x", "y", "z", "o_x", "o_y", "o_z", "theta")


def leader(t):  # a 40 mm-radius circle at 0.5 Hz, then rest after 6 s
    if t > 6:
        t = 6
    return 200 + 40 * math.cos(math.pi * t), 40 * math.sin(math.pi * t), 150


def main():
    d = tempfile.mkdtemp()
    pre = os.path.join(d, "run")
    with open(pre + "-ticks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", *[f"l_{k}" for k in KEYS], *[f"s_{k}" for k in KEYS],
                    "held", "clamped", "state", "err_mm", "err_deg", "tick_ms"])
        for k in range(450):  # 9 s at 50 Hz
            t = k * 0.02
            x, y, z = leader(t)
            rest = t > 6.5
            pause = 2.0 <= t < 2.3  # a short held pause mid-motion: must not count as rest
            row = [x, y, z, 0, 0, -1, 0]
            w.writerow([f"{t:.4f}", "track", *row, *row, int(rest or pause), 0,
                        "converged" if rest else "running", 0.5 if rest else 8.0, 0.1, 6.0])
    with open(pre + "-follower.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "kind", *[f"f_{k}" for k in KEYS], "rate_hz", "tick_p99_ms", "overruns", "resyncs"])
        for k in range(90):  # 10 Hz, follower = leader delayed 100 ms
            t = k * 0.1
            x, y, z = leader(max(0.0, t - 0.1))
            kind = "status" if k % 2 == 0 else "pose"
            extra = [50.0, 3.0, 0, 0] if kind == "status" else ["", "", "", ""]
            w.writerow([f"{t:.4f}", kind, x, y, z, 0, 0, -1, 0, *extra])
    r = analyze(pre)
    assert abs(r["lag_ms"] - 100) <= 10, r["lag_ms"]
    assert r["track_err_mm_at_lag"] < r["track_err_mm_raw"], r
    assert r["rest_converged_frac"] >= 0.9, r
    assert r["max_tick_p99_ms"] == 3.0 and r["max_overruns"] == 0, r
    print("ok", {k: r[k] for k in ("lag_ms", "track_err_mm_raw", "track_err_mm_at_lag", "rest_converged_frac")})

    # every tick held (e.g. the self-tracking rehearsal): no motion, so lag fields are None, no crash
    rows = list(csv.reader(open(pre + "-ticks.csv")))
    hi = rows[0].index("held")
    with open(pre + "-ticks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(rows[0])
        for r in rows[1:]:
            r[hi] = "1"
            w.writerow(r)
    r = analyze(pre)
    assert r["lag_ms"] is None and r["track_err_mm_raw"] is None, r
    print("ok all-held")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run.** `uv run clients/python/test_teleop_lag.py` → FAILS (no module `teleop_lag`).

- [ ] **Step 3: Implement** `clients/python/teleop_lag.py`:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Analyse a teleop run: uv run clients/python/teleop_lag.py PREFIX
Reads PREFIX-ticks.csv and PREFIX-follower.csv (written by teleop.py --csv PREFIX)."""
import bisect
import csv
import math
import statistics
import sys


def _rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _interp(ts, vs, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return tuple(a + f * (b - a) for a, b in zip(vs[i - 1], vs[i]))


def analyze(prefix: str) -> dict:
    ticks = [r for r in _rows(prefix + "-ticks.csv") if r["phase"] == "track"]
    foll = _rows(prefix + "-follower.csv")
    lt = [float(r["t"]) for r in ticks]
    lp = [(float(r["l_x"]), float(r["l_y"]), float(r["l_z"])) for r in ticks]
    ft = [float(r["t"]) for r in foll]
    fp = [(float(r["f_x"]), float(r["f_y"]), float(r["f_z"])) for r in foll]
    t0, t1 = max(lt[0], ft[0]), min(lt[-1], ft[-1])
    grid = [t0 + 0.5 + k * 0.005 for k in range(int((t1 - t0 - 1.0) / 0.005))]  # 200 Hz, 0.5 s trimmed

    # motion only: drop grid points inside held runs (rest)
    held_t = [float(r["t"]) for r in ticks if r["held"] == "1"]
    hs = set(round(t, 2) for t in held_t)
    grid = [t for t in grid if round(t, 2) not in hs and round(t - 0.01, 2) not in hs and round(t + 0.01, 2) not in hs]

    def err_at(lag):  # follower(t) vs leader(t - lag), during motion
        return statistics.fmean(math.dist(_interp(ft, fp, t), _interp(lt, lp, t - lag)) for t in grid)

    lags = [k * 0.005 for k in range(0, 101)]  # 0–500 ms
    best = min(lags, key=err_at) if len(grid) >= 50 else None  # mostly-at-rest runs have no motion to correlate
    # rest (spec): ticks inside a held run lasting >= 2 s, at least 1 s into it, with no clamp
    held, run = [], []
    for r in ticks + [None]:
        if r is not None and r["held"] == "1":
            run.append(r)
            continue
        if run and float(run[-1]["t"]) - float(run[0]["t"]) >= 2.0:
            held += [x for x in run if float(x["t"]) - float(run[0]["t"]) >= 1.0 and x["clamped"] == "0"]
        run = []
    status = [r for r in foll if r["kind"] == "status" and r["tick_p99_ms"] != ""]
    tick_ms = sorted(float(r["tick_ms"]) for r in ticks)
    dur = lt[-1] - lt[0]
    return {
        "lag_ms": round(best * 1000) if best is not None else None,
        "track_err_mm_raw": round(err_at(0.0), 2) if best is not None else None,
        "track_err_mm_at_lag": round(err_at(best), 2) if best is not None else None,
        "rest_converged_frac": round(sum(r["state"] == "converged" for r in held) / len(held), 3) if held else None,
        "rest_err_mm_median": round(statistics.median(float(r["err_mm"]) for r in held), 2) if held else None,
        "rest_err_deg_median": round(statistics.median(float(r["err_deg"]) for r in held), 2) if held else None,
        "clamped_ticks": sum(r["clamped"] == "1" for r in ticks),
        "client_rate_hz": round((len(ticks) - 1) / dur, 2) if dur > 0 else None,
        "client_tick_ms_p50": tick_ms[len(tick_ms) // 2],
        "client_tick_ms_p99": tick_ms[int(len(tick_ms) * 0.99)],
        "max_tick_p99_ms": max(float(r["tick_p99_ms"]) for r in status) if status else None,
        "max_overruns": max(float(r["overruns"]) for r in status) if status else None,
        "max_resyncs": max(float(r["resyncs"]) for r in status) if status else None,
    }


if __name__ == "__main__":
    for k, v in analyze(sys.argv[1]).items():
        print(f"{k:24s} {v}")
```

  Note: this is O(lags × grid × log n). For a 60 s run that's about 100 × 12 000 interpolations of each stream, a few seconds. Fine.

- [ ] **Step 4: Run.** `uv run clients/python/test_teleop_lag.py` → `ok {…}` with `lag_ms` ≈ 100.
- [ ] **Step 5: Commit.** `git add clients/python/teleop_lag.py clients/python/test_teleop_lag.py && git commit -m "teleop_lag: lag, tracking error and bar checks from teleop CSVs"`.

### Task 4: runbook section

**Files:** modify `dev/so101-hw.md` (append).

- [ ] **Step 1: Append** a `## Stage 2: leader-arm teleop` section with:
  1. **Setup:**
     - enable `leader-arm` and `leader-gripper` on `laptop-main` (config change, with the user's OK and a backup);
     - find the listener;
     - check that both arms' calibrated limits match (`bin/cli … --arm leader-arm pose` and `--arm follower-arm pose`, no `--start-joints`, so it's read-only) and record them.
  2. **Safety:**
     - the leader goes limp at release, so hold it;
     - keep a hand near the follower's power switch;
     - Ctrl-C stops the follower through `servo_stop`;
     - if the client crashes, the follower stops within 300 ms but the leader falls;
     - the fast move must stay at or under about 300 mm/s for 0.5 s.
  3. **Commands:**
     - `uv run clients/python/teleop.py --address … --insecure --dry-run`;
     - the tuned session: `… --csv .runs/3c2-tuned --duration 60`;
     - the smooth session: `… --csv .runs/3c2-smooth --duration 60 --tol-mm 13 --tol-deg 5`;
     - the analysis: `uv run clients/python/teleop_lag.py .runs/3c2-tuned`.
  4. **Protocol:** about 60 s per run: slow sweeps, one bounded fast move, pick-like motions with the gripper, then rest the leader on a support above the floor for ≥ 2 s.
     - **Engage:** rest the leader on a support. Hand tremor can stop the follower converging, which runs into the 15 s engage timeout.
     - **Smooth profile:** about 11 mm of droop at a target clamped to floor + 15 mm leaves only about 4 mm before the floor watchdog trips, so keep the leader well above the table.
     - **Ctrl-C:** the client prints "stopping, wait…" and ignores a second Ctrl-C until the leader is re-torqued.
  5. **After every run:** the driver's `streamed … late` lines (`read_machine_logs`).
- [ ] **Step 2: Commit.** `git commit -am "dev: stage-2 teleop runbook"`.

### Task 5: Part A review (Opus)

Review the whole diff `git diff 5ce4f91..HEAD` against the spec. Focus on the exit paths, the hold logic, the floor checks and the CSV columns that `teleop_lag.py` relies on. Fix findings before Part B.

---

## Part B — rehearsal on the MuJoCo SO-101 (controller)

`mujoco-sim-mac` has one SO-101 (`arm`) and the `servo` service; find its listener as in stage 1.

- [ ] **B1.** `uv run clients/python/teleop.py --address 127.0.0.1:<port> --insecure --leader arm --follower arm --min-z-mm -70 --dry-run` prints the read timing.
- [ ] **B2.** **Self-tracking rehearsal** (leader = follower, so no release and no gripper): `… --leader arm --follower arm --min-z-mm -70 --duration 10 --csv .runs/3c2-sim-self`. Expect:
  - engage converges at once;
  - 10 s of tracking with `held` = 1 almost throughout;
  - a clean exit, with `servo_stop` logged;
  - `teleop_lag.py` runs (its lag is meaningless here).
- [ ] **B3.** **Ctrl-C mid-run:** the same command without `--duration`; press Ctrl-C after 3 s. Expect "stopping", `servo_stop` and closed connections, with no traceback.
- [ ] **B4.** **Start-up floor check:** rerun with `--min-z-mm 500`. It must refuse to engage and prompt; answer with Ctrl-C, which must exit at once without a hang. That proves the check without moving anything.
- [ ] **B4b.** **Fault path:** `… --leader arm --follower arm --min-z-mm -70 --fault-after 5 --csv .runs/3c2-sim-fault`. Expect the injected fault after about 5 s, then `servo_stop` and a clean exit with code 1.
- [ ] **B5.** Record any fixes; commit.

---

## Part C — hardware (controller with the user; never a subagent)

- [ ] **C1.** Ask the user's OK, back up the config, and enable `leader-arm` and `leader-gripper` on `laptop-main`. Confirm the leader isn't torqued into a strange pose at startup (the driver holds its current pose).
- [ ] **C2.** Read-only calibration check: both arms' `pose` limits; record them in the runbook.
- [ ] **C3.** `--dry-run` against the real leader. If it doesn't fit, choose `--rate 40` or 30 and note it.
- [ ] **C3.5.** **Smoke run with the leader still torqued:** `--no-release --duration 10`, leader torqued and still. It checks engage and 5-DOF rest convergence on the real follower without the limp-leader risk.
- [ ] **C4.** **Tuned session**, 60 s, following the protocol. Then:
  - `teleop_lag.py`;
  - the driver's `late` lines;
  - the user's verdict on jitter and on the gripper.

  Bars: late 0, max overruns 0, max tick p99 < the period, rest converged ≥ 90 %, client rate ≥ 0.99 × `--rate`, no visible jitter.
- [ ] **C5.** **Smooth session:**
  1. In `~/src/so-101`, create a throwaway local branch `teleop-smooth` from `stream-speed-profile`, merge `servo-deadband` into it, and run `make bin/arm`.
  2. Restart the local driver module.
  3. Set `servo_gain_profile: "smooth"` on `follower-arm`.
  4. Run 60 s with `--tol-mm 13 --tol-deg 5`.
  5. Do the same analysis and get the user's verdict.
- [ ] **C6.** **Restore:**
  - set `servo_gain_profile` to `"tuned"` (or remove it);
  - leave the merged driver build running, or switch back to `stream-speed-profile`, whichever the user prefers;
  - verify droop is ≈ 0.9 mm after a 5 s hold.
- [ ] **C7.** Write `docs/results/2026-11-phase3c2.md`:
  - a per-profile table against the bars;
  - lag and tracking (tuned vs smooth);
  - rest convergence of the pose task on 5 DOF;
  - findings;
  - a recommendation on `smooth` for teleop.

  Commit.
