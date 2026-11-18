# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 2 bench for the diff-ik servo DoCommand surface.

Usage (sim, local plaintext listener):
  uv run clients/python/servo_bench.py --address 127.0.0.1:<port> --insecure --arm pick-arm --joint-deg-s 60
Level 1 (hardware):
  uv run clients/python/servo_bench.py --address … --arm follower-arm --task pointing --joint-deg-s 30 --linear-mm-s 40 --angular-deg-s 20 --timeout-ms 300 --twist-mm-s 30 --circle-mm-s 32 --resync-deg <from check 4> --singular-eps 0.03 --max-damping 0.05
With API keys instead: export VIAM_API_KEY_ID / VIAM_API_KEY and drop --insecure.
Spec: docs/superpowers/specs/2026-11-15-diff-ik-phase2-design.md, "Measurements".
"""

import argparse
import asyncio
import math
import os
import statistics
import sys
import time

FAILS: list[str] = []
MOTION_EPS = 0.05  # deg; --motion-eps-deg. Use ≥ 2 encoder counts on hardware (STS3215 count = 0.088°).


def report(metric: str, value: str, bar: str = "report", ok: bool | None = None) -> None:
    mark = "" if ok is None else ("PASS" if ok else "FAIL")
    if ok is False:
        FAILS.append(metric)
    print(f"{metric:<38} {value:<26} {bar:<20} {mark}", flush=True)


def moved(q0: list[float], q: list[float], eps: float | None = None) -> bool:
    eps = MOTION_EPS if eps is None else eps
    return any(abs(a - b) > eps for a, b in zip(q0, q))


def first_stationary(samples: list[tuple[float, list[float]]], after: float, eps: float | None = None,
                     window: float = 0.1) -> float | None:
    """Time of the first sample after `after` from which every joint stays within eps for `window` seconds.

    A time window, not a sample count: polls over loopback arrive ~1 ms apart, so consecutive
    samples of a moving arm differ by far less than eps."""
    eps = MOTION_EPS if eps is None else eps
    s = [x for x in samples if x[0] >= after]
    for i, (t, q) in enumerate(s):
        if s[-1][0] < t + window:
            return None  # not enough samples left to prove it
        if all(not moved(q, q2, eps) for t2, q2 in s[i + 1:] if t2 <= t + window):
            return t
    return None


async def connect(args):
    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions

    if args.insecure:
        dial = DialOptions(insecure=True, disable_webrtc=True)
    else:
        dial = DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    return await RobotClient.at_address(args.address, RobotClient.Options(dial_options=dial))


async def cmd(svc, **kw):
    return await svc.do_command(kw)


async def expect_err(svc, label: str, substr: str, **kw) -> None:
    try:
        await cmd(svc, **kw)
        report(f"smoke: {label}", "no error", f"error ∋ '{substr}'", False)
    except Exception as e:  # SDK wraps server errors in several exception types
        report(f"smoke: {label}", "error", f"error ∋ '{substr}'", substr in str(e))


class Poller:
    """Polls the arm on a second connection: joint positions (deg) or end positions (mm)."""

    def __init__(self, arm, end: bool = False):
        self.arm, self.end = arm, end
        self.samples: list[tuple[float, list[float]]] = []
        self.task: asyncio.Task | None = None

    async def _run(self) -> None:
        while True:
            if self.end:
                p = await self.arm.get_end_position()
                v = [p.x, p.y, p.z]
            else:
                v = list((await self.arm.get_joint_positions()).values)
            self.samples.append((time.monotonic(), v))

    def start(self) -> None:
        self.samples = []
        self.task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass


def level_limits(args) -> dict:
    lim = {"joint_deg_s": args.joint_deg_s}
    for key, val in (("linear_mm_s", args.linear_mm_s), ("angular_deg_s", args.angular_deg_s),
                     ("resync_deg", args.resync_deg), ("singular_eps", args.singular_eps),
                     ("max_damping", args.max_damping), ("tol_mm", args.tol_mm),
                     ("tol_deg", args.tol_deg)):
        if val is not None:
            lim[key] = val
    return lim


def start_kw(args, **kw):
    base = {"command": "servo_start", "component_name": args.arm, "frame": f"{args.arm}_origin",
            "rate_hz": args.rate, "limits": level_limits(args)}
    if args.task:
        base["task"] = args.task
    if args.no_point_velocities:
        base["point_velocities"] = False
    if args.solver:
        base["solver"] = args.solver
    base.update(kw)
    return base


async def hold_until_converged(svc, sid: str, pose: dict, timeout: float = 6.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = await cmd(svc, command="servo_target", session_id=sid, pose=pose)
        if r["state"] == "converged":
            return True
        await asyncio.sleep(0.1)
    return False


async def smoke(svc, args) -> None:
    r = await svc.do_command(start_kw(args, mode="pose"))
    sid, p0 = r["session_id"], r["pose"]
    await expect_err(svc, "busy arm", "busy", **start_kw(args))
    await expect_err(svc, "wrong mode", "wrong mode", command="servo_twist", session_id=sid,
                     linear=[0, 0, 0], angular=[0, 0, 0])
    await expect_err(svc, "unknown session", "unknown session_id", command="servo_status", session_id="nope")
    await expect_err(svc, "unknown command", "unknown command", command="servo_fly")
    goal = dict(p0, z=p0["z"] + 10)
    ok = await hold_until_converged(svc, sid, goal)
    st = await cmd(svc, command="servo_status", session_id=sid)
    report("smoke: pose converges", st["state"], "converged", ok)
    fin = await cmd(svc, command="servo_stop", session_id=sid)
    report("smoke: pose stop", fin["state"], "stopped", fin["state"] == "stopped")
    await expect_err(svc, "ended session", "session ended", command="servo_target", session_id=sid, pose=goal)

    r = await svc.do_command(start_kw(args, mode="twist"))
    sid = r["session_id"]
    for _ in range(5):
        await cmd(svc, command="servo_twist", session_id=sid, linear=[0, 0, -10], angular=[0, 0, 0])
        await asyncio.sleep(0.1)
    st = await cmd(svc, command="servo_status", session_id=sid)
    report("smoke: twist running", st["state"], "running", st["state"] == "running")
    fin = await cmd(svc, command="servo_stop", session_id=sid)
    report("smoke: twist stop", fin["state"], "stopped", fin["state"] == "stopped")


async def latency(svc, arm2, args) -> None:
    r = await svc.do_command(start_kw(args, mode="pose"))
    sid, base = r["session_id"], r["pose"]
    poll = Poller(arm2)
    lats, rtts = [], []
    for i in range(10):
        prev = dict(base, x=base["x"] + (0 if i % 2 == 0 else 20))
        tgt = dict(base, x=base["x"] + (20 if i % 2 == 0 else 0))
        if not await hold_until_converged(svc, sid, prev):
            report(f"latency step {i}", "no convergence", "converged", False)
            break
        poll.start()
        await asyncio.sleep(0.15)
        if not poll.samples:
            await poll.stop()
            report(f"latency step {i}", "no joint samples", "samples", False)
            break
        q0 = poll.samples[-1][1]
        t0 = time.monotonic()
        await cmd(svc, command="servo_target", session_id=sid, pose=tgt)
        rtts.append(time.monotonic() - t0)
        lat, last_send = None, t0
        while lat is None and time.monotonic() - t0 < 2:
            await asyncio.sleep(0.01)
            hit = next((ts for ts, q in poll.samples if ts > t0 and moved(q0, q)), None)
            if hit is not None:
                lat = hit - t0
            elif time.monotonic() - last_send > 0.1:
                await cmd(svc, command="servo_target", session_id=sid, pose=tgt)  # feed the dead-man
                last_send = time.monotonic()
        await poll.stop()
        if lat is not None:
            lats.append(lat)
    await cmd(svc, command="servo_stop", session_id=sid)
    if lats:
        report("command→motion latency (median)", f"{statistics.median(lats) * 1000:.0f} ms")
        report("command→motion latency (max)", f"{max(lats) * 1000:.0f} ms")
    report("servo_target round-trip (median)", f"{statistics.median(rtts) * 1000:.1f} ms")
    report("latency steps measured", f"{len(lats)}/10", "10/10", len(lats) == 10)


async def twist_for(svc, sid: str, linear: list[float], seconds: float) -> float:
    """Re-send a twist every 100 ms for `seconds`; returns the time of the last send."""
    end, last = time.monotonic() + seconds, 0.0
    while time.monotonic() < end:
        last = time.monotonic()
        await cmd(svc, command="servo_twist", session_id=sid, linear=linear, angular=[0, 0, 0])
        await asyncio.sleep(0.1)
    return last


async def stop_latency(svc, arm2, args) -> None:
    sid = (await svc.do_command(start_kw(args, mode="twist")))["session_id"]
    poll = Poller(arm2)
    poll.start()
    await twist_for(svc, sid, [args.twist_mm_s, 0, 0], 1.0)
    p_before = await arm2.get_end_position()
    t0 = time.monotonic()
    fin = await cmd(svc, command="servo_stop", session_id=sid)
    rtt = time.monotonic() - t0
    await asyncio.sleep(1.5)
    await poll.stop()
    p_after = await arm2.get_end_position()
    still = first_stationary(poll.samples, t0)
    report("servo_stop round-trip (incl. ramp)", f"{rtt * 1000:.0f} ms")
    report("stop sent → arm stationary", "never" if still is None else f"{(still - t0) * 1000:.0f} ms")
    d = math.dist((p_before.x, p_before.y, p_before.z), (p_after.x, p_after.y, p_after.z))
    report("travel after stop sent", f"{d:.1f} mm")
    report("stop final state", fin["state"], "stopped", fin["state"] == "stopped")
    report("ideal ramp travel", f"{args.twist_mm_s * 0.2 / 2:.1f} mm")  # ramp_ms default 200 (module.go)


async def deadman(svc, arm2, args) -> None:
    timeout_ms, ramp_ms, rate = args.timeout_ms, 200, args.rate
    sid = (await svc.do_command(start_kw(args, mode="twist", timeout_ms=timeout_ms, ramp_ms=ramp_ms,
                                         rate_hz=rate)))["session_id"]
    poll = Poller(arm2)
    poll.start()
    t_last = await twist_for(svc, sid, [-args.twist_mm_s, 0, 0], 1.0)
    t_to = None
    while time.monotonic() - t_last < 3:
        st = await cmd(svc, command="servo_status", session_id=sid)
        if st["state"] == "stopped_timeout":
            t_to = time.monotonic() - t_last
            break
        await asyncio.sleep(0.02)
    await asyncio.sleep(1.0)
    await poll.stop()
    bar = timeout_ms + 2 * 1000 / rate + ramp_ms + 50
    report("last cmd → stopped_timeout", "never" if t_to is None else f"{t_to * 1000:.0f} ms",
           f"≤ {bar:.0f} ms", t_to is not None and t_to * 1000 <= bar)
    still = first_stationary(poll.samples, t_last)
    report("last cmd → arm stationary", "never" if still is None else f"{(still - t_last) * 1000:.0f} ms")


async def circle(svc, arm2, args, home=None) -> None:
    if home is not None:
        # start every circle from the same joints: the previous run leaves the arm 50 mm out,
        # so chaining circles walks toward the edge of a small arm's reach
        await arm2.move_to_joint_positions(home)
    r = await svc.do_command(start_kw(args, mode="pose"))
    sid, c = r["session_id"], r["pose"]
    radius, speed = 50.0, args.circle_mm_s
    omega = speed / radius
    laps_t = 2 * 2 * math.pi / omega
    if not await hold_until_converged(svc, sid, dict(c, x=c["x"] + radius)):
        report(f"circle {speed:.0f} mm/s", "no convergence at start", "converged", False)
        await cmd(svc, command="servo_stop", session_id=sid)
        return
    poll = Poller(arm2, end=True)
    poll.start()
    t0, n = time.monotonic(), 0
    while (t := time.monotonic() - t0) < laps_t:
        th = omega * t
        await cmd(svc, command="servo_target", session_id=sid,
                  pose=dict(c, x=c["x"] + radius * math.cos(th), y=c["y"] + radius * math.sin(th)))
        n += 1
        await asyncio.sleep(max(0.0, t0 + n / args.rate - time.monotonic()))
    await poll.stop()
    st = await cmd(svc, command="servo_status", session_id=sid)
    await cmd(svc, command="servo_stop", session_id=sid)
    errs = []
    for ts, (x, y, z) in poll.samples:
        th = omega * (ts - t0)
        if 0 <= ts - t0 <= laps_t:
            errs.append(math.dist((x, y, z), (c["x"] + radius * math.cos(th), c["y"] + radius * math.sin(th), c["z"])))
    label = f"circle {speed:.0f} mm/s"
    if not errs:
        report(f"{label}: samples", "none", "> 0", False)
        return
    mean = statistics.mean(errs)
    report(f"{label}: mean error", f"{mean:.1f} mm", "≤ 22.1 mm", mean <= 22.1)
    report(f"{label}: max error", f"{max(errs):.1f} mm")
    report(f"{label}: resyncs / overruns", f"{st['resyncs']:.0f} / {st['overruns']:.0f}")
    if "pointing_residual_deg" in st:
        report(f"{label}: pointing residual (geometry, not lag)", f"{st['pointing_residual_deg']:.1f}°")
    report(f"{label}: target rate", f"{n / laps_t:.1f} Hz")


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8080")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", required=True)
    ap.add_argument("--task", default=None, help="pose|pointing (default: the server's default for the arm)")
    ap.add_argument("--no-point-velocities", action="store_true",
                    help="servo_start point_velocities=false (A/B: stream positions without per-point velocities)")
    ap.add_argument("--joint-deg-s", type=float, default=90)
    ap.add_argument("--solver", choices=["dls", "qp"], default=None, help="servo_start solver (default: the server's, dls)")
    ap.add_argument("--linear-mm-s", type=float, default=None)
    ap.add_argument("--angular-deg-s", type=float, default=None)
    ap.add_argument("--rate", type=float, default=50)
    ap.add_argument("--timeout-ms", type=float, default=500, help="dead-man step's timeout")
    ap.add_argument("--resync-deg", type=float, default=None)
    ap.add_argument("--singular-eps", type=float, default=None)
    ap.add_argument("--max-damping", type=float, default=None)
    ap.add_argument("--tol-mm", type=float, default=None)
    ap.add_argument("--tol-deg", type=float, default=None)
    ap.add_argument("--twist-mm-s", type=float, default=50, help="stop and dead-man twist speed")
    ap.add_argument("--circle-mm-s", type=float, default=50)
    ap.add_argument("--motion-eps-deg", type=float, default=0.05,
                    help="joint change that counts as motion (latency, stationary); ≥ 0.2 on the real SO-101")
    ap.add_argument("--steps", default="smoke,latency,stop,deadman,circle")
    args = ap.parse_args()
    global MOTION_EPS
    MOTION_EPS = args.motion_eps_deg

    from viam.components.arm import Arm
    from viam.services.motion import MotionClient

    robot, robot2 = await connect(args), await connect(args)
    try:
        svc = MotionClient.from_robot(robot, args.service)
        arm2 = Arm.from_robot(robot2, args.arm)
        home = await arm2.get_joint_positions()  # the start pose every circle returns to
        steps = args.steps.split(",")
        if "smoke" in steps:
            await smoke(svc, args)
        if "latency" in steps:
            await latency(svc, arm2, args)
        if "stop" in steps:
            await stop_latency(svc, arm2, args)
        if "deadman" in steps:
            await deadman(svc, arm2, args)
        if "circle" in steps:
            await circle(svc, arm2, args, home)
    finally:
        await robot.close()
        await robot2.close()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASS'}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
