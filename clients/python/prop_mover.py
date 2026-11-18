# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Move a sim prop in a straight line A -> B (world mm) at a set speed, hold at B, optionally come back, and log
its true pose with Unix time (the props CSV that `cli clearance` replays against).

  uv run clients/python/prop_mover.py --address localhost:8084 --insecure --prop intruder --from 600,-300,100 \\
      --to 600,300,100 --mm-s 50 --hold-s 3 --back --size-mm 80,80,120 --csv .runs/props.csv
"""
import argparse
import math


def vec3(s: str) -> tuple:
    v = tuple(float(x) for x in s.split(","))
    if len(v) != 3 or not all(math.isfinite(x) for x in v):
        raise argparse.ArgumentTypeError(f"want three finite numbers x,y,z, got {s!r}")
    return v


def path_state(t: float, a: tuple, b: tuple, mm_s: float, hold_s: float, back: bool) -> tuple:
    """(position, done) t s after the mover starts: A -> B at mm_s, hold_s at B, then B -> A if back."""
    d = math.dist(a, b)
    if d == 0:  # no leg: sit at A through the hold
        return a, t >= hold_s
    leg = d / mm_s
    if t <= 0:
        return a, False
    f = min(t / leg, 1.0)
    if t < leg + hold_s or not back:
        return tuple(p + (q - p) * f for p, q in zip(a, b)), t >= leg + hold_s
    g = min((t - leg - hold_s) / leg, 1.0)
    return tuple(q + (p - q) * g for p, q in zip(a, b)), g >= 1.0


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8080")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--world", default="sim-world")
    ap.add_argument("--prop", required=True)
    ap.add_argument("--from", dest="a", type=vec3, required=True, help="x,y,z world mm")
    ap.add_argument("--to", dest="b", type=vec3, required=True, help="x,y,z world mm")
    ap.add_argument("--mm-s", type=float, required=True)
    ap.add_argument("--hold-s", type=float, default=0.0)
    ap.add_argument("--back", action="store_true", help="return to --from after the hold")
    ap.add_argument("--size-mm", type=vec3, required=True, help="prop box full dims sx,sy,sz (its centre is the pose), logged for clearance")
    ap.add_argument("--csv", help="props CSV to write")
    ap.add_argument("--start-delay-s", type=float, default=0.0, help="sit at --from this long before moving")
    ap.add_argument("--hz", type=float, default=50.0)
    args = ap.parse_args()
    if not args.mm_s > 0 or args.hold_s < 0 or args.start_delay_s < 0 or not args.hz > 0:
        ap.error("--mm-s and --hz must be > 0; --hold-s and --start-delay-s >= 0")

    import asyncio
    import csv
    import time

    import viam.components.generic  # noqa: F401  (before connecting, so the client builds this API)
    from viam.components.generic import Generic
    from servo_client import connect

    robot = await connect(args)
    f = None
    try:
        world = Generic.from_robot(robot, args.world)
        w = None
        if args.csv:
            f = open(args.csv, "w", newline="")
            w = csv.writer(f)
            w.writerow(["t", "name", "x", "y", "z", "qw", "qx", "qy", "qz", "sx", "sy", "sz"])
        t0 = time.monotonic()
        while True:
            now = time.time()
            pos, done = path_state(time.monotonic() - t0 - args.start_delay_s, args.a, args.b, args.mm_s, args.hold_s, args.back)
            await world.do_command({"command": "set_prop_pose", "prop": args.prop,
                                    "position": [p / 1000 for p in pos], "orientation_wxyz": [1, 0, 0, 0]})
            if w:
                w.writerow([f"{now:.4f}", args.prop, *(f"{p:.3f}" for p in pos), 1, 0, 0, 0, *args.size_mm])
            if done:
                break
            await asyncio.sleep(max(0.0, 1 / args.hz - (time.time() - now)))
    finally:
        if f:
            f.close()
        await robot.close()


if __name__ == "__main__":
    import asyncio
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    asyncio.run(main())
