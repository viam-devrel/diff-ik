# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for policy_stream.py. Run: uv run clients/python/test_policy_stream.py"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_stream import WAYPOINTS, blend, chunk_action, latency_s, make_chunk, minjerk, reference  # noqa: E402

assert minjerk(0) == 0 and minjerk(1) == 1 and minjerk(0.5) == 0.5 and minjerk(-1) == 0
assert reference(0, 2.5) == WAYPOINTS[0] and reference(2.5, 2.5) == WAYPOINTS[1] and reference(99, 2.5) == WAYPOINTS[-1]
c = {"t_obs": 10.0, "actions": [(i, 0, 0) for i in range(5)]}
assert chunk_action(c, 10.1, 30, True, 10.2) == (3, 0, 0)  # aligned: 0.1 s after the observation is action 3
assert chunk_action(c, 10.25, 30, False, 10.2) == (1, 0, 0)  # naive: counts from arrival
assert chunk_action(c, 11.0, 30, True, 10.2) is None  # used up
assert blend((0, 0, 0), (10, 0, 0), 0.25) == (2.5, 0, 0)
rng = random.Random(1)
lats = [latency_s(rng, 150, 0.4, 0.0, 1000) for _ in range(500)]
assert 0.12 < sorted(lats)[250] < 0.18, sorted(lats)[250]  # median near 150 ms
assert all(latency_s(random.Random(s), 150, 0.0, 1.0, 1000) == 1.15 for s in range(3))  # a stall adds stall_ms
ch = make_chunk(random.Random(2), 0.0, 50, 30, 2.5, 0.0, 0.0)
assert len(ch) == 50 and ch[0] == WAYPOINTS[0] and ch[30] == reference(1.0, 2.5)
print("ok")
