"""The client-side chunk selection shared by policy_stream.py and policy_exec.py."""


def blended(cur, prev, t, w, at, mix):
    """cur's action at time t, crossfaded from prev's by w in [0, 1]; None when cur has none at t.
    at(chunk, t) samples a chunk (None when used up); mix(a, b, w) is (1 − w)·a + w·b."""
    target = at(cur, t)
    if prev is not None and target is not None and w < 1:
        old = at(prev, t)
        if old is not None:
            target = mix(old, target, w)
    return target
