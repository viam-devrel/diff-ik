# /// script
# requires-python = ">=3.11"
# ///
"""Pure-path checks for builtin_peg.py. Run: uv run clients/python/test_builtin_peg.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from builtin_peg import ft_vec, to_base_side  # noqa: E402

nested = {"force": {"x": 1.0, "y": 2.0, "z": 3.0}, "torque": {"x": 0.1, "y": 0.2, "z": 0.3}, "tared": False}
flat = {"Fx_N": 1.0, "Fy_N": 2.0, "Fz_N": 3.0, "TRx_Nm": 0.1, "TRy_Nm": 0.2, "TRz_Nm": 0.3}
assert ft_vec(nested) == ft_vec(flat) == ((1.0, 2.0, 3.0), (0.1, 0.2, 0.3))
assert to_base_side((4.0, 3.0, -8.0)) == (4.0, -3.0)  # tool down (180° about x): tool y is base -y
print("ok")
