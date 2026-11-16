"""Python float controller vs C++ Q15 controller on recorded closed-loop vectors.

Builds the C++ replay tool with whatever C++ compiler is installed (clang++/g++/c++) and skips if
there is none. Thresholds are calibrated bounds with margin, not measured results.
"""
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import verify_q15 as v  # noqa: E402

NAMES = ("speed_step_30", "voltage_limited_reversal")


def _have_compiler():
    return any(shutil.which(c) for c in ("clang++", "g++", "c++"))


def test_input_quantization_round_trip_and_saturation():
    phys = np.array([[1.0, -2.0, 0.5, 3.3, 100.0]])
    err = np.abs(v.from_raw(v.to_raw(phys)) - phys)
    assert np.all(err <= 0.5 * v.IN_SCALE / v.Q + 1e-12)          # within half an LSB
    big = v.to_raw(np.array([[1e3, -1e3, 1e3, -1e3, 1e5]]))
    assert list(big[0]) == [32767, -32768, 32767, -32768, 32767]    # saturates, never wraps


def test_cpp_q15_matches_python_float_on_recorded_vectors():
    if not _have_compiler():
        pytest.skip("no C++ compiler available")
    exes = v.build_tools()
    res = v.verify_scenarios(exes, scale=0.25, names=NAMES)
    agg = res["_aggregate"]
    assert all(res[n]["clipped_input_samples"] == 0 for n in NAMES)   # inputs fit the Q15 scaling
    assert res["voltage_limited_reversal"]["limited_steps"] > 0        # limiter really exercised
    assert agg["A_all"]["max_lsb"] < 6.0                               # controller error only
    assert agg["A_all"]["rms_lsb"] < 2.0
    assert agg["B_all"]["max_lsb"] < 20.0                              # incl. input quantization
    assert agg["B_all"]["rms_lsb"] < 6.0


def test_comparison_detects_a_wrong_reference():
    # Guard against a comparison that cannot fail: a float controller with decoupling disabled
    # must disagree with the Q15 controller (which has it) by far more than the pass thresholds.
    if not _have_compiler():
        pytest.skip("no C++ compiler available")
    exes = v.build_tools()
    phys = v.make_vectors(scale=0.25, names=("speed_step_30",))["speed_step_30"]
    raw = v.to_raw(phys)
    v_cpp, _, _ = v.run_cpp(exes["replay_q15"], raw, "canary")
    wrong = v.run_python(v.from_raw(raw), decoupling=False)
    assert np.abs(v_cpp - wrong).max() > 100.0
