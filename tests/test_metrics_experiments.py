import math

import numpy as np
import pytest

from pmsm_foc.experiments import load_disturbance, speed_step, torque_step
from pmsm_foc.metrics import disturbance_metrics, step_metrics


def _record(fn, t_step, t_end, dt=1e-6):
    t = np.arange(0.0, t_end, dt)
    y = np.where(t >= t_step, fn(np.maximum(t - t_step, 0.0)), 0.0)
    return t, y


def test_first_order_metrics_match_analytic():
    tau = 1e-3
    t, y = _record(lambda s: 1 - np.exp(-s / tau), t_step=2e-3, t_end=30e-3)
    m = step_metrics(t, y, 1.0, 2e-3, y_init=0.0)
    assert m["rise_time_s"] == pytest.approx(tau * math.log(9), rel=2e-3)     # 2.197 tau
    assert m["overshoot_pct"] == 0.0
    assert m["settling_time_s"] == pytest.approx(tau * math.log(50), rel=2e-3)  # 3.912 tau
    assert m["steady_state_error"] == pytest.approx(0.0, abs=1e-6)


def test_second_order_overshoot_matches_analytic():
    zeta, wn = 0.5, 1000.0
    wd, phi = wn * math.sqrt(1 - zeta ** 2), math.acos(zeta)
    t, y = _record(lambda s: 1 - np.exp(-zeta * wn * s) / math.sqrt(1 - zeta ** 2)
                   * np.sin(wd * s + phi), t_step=1e-3, t_end=40e-3)
    m = step_metrics(t, y, 1.0, 1e-3, y_init=0.0)
    expected = 100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))      # 16.30 %
    assert m["overshoot_pct"] == pytest.approx(expected, abs=0.05)


def test_non_settling_response_reports_none():
    t, y = _record(lambda s: 1 + 0.1 * np.sin(300 * s) + 0 * s, t_step=0.0, t_end=0.2)
    assert step_metrics(t, y, 1.0, 0.0, y_init=0.0)["settling_time_s"] is None


def test_disturbance_metrics_match_analytic():
    tau, dip, ref, t_d = 5e-3, 5.0, 30.0, 0.01
    t = np.arange(0.0, 0.15, 1e-6)
    s = np.maximum(t - t_d, 0.0) / tau
    y = ref - dip * s * np.exp(1 - s)                 # peak deviation -dip at s = 1
    m = disturbance_metrics(t, y, ref, t_d)
    assert m["peak_deviation"] == pytest.approx(-dip, abs=1e-3)
    assert m["time_of_peak_s"] == pytest.approx(tau, abs=2e-6)
    lo, hi = 1.0, 20.0                                # solve dip*x*exp(1-x) = 0.02*ref for x > 1
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if dip * mid * math.exp(1 - mid) > 0.02 * ref else (lo, mid)
    assert m["recovery_time_s"] == pytest.approx(lo * tau, abs=5e-6)


def test_torque_step_experiment_is_sane():
    _, m, _ = torque_step()
    assert m["overshoot_pct"] < 12.0
    assert m["rise_time_ms"] < 0.5
    assert m["settling_time_ms"] < 3.0
    assert abs(m["steady_state_error"]) < 0.01
    assert m["peak_abs_id_A"] < 1e-6                  # locked rotor: no d-axis disturbance


def test_speed_step_experiment_is_sane():
    _, m, _ = speed_step()
    assert abs(m["steady_state_error"]) < 0.01
    assert m["settling_time_ms"] is not None and m["settling_time_ms"] < 100.0
    assert m["peak_iq_ref_A"] <= 10.0 + 1e-9
    assert m["peak_voltage_V"] <= m["voltage_limit_V"] * (1 + 1e-6)
    assert m["peak_abs_id_A"] < 0.5


def test_load_disturbance_experiment_is_sane():
    _, m, _ = load_disturbance()
    assert m["speed_just_before_load"] == pytest.approx(30.0, abs=0.05)   # settled before the hit
    assert m["peak_deviation"] < 0.0                                       # speed dips
    assert m["recovery_time_ms"] is not None
    assert abs(m["steady_state_error"]) < 0.01
