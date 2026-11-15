import math
from dataclasses import replace

import numpy as np
import pytest

from pmsm_foc import FocDrive, PMSMParams
from pmsm_foc.control import (design_current_gains, design_speed_gains, limit_dq,
                              pole_placement_pi, torque_constant)

P = PMSMParams()
P_LOCKED = replace(P, J=1e12)      # speed held constant: isolates the current loops


def test_pole_placement_matches_target_polynomial():
    a, b = P.Rs / P.Ld, 1.0 / P.Ld
    for wn, zeta in ((2 * math.pi * 200, 1.5), (2 * math.pi * 25, 1.0), (1000.0, 0.8)):
        kp, ki = pole_placement_pi(a, b, wn, zeta)
        assert a + b * kp == pytest.approx(2 * zeta * wn)      # s^1 coefficient
        assert b * ki == pytest.approx(wn ** 2)                # s^0 coefficient
    kp, ki = pole_placement_pi(a, b, 1000.0, 0.8)
    roots = np.roots([1.0, a + b * kp, b * ki])
    assert sorted(roots.real) == pytest.approx([-800.0, -800.0])
    assert sorted(abs(roots.imag)) == pytest.approx([600.0, 600.0])   # wn*sqrt(1-zeta^2)


def test_gain_design_formulas():
    g = design_current_gains(P.Rs, P.Ld, 200.0, 1.5)
    wn = 2 * math.pi * 200
    assert g.kp == pytest.approx(2 * 1.5 * wn * P.Ld - P.Rs)
    assert g.ki == pytest.approx(wn ** 2 * P.Ld)
    s = design_speed_gains(P, 25.0, 1.0)
    wn = 2 * math.pi * 25
    kt = torque_constant(P)
    assert s.kp == pytest.approx((2 * wn * P.J - P.B) / kt)
    assert s.ki == pytest.approx(wn ** 2 * P.J / kt)


def test_limit_dq_is_circular_with_d_priority():
    vmax = 27.0
    assert limit_dq(10.0, 10.0, vmax) == (10.0, 10.0)
    vd, vq = limit_dq(20.0, 50.0, vmax)
    assert vd == 20.0 and vq == pytest.approx(math.sqrt(vmax ** 2 - 20.0 ** 2))
    vd, vq = limit_dq(-100.0, 5.0, vmax)
    assert vd == -vmax and vq == 0.0
    for vd_in, vq_in in ((30, 30), (-40, 10), (5, -90), (0, 27.5)):
        assert math.hypot(*limit_dq(vd_in, vq_in, vmax)) <= vmax + 1e-12


def test_current_step_response_at_standstill():
    log = FocDrive(P_LOCKED).run(0.010, iq_ref=5.0)
    assert log["iq"][-1] == pytest.approx(5.0, abs=0.05)
    assert abs(log["id"][-1]) < 0.01
    assert log["iq"].max() < 5.0 * 1.12            # < 12 % overshoot incl. 1.5 Ts delay


def test_decoupling_reduces_cross_axis_error():
    def peak_id(decoupling):
        d = FocDrive(P_LOCKED, decoupling=decoupling)
        d.plant.x.wm = 40.0                         # we = 160 rad/s, speed locked
        log = d.run(0.010, iq_ref=10.0)
        assert log["iq"][-1] == pytest.approx(10.0, abs=0.1)
        return float(np.abs(log["id"]).max())
    on, off = peak_id(True), peak_id(False)
    assert on < 0.1
    assert off > 2.5 * on


def test_current_loop_anti_windup_recovers_after_saturation():
    def iq_after_release(anti_windup):
        d = FocDrive(P_LOCKED, anti_windup=anti_windup)
        d.run(0.050, iq_ref=100.0)                 # unreachable reference: voltage saturated
        return d.run(0.010, iq_ref=2.0)["iq"][-1]
    assert iq_after_release(True) == pytest.approx(2.0, abs=0.1)
    assert iq_after_release(False) > 20.0          # integrator wound up: still saturated 10 ms later


def test_speed_step_with_load_reaches_setpoint_and_respects_current_limit():
    d = FocDrive(P, i_max=10.0)
    log = d.run(0.4, speed_ref=30.0, t_load=1.0)
    assert log["wm"][-1] == pytest.approx(30.0, abs=0.05)
    assert log["iq"][-1] == pytest.approx((1.0 + P.B * 30.0) / torque_constant(P), abs=0.02)
    assert np.abs(log["iq_ref"]).max() <= 10.0 + 1e-9
    assert log["wm"].max() < 30.0 * 1.10
    assert np.abs(log["id"]).max() < 0.2


def test_speed_step_beyond_current_limit_saturates_then_converges():
    d = FocDrive(P, i_max=10.0)
    log = d.run(0.5, speed_ref=45.0)
    assert np.abs(log["iq_ref"]).max() == pytest.approx(10.0)    # limit was active
    assert log["wm"][-1] == pytest.approx(45.0, abs=0.05)
