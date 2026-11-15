import math
from dataclasses import replace

import pytest

from pmsm_foc import AveragedInverter, PMSM, PMSMParams, PMSMState
from pmsm_foc.svpwm import limit_voltage, svpwm_duties
from pmsm_foc.transforms import clarke, inv_clarke, inv_park, park

P = PMSMParams()
P_LOCKED = replace(P, J=1e12)   # speed locked: isolates the electrical/PWM path
ANGLES = [2 * math.pi * k / 360 for k in range(360)]
TWO_THIRDS_PI = 2 * math.pi / 3


def test_clarke_is_amplitude_invariant():
    A = 12.0
    for phi in ANGLES:
        al, be = clarke(A * math.cos(phi), A * math.cos(phi - TWO_THIRDS_PI),
                        A * math.cos(phi + TWO_THIRDS_PI))
        assert al == pytest.approx(A * math.cos(phi), abs=1e-12)
        assert be == pytest.approx(A * math.sin(phi), abs=1e-12)


def test_clarke_round_trip():
    a, b, c = inv_clarke(3.0, -4.0)
    assert a + b + c == pytest.approx(0.0, abs=1e-12)
    assert clarke(a, b, c) == pytest.approx((3.0, -4.0), abs=1e-12)


def test_park_gives_constant_dq_for_rotating_vector():
    A = 7.0
    for th in ANGLES:
        d, q = park(A * math.cos(th), A * math.sin(th), th)
        assert d == pytest.approx(A, abs=1e-12)
        assert q == pytest.approx(0.0, abs=1e-12)


def test_park_round_trip():
    al, be = inv_park(2.0, -5.0, 1.234)
    assert park(al, be, 1.234) == pytest.approx((2.0, -5.0), abs=1e-12)


def test_svpwm_reproduces_reference_and_is_centered():
    inv = AveragedInverter(P.Vdc)
    vmax = P.Vdc / math.sqrt(3)
    for mag in (0.0, 10.0, 0.5 * vmax, 0.9999 * vmax):
        for th in ANGLES:
            va, vb = mag * math.cos(th), mag * math.sin(th)
            duties = svpwm_duties(va, vb, P.Vdc)
            assert all(0.0 <= d <= 1.0 for d in duties)
            assert max(duties) + min(duties) == pytest.approx(1.0, abs=1e-12)  # centered
            assert inv.realized_alpha_beta(duties) == pytest.approx((va, vb), abs=1e-9)


def test_voltage_limit_clips_to_linear_range_and_keeps_direction():
    va, vb, sat = limit_voltage(60.0, 80.0, P.Vdc)             # |v| = 100 V
    assert sat
    assert math.hypot(va, vb) == pytest.approx(P.Vdc / math.sqrt(3))
    assert math.atan2(vb, va) == pytest.approx(math.atan2(80.0, 60.0))
    assert limit_voltage(1.0, 1.0, P.Vdc)[2] is False


def test_pwm_chain_matches_direct_dq_at_standstill():
    # wm = 0, vq = 2.5 V -> iq(tau) = 5*(1 - exp(-1)) with tau = Lq/Rs = 2.8 ms = 28 PWM periods
    m = PMSM(P_LOCKED, PMSMState())
    inv = AveragedInverter(P.Vdc)
    for _ in range(28):
        inv.run_period(m, 0.0, 2.5)
    assert m.x.iq == pytest.approx(5.0 * (1 - math.exp(-1)), rel=1e-6)
    assert m.x.id == pytest.approx(0.0, abs=1e-9)


def test_angle_compensation_fixes_zoh_rotation_error():
    # we = 200 rad/s -> rotor turns 0.02 rad per 100 us PWM period.
    def mean_error(angle_comp):
        m = PMSM(P_LOCKED, PMSMState(wm=50.0, theta_e=0.7))
        r = AveragedInverter(P.Vdc).run_period(m, -1.4, 22.5, angle_comp=angle_comp)
        assert not r.saturated
        return math.hypot(r.vd_avg + 1.4, r.vq_avg - 22.5)
    assert mean_error(True) < 5e-3     # residual ~2e-3 V: angle sampled at 1 us step starts
    assert mean_error(False) > 0.15    # ~0.22 V error without compensation
