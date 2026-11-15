from dataclasses import replace

import pytest

from pmsm_foc import PMSM, PMSMParams, PMSMState, electromagnetic_torque

P = PMSMParams()
# Effectively infinite inertia -> speed stays locked, isolating the electrical dynamics.
P_LOCKED = replace(P, J=1e12)


def test_torque_surface_pm():
    # Te = 1.5 * p * psi_f * iq = 1.5 * 4 * 0.1 * 10 = 6.0 N m
    assert electromagnetic_torque(P, 0.0, 10.0) == pytest.approx(6.0)


def test_torque_with_reluctance_term():
    p = replace(P, Ld=1.0e-3, Lq=2.0e-3)
    # 1.5*4*(0.1*10 + (1e-3-2e-3)*(-5)*10) = 6*(1.0 + 0.05) = 6.3 N m
    assert electromagnetic_torque(p, -5.0, 10.0) == pytest.approx(6.3)


def test_back_emf_and_mechanical_derivatives():
    m = PMSM(P)
    wm = 100.0
    we = 4 * wm
    did, diq, dwm, dth = m.derivs(0.0, 0.0, wm, 0.0, 0.0, 0.0)
    assert dth == pytest.approx(we)
    assert did == pytest.approx(0.0)
    assert diq == pytest.approx(-we * P.psi_f / P.Lq)       # back-EMF e = we*psi_f = 40 V
    _, _, dwm, _ = m.derivs(0.0, 10.0, 10.0, 0.0, 0.0, 2.0)
    assert dwm == pytest.approx((6.0 - P.B * 10.0 - 2.0) / P.J)


def test_rl_step_response_matches_analytic():
    # wm = 0: no back-EMF/coupling -> iq(t) = (vq/Rs)(1 - exp(-t/tau)), tau = Lq/Rs = 2.8 ms
    m = PMSM(P_LOCKED, PMSMState())
    for _ in range(2800):                                    # t = tau
        m.step(0.0, 2.5)
    assert m.x.iq == pytest.approx(5.0 * (1 - 2.718281828459045 ** -1), rel=1e-6)
    assert m.x.id == pytest.approx(0.0, abs=1e-9)


def test_steady_state_with_back_emf():
    # wm = 50 rad/s -> we = 200 rad/s, e = we*psi_f = 20 V.
    # Target id=0, iq=5 A: vq = Rs*iq + e = 22.5 V, vd = -we*Lq*iq = -1.4 V.
    m = PMSM(P_LOCKED, PMSMState(wm=50.0))
    for _ in range(50_000):                                  # 50 ms >> 2.8 ms
        m.step(-1.4, 22.5)
    assert m.x.iq == pytest.approx(5.0, abs=1e-6)
    assert m.x.id == pytest.approx(0.0, abs=1e-6)
    assert m.torque() == pytest.approx(1.5 * 4 * 0.1 * 5.0, abs=1e-5)
