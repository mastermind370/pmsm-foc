"""PI controllers tuned by pole placement, with back-calculation anti-windup.

Both loops are first-order plants G(s) = b / (s + a) under PI control C(s) = Kp + Ki/s.
Closed-loop characteristic polynomial:  s^2 + (a + b*Kp) s + b*Ki.
Matching it to s^2 + 2*zeta*wn*s + wn^2 gives
    Kp = (2*zeta*wn - a) / b          Ki = wn^2 / b
    current loop (per axis): a = R/L,  b = 1/L      (after decoupling feed-forward)
    speed loop             : a = B/J,  b = Kt/J     (Kt = 1.5*p*psi_f, current loop ~ ideal)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .params import PMSMParams
from .transforms import SQRT3


def pole_placement_pi(a: float, b: float, wn: float, zeta: float) -> tuple[float, float]:
    return (2.0 * zeta * wn - a) / b, wn * wn / b


@dataclass(frozen=True)
class PIGains:
    kp: float
    ki: float
    kaw: float      # back-calculation gain [1/s]; Tt = Ti = Kp/Ki  ->  kaw = Ki/Kp


def design_current_gains(R: float, L: float, f_bw_hz: float = 200.0, zeta: float = 1.5) -> PIGains:
    kp, ki = pole_placement_pi(R / L, 1.0 / L, 2.0 * math.pi * f_bw_hz, zeta)
    return PIGains(kp, ki, ki / kp)


def torque_constant(p: PMSMParams) -> float:
    return 1.5 * p.pole_pairs * p.psi_f


def design_speed_gains(p: PMSMParams, f_bw_hz: float = 25.0, zeta: float = 1.0) -> PIGains:
    kp, ki = pole_placement_pi(p.B / p.J, torque_constant(p) / p.J, 2.0 * math.pi * f_bw_hz, zeta)
    return PIGains(kp, ki, ki / kp)


def limit_dq(vd: float, vq: float, vmax: float) -> tuple[float, float]:
    """Circular voltage limit |v| <= vmax with d-axis priority."""
    vd_l = max(-vmax, min(vmax, vd))
    vq_max = math.sqrt(max(0.0, vmax * vmax - vd_l * vd_l))
    return vd_l, max(-vq_max, min(vq_max, vq))


class CurrentController:
    """Discrete PI (forward-Euler integrator) for id/iq with decoupling feed-forward."""

    def __init__(self, p: PMSMParams, Ts: float, f_bw_hz: float = 200.0, zeta: float = 1.5,
                 vmax: float | None = None, decoupling: bool = True, anti_windup: bool = True):
        self.p, self.Ts = p, Ts
        self.gd = design_current_gains(p.Rs, p.Ld, f_bw_hz, zeta)
        self.gq = design_current_gains(p.Rs, p.Lq, f_bw_hz, zeta)
        self.vmax = p.Vdc / SQRT3 if vmax is None else vmax
        self.decoupling, self.anti_windup = decoupling, anti_windup
        self.Id = 0.0
        self.Iq = 0.0

    def update(self, id_ref: float, iq_ref: float, id_: float, iq: float, we: float):
        p, Ts = self.p, self.Ts
        ed, eq = id_ref - id_, iq_ref - iq
        ffd = -we * p.Lq * iq if self.decoupling else 0.0
        ffq = we * (p.Ld * id_ + p.psi_f) if self.decoupling else 0.0
        ud = self.gd.kp * ed + self.Id + ffd
        uq = self.gq.kp * eq + self.Iq + ffq
        vd, vq = limit_dq(ud, uq, self.vmax)
        kawd = self.gd.kaw if self.anti_windup else 0.0
        kawq = self.gq.kaw if self.anti_windup else 0.0
        self.Id += Ts * (self.gd.ki * ed + kawd * (vd - ud))
        self.Iq += Ts * (self.gq.ki * eq + kawq * (vq - uq))
        return vd, vq


class SpeedController:
    """Outer PI: speed error -> iq reference (torque command), limited to +/- i_max."""

    def __init__(self, p: PMSMParams, Ts: float, i_max: float, f_bw_hz: float = 25.0,
                 zeta: float = 1.0, anti_windup: bool = True):
        self.Ts, self.i_max, self.anti_windup = Ts, i_max, anti_windup
        self.g = design_speed_gains(p, f_bw_hz, zeta)
        self.I = 0.0

    def update(self, w_ref: float, w: float) -> float:
        e = w_ref - w
        u = self.g.kp * e + self.I
        u_sat = max(-self.i_max, min(self.i_max, u))
        kaw = self.g.kaw if self.anti_windup else 0.0
        self.I += self.Ts * (self.g.ki * e + kaw * (u_sat - u))
        return u_sat
