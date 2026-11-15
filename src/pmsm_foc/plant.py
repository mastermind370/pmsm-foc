"""PMSM d-q plant in the rotor reference frame, integrated with fixed-step RK4.

Conventions (motor convention, amplitude-invariant transforms, rotor frame):
    Ld did/dt = vd - Rs id + we Lq iq
    Lq diq/dt = vq - Rs iq - we Ld id - we psi_f
    Te        = 1.5 p [psi_f iq + (Ld - Lq) id iq]
    J dwm/dt  = Te - B wm - T_load
    dtheta_e/dt = we = p wm
Inputs vd, vq, T_load are held constant (zero-order hold) across each step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .params import PMSMParams

TWO_PI = 2.0 * math.pi


@dataclass
class PMSMState:
    id: float = 0.0        # d-axis current [A]
    iq: float = 0.0        # q-axis current [A]
    wm: float = 0.0        # mechanical speed [rad/s]
    theta_e: float = 0.0   # electrical rotor angle [rad], wrapped to [0, 2pi)


def electromagnetic_torque(p: PMSMParams, id_: float, iq: float) -> float:
    return 1.5 * p.pole_pairs * (p.psi_f * iq + (p.Ld - p.Lq) * id_ * iq)


class PMSM:
    def __init__(self, params: PMSMParams | None = None, state: PMSMState | None = None):
        self.p = params or PMSMParams()
        self.x = state or PMSMState()
        self.t = 0.0

    def torque(self) -> float:
        return electromagnetic_torque(self.p, self.x.id, self.x.iq)

    def derivs(self, id_, iq, wm, vd, vq, t_load):
        """Returns (did/dt, diq/dt, dwm/dt, dtheta_e/dt)."""
        p = self.p
        we = p.pole_pairs * wm
        did = (vd - p.Rs * id_ + we * p.Lq * iq) / p.Ld
        diq = (vq - p.Rs * iq - we * p.Ld * id_ - we * p.psi_f) / p.Lq
        te = 1.5 * p.pole_pairs * (p.psi_f * iq + (p.Ld - p.Lq) * id_ * iq)
        dwm = (te - p.B * wm - t_load) / p.J
        return did, diq, dwm, we

    def step(self, vd: float, vq: float, t_load: float = 0.0) -> PMSMState:
        h = self.p.dt
        h2, h6 = 0.5 * h, h / 6.0
        x = self.x
        f = self.derivs
        a1, b1, c1, d1 = f(x.id, x.iq, x.wm, vd, vq, t_load)
        a2, b2, c2, d2 = f(x.id + h2 * a1, x.iq + h2 * b1, x.wm + h2 * c1, vd, vq, t_load)
        a3, b3, c3, d3 = f(x.id + h2 * a2, x.iq + h2 * b2, x.wm + h2 * c2, vd, vq, t_load)
        a4, b4, c4, d4 = f(x.id + h * a3, x.iq + h * b3, x.wm + h * c3, vd, vq, t_load)
        x.id += h6 * (a1 + 2 * a2 + 2 * a3 + a4)
        x.iq += h6 * (b1 + 2 * b2 + 2 * b3 + b4)
        x.wm += h6 * (c1 + 2 * c2 + 2 * c3 + c4)
        x.theta_e = (x.theta_e + h6 * (d1 + 2 * d2 + 2 * d3 + d4)) % TWO_PI
        self.t += h
        return x
