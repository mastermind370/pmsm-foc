"""Averaged two-level inverter (dead-time ignored) coupled to the 1 us PMSM plant.

Each PWM period (Ts = 1/f_sw = 100 us) the controller's dq voltage command is turned
into alpha/beta, limited, converted to duties, and held (zero-order hold) for the whole
period. The *stator-frame* voltage is constant while the rotor keeps turning, so the
plant sees vd/vq re-projected with the current rotor angle at every 1 us step.

angle_comp=True builds the command with theta + 0.5*we*Ts (rotor angle at mid-period),
which makes the period-average of the applied dq voltage match the command.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .plant import PMSM
from .svpwm import limit_voltage, svpwm_duties
from .transforms import clarke, inv_park


@dataclass
class PeriodResult:
    duties: tuple[float, float, float]
    vd_avg: float          # mean applied d voltage over the period [V]
    vq_avg: float          # mean applied q voltage over the period [V]
    saturated: bool        # True if the command was clipped to Vdc/sqrt(3)


class AveragedInverter:
    def __init__(self, vdc: float = 48.0, f_sw: float = 10_000.0):
        self.vdc = vdc
        self.f_sw = f_sw
        self.Ts = 1.0 / f_sw

    def realized_alpha_beta(self, duties) -> tuple[float, float]:
        """Averaged pole voltages -> phase voltages (clarke drops the common mode)."""
        vao, vbo, vco = ((d - 0.5) * self.vdc for d in duties)
        return clarke(vao, vbo, vco)

    def run_period(self, plant: PMSM, vd_ref: float, vq_ref: float,
                   t_load: float = 0.0, angle_comp: bool = True) -> PeriodResult:
        x = plant.x
        we = plant.p.pole_pairs * x.wm
        theta_cmd = x.theta_e + (0.5 * we * self.Ts if angle_comp else 0.0)
        v_a, v_b = inv_park(vd_ref, vq_ref, theta_cmd)
        v_a, v_b, sat = limit_voltage(v_a, v_b, self.vdc)
        duties = svpwm_duties(v_a, v_b, self.vdc)
        v_a, v_b = self.realized_alpha_beta(duties)

        n_sub = max(1, round(self.Ts / plant.p.dt))
        sum_d = sum_q = 0.0
        for _ in range(n_sub):
            c, s = math.cos(x.theta_e), math.sin(x.theta_e)
            vd = v_a * c + v_b * s
            vq = -v_a * s + v_b * c
            plant.step(vd, vq, t_load)
            sum_d += vd
            sum_q += vq
        return PeriodResult(duties, sum_d / n_sub, sum_q / n_sub, sat)
