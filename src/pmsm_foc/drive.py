"""Closed-loop FOC drive: plant + averaged inverter + current loop (10 kHz) + speed loop (1 kHz).

Timing per 100 us PWM period: sample id/iq/wm/theta -> run controllers -> the new voltage
command is applied in the NEXT period (one-sample computation delay, delay=True).
"""
from __future__ import annotations

import numpy as np

from .control import CurrentController, SpeedController
from .inverter import AveragedInverter
from .params import PMSMParams
from .plant import PMSM


def _val(x, t):
    return x(t) if callable(x) else x


class FocDrive:
    def __init__(self, params: PMSMParams | None = None, i_max: float = 10.0,
                 f_sw: float = 10_000.0, f_speed: float = 1_000.0,
                 f_bw_current: float = 200.0, zeta_i: float = 1.5,
                 f_bw_speed: float = 25.0, zeta_w: float = 1.0,
                 decoupling: bool = True, anti_windup: bool = True, delay: bool = True):
        self.p = params or PMSMParams()
        self.plant = PMSM(self.p)
        self.inv = AveragedInverter(self.p.Vdc, f_sw)
        self.Ts = 1.0 / f_sw
        self.speed_div = round(f_sw / f_speed)
        self.cc = CurrentController(self.p, self.Ts, f_bw_current, zeta_i,
                                    decoupling=decoupling, anti_windup=anti_windup)
        self.sc = SpeedController(self.p, 1.0 / f_speed, i_max, f_bw_speed, zeta_w, anti_windup)
        self.delay = delay
        self.iq_ref = 0.0
        self._pending = (0.0, 0.0)
        self._k = 0

    def step(self, speed_ref=None, id_ref: float = 0.0, iq_ref: float = 0.0, t_load: float = 0.0):
        """One PWM period. speed_ref=None -> torque mode (iq_ref used directly, NOT limited)."""
        x = self.plant.x
        if speed_ref is None:
            self.iq_ref = iq_ref
        elif self._k % self.speed_div == 0:
            self.iq_ref = self.sc.update(speed_ref, x.wm)
        v_new = self.cc.update(id_ref, self.iq_ref, x.id, x.iq, self.p.pole_pairs * x.wm)
        v_apply = self._pending if self.delay else v_new
        self._pending = v_new
        res = self.inv.run_period(self.plant, v_apply[0], v_apply[1], t_load)
        self._k += 1
        return res

    def run(self, t_end: float, speed_ref=None, id_ref=0.0, iq_ref=0.0, t_load=0.0):
        """Simulate t_end seconds; each input may be a float or a function of time [s].
        Returns numpy arrays logged once per PWM period (values at the end of the period)."""
        n = round(t_end / self.Ts)
        keys = ("t", "id", "iq", "wm", "te", "iq_ref", "id_ref", "wm_ref", "vd", "vq", "tload")
        log = {k: np.empty(n) for k in keys}
        x = self.plant.x
        for i in range(n):
            t = self.plant.t
            sr, ir, qr, tl = _val(speed_ref, t), _val(id_ref, t), _val(iq_ref, t), _val(t_load, t)
            res = self.step(sr, ir, qr, tl)
            log["t"][i] = self.plant.t
            log["id"][i], log["iq"][i], log["wm"][i] = x.id, x.iq, x.wm
            log["te"][i] = self.plant.torque()
            log["iq_ref"][i], log["id_ref"][i] = self.iq_ref, ir
            log["wm_ref"][i] = np.nan if sr is None else sr
            log["vd"][i], log["vq"][i], log["tload"][i] = res.vd_avg, res.vq_avg, tl
        return log
