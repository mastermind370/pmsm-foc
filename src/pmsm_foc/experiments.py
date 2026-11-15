"""The three standard experiments. Each returns (log, metrics, info) for plotting/reporting."""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from .drive import FocDrive
from .metrics import disturbance_metrics, step_metrics
from .params import PMSMParams

BAND = 0.02          # +/-2 % settling band


def _ms(v):
    return None if v is None else v * 1e3


def torque_step(i_step: float = 5.0, t_step: float = 1e-3, t_end: float = 15e-3):
    """iq* steps 0 -> i_step with the rotor locked (J -> inf): pure current-loop response."""
    drive = FocDrive(replace(PMSMParams(), J=1e12))
    log = drive.run(t_end, iq_ref=lambda t: i_step if t >= t_step - 1e-9 else 0.0)
    m = step_metrics(log["t"], log["iq"], i_step, t_step, y_init=0.0, band=BAND)
    metrics = {"rise_time_ms": _ms(m["rise_time_s"]), "overshoot_pct": m["overshoot_pct"],
               "settling_time_ms": _ms(m["settling_time_s"]),
               "steady_state_error": m["steady_state_error"], "unit": "A",
               "peak_abs_id_A": float(np.abs(log["id"]).max())}
    info = dict(signal="iq", ref=i_step, t_event=t_step, unit="A", band=BAND,
                title=f"Torque step: iq* 0 -> {i_step:g} A (rotor locked)")
    return log, metrics, info


def speed_step(w_ref: float = 30.0, t_step: float = 10e-3, t_end: float = 0.3, i_max: float = 10.0):
    """Speed reference steps 0 -> w_ref [rad/s], no load."""
    drive = FocDrive(i_max=i_max)
    log = drive.run(t_end, speed_ref=lambda t: w_ref if t >= t_step - 1e-9 else 0.0)
    m = step_metrics(log["t"], log["wm"], w_ref, t_step, y_init=0.0, band=BAND)
    vlim = drive.cc.vmax
    metrics = {"rise_time_ms": _ms(m["rise_time_s"]), "overshoot_pct": m["overshoot_pct"],
               "settling_time_ms": _ms(m["settling_time_s"]),
               "steady_state_error": m["steady_state_error"], "unit": "rad/s",
               "peak_iq_ref_A": float(np.abs(log["iq_ref"]).max()),
               "peak_voltage_V": float(np.hypot(log["vd"], log["vq"]).max()),
               "voltage_limit_V": float(vlim), "peak_abs_id_A": float(np.abs(log["id"]).max())}
    info = dict(signal="wm", ref=w_ref, t_event=t_step, unit="rad/s", band=BAND,
                title=f"Speed step: 0 -> {w_ref:g} rad/s ({w_ref * 60 / (2 * np.pi):.0f} rpm), no load")
    return log, metrics, info


def load_disturbance(w_ref: float = 30.0, t_load: float = 0.2, load: float = 2.0, t_end: float = 0.4):
    """Speed held at w_ref; a load torque step 0 -> `load` N m hits at t_load."""
    drive = FocDrive()
    log = drive.run(t_end, speed_ref=w_ref, t_load=lambda t: load if t >= t_load - 1e-9 else 0.0)
    m = disturbance_metrics(log["t"], log["wm"], w_ref, t_load, band=BAND)
    i0 = int(np.searchsorted(log["t"], t_load - 1e-9))
    pre = log["wm"][i0 - 1]
    metrics = {"peak_deviation": m["peak_deviation"], "peak_deviation_pct": m["peak_deviation_pct"],
               "time_of_peak_ms": _ms(m["time_of_peak_s"]),
               "recovery_time_ms": _ms(m["recovery_time_s"]),
               "steady_state_error": m["steady_state_error"], "unit": "rad/s",
               "speed_just_before_load": float(pre),
               "peak_iq_ref_after_load_A": float(np.abs(log["iq_ref"][i0:]).max())}
    info = dict(signal="wm", ref=w_ref, t_event=t_load, unit="rad/s", band=BAND,
                title=f"Load disturbance: {load:g} N m step at {t_load * 1e3:.0f} ms, speed ref {w_ref:g} rad/s")
    return log, metrics, info
