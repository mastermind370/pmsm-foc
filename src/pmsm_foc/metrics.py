"""Step-response and disturbance metrics computed from sampled time series."""
from __future__ import annotations

import numpy as np


def _first_crossing(t, yn, level):
    idx = np.nonzero(yn >= level)[0]
    if idx.size == 0:
        return None
    i = int(idx[0])
    if i == 0:
        return float(t[0])
    t0, t1, y0, y1 = t[i - 1], t[i], yn[i - 1], yn[i]
    return float(t0 + (level - y0) * (t1 - t0) / (y1 - y0))


def step_metrics(t, y, y_ref, t_step, y_init=None, band=0.02, tail_frac=0.05):
    """Metrics for a step from y_init to y_ref applied at t_step.

    rise_time      10 %-90 % of the step, linearly interpolated between samples [s]
    overshoot_pct  max(0, peak - y_ref) / |step| * 100
    settling_time  time after t_step of the last sample outside +/-band*|step| [s]
                   (None if the response never settles inside the record)
    sse            y_ref - mean(y over the last tail_frac of the record)
    """
    t, y = np.asarray(t, float), np.asarray(y, float)
    m = t >= t_step - 1e-9
    ts, ys = t[m], y[m]
    y0 = float(ys[0]) if y_init is None else float(y_init)
    delta = y_ref - y0
    yn = (ys - y0) / delta
    t10, t90 = _first_crossing(ts, yn, 0.1), _first_crossing(ts, yn, 0.9)
    rise = None if (t10 is None or t90 is None) else t90 - t10
    overshoot = max(0.0, float(yn.max()) - 1.0) * 100.0
    outside = np.nonzero(np.abs(yn - 1.0) > band)[0]
    if outside.size == 0:
        settle = 0.0
    elif outside[-1] == len(yn) - 1:
        settle = None
    else:
        settle = float(ts[outside[-1] + 1] - t_step)
    n_tail = max(1, int(tail_frac * len(ys)))
    sse = float(y_ref - ys[-n_tail:].mean())
    return {"rise_time_s": rise, "overshoot_pct": overshoot, "settling_time_s": settle,
            "steady_state_error": sse}


def disturbance_metrics(t, y, y_ref, t_dist, band=0.02, tail_frac=0.05):
    """Regulation metrics after a disturbance at t_dist (output was at y_ref before it).

    peak_deviation recovery_time (last sample outside +/-band*|y_ref| after t_dist) and sse.
    """
    t, y = np.asarray(t, float), np.asarray(y, float)
    m = t >= t_dist - 1e-9
    ts, ys = t[m], y[m]
    dev = ys - y_ref
    k = int(np.argmax(np.abs(dev)))
    outside = np.nonzero(np.abs(dev) > band * abs(y_ref))[0]
    if outside.size == 0:
        rec = 0.0
    elif outside[-1] == len(ys) - 1:
        rec = None
    else:
        rec = float(ts[outside[-1] + 1] - t_dist)
    n_tail = max(1, int(tail_frac * len(ys)))
    return {"peak_deviation": float(dev[k]),
            "peak_deviation_pct": float(abs(dev[k]) / abs(y_ref) * 100.0),
            "time_of_peak_s": float(ts[k] - t_dist),
            "recovery_time_s": rec,
            "steady_state_error": float(y_ref - ys[-n_tail:].mean())}
