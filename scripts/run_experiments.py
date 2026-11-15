"""Run the three experiments; write plots and results/metrics.json (+ metrics.md)."""
import json
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pmsm_foc import PMSMParams
from pmsm_foc.control import design_current_gains, design_speed_gains
from pmsm_foc.experiments import BAND, load_disturbance, speed_step, torque_step

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results"
OUT.mkdir(exist_ok=True)


def plot_experiment(name, log, info, panels, xlim_ms=None):
    """panels: list of (y-label, [(log key, legend label), ...]), one subplot each."""
    t_ms = log["t"] * 1e3
    fig, axes = plt.subplots(len(panels), 1, figsize=(8, 2.6 * len(panels)), sharex=True)
    axes = np.atleast_1d(axes)
    for ax, (ylabel, series) in zip(axes, panels):
        for key, label in series:
            ax.plot(t_ms, log[key], label=label, lw=1.4)
        ax.axvline(info["t_event"] * 1e3, color="gray", ls=":", lw=1)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    ref, b = info["ref"], info["band"]
    axes[0].axhspan(ref * (1 - b), ref * (1 + b), color="green", alpha=0.15, lw=0)
    axes[0].set_title(info["title"] + f"   (green band = ±{b:.0%} of final value)", fontsize=9)
    if xlim_ms:
        axes[0].set_xlim(*xlim_ms)
    axes[-1].set_xlabel("time [ms]")
    fig.tight_layout()
    path = OUT / f"{name}.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fmt(v, nd=2):
    return "n/a" if v is None else f"{v:.{nd}f}"


def git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return None


def main():
    t0 = time.perf_counter()
    p = PMSMParams()
    runs = {}

    log, m, info = torque_step()
    plot_experiment("torque_step", log, info,
                    [("current [A]", [("iq_ref", "iq*"), ("iq", "iq")]),
                     ("current [A]", [("id", "id")]),
                     ("torque [N m]", [("te", "electromagnetic torque")])])
    runs["torque_step"] = (info["title"], m)

    log, m, info = speed_step()
    plot_experiment("speed_step", log, info,
                    [("speed [rad/s]", [("wm_ref", "speed ref"), ("wm", "speed")]),
                     ("current [A]", [("iq_ref", "iq*"), ("iq", "iq"), ("id", "id")]),
                     ("voltage [V]", [("vd", "vd (avg)"), ("vq", "vq (avg)")])],
                    xlim_ms=(0, 100))
    runs["speed_step"] = (info["title"], m)

    log, m, info = load_disturbance()
    plot_experiment("load_disturbance", log, info,
                    [("speed [rad/s]", [("wm_ref", "speed ref"), ("wm", "speed")]),
                     ("torque [N m]", [("tload", "load torque"), ("te", "motor torque")]),
                     ("current [A]", [("iq_ref", "iq*"), ("iq", "iq")])],
                    xlim_ms=(info["t_event"] * 1e3 - 20, info["t_event"] * 1e3 + 120))
    runs["load_disturbance"] = (info["title"], m)

    cg, sg = design_current_gains(p.Rs, p.Ld), design_speed_gains(p)
    report = {
        "meta": {
            "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "git_commit": git_commit(),
            "python": platform.python_version(), "numpy": np.__version__,
            "machine": platform.machine(), "wall_time_s": None,
            "plant_step_s": p.dt, "pwm_period_s": 1e-4, "speed_loop_period_s": 1e-3,
            "log_sample_s": 1e-4, "settling_band": BAND,
            "motor": {"Rs": p.Rs, "Ld": p.Ld, "Lq": p.Lq, "psi_f": p.psi_f,
                      "pole_pairs": p.pole_pairs, "J": p.J, "B": p.B, "Vdc": p.Vdc},
            "current_loop": {"bandwidth_hz": 200.0, "zeta": 1.5, "kp": cg.kp, "ki": cg.ki},
            "speed_loop": {"bandwidth_hz": 25.0, "zeta": 1.0, "kp": sg.kp, "ki": sg.ki,
                           "i_max_A": 10.0},
            "notes": "Ideal speed/current measurement; one-sample computation delay modeled; "
                     "times resolved to the 100 us logging period (linear interpolation).",
        },
        "experiments": {k: {"description": d, "metrics": m} for k, (d, m) in runs.items()},
    }
    report["meta"]["wall_time_s"] = round(time.perf_counter() - t0, 2)
    (OUT / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")

    tq, sp, ld = (runs[k][1] for k in ("torque_step", "speed_step", "load_disturbance"))
    rows = [
        "| Experiment | Signal | Rise 10-90 % [ms] | Overshoot [%] | Settling 2 % [ms] | Steady-state error |",
        "|---|---|---|---|---|---|",
        f"| Torque step (5 A, locked rotor) | iq | {fmt(tq['rise_time_ms'])} | {fmt(tq['overshoot_pct'], 1)} | "
        f"{fmt(tq['settling_time_ms'])} | {tq['steady_state_error']:.4f} A |",
        f"| Speed step (30 rad/s) | speed | {fmt(sp['rise_time_ms'], 1)} | {fmt(sp['overshoot_pct'], 1)} | "
        f"{fmt(sp['settling_time_ms'], 1)} | {sp['steady_state_error']:.4f} rad/s |",
        f"| Load step (2 N m) | speed | n/a | dip {fmt(abs(ld['peak_deviation']), 2)} rad/s "
        f"({fmt(ld['peak_deviation_pct'], 1)} %) | recovery {fmt(ld['recovery_time_ms'], 1)} | "
        f"{ld['steady_state_error']:.4f} rad/s |",
    ]
    (OUT / "metrics.md").write_text("\n".join(rows) + "\n")
    print("\n".join(rows))
    print(f"\nwrote {OUT / 'metrics.json'}, {OUT / 'metrics.md'} and 3 PNGs in {OUT} "
          f"({report['meta']['wall_time_s']} s)")


if __name__ == "__main__":
    sys.exit(main())
