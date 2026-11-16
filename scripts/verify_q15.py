"""Stage 6: replay identical input vectors through the Python float controller and the C++ Q15
controller; report max / RMS error and time one control-loop iteration.

Vectors are recorded from closed-loop Python runs (the controller inputs at every 100 us period),
then replayed open-loop through both controllers from zero state.

Two comparisons, so the error sources can be told apart:
  A  Python float fed the *dequantized* Q15 inputs   -> controller error only
       (gain quantization + fixed-point rounding/saturation)
  B  Python float fed the original float inputs       -> end-to-end error
       (A plus the quantization of the 5 input signals to Q15)
Errors are output-voltage errors in Q15 LSB (1 LSB = V_base / 32768 = 0.846 mV).

Run:  PYTHONPATH=src python scripts/verify_q15.py
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pmsm_foc import FocDrive, PMSMParams
from pmsm_foc.control import CurrentController

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "verify"
OUT = ROOT / "results"

P = PMSMParams()
TS = 1e-4
I_BASE, W_BASE = 20.0, 400.0            # must match cpp/include/default_config.hpp
V_BASE = P.Vdc / np.sqrt(3.0)
Q = 32768.0
LSB_V = V_BASE / Q                      # volts per output LSB
IN_SCALE = np.array([I_BASE, I_BASE, I_BASE, I_BASE, W_BASE])   # id_ref iq_ref id iq we


# ----------------------------------------------------------------------------- C++ tools
def find_cxx() -> str:
    for c in (os.environ.get("CXX"), "clang++", "g++", "c++"):
        if c and shutil.which(c):
            return shutil.which(c)
    raise RuntimeError("no C++ compiler found (install Xcode command line tools)")


def build_tools() -> dict:
    BUILD.mkdir(parents=True, exist_ok=True)
    cxx = find_cxx()
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra"]
    exes = {}
    for name in ("replay_q15", "bench_q15"):
        exe = BUILD / name
        cmd = [cxx, *flags, "-I", str(ROOT / "cpp" / "include"),
               str(ROOT / "cpp" / "src" / "current_controller_q15.cpp"),
               str(ROOT / "cpp" / "tools" / f"{name}.cpp"), "-o", str(exe)]
        subprocess.run(cmd, check=True)
        exes[name] = exe
    version = subprocess.run([cxx, "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    exes["compiler"] = version
    exes["flags"] = " ".join(flags)
    return exes


# ----------------------------------------------------------------------------- quantization
def to_raw(phys: np.ndarray) -> np.ndarray:
    """Physical (A, A, A, A, rad/s) -> raw int16 Q15, round to nearest, saturating."""
    return np.clip(np.rint(phys / IN_SCALE * Q), -32768, 32767).astype(np.int64)


def from_raw(raw: np.ndarray) -> np.ndarray:
    return raw.astype(float) * IN_SCALE / Q


# ----------------------------------------------------------------------------- vectors
def record(drive: FocDrive, **run_kwargs) -> np.ndarray:
    """Run `drive` and return the controller inputs (id_ref, iq_ref, id, iq, we) per period."""
    rows = []
    orig = drive.cc.update

    def spy(*args):
        rows.append(args)
        return orig(*args)

    drive.cc.update = spy
    drive.run(**run_kwargs)
    return np.array(rows, dtype=float)


def square(amp: float, period: float):
    return lambda t: amp if int(t / period) % 2 == 0 else -amp


def make_vectors(scale: float = 1.0, names=None) -> dict:
    """Closed-loop recordings. `scale` shortens every run (used by the unit test)."""
    sc = {
        "torque_step_locked": lambda: record(
            FocDrive(replace(P, J=1e12)), t_end=0.2 * scale, iq_ref=square(5.0, 0.02)),
        "speed_step_30": lambda: record(
            FocDrive(), t_end=0.3 * scale,
            speed_ref=lambda t: 30.0 if t >= 10e-3 - 1e-9 else 0.0),
        "load_step_2Nm": lambda: record(
            FocDrive(), t_end=0.4 * scale, speed_ref=30.0,
            t_load=lambda t: 2.0 if t >= 0.2 - 1e-9 else 0.0),
        "voltage_limited_reversal": lambda: record(
            FocDrive(), t_end=1.0 * scale, speed_ref=square(60.0, 0.25)),
    }
    return {k: f() for k, f in sc.items() if names is None or k in names}


# ----------------------------------------------------------------------------- replays
def run_cpp(exe: Path, raw: np.ndarray, tag: str):
    BUILD.mkdir(parents=True, exist_ok=True)
    fin, fout = BUILD / f"{tag}_in.txt", BUILD / f"{tag}_out.txt"
    np.savetxt(fin, raw, fmt="%d")
    res = subprocess.run([str(exe), str(fin), str(fout)], check=True, capture_output=True, text=True)
    out = np.loadtxt(fout, dtype=np.int64, ndmin=2)
    return out[:, :2].astype(float), out[:, 2].astype(bool), res.stderr


def run_python(phys: np.ndarray, **ctl_kwargs) -> np.ndarray:
    """Stage 3 float controller (defaults) -> output voltage in Q15 LSB (float, not rounded)."""
    ctl = CurrentController(P, TS, **ctl_kwargs)
    v = np.empty((len(phys), 2))
    for k, row in enumerate(phys):
        v[k] = ctl.update(*row)
    return v / V_BASE * Q


# ----------------------------------------------------------------------------- statistics
def stats(e: np.ndarray):
    if e.size == 0:
        return None
    mx, rms = float(np.abs(e).max()), float(np.sqrt(np.mean(e ** 2)))
    return {"n_steps": int(e.shape[0]), "max_lsb": mx, "rms_lsb": rms,
            "max_mV": mx * LSB_V * 1e3, "rms_mV": rms * LSB_V * 1e3,
            "max_pct_full_scale": mx / Q * 100.0}


def verify_scenarios(exes: dict, scale: float = 1.0, names=None) -> dict:
    vectors = make_vectors(scale, names)
    res, all_a, all_b, all_lim = {}, [], [], []
    gains_line = ""
    for name, phys in vectors.items():
        raw = to_raw(phys)
        clipped = int(np.count_nonzero(np.abs(phys / IN_SCALE * Q) > 32767.5))
        v_cpp, limited, stderr = run_cpp(exes["replay_q15"], raw, name)
        gains_line = stderr.splitlines()[0]
        v_a = run_python(from_raw(raw))         # A: same numbers the C++ saw
        v_b = run_python(phys)                  # B: original float inputs
        err_a, err_b = v_cpp - v_a, v_cpp - v_b
        res[name] = {
            "steps": int(len(raw)), "limited_steps": int(limited.sum()),
            "limited_pct": float(limited.mean() * 100.0), "clipped_input_samples": clipped,
            "A_all": stats(err_a), "A_linear": stats(err_a[~limited]),
            "A_limited": stats(err_a[limited]), "B_all": stats(err_b),
            "_err_a": err_a, "_limited": limited, "_raw": raw,
        }
        all_a.append(err_a); all_b.append(err_b); all_lim.append(limited)
    ea, eb, lim = np.vstack(all_a), np.vstack(all_b), np.concatenate(all_lim)
    res["_aggregate"] = {"A_all": stats(ea), "A_linear": stats(ea[~lim]),
                         "A_limited": stats(ea[lim]), "B_all": stats(eb),
                         "steps": int(len(lim)), "limited_steps": int(lim.sum())}
    res["_gains"] = gains_line
    return res


def time_one_iteration(exes: dict, raw: np.ndarray, iters: int, reps: int) -> dict:
    fin = BUILD / "bench_in.txt"
    np.savetxt(fin, raw, fmt="%d")
    out = subprocess.run([str(exes["bench_q15"]), str(fin), str(iters), str(reps)],
                         check=True, capture_output=True, text=True).stdout
    return json.loads(out)


# ----------------------------------------------------------------------------- reporting
def plot_errors(res: dict):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [k for k in res if not k.startswith("_")]
    fig, axes = plt.subplots(len(names), 1, figsize=(8, 2.1 * len(names)), sharex=False)
    for ax, name in zip(np.atleast_1d(axes), names):
        e = np.abs(res[name]["_err_a"]).max(axis=1)
        lim = res[name]["_limited"]
        t_ms = np.arange(len(e)) * TS * 1e3
        ax.plot(t_ms, e, lw=0.8, label="|error| (max of vd, vq)")
        ax.plot(t_ms[lim], e[lim], ".", ms=2.5, color="tab:red", label="voltage limiter active")
        ax.set_ylabel("LSB")
        ax.set_title(name, fontsize=9, loc="left")
        ax.grid(alpha=0.3)
    np.atleast_1d(axes)[0].legend(loc="upper right", fontsize=7)
    np.atleast_1d(axes)[-1].set_xlabel("time [ms]")
    fig.suptitle("Q15 vs Python float controller, output-voltage error (same inputs)", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "q15_error.png", dpi=150)
    plt.close(fig)


def fmt(s, key):
    return "n/a" if s is None else f"{s[key]:.2f}"


def write_reports(res: dict, timing: dict, exes: dict):
    names = [k for k in res if not k.startswith("_")]
    clean = {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")}
             for k, v in res.items() if not k.startswith("_")}
    cpu = ""
    if platform.system() == "Darwin":
        try:
            cpu = subprocess.run(["sysctl", "-n", "machdep.cpu.brand_string"], capture_output=True,
                                 text=True).stdout.strip()
        except Exception:
            pass
    try:
        commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                                         stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        commit = None
    report = {
        "meta": {
            "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "git_commit": commit, "python": platform.python_version(), "numpy": np.__version__,
            "machine": platform.machine(), "cpu": cpu, "compiler": exes["compiler"],
            "compile_flags": exes["flags"], "q15_gains_raw": res["_gains"],
            "lsb_volts": float(LSB_V), "bases": {"I_A": I_BASE, "V_V": float(V_BASE), "W_rad_s": W_BASE},
            "comparison_A": "Python float on dequantized Q15 inputs (controller error only)",
            "comparison_B": "Python float on original float inputs (includes input quantization)",
            "note": "Python limits at exactly 1.0 pu (32768 LSB), Q15 tops out at 32767: up to "
                    "1 LSB of error on limited steps is full-scale representation, not arithmetic.",
        },
        "scenarios": clean, "aggregate": res["_aggregate"], "timing_one_iteration_ns": timing,
    }
    (OUT / "verification.json").write_text(json.dumps(report, indent=2) + "\n")

    rows = ["| Scenario | Steps | Limiter active | Max err [LSB] | RMS err [LSB] | Linear-only max [LSB] | Linear-only RMS [LSB] |",
            "|---|---|---|---|---|---|---|"]
    for n in names:
        r = res[n]
        rows.append(f"| {n} | {r['steps']} | {r['limited_pct']:.1f} % | {fmt(r['A_all'], 'max_lsb')} | "
                    f"{fmt(r['A_all'], 'rms_lsb')} | {fmt(r['A_linear'], 'max_lsb')} | {fmt(r['A_linear'], 'rms_lsb')} |")
    a = res["_aggregate"]
    rows.append(f"| **all** | {a['steps']} | {100.0 * a['limited_steps'] / a['steps']:.1f} % | "
                f"{fmt(a['A_all'], 'max_lsb')} | {fmt(a['A_all'], 'rms_lsb')} | "
                f"{fmt(a['A_linear'], 'max_lsb')} | {fmt(a['A_linear'], 'rms_lsb')} |")
    md = "\n".join(rows) + "\n"
    md += (f"\nLimited steps only: max {fmt(a['A_limited'], 'max_lsb')} LSB, RMS {fmt(a['A_limited'], 'rms_lsb')} LSB. "
           f"End-to-end (comparison B, all steps): max {fmt(a['B_all'], 'max_lsb')} LSB, RMS {fmt(a['B_all'], 'rms_lsb')} LSB.\n"
           f"1 LSB = {LSB_V * 1e3:.3f} mV. Time per control iteration: min {timing['ns_per_iter_min']:.1f} / "
           f"median {timing['ns_per_iter_median']:.1f} / max {timing['ns_per_iter_max']:.1f} ns "
           f"({cpu or platform.machine()}, {exes['compiler']}, {exes['flags']}).\n")
    (OUT / "verification.md").write_text(md)
    return md


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=1_000_000, help="iterations per timing rep")
    ap.add_argument("--reps", type=int, default=15, help="timing reps")
    args = ap.parse_args()

    OUT.mkdir(exist_ok=True)
    exes = build_tools()
    res = verify_scenarios(exes)
    timing = time_one_iteration(exes, res["speed_step_30"]["_raw"], args.iters, args.reps)
    plot_errors(res)
    print(write_reports(res, timing, exes))
    print(f"wrote {OUT / 'verification.json'}, {OUT / 'verification.md'}, {OUT / 'q15_error.png'}")


if __name__ == "__main__":
    sys.exit(main())
