"""Current-loop step response (iq 0 -> 5 A, rotor locked) vs bandwidth/damping,
with the 1.5*Ts computation+ZOH delay included. Prints a table you can measure yourself."""
from dataclasses import replace

import numpy as np

from pmsm_foc import FocDrive, PMSMParams

locked = replace(PMSMParams(), J=1e12)
REF = 5.0
print(f"{'f_bw[Hz]':>8} {'zeta':>5} {'overshoot[%]':>13} {'rise10-90[us]':>14} {'settle2%[us]':>13}")
for f_bw in (150, 200, 300, 500):
    for zeta in (1.0, 1.5):
        log = FocDrive(locked, f_bw_current=f_bw, zeta_i=zeta).run(0.012, iq_ref=REF)
        iq, t = log["iq"], log["t"]
        over = max(0.0, (iq.max() - REF) / REF * 100)
        rise = t[np.argmax(iq >= 0.9 * REF)] - t[np.argmax(iq >= 0.1 * REF)]
        out = np.where(np.abs(iq - REF) > 0.02 * REF)[0]
        settle = t[out[-1]] if len(out) else 0.0
        print(f"{f_bw:8d} {zeta:5.2f} {over:13.1f} {rise * 1e6:14.0f} {settle * 1e6:13.0f}")
