"""Plot SVPWM duty cycles over one electrical revolution -> results/svpwm_duties.png."""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pmsm_foc import PMSMParams
from pmsm_foc.svpwm import svpwm_duties

p = PMSMParams()
vlin = p.Vdc / math.sqrt(3)
amp = 0.9 * vlin
deg = [k * 0.5 for k in range(721)]
duty = [svpwm_duties(amp * math.cos(math.radians(a)), amp * math.sin(math.radians(a)), p.Vdc)
        for a in deg]

fig, ax = plt.subplots(figsize=(8, 3.6))
for i, name in enumerate("abc"):
    ax.plot(deg, [d[i] for d in duty], label=f"duty {name}")
ax.set(xlabel="electrical angle [deg]", ylabel="duty cycle", xlim=(0, 360), ylim=(0, 1),
       title=f"SVPWM duties, |v| = {amp:.1f} V (90% of linear limit), Vdc = {p.Vdc:.0f} V")
ax.grid(alpha=0.3)
ax.legend(loc="upper right", ncol=3)
fig.tight_layout()
out = Path(__file__).resolve().parents[1] / "results" / "svpwm_duties.png"
out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=150)
flat = [d for tri in duty for d in tri]
print(f"linear limit Vdc/sqrt(3) = {vlin:.2f} V phase-peak; demo amplitude = {amp:.2f} V")
print(f"duty min = {min(flat):.4f}, max = {max(flat):.4f}")
print(f"saved {out}")
