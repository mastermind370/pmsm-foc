"""SVPWM via min-max common-mode injection.

Adding v0 = -(max+min)/2 to the three phase references is equivalent to space-vector
modulation with the two zero vectors split equally (centered pulses). The linear
range is the inscribed circle of the voltage hexagon: |v| <= Vdc/sqrt(3).
"""
import math

from .transforms import SQRT3, inv_clarke


def limit_voltage(v_alpha: float, v_beta: float, vdc: float) -> tuple[float, float, bool]:
    """Scale the reference vector (direction kept) into the linear range."""
    vmax = vdc / SQRT3
    mag = math.hypot(v_alpha, v_beta)
    if mag <= vmax:
        return v_alpha, v_beta, False
    k = vmax / mag
    return v_alpha * k, v_beta * k, True


def svpwm_duties(v_alpha: float, v_beta: float, vdc: float) -> tuple[float, float, float]:
    """Phase duty cycles in [0, 1] (0.5 = half the DC bus at the pole)."""
    va, vb, vc = inv_clarke(v_alpha, v_beta)
    v0 = -0.5 * (max(va, vb, vc) + min(va, vb, vc))
    k = 1.0 / vdc
    return tuple(min(1.0, max(0.0, 0.5 + (v + v0) * k)) for v in (va, vb, vc))
