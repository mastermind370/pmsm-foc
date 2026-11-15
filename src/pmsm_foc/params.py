"""Motor and drive parameters (SI units). Single source of truth for the project."""
from dataclasses import dataclass


@dataclass(frozen=True)
class PMSMParams:
    Rs: float = 0.5          # stator resistance [ohm]
    Ld: float = 1.4e-3       # d-axis inductance [H]
    Lq: float = 1.4e-3       # q-axis inductance [H]
    psi_f: float = 0.1       # PM flux linkage [Wb]
    pole_pairs: int = 4
    J: float = 5.0e-4        # rotor + load inertia [kg m^2]   (assumed)
    B: float = 1.0e-4        # viscous friction [N m s/rad]    (assumed)
    Vdc: float = 48.0        # DC bus [V]
    dt: float = 1.0e-6       # plant integration step [s]
