# Q15 current controller: scaling and arithmetic

Source: `cpp/include/current_controller_q15.hpp`, `cpp/src/current_controller_q15.cpp`.
Float reference: `src/pmsm_foc/control.py` (`CurrentController`).

## Per-unit bases

| Quantity | Base | Why |
|---|---|---|
| Current | I_base = 20 A | +/-10 A limit uses half the range; 0.61 mA per LSB |
| Voltage | V_base = Vdc/sqrt(3) = 27.71 V | 1.0 pu = linear SVPWM limit, so the voltage circle is radius 1.0 |
| Speed (electrical) | W_base = 400 rad/s | 100 rad/s mechanical at 4 pole pairs; headroom above the ~277 rad/s voltage-limited speed |

Signals are stored as Q15 (`int16`, value = raw / 32768, range [-1, 1)).

## Formats

| Item | Format | Type | Range | Notes |
|---|---|---|---|---|
| id, iq, id_ref, iq_ref, we, vd, vq | Q15 | int16 | [-1, 1) | per-unit |
| kp_d, kp_q | Q12 | int16 | [-8, 8) | Kp * I_base / V_base |
| c_ffd, c_ffq, c_bemf | Q12 | int16 | [-8, 8) | decoupling and back-EMF coefficients |
| ki*Ts (per sample) | Q15 | int16 | [-1, 1) | Ki * Ts * I_base / V_base |
| kaw*Ts (per sample) | Q15 | int16 | [-1, 1) | back-calculation gain |
| integrator accumulators | Q27 | int32 | [-16, 16) pu | 12 guard bits over Q15 |
| errors, P/I/FF terms, u | Q15 held in int32 | int32 | (-4, 4) pu | not narrowed until after the limiter |

Gains that do not fit their format make `quantize_gains` throw at start-up
(`std::range_error`) instead of silently wrapping.

## Equations (per control period)

```
e      = ref - meas                              (int32, wide Q15)
P      = kp * e            Q12*Q15 = Q27  -> >>12 (round) -> Q15
I      = acc               Q27            -> >>12 (round) -> Q15
ff_d   = -we * iq * c_ffd   Q15*Q15*Q12 = Q42 -> >>27 (round) -> Q15
ff_q   =  we * id * c_ffq   (same)  +  we * c_bemf  (Q27 -> >>12)
u      = P + I + ff                                (wide Q15)
v_d    = clamp(u_d, -vmax, vmax)
v_q    = clamp(u_q, -sqrt(vmax^2 - v_d^2), +sqrt(...))     d-axis priority
acc   += (ki*e + kaw*(v - u))   Q15*Q15 = Q30 -> >>3 (round) -> Q27, saturating at int32
```

All products are widened to `int64` before shifting; rounding is round-to-nearest
(ties toward +infinity). `isqrt32` is an integer bitwise square root (floor).

## Overflow argument

* Largest product `we*iq*c_ffd`: 2^15 * 2^15 * 2^15 = 2^45, far inside int64.
* `vmax^2 - vd^2` is at most 32767^2 = 1.07e9 < 2^31, and non-negative because `vd` is clamped first.
* Wide terms: |e| < 2, |P| < 16, |I| < 16, |ff| < 8, so |u| < 40 pu in Q15 = 1.3e6, no int32 risk.
* Accumulator increments are int64 then saturated to int32, so a permanent worst-case error pins the
  accumulator instead of wrapping (tested). In normal operation the anti-windup term keeps it near
  the saturation boundary.

## Known limitation: limiter sensitivity

Near `vd = vmax` the q-axis limit `sqrt(vmax^2 - vd^2)` is very steep (slope = vd / vq_limit). A one-LSB
rounding difference in `vd` can move `vq` by tens of LSB. That is a property of a circular limiter with
d-axis priority, not an arithmetic bug, but it means test vectors that sit deep in saturation will show
larger max error than linear-region vectors. Report both separately in Stage 6.

## Not modelled

No dead-time, no ADC quantization or offset, no angle sensor quantization, one fixed sample time.
