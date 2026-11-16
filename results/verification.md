| Scenario | Steps | Limiter active | Max err [LSB] | RMS err [LSB] | Linear-only max [LSB] | Linear-only RMS [LSB] |
|---|---|---|---|---|---|---|
| torque_step_locked | 2000 | 1.8 % | 1.21 | 0.30 | 1.21 | 0.29 |
| speed_step_30 | 3000 | 0.1 % | 2.00 | 0.35 | 1.81 | 0.35 |
| load_step_2Nm | 4000 | 0.1 % | 2.00 | 0.50 | 1.81 | 0.50 |
| voltage_limited_reversal | 10000 | 5.5 % | 3.63 | 1.18 | 3.63 | 1.17 |
| **all** | 19000 | 3.1 % | 3.63 | 0.90 | 3.63 | 0.89 |

Limited steps only: max 2.00 LSB, RMS 1.21 LSB. End-to-end (comparison B, all steps): max 9.28 LSB, RMS 2.46 LSB.
1 LSB = 0.846 mV. Time per control iteration: min 23.3 / median 23.4 / max 23.7 ns (Apple M4, Apple clang version 17.0.0 (clang-1700.0.13.5), -std=c++17 -O2 -Wall -Wextra).
