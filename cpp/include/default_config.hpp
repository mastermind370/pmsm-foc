// default_config.hpp - the exact controller configuration used by the Python project (Stage 3
// defaults), so the replay and benchmark tools build the same Q15 gains the tests use.
#pragma once

#include "current_controller_q15.hpp"

namespace foc::defaults {

constexpr double kTs = 1e-4;   // 10 kHz control period

inline MotorParams motor() { return {0.5, 1.4e-3, 1.4e-3, 0.1, 48.0}; }

// I_base = 20 A, V_base = 48/sqrt(3) = 27.71 V, W_base = 400 rad/s (electrical)
inline Scaling scaling() { return make_scaling(48.0); }

// 200 Hz current-loop bandwidth, zeta = 1.5 (same as FocDrive / CurrentController defaults).
inline Gains gains() {
    const PiDesign d = design_current_pi(0.5, 1.4e-3, 200.0, 1.5);
    return quantize_gains(motor(), d, d, scaling(), kTs);
}

}  // namespace foc::defaults
