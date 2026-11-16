// current_controller_q15.hpp - Q15 fixed-point port of the d/q PI current controller.
//
// Mirrors pmsm_foc.control.CurrentController: per-axis PI (forward-Euler integrator),
// decoupling feed-forward, circular voltage limit with d-axis priority, and
// back-calculation anti-windup. See docs/q15_scaling.md for the full scaling table.
//
// Per-unit scaling (signal_pu = physical / base, stored as Q15):
//     current  I_base = 20 A         (+/-10 A limit uses half of the range, 0.61 mA / LSB)
//     voltage  V_base = Vdc/sqrt(3)  (= linear-modulation limit, 27.71 V for Vdc = 48 V)
//     speed    W_base = 400 rad/s    (electrical; 100 rad/s mechanical for 4 pole pairs)
//
// Fixed-point formats used inside the controller:
//     signals id, iq, id_ref, iq_ref, we, vd, vq   Q15  (int16)
//     kp, decoupling and back-EMF coefficients    Q12  (int16, range +/-8 per-unit gain)
//     ki*Ts, kaw*Ts (per-sample integrator gains) Q15  (int16)
//     integrator accumulators                     Q27  (int32, range +/-16 pu, 12 guard bits)
//     intermediate sums (errors, u)               "wide Q15" int32 -- never narrowed
//                                                 before the voltage limiter
#pragma once

#include "q15.hpp"

namespace foc {

constexpr int kKpFrac = 12;    // kp, c_ffd, c_ffq, c_bemf
constexpr int kKiFrac = 15;    // ki*Ts, kaw*Ts
constexpr int kAccFrac = 27;   // integrator accumulators

struct Scaling {
    double i_base;   // [A]
    double v_base;   // [V]
    double w_base;   // [rad/s electrical]
};

// Standard scaling for a given DC bus: V_base = Vdc / sqrt(3).
Scaling make_scaling(double vdc, double i_base = 20.0, double w_base = 400.0);

struct MotorParams {
    double Rs, Ld, Lq, psi_f, vdc;
};

// PI gains in physical units: kp [V/A], ki [V/(A s)], kaw [1/s].
struct PiDesign {
    double kp, ki, kaw;
};

// Pole placement for G(s) = 1/(L s + R): Kp = 2*zeta*wn*L - R, Ki = wn^2 * L, kaw = Ki/Kp.
PiDesign design_current_pi(double R, double L, double f_bw_hz, double zeta);

struct Gains {
    q15::q15_t kp_d, kp_q;      // Q12  [pu V / pu A]
    q15::q15_t ki_d, ki_q;      // Q15  per-sample: Ki * Ts * I_base / V_base
    q15::q15_t kaw_d, kaw_q;    // Q15  per-sample: Kaw * Ts
    q15::q15_t c_ffd;           // Q12  W_base * Lq * I_base / V_base   (d: -we*Lq*iq)
    q15::q15_t c_ffq;           // Q12  W_base * Ld * I_base / V_base   (q: +we*Ld*id)
    q15::q15_t c_bemf;          // Q12  W_base * psi_f / V_base         (q: +we*psi_f)
    q15::q15_t v_max;           // Q15  voltage-circle radius (32767 = 1.0 pu)
};

// Convert physical gains to Q-format. Throws std::range_error if any gain does not fit
// (start-up only; choose different bases or a lower bandwidth).
Gains quantize_gains(const MotorParams& m, const PiDesign& d_axis, const PiDesign& q_axis,
                     const Scaling& s, double Ts);

struct Output {
    q15::q15_t vd;
    q15::q15_t vq;
    bool saturated;   // voltage limiter was active
};

class CurrentControllerQ15 {
public:
    explicit CurrentControllerQ15(const Gains& g) noexcept : g_(g) {}

    // One control period. All arguments are Q15 per-unit. Integer-only, no exceptions.
    Output update(q15::q15_t id_ref, q15::q15_t iq_ref, q15::q15_t id, q15::q15_t iq,
                  q15::q15_t we) noexcept;

    void reset() noexcept { acc_d_ = 0; acc_q_ = 0; }
    std::int32_t acc_d() const noexcept { return acc_d_; }   // Q27, for tests/diagnostics
    std::int32_t acc_q() const noexcept { return acc_q_; }
    const Gains& gains() const noexcept { return g_; }

private:
    Gains g_;
    std::int32_t acc_d_ = 0;
    std::int32_t acc_q_ = 0;
};

}  // namespace foc
