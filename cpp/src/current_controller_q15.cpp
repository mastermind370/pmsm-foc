#include "current_controller_q15.hpp"

#include <cmath>

namespace foc {

using q15::q15_t;
using q15::round_shift;

namespace {
constexpr double kPi = 3.14159265358979323846;
// ki(Q15) * error(Q15) = Q30 -> accumulator Q27
constexpr int kIncShift = kKiFrac + 15 - kAccFrac;
static_assert(kIncShift >= 1, "accumulator must have fewer fractional bits than the product");
}  // namespace

Scaling make_scaling(double vdc, double i_base, double w_base) {
    return Scaling{i_base, vdc / std::sqrt(3.0), w_base};
}

PiDesign design_current_pi(double R, double L, double f_bw_hz, double zeta) {
    const double wn = 2.0 * kPi * f_bw_hz;
    const double kp = 2.0 * zeta * wn * L - R;
    const double ki = wn * wn * L;
    return PiDesign{kp, ki, ki / kp};
}

Gains quantize_gains(const MotorParams& m, const PiDesign& d, const PiDesign& q,
                     const Scaling& s, double Ts) {
    const double zi = s.i_base / s.v_base;   // converts V/A gains to per-unit gains
    Gains g{};
    g.kp_d = q15::to_fixed16(d.kp * zi, kKpFrac);
    g.kp_q = q15::to_fixed16(q.kp * zi, kKpFrac);
    g.ki_d = q15::to_fixed16(d.ki * Ts * zi, kKiFrac);
    g.ki_q = q15::to_fixed16(q.ki * Ts * zi, kKiFrac);
    g.kaw_d = q15::to_fixed16(d.kaw * Ts, kKiFrac);
    g.kaw_q = q15::to_fixed16(q.kaw * Ts, kKiFrac);
    g.c_ffd = q15::to_fixed16(s.w_base * m.Lq * zi, kKpFrac);
    g.c_ffq = q15::to_fixed16(s.w_base * m.Ld * zi, kKpFrac);
    g.c_bemf = q15::to_fixed16(s.w_base * m.psi_f / s.v_base, kKpFrac);
    g.v_max = static_cast<q15_t>(q15::kMax);
    return g;
}

Output CurrentControllerQ15::update(q15_t id_ref, q15_t iq_ref, q15_t id, q15_t iq,
                                    q15_t we) noexcept {
    // Errors: wide Q15 (|e| < 2.0), never narrowed.
    const std::int32_t ed = std::int32_t{id_ref} - std::int32_t{id};
    const std::int32_t eq = std::int32_t{iq_ref} - std::int32_t{iq};

    // Proportional terms: Q15 * Q12 = Q27 -> Q15 (wide).
    const auto pd = static_cast<std::int32_t>(round_shift(std::int64_t{g_.kp_d} * ed, kKpFrac));
    const auto pq = static_cast<std::int32_t>(round_shift(std::int64_t{g_.kp_q} * eq, kKpFrac));

    // Integrator outputs: Q27 -> Q15.
    const auto itd = static_cast<std::int32_t>(round_shift(acc_d_, kAccFrac - 15));
    const auto itq = static_cast<std::int32_t>(round_shift(acc_q_, kAccFrac - 15));

    // Decoupling feed-forward: we*iq*c = Q15*Q15*Q12 = Q42 -> Q15 (shift 27).
    const std::int64_t we64 = we;
    const auto ffd = -static_cast<std::int32_t>(
        round_shift(we64 * iq * g_.c_ffd, 15 + 15 + kKpFrac - 15));
    const auto ffq = static_cast<std::int32_t>(
        round_shift(we64 * id * g_.c_ffq, 15 + 15 + kKpFrac - 15) +
        round_shift(we64 * g_.c_bemf, kKpFrac));

    // Unsaturated commands (wide Q15).
    const std::int32_t ud = pd + itd + ffd;
    const std::int32_t uq = pq + itq + ffq;

    // Circular limit |v| <= v_max with d-axis priority.
    const std::int32_t vmax = g_.v_max;
    const std::int32_t vd = q15::clamp32(ud, -vmax, vmax);
    const auto remaining = static_cast<std::uint32_t>(vmax * vmax - vd * vd);
    const std::int32_t vq_lim = q15::isqrt32(remaining);
    const std::int32_t vq = q15::clamp32(uq, -vq_lim, vq_lim);

    // Integrators: forward Euler + back-calculation anti-windup, saturating at int32.
    acc_d_ = q15::sat32(std::int64_t{acc_d_} + round_shift(std::int64_t{g_.ki_d} * ed, kIncShift) +
                        round_shift(std::int64_t{g_.kaw_d} * (vd - ud), kIncShift));
    acc_q_ = q15::sat32(std::int64_t{acc_q_} + round_shift(std::int64_t{g_.ki_q} * eq, kIncShift) +
                        round_shift(std::int64_t{g_.kaw_q} * (vq - uq), kIncShift));

    return Output{static_cast<q15_t>(vd), static_cast<q15_t>(vq), (vd != ud) || (vq != uq)};
}

}  // namespace foc
