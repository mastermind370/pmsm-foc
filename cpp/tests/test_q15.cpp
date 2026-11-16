// Self-contained tests (no framework). Exit code 0 = all passed.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>

#include "current_controller_q15.hpp"

using namespace foc;
using q15::q15_t;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond)                                                                    \
    do {                                                                               \
        ++g_checks;                                                                    \
        if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
    } while (0)

static constexpr double kTs = 1e-4;
static constexpr double Q15 = 32768.0;

// Reference design used throughout: same numbers as the Python project (Stage 3 defaults).
static MotorParams motor() { return {0.5, 1.4e-3, 1.4e-3, 0.1, 48.0}; }
static Scaling scaling() { return make_scaling(48.0); }
static Gains make_gains() {
    const PiDesign d = design_current_pi(0.5, 1.4e-3, 200.0, 1.5);
    return quantize_gains(motor(), d, d, scaling(), kTs);
}

// ---- double-precision model of the SAME algorithm, using the dequantized gains --------
struct RefOut { double vd, vq; };
struct RefController {
    double kp_d, kp_q, ki_d, ki_q, kaw_d, kaw_q, cffd, cffq, cbemf, vmax;
    double Id = 0, Iq = 0;
    explicit RefController(const Gains& g)
        : kp_d(g.kp_d / 4096.0), kp_q(g.kp_q / 4096.0), ki_d(g.ki_d / Q15), ki_q(g.ki_q / Q15),
          kaw_d(g.kaw_d / Q15), kaw_q(g.kaw_q / Q15), cffd(g.c_ffd / 4096.0),
          cffq(g.c_ffq / 4096.0), cbemf(g.c_bemf / 4096.0), vmax(g.v_max / Q15) {}
    RefOut update(double idr, double iqr, double id, double iq, double we) {
        const double ed = idr - id, eq = iqr - iq;
        const double ud = kp_d * ed + Id - we * iq * cffd;
        const double uq = kp_q * eq + Iq + we * id * cffq + we * cbemf;
        const double vd = std::fmax(-vmax, std::fmin(vmax, ud));
        const double lim = std::sqrt(std::fmax(0.0, vmax * vmax - vd * vd));
        const double vq = std::fmax(-lim, std::fmin(lim, uq));
        Id += ki_d * ed + kaw_d * (vd - ud);
        Iq += ki_q * eq + kaw_q * (vq - uq);
        return {vd, vq};
    }
};

static q15_t to_q15(double x) { return q15::sat16(static_cast<std::int32_t>(std::lrint(x * Q15))); }

// ---------------------------------------------------------------------------------------
static void test_primitives() {
    CHECK(q15::sat16(40000) == 32767 && q15::sat16(-40000) == -32768);
    CHECK(q15::sat16(32767) == 32767 && q15::sat16(-32768) == -32768 && q15::sat16(0) == 0);
    CHECK(q15::sat32(std::int64_t{1} << 40) == INT32_MAX);
    CHECK(q15::sat32(-(std::int64_t{1} << 40)) == INT32_MIN);
    CHECK(q15::sat32(123456) == 123456);
    CHECK(q15::round_shift(5, 1) == 3 && q15::round_shift(4, 1) == 2);        // 2.5->3, 2->2
    CHECK(q15::round_shift(-5, 1) == -2 && q15::round_shift(-6, 1) == -3);    // -2.5->-2 (ties up)
    CHECK(q15::round_shift(1000, 3) == 125 && q15::round_shift(-1000, 3) == -125);

    // isqrt32 = floor(sqrt(x)): exhaustive to 2^22, then random and edge values.
    bool ok = true;
    for (std::uint32_t x = 0; x < (1u << 22); ++x) {
        const auto r = static_cast<std::uint32_t>(q15::isqrt32(x));
        if (r * r > x || (r + 1) * (r + 1) <= x) { ok = false; break; }
    }
    CHECK(ok);
    std::mt19937 rng(1);
    for (int i = 0; i < 200000; ++i) {
        const auto x = static_cast<std::uint32_t>(rng());
        const std::uint64_t r = q15::isqrt32(x);
        CHECK(r * r <= x && (r + 1) * (r + 1) > x);
    }
    CHECK(q15::isqrt32(0xFFFFFFFFu) == 65535 && q15::isqrt32(32767u * 32767u) == 32767);

    CHECK(q15::to_fixed16(0.5, 15) == 16384);
    CHECK(q15::to_fixed16(-1.0, 15) == -32768);
    bool threw = false;
    try { q15::to_fixed16(1.0, 15); } catch (const std::range_error&) { threw = true; }
    CHECK(threw);                                                             // 32768 does not fit
}

static void test_gain_quantization() {
    const Gains g = make_gains();
    const Scaling s = scaling();
    const PiDesign d = design_current_pi(0.5, 1.4e-3, 200.0, 1.5);
    const double zi = s.i_base / s.v_base;
    CHECK(std::abs(g.kp_d - d.kp * zi * 4096.0) <= 0.5);
    CHECK(std::abs(g.ki_d - d.ki * kTs * zi * Q15) <= 0.5);
    CHECK(std::abs(g.kaw_d - d.kaw * kTs * Q15) <= 0.5);
    CHECK(std::abs(g.c_bemf - s.w_base * 0.1 / s.v_base * 4096.0) <= 0.5);
    CHECK(g.kp_d == g.kp_q && g.ki_d == g.ki_q);                              // Ld == Lq
    CHECK(g.v_max == 32767);

    // A gain that cannot be represented must be rejected at start-up, not silently wrapped.
    PiDesign huge = d;
    huge.kp *= 10.0;                                                          // ~34 pu > Q12 range 8
    bool threw = false;
    try { quantize_gains(motor(), huge, huge, s, kTs); } catch (const std::range_error&) { threw = true; }
    CHECK(threw);
}

static void test_zero_and_polarity() {
    CurrentControllerQ15 c(make_gains());
    Output o = c.update(0, 0, 0, 0, 0);
    CHECK(o.vd == 0 && o.vq == 0 && !o.saturated && c.acc_d() == 0 && c.acc_q() == 0);
    // Positive iq error -> positive vq, and d is untouched at we = 0.
    o = c.update(0, to_q15(0.25), 0, 0, 0);
    CHECK(o.vq > 0 && o.vd == 0 && c.acc_q() > 0);
    // Speed term: with zero error and positive we the back-EMF feed-forward gives vq > 0.
    CurrentControllerQ15 c2(make_gains());
    o = c2.update(0, 0, 0, 0, to_q15(0.5));
    CHECK(o.vq > 0 && o.vd == 0);
    const double expect = 0.5 * 0.1 * 400.0 / scaling().v_base;              // we*psi_f / V_base
    CHECK(std::abs(o.vq / Q15 - expect) < 2e-4);
}

// Locked-rotor d/q plant in physical units, sampled once per control period (1 us Euler sub-steps).
struct Plant {
    double id = 0, iq = 0;
    void step(double vd, double vq, double we) {
        const MotorParams m = motor();
        const double h = 1e-6;
        for (int k = 0; k < 100; ++k) {
            const double did = (vd - m.Rs * id + we * m.Lq * iq) / m.Ld;
            const double diq = (vq - m.Rs * iq - we * m.Ld * id - we * m.psi_f) / m.Lq;
            id += h * did;
            iq += h * diq;
        }
    }
};

static void test_matches_double_reference() {
    // Closed loop: the plant is driven by the Q15 controller; the double model gets exactly the
    // same quantized inputs each step, so any difference is arithmetic (rounding/truncation).
    const Gains g = make_gains();
    const Scaling s = scaling();
    CurrentControllerQ15 c(g);
    RefController r(g);
    Plant pl;
    double max_err = 0.0, sum_sq = 0.0;
    int n = 0, sat_count = 0;
    const int steps = 40000;                                    // 4 s of control time
    for (int k = 0; k < steps; ++k) {
        const double t = k * kTs;
        const double iq_ref = ((k / 2000) % 2 == 0) ? 0.3 : -0.3;           // +/-6 A, every 200 ms
        const double id_ref = ((k / 3500) % 2 == 0) ? 0.0 : -0.1;           // 0 / -2 A
        const double we_pu = 0.3 * std::sin(2.0 * 3.14159265358979 * 1.5 * t);  // slow speed sweep
        const q15_t qidr = to_q15(id_ref), qiqr = to_q15(iq_ref), qwe = to_q15(we_pu);
        const q15_t qid = to_q15(pl.id / s.i_base), qiq = to_q15(pl.iq / s.i_base);
        const Output o = c.update(qidr, qiqr, qid, qiq, qwe);
        const RefOut e = r.update(qidr / Q15, qiqr / Q15, qid / Q15, qiq / Q15, qwe / Q15);
        const double ed = o.vd / Q15 - e.vd, eq = o.vq / Q15 - e.vq;
        max_err = std::fmax(max_err, std::fmax(std::fabs(ed), std::fabs(eq)));
        sum_sq += ed * ed + eq * eq;
        n += 2;
        sat_count += o.saturated ? 1 : 0;
        pl.step(o.vd / Q15 * s.v_base, o.vq / Q15 * s.v_base, we_pu * s.w_base);
    }
    const double rms = std::sqrt(sum_sq / n);
    std::printf("  [info] fixed vs double (same gains): max err %.3g pu (%.2f LSB), rms %.3g pu "
                "(%.2f LSB), saturated in %d of %d steps\n",
                max_err, max_err * Q15, rms, rms * Q15, sat_count, steps);
    CHECK(max_err < 4.0 / Q15);          // calibrated bound: a few LSB of Q15
    CHECK(rms < 1.0 / Q15);
    CHECK(sat_count < steps / 20);       // mostly the linear region; saturation only on step edges
}

static void test_voltage_circle_and_d_priority() {
    CurrentControllerQ15 c(make_gains());
    const auto full = static_cast<q15_t>(32767);
    // Huge d and q demands.
    Output o = c.update(full, full, static_cast<q15_t>(-32768), static_cast<q15_t>(-32768), 0);
    CHECK(o.saturated);
    CHECK(static_cast<std::int64_t>(o.vd) * o.vd + static_cast<std::int64_t>(o.vq) * o.vq <=
          static_cast<std::int64_t>(32767) * 32767);
    CHECK(o.vd == 32767 && o.vq == 0);                    // d-axis priority: q gets what is left
    // Moderate d demand leaves room for q, and the pair stays inside the circle.
    CurrentControllerQ15 c2(make_gains());
    o = c2.update(to_q15(0.1), to_q15(0.9), 0, 0, 0);
    CHECK(o.vd > 0 && o.vq > 0);
    CHECK(static_cast<std::int64_t>(o.vd) * o.vd + static_cast<std::int64_t>(o.vq) * o.vq <=
          static_cast<std::int64_t>(32767) * 32767);
}

static void test_anti_windup() {
    // we = 0.6 pu (240 rad/s): back-EMF eats 0.87 pu of the voltage circle, so an iq* of 0.9 pu
    // (18 A) is unreachable. Hold it for 50 ms, then drop iq* to a reachable 0.2 pu and see how
    // quickly the current gets there. Anti-windup should recover almost immediately.
    auto run = [](const Gains& g, double* acc_peak_pu, int* steps_to_recover) {
        const Scaling s = scaling();
        const double we_pu = 0.6, we = we_pu * s.w_base;
        CurrentControllerQ15 c(g);
        Plant pl;
        double peak = 0;
        auto step = [&](double iq_ref_pu) {
            const Output o = c.update(0, to_q15(iq_ref_pu), to_q15(pl.id / s.i_base),
                                      to_q15(pl.iq / s.i_base), to_q15(we_pu));
            pl.step(o.vd / Q15 * s.v_base, o.vq / Q15 * s.v_base, we);
            peak = std::fmax(peak, std::fabs(c.acc_q() / 134217728.0));
            return o;
        };
        for (int k = 0; k < 500; ++k) step(0.9);
        *acc_peak_pu = peak;
        int n = 0;
        for (; n < 1000; ++n) {
            step(0.2);
            if (std::fabs(pl.iq / s.i_base - 0.2) < 0.02) break;
        }
        *steps_to_recover = n;
    };
    const Gains g = make_gains();
    Gains no_aw = g;
    no_aw.kaw_d = no_aw.kaw_q = 0;
    double pk_aw, pk_no;
    int rec_aw, rec_no;
    run(g, &pk_aw, &rec_aw);
    run(no_aw, &pk_no, &rec_no);
    std::printf("  [info] anti-windup on: peak |acc| %.2f pu, recovery %d steps; "
                "off: peak %.2f pu, recovery %d steps\n", pk_aw, rec_aw, pk_no, rec_no);
    CHECK(pk_aw < 2.0);                       // back-calculation holds the state near the limit
    CHECK(pk_no > 4.0 * pk_aw);               // without it the integrator winds far up
    CHECK(rec_aw < 100);                      // < 10 ms to settle within 0.02 pu (0.4 A)
    CHECK(rec_no > 3 * rec_aw);               // windup makes recovery several times slower
}

static void test_extreme_inputs_are_safe() {
    // Run under -fsanitize=undefined to prove there is no signed overflow anywhere.
    const std::int16_t vals[] = {INT16_MIN, -32767, -16384, -1, 0, 1, 16384, 32766, INT16_MAX};
    CurrentControllerQ15 c(make_gains());
    for (int rep = 0; rep < 50; ++rep)
        for (auto a : vals) for (auto b : vals) for (auto d : vals) for (auto w : {vals[0], vals[4], vals[8]}) {
            const Output o = c.update(a, b, d, a, w);
            CHECK(static_cast<std::int64_t>(o.vd) * o.vd + static_cast<std::int64_t>(o.vq) * o.vq <=
                  static_cast<std::int64_t>(32767) * 32767);
        }
    // Accumulators must be pinned (saturating), never wrapped, under a permanent worst-case error.
    CurrentControllerQ15 no_aw_c([] { Gains g = make_gains(); g.kaw_d = g.kaw_q = 0; return g; }());
    for (int k = 0; k < 2000000; ++k) no_aw_c.update(INT16_MAX, INT16_MAX, INT16_MIN, INT16_MIN, 0);
    CHECK(no_aw_c.acc_d() == INT32_MAX && no_aw_c.acc_q() == INT32_MAX);
}

int main() {
    test_primitives();
    test_gain_quantization();
    test_zero_and_polarity();
    test_matches_double_reference();
    test_voltage_circle_and_d_priority();
    test_anti_windup();
    test_extreme_inputs_are_safe();
    if (g_fail == 0) std::printf("ALL PASSED (%d checks)\n", g_checks);
    else std::printf("%d of %d checks FAILED\n", g_fail, g_checks);
    return g_fail == 0 ? 0 : 1;
}
