// q15.hpp - fixed-point primitives (Q15 = int16 with 15 fractional bits, range [-1, 1)).
//
// Control-loop code (everything used by CurrentControllerQ15::update) uses only integer
// arithmetic, is noexcept, and never allocates. The to_fixed16()/from_fixed() helpers use
// floating point and may throw; they are for start-up configuration and tests only.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace q15 {

static_assert((-1 >> 1) == -1, "arithmetic right shift of negative integers is required");

using q15_t = std::int16_t;                       // value = raw / 2^15
constexpr std::int32_t kMax = 32767;              // 0.99997
constexpr std::int32_t kMin = -32768;             // -1.0

// Saturating narrowing: int32 -> Q15 (int16).
constexpr q15_t sat16(std::int32_t x) noexcept {
    return static_cast<q15_t>(x > kMax ? kMax : (x < kMin ? kMin : x));
}

// Saturating narrowing: int64 -> int32.
constexpr std::int32_t sat32(std::int64_t x) noexcept {
    constexpr std::int64_t hi = std::numeric_limits<std::int32_t>::max();
    constexpr std::int64_t lo = std::numeric_limits<std::int32_t>::min();
    return static_cast<std::int32_t>(x > hi ? hi : (x < lo ? lo : x));
}

constexpr std::int32_t clamp32(std::int32_t x, std::int32_t lo, std::int32_t hi) noexcept {
    return x < lo ? lo : (x > hi ? hi : x);
}

// x / 2^n rounded to nearest (ties toward +infinity). Requires 1 <= n <= 62.
constexpr std::int64_t round_shift(std::int64_t x, int n) noexcept {
    return (x + (std::int64_t{1} << (n - 1))) >> n;
}

// floor(sqrt(x)) for a 32-bit unsigned value, integer-only (bitwise method).
constexpr std::uint16_t isqrt32(std::uint32_t x) noexcept {
    std::uint32_t res = 0;
    std::uint32_t bit = std::uint32_t{1} << 30;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<std::uint16_t>(res);
}

// ---- host-side helpers (floating point; NOT for the control loop) ----------------------

// Round x * 2^frac_bits to an int16. Throws std::range_error if it does not fit.
inline std::int16_t to_fixed16(double x, int frac_bits) {
    const double scaled = std::nearbyint(std::ldexp(x, frac_bits));
    if (!(scaled >= -32768.0 && scaled <= 32767.0)) {
        throw std::range_error("value does not fit in int16 at the requested Q format");
    }
    return static_cast<std::int16_t>(scaled);
}

inline double from_fixed(std::int64_t raw, int frac_bits) {
    return std::ldexp(static_cast<double>(raw), -frac_bits);
}

}  // namespace q15
