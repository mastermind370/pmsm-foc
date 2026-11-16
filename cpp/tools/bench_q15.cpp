// bench_q15 - time one iteration of CurrentControllerQ15::update.
//
//   usage: bench_q15 <inputs.txt> [iterations_per_rep=1000000] [reps=15]
//
// Method: the recorded vectors are cycled through in order, so the integrator state evolves as in
// a real run (each iteration depends on the previous one: this measures latency, not throughput).
// One warm-up rep is discarded. Each rep is timed as a batch and divided by the iteration count,
// which keeps clock-read overhead (tens of ns) out of the per-iteration figure. Prints one JSON
// line with min / median / max ns per iteration over the reps.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "default_config.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <inputs.txt> [iters_per_rep] [reps]\n", argv[0]);
        return 2;
    }
    const long iters = argc > 2 ? std::atol(argv[2]) : 1000000;
    const int reps = argc > 3 ? std::atoi(argv[3]) : 15;
    if (iters <= 0 || reps <= 0) return 2;

    std::FILE* fi = std::fopen(argv[1], "r");
    if (fi == nullptr) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }
    std::vector<std::array<q15::q15_t, 5>> vec;
    int a, b, c, d, e;
    while (std::fscanf(fi, "%d %d %d %d %d", &a, &b, &c, &d, &e) == 5) {
        vec.push_back({static_cast<q15::q15_t>(a), static_cast<q15::q15_t>(b),
                       static_cast<q15::q15_t>(c), static_cast<q15::q15_t>(d),
                       static_cast<q15::q15_t>(e)});
    }
    std::fclose(fi);
    if (vec.empty()) return 1;

    foc::CurrentControllerQ15 ctl(foc::defaults::gains());
    volatile std::int32_t sink = 0;   // keeps the optimizer from deleting the work

    auto run = [&](long n) {
        std::int32_t acc = 0;
        std::size_t k = 0;
        for (long i = 0; i < n; ++i) {
            const auto& v = vec[k];
            const foc::Output o = ctl.update(v[0], v[1], v[2], v[3], v[4]);
            acc += o.vd + o.vq;
            if (++k == vec.size()) k = 0;
        }
        sink = acc;
    };

    run(iters);   // warm-up
    std::vector<double> ns;
    for (int r = 0; r < reps; ++r) {
        const auto t0 = std::chrono::steady_clock::now();
        run(iters);
        const auto t1 = std::chrono::steady_clock::now();
        ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() /
                     static_cast<double>(iters));
    }
    std::sort(ns.begin(), ns.end());
    std::printf("{\"vectors\": %zu, \"iterations_per_rep\": %ld, \"reps\": %d, "
                "\"ns_per_iter_min\": %.3f, \"ns_per_iter_median\": %.3f, "
                "\"ns_per_iter_max\": %.3f}\n",
                vec.size(), iters, reps, ns.front(), ns[ns.size() / 2], ns.back());
    return 0;
}
