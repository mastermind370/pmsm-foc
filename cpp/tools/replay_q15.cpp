// replay_q15 - run recorded input vectors through the Q15 current controller.
//
//   usage: replay_q15 <inputs.txt> <outputs.txt>
//   input  line: id_ref iq_ref id iq we        (five int16 raw Q15 values, space separated)
//   output line: vd vq saturated               (two int16 raw Q15 values, 0/1 limiter flag)
//
// The controller starts from zero state, exactly like the Python reference it is compared to.
#include <cstdio>
#include <cstdlib>

#include "default_config.hpp"

namespace {
bool fits16(int v) { return v >= -32768 && v <= 32767; }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <inputs.txt> <outputs.txt>\n", argv[0]);
        return 2;
    }
    std::FILE* fi = std::fopen(argv[1], "r");
    std::FILE* fo = std::fopen(argv[2], "w");
    if (fi == nullptr || fo == nullptr) {
        std::fprintf(stderr, "cannot open input or output file\n");
        return 1;
    }

    const foc::Gains g = foc::defaults::gains();
    std::fprintf(stderr,
                 "gains(raw): kp_d=%d ki_d=%d kaw_d=%d c_ffd=%d c_ffq=%d c_bemf=%d v_max=%d\n",
                 g.kp_d, g.ki_d, g.kaw_d, g.c_ffd, g.c_ffq, g.c_bemf, g.v_max);
    foc::CurrentControllerQ15 ctl(g);

    int a, b, c, d, e;
    long n = 0;
    while (std::fscanf(fi, "%d %d %d %d %d", &a, &b, &c, &d, &e) == 5) {
        if (!fits16(a) || !fits16(b) || !fits16(c) || !fits16(d) || !fits16(e)) {
            std::fprintf(stderr, "line %ld: value outside int16 range\n", n + 1);
            return 1;
        }
        const foc::Output o = ctl.update(
            static_cast<q15::q15_t>(a), static_cast<q15::q15_t>(b), static_cast<q15::q15_t>(c),
            static_cast<q15::q15_t>(d), static_cast<q15::q15_t>(e));
        std::fprintf(fo, "%d %d %d\n", o.vd, o.vq, o.saturated ? 1 : 0);
        ++n;
    }
    std::fclose(fi);
    std::fclose(fo);
    std::fprintf(stderr, "replayed %ld vectors\n", n);
    return 0;
}
