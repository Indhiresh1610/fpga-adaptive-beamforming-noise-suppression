// ============================================================================
// tb_wiener_block.cpp -- self-checking testbench for wiener_block_top
//   TEST 1  Givens solver vs double Gauss-Jordan on random Toeplitz SPD systems
//   TEST 2  Reset / pass-through: block 0 output == input; reset reproduces block 0
//   TEST 3  Streaming scenarios (REF mode and NOISE-learn + RUN mode) vs a double-precision
//           reference of the SAME causal algorithm; checks SNR gain and DUT-vs-ref agreement.
// Returns 0 on pass (required by Vitis HLS C-sim).
// Host build:  g++ -O2 -DWIENER_NO_AP tb_wiener_block.cpp wiener_block.cpp -o tb
// ============================================================================
#include <cstdio>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include "wiener_block.h"

static const double FS_HZ = 8000.0, DURATION_S = 10.0;
static const double TOL_DB = 0.3;        // DUT vs double reference SNR
static const double MIN_GAIN_DB = 1.0;   // required SNR improvement over the noisy input
static const int    NOISE_BLOCKS = 2;    // leading noise-only blocks in NOISE/RUN scenarios

static std::vector<double> gauss_jordan(std::vector<std::vector<double>> A, std::vector<double> b) {
    int n = (int)b.size();
    for (int c = 0; c < n; c++) {
        int p = c;
        for (int r = c + 1; r < n; r++) if (std::fabs(A[r][c]) > std::fabs(A[p][c])) p = r;
        std::swap(A[c], A[p]); std::swap(b[c], b[p]);
        double piv = A[c][c];
        for (int k = c; k < n; k++) A[c][k] /= piv;
        b[c] /= piv;
        for (int r = 0; r < n; r++) if (r != c) {
            double f = A[r][c];
            for (int k = c; k < n; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    return b;
}
static std::vector<double> ref_solve(const std::vector<double>& rxx, const std::vector<double>& rdx) {
    std::vector<std::vector<double>> A(TAPS, std::vector<double>(TAPS));
    for (int i = 0; i < TAPS; i++) for (int j = 0; j < TAPS; j++)
        A[i][j] = (i == j) ? rxx[0] * (1.0 + (double)LOAD_FRAC) : rxx[std::abs(i - j)];
    return gauss_jordan(A, rdx);
}

// ------------------------------ TEST 1 ---------------------------------------
static int test_solver() {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> fr(0.02, 0.48), amp(0.1, 1.0);
    std::uniform_real_distribution<double> ls(-3, 0);
    double worst = 0;
    for (int t = 0; t < 2000; t++) {
        // autocorrelation of (t%4+1) real tones + white noise  => valid SPD Toeplitz
        std::vector<double> r(TAPS, 0.0), p(TAPS);
        int nt = t % 4 + 1;
        for (int k = 0; k < nt; k++) {
            double a = amp(rng), w = 2 * M_PI * fr(rng);
            for (int l = 0; l < TAPS; l++) r[l] += 0.5 * a * a * std::cos(w * l);
        }
        r[0] += std::pow(10.0, 2 * ls(rng));
        for (int l = 0; l < TAPS; l++) p[l] = r[l] * (0.5 + 0.5 * amp(rng)) + 0.01 * (l % 3);
        float rf[TAPS], pf[TAPS], hf[TAPS];
        for (int l = 0; l < TAPS; l++) { rf[l] = (float)r[l]; pf[l] = (float)p[l]; }
        solve_wiener(rf, pf, hf);
        auto href = ref_solve(r, p);
        double num = 0, den = 0;
        for (int l = 0; l < TAPS; l++) { num += (hf[l] - href[l]) * (hf[l] - href[l]); den += href[l] * href[l]; }
        worst = std::max(worst, std::sqrt(num / std::max(den, 1e-30)));
    }
    bool ok = worst < 1e-2;
    std::printf("TEST 1 Givens solver vs double reference: worst relative tap error = %.2e -> %s\n", worst, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ------------------------------ TEST 2 ---------------------------------------
static int test_reset() {
    std::mt19937 rng(3);
    std::normal_distribution<double> g(0.0, 0.3);
    static sample_t x[BLOCK]; static out_t y1[BLOCK], y2[BLOCK], y3[BLOCK];
    for (int n = 0; n < BLOCK; n++) x[n] = g(rng);
    h_t hu[TAPS];
    wiener_block_top(x, x, y1, WF_MODE_REF, true, hu);
    bool passthru = std::fabs(WF_D(hu[0]) - 1.0) < 1e-5;
    for (int l = 1; l < TAPS; l++) passthru &= std::fabs(WF_D(hu[l])) < 1e-5;
    for (int n = 0; n < BLOCK; n++) passthru &= std::fabs(WF_D(y1[n]) - WF_D(x[n])) < 1e-3;
    wiener_block_top(x, x, y2, WF_MODE_REF, false, hu);              // d = x  -> h should adapt
    bool adapted = false;
    for (int l = 0; l < TAPS; l++) if (std::fabs(WF_D(hu[l]) - (l == 0)) > 1e-3) adapted = true;
    wiener_block_top(x, x, y3, WF_MODE_REF, true, hu);               // reset, same data
    bool same = true;
    for (int n = 0; n < BLOCK; n++) if (y1[n] != y3[n]) same = false;
    // NOTE: with d = x the optimum is h = delta, so "adapted" may legitimately be false; only
    // report it, do not fail on it.
    std::printf("TEST 2 block0 pass-through: %s | taps differ after block1: %s | reset reproduces block 0: %s\n",
                passthru ? "yes" : "NO", adapted ? "yes" : "no (ok, d=x => delta)", same ? "yes" : "NO");
    bool ok = passthru && same;
    std::printf("TEST 2 -> %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ------------------------------ TEST 3 ---------------------------------------
struct Case { const char* name; double sigma; int mode; };

static double snr_db(const std::vector<double>& est, const std::vector<double>& s, size_t i0, size_t i1) {
    double ps = 0, pe = 0;
    for (size_t n = i0; n < i1; n++) { ps += s[n] * s[n]; pe += (est[n] - s[n]) * (est[n] - s[n]); }
    return 10 * std::log10(ps / std::max(pe, 1e-15));
}

static int test_scenarios() {
    int fails = 0;
    const int NB = (int)(FS_HZ * DURATION_S) / BLOCK, N = NB * BLOCK;
    std::printf("TEST 3 scenarios: %d blocks of %d samples, TAPS=%d, evaluation = second half\n", NB, BLOCK, TAPS);
    std::printf("%-26s %8s %8s %8s %8s %8s %9s\n", "case", "in SNR", "DUT SNR", "ref SNR", "gain dB", "diff dB", "max|x|");

    std::vector<Case> cases = {
        {"REF  sigma=0.10", 0.10, WF_MODE_REF}, {"REF  sigma=0.30", 0.30, WF_MODE_REF}, {"REF  sigma=0.60", 0.60, WF_MODE_REF},
        {"RUN  sigma=0.10", 0.10, WF_MODE_RUN}, {"RUN  sigma=0.30", 0.30, WF_MODE_RUN}, {"RUN  sigma=0.60", 0.60, WF_MODE_RUN}};

    static sample_t xb[BLOCK], db[BLOCK]; static out_t yb[BLOCK];
    for (const auto& cs : cases) {
        std::mt19937 rng(123);
        std::normal_distribution<double> g(0.0, 1.0);
        // clean "speech-like" signal: 3 tones with slow amplitude modulation
        std::vector<double> s(N), nz(N), x(N);
        for (int n = 0; n < N; n++) {
            double t = n / FS_HZ, am = 0.75 + 0.25 * std::sin(2 * M_PI * 3.0 * t);
            s[n] = am * (0.4 * std::sin(2 * M_PI * 300 * t) + 0.3 * std::sin(2 * M_PI * 1000 * t + 1) + 0.2 * std::sin(2 * M_PI * 2200 * t + 2));
            nz[n] = cs.sigma * g(rng);
        }
        const bool run = (cs.mode == WF_MODE_RUN);
        std::vector<int> bmode(NB, cs.mode);
        for (int n = 0; n < N; n++) {
            bool noise_only = run && (n / BLOCK < NOISE_BLOCKS);
            x[n] = noise_only ? nz[n] : s[n] + nz[n];
        }
        if (run) for (int b = 0; b < NOISE_BLOCKS; b++) bmode[b] = WF_MODE_NOISE;
        double maxabs = 0; for (double v : x) maxabs = std::max(maxabs, std::fabs(v));

        // ---- DUT ----
        std::vector<double> y_dut(N);
        for (int b = 0; b < NB; b++) {
            for (int n = 0; n < BLOCK; n++) { xb[n] = x[(size_t)b * BLOCK + n]; db[n] = s[(size_t)b * BLOCK + n]; }
            h_t hu[TAPS];
            wiener_block_top(xb, db, yb, bmode[b], b == 0, hu);
            for (int n = 0; n < BLOCK; n++) y_dut[(size_t)b * BLOCK + n] = WF_D(yb[n]);
        }

        // ---- double-precision reference of the same causal algorithm ----
        std::vector<double> y_ref(N), h(TAPS, 0.0), rxx(TAPS, 0.0), rnn(TAPS, 0.0), rdx(TAPS, 0.0);
        h[0] = 1.0; bool hc = false, hn = false, hr = false;
        auto X = [&](long n) { return n >= 0 ? x[n] : 0.0; };
        for (int b = 0; b < NB; b++) {
            std::vector<double> br(TAPS, 0.0), bd(TAPS, 0.0);
            for (int n = 0; n < BLOCK; n++) {
                long i = (long)b * BLOCK + n; double acc = 0;
                for (int k = 0; k < TAPS; k++) acc += h[k] * X(i - k);
                y_ref[i] = acc;
                for (int l = 0; l < TAPS; l++) { br[l] += X(i) * X(i - l); bd[l] += s[i] * X(i - l); }
            }
            for (int l = 0; l < TAPS; l++) { br[l] /= BLOCK; bd[l] /= BLOCK; }
            bool use_ref = (bmode[b] == WF_MODE_REF);
            if (bmode[b] == WF_MODE_NOISE) {
                for (int l = 0; l < TAPS; l++) rnn[l] = hn ? NOISE_FORGET * rnn[l] + (1 - NOISE_FORGET) * br[l] : br[l];
                hn = true;
            } else {
                for (int l = 0; l < TAPS; l++) {
                    rxx[l] = hc ? FORGET * rxx[l] + (1 - FORGET) * br[l] : br[l];
                    rdx[l] = use_ref ? (hr ? FORGET * rdx[l] + (1 - FORGET) * bd[l] : bd[l]) : rxx[l] - NOISE_OVERSUB * rnn[l];
                }
                hc = true; if (use_ref) hr = true;
                if (use_ref || hn) {
                    if (!use_ref && rdx[0] < 0) std::fill(rdx.begin(), rdx.end(), 0.0);
                    h = ref_solve(rxx, rdx);
                }
            }
        }

        size_t i0 = (size_t)(NB / 2) * BLOCK;
        double sin_ = snr_db(x, s, i0, N), sd = snr_db(y_dut, s, i0, N), sr = snr_db(y_ref, s, i0, N);
        bool ok = std::fabs(sd - sr) <= TOL_DB && (sd - sin_) >= MIN_GAIN_DB && maxabs < 15.9;
        if (!ok) fails++;
        std::printf("%-26s %8.2f %8.2f %8.2f %+8.2f %+8.2f %9.2f  %s\n", cs.name, sin_, sd, sr, sd - sin_, sd - sr, maxabs, ok ? "PASS" : "FAIL");
    }
    return fails ? 1 : 0;
}

int main() {
    int rc = 0;
    rc |= test_solver();
    rc |= test_reset();
    rc |= test_scenarios();
    std::printf("\nOVERALL: %s\n", rc ? "FAIL" : "PASS");
    return rc;
}
