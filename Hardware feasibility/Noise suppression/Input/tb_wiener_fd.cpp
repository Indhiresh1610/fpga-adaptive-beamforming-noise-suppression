// ============================================================================
// tb_wiener_fd.cpp -- self-checking testbench for wiener_fd_top
//   TEST 1  FFT engine vs naive double DFT
//   TEST 2  Streaming control: noise-learn flag timing, muted output while learning, reset repeatability
//   TEST 3  Perfect reconstruction: noise-free input must come back unchanged (delay 384)
//           -> proves windowing / overlap-add / normalisation / circular-buffer alignment
//   TEST 4  Scenarios (signal x noise x SNR, same generators as the Python sweep) vs a double-precision
//           reference of the SAME causal algorithm; checks DUT-vs-ref agreement and SNR gain.
// Returns 0 on pass (required by Vitis HLS C-sim).
// Host build:  g++ -O2 -DWFD_NO_AP tb_wiener_fd.cpp wiener_fd.cpp -o tb
// ============================================================================
#include <cstdio>
#include <cmath>
#include <complex>
#include <vector>
#include <random>
#include <algorithm>
#include "wiener_fd.h"

static const double FS_HZ = 16000.0, DURATION_S = 3.0, LEAD_S = 0.3;
static const double TOL_DB      = 0.3;    // DUT vs double reference SNR
static const double MIN_GAIN_DB = 1.0;    // required SNR improvement for scenarios with input SNR >= 0 dB
static const double PI = 3.14159265358979323846;

typedef std::complex<double> cd;

// ---------------- double FFT (reference only) ----------------
static void dfft(std::vector<cd>& a, bool inverse) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = 2 * PI / (double)len * (inverse ? 1 : -1);
        cd wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            cd w(1);
            for (size_t k = 0; k < len / 2; k++) {
                cd u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v; a[i + k + len / 2] = u - v; w *= wl;
            }
        }
    }
    if (inverse) for (auto& v : a) v /= (double)n;
}

// ------------------------------ TEST 1 ---------------------------------------
static int test_fft() {
    std::mt19937 rng(11);
    std::normal_distribution<double> g(0.0, 1.0);
    double worst = 0;
    for (int t = 0; t < 10; t++) {
        std::vector<double> x(FRAME), xi(FRAME);
        for (int n = 0; n < FRAME; n++) { x[n] = g(rng); xi[n] = g(rng); }
        float re[FRAME], im[FRAME];
        for (int n = 0; n < FRAME; n++) { int r = wfd_bitrev(n); re[r] = (float)x[n]; im[r] = (float)xi[n]; }
        wfd_fft(re, im);
        std::vector<cd> a(FRAME);
        for (int n = 0; n < FRAME; n++) a[n] = cd(x[n], xi[n]);
        dfft(a, false);
        double mx = 0, err = 0;
        for (int k = 0; k < FRAME; k++) {
            mx = std::max(mx, std::abs(a[k]));
            err = std::max(err, std::abs(a[k] - cd(re[k], im[k])));
        }
        worst = std::max(worst, err / mx);
    }
    bool ok = worst < 1e-5;
    std::printf("TEST 1 FFT engine vs double FFT: worst relative error = %.2e -> %s\n", worst, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ------------------------------ TEST 2 ---------------------------------------
static int test_control() {
    std::mt19937 rng(5);
    std::normal_distribution<double> g(0.0, 0.3);
    const int NBLK = 40;
    static fd_sample_t xb[HOP]; static fd_out_t yb[HOP];
    std::vector<std::vector<double>> y1(NBLK, std::vector<double>(HOP)), y2 = y1;
    std::vector<std::vector<double>> in(NBLK, std::vector<double>(HOP));
    for (auto& b : in) for (auto& v : b) v = g(rng);

    bool flag_ok = true, mute_ok = true, nonzero_after = false;
    const int LAST_LEARN = N_SKIP_FRAMES + N_INIT_FRAMES - 1;      // block index that finishes learning
    for (int pass = 0; pass < 2; pass++) {
        for (int b = 0; b < NBLK; b++) {
            for (int i = 0; i < HOP; i++) xb[i] = in[b][i];
            bool rdy = false;
            wiener_fd_top(xb, yb, (b == 0), &rdy);                 // reset on first block of each pass
            for (int i = 0; i < HOP; i++) (pass ? y2 : y1)[b][i] = WFD_D(yb[i]);
            if (pass == 0) {
                if (rdy != (b >= LAST_LEARN)) flag_ok = false;
                double e = 0; for (int i = 0; i < HOP; i++) e += std::fabs(y1[b][i]);
                if (b <= LAST_LEARN && e != 0.0) mute_ok = false;
                if (b > LAST_LEARN && e > 0.0) nonzero_after = true;
            }
        }
    }
    bool same = (y1 == y2);
    bool ok = flag_ok && mute_ok && nonzero_after && same;
    std::printf("TEST 2 noise_ready timing: %s | muted while learning: %s | output after learning: %s | reset repeatable: %s -> %s\n",
                flag_ok ? "ok" : "NO", mute_ok ? "ok" : "NO", nonzero_after ? "ok" : "NO", same ? "ok" : "NO", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ------------------------------ signal generators (match the Python) ---------
static std::vector<double> make_signal(const std::string& type, int N_total) {
    const int lead = (int)(LEAD_S * FS_HZ), L = (int)(DURATION_S * FS_HZ);
    std::vector<double> x(L);
    for (int n = 0; n < L; n++) {
        double t = n / FS_HZ;
        if (type == "single_tone")       x[n] = std::sin(2 * PI * 1000 * t);
        else if (type == "multi_tone")   x[n] = std::sin(2*PI*300*t) + std::sin(2*PI*1000*t) + std::sin(2*PI*2500*t) + std::sin(2*PI*4000*t);
        else if (type == "chirp")        x[n] = std::sin(2 * PI * (200 * t + 0.5 * ((6000.0 - 200.0) / DURATION_S) * t * t));
        else if (type == "am_modulated") x[n] = std::sin(2 * PI * 1500 * t) * 0.5 * (1 + std::sin(2 * PI * 3 * t));
        else /* square_wave */           x[n] = (std::fmod(400.0 * t, 1.0) < 0.5) ? 1.0 : -1.0;
    }
    double mx = 0; for (double v : x) mx = std::max(mx, std::fabs(v));
    std::vector<double> out(N_total, 0.0);                          // lead silence, signal, zero pad up to whole blocks
    for (int n = 0; n < L && lead + n < N_total; n++) out[lead + n] = x[n] / mx * 0.5;
    return out;
}

static std::vector<double> make_noise(const std::string& type, const std::vector<double>& clean, double snr_db, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> g(0.0, 1.0);
    const size_t N = clean.size();
    std::vector<double> nz(N);
    if (type == "white") {
        for (auto& v : nz) v = g(rng);
    } else {                                                        // "pink": Kellet 3-pole approximation (1/f-like)
        double b0 = 0, b1 = 0, b2 = 0;
        for (auto& v : nz) {
            double w = g(rng);
            b0 = 0.99765 * b0 + w * 0.0990460; b1 = 0.96300 * b1 + w * 0.2965164; b2 = 0.57000 * b2 + w * 1.0526913;
            v = b0 + b1 + b2 + w * 0.1848;
        }
    }
    double pc = 0, pn = 0;
    for (size_t n = 0; n < N; n++) { pc += clean[n] * clean[n]; pn += nz[n] * nz[n]; }
    double sc = std::sqrt((pc / N) / std::pow(10.0, snr_db / 10.0) / (pn / N));
    for (auto& v : nz) v *= sc;
    return nz;
}

// ------------------------------ double reference of the SAME causal algorithm -
// Works on the whole signal (no circular buffers): independent check of the DUT data-flow.
static std::vector<double> ref_wiener(const std::vector<double>& xq, int NF) {
    const int off = FRAME;                                          // avoid negative indices
    std::vector<double> ola((size_t)NF * HOP + 2 * FRAME, 0.0);
    std::vector<double> noise(NBINS, 0.0), inv(NBINS, 0.0), aprev2(NBINS, 0.0);
    bool ready = false;
    for (int f = 0; f < NF; f++) {
        const bool learn = f >= N_SKIP_FRAMES && f < N_SKIP_FRAMES + N_INIT_FRAMES;
        const bool enh = ready && !learn;
        if (!learn && !enh) continue;
        const long s0 = (long)HOP * (f + 1) - FRAME;
        std::vector<cd> Y(FRAME);
        for (int n = 0; n < FRAME; n++) {
            long i = s0 + n;
            double v = (i >= 0 && i < (long)xq.size()) ? xq[i] : 0.0;
            Y[n] = v * (0.5 - 0.5 * std::cos(2 * PI * n / FRAME));
        }
        dfft(Y, false);
        if (learn) {
            for (int k = 0; k < NBINS; k++) noise[k] += std::norm(Y[k]);
            if (f == N_SKIP_FRAMES + N_INIT_FRAMES - 1) {
                for (int k = 0; k < NBINS; k++) inv[k] = 1.0 / (noise[k] / N_INIT_FRAMES + 1e-12);
                ready = true;
            }
            continue;
        }
        std::vector<cd> Z(FRAME);
        for (int k = 0; k < NBINS; k++) {
            double P = std::norm(Y[k]);
            double xi = 0.98 * aprev2[k] * inv[k] + 0.02 * std::max(P * inv[k] - 1.0, 0.0);
            xi = std::max(xi, 1e-6);
            double G = xi / (1 + xi);
            aprev2[k] = G * G * P;
            Z[k] = G * Y[k];
            if (k > 0 && k < FRAME / 2) Z[FRAME - k] = std::conj(Z[k]);
        }
        dfft(Z, true);
        for (int n = 0; n < FRAME; n++)
            ola[(size_t)(s0 + n + off)] += Z[n].real() * (0.5 - 0.5 * std::cos(2 * PI * n / FRAME));
    }
    std::vector<double> y(xq.size());
    for (size_t i = 0; i < xq.size(); i++) y[i] = ola[i + off] / 1.5;
    return y;
}

// ------------------------------ run DUT over a whole signal -------------------
// returns y aligned to the input time line (removes the 384-sample latency).
static std::vector<double> run_dut(const std::vector<double>& x, std::vector<double>& xq_out) {
    const int NB = (int)(x.size() / HOP);
    const int NF = NB + (FRAME - HOP) / HOP;                        // + flush blocks
    static fd_sample_t xb[HOP]; static fd_out_t yb[HOP];
    std::vector<double> stream((size_t)NF * HOP);
    xq_out.assign(x.size(), 0.0);
    for (int b = 0; b < NF; b++) {
        for (int i = 0; i < HOP; i++) {
            size_t n = (size_t)b * HOP + i;
            xb[i] = (n < x.size()) ? x[n] : 0.0;
            if (n < x.size()) xq_out[n] = WFD_D(xb[i]);             // quantised input seen by the DUT
        }
        bool rdy;
        wiener_fd_top(xb, yb, b == 0, &rdy);
        for (int i = 0; i < HOP; i++) stream[(size_t)b * HOP + i] = WFD_D(yb[i]);
    }
    std::vector<double> y(x.size());
    for (size_t n = 0; n < x.size(); n++) y[n] = stream[n + LATENCY_SAMPLES];
    return y;
}

static double snr_db(const std::vector<double>& est, const std::vector<double>& s, size_t i0, size_t i1) {
    double ps = 0, pe = 0;
    for (size_t n = i0; n < i1; n++) { ps += s[n] * s[n]; pe += (est[n] - s[n]) * (est[n] - s[n]); }
    return 10 * std::log10(ps / std::max(pe, 1e-15));
}

// ------------------------------ TEST 3 ---------------------------------------
static int test_reconstruction() {
    const int NB = 413, N = NB * HOP;
    int fails = 0;
    for (std::string type : {"multi_tone", "chirp"}) {
        std::vector<double> x = make_signal(type, N), xq;
        std::vector<double> y = run_dut(x, xq);
        // zero noise PSD -> gain ~1 on every bin that carries signal; evaluate where the signal lives
        size_t i0 = (size_t)(LEAD_S * FS_HZ) + 2 * FRAME, i1 = (size_t)N - 2 * FRAME;
        double mx = 0; for (size_t n = i0; n < i1; n++) mx = std::max(mx, std::fabs(y[n] - x[n]));
        bool ok = mx < 5e-3;
        if (!ok) fails++;
        std::printf("TEST 3 reconstruction (%-10s): max|y-x| = %.2e (signal peak 0.5) -> %s\n", type.c_str(), mx, ok ? "PASS" : "FAIL");
    }
    return fails ? 1 : 0;
}

// ------------------------------ TEST 4 ---------------------------------------
static int test_scenarios() {
    const int NB = 413, N = NB * HOP;
    const size_t i0 = (size_t)(LEAD_S * FS_HZ);                     // evaluate on the signal part only
    int fails = 0;
    std::printf("TEST 4 scenarios: %d blocks of %d samples, FRAME=%d, evaluation = signal part (after %.1f s lead-in)\n",
                NB, HOP, FRAME, LEAD_S);
    std::printf("%-30s %8s %8s %8s %8s %8s %8s\n", "case", "in SNR", "DUT SNR", "ref SNR", "gain dB", "diff dB", "max|x|");
    const char* sigs[]   = {"single_tone", "multi_tone", "chirp", "am_modulated", "square_wave"};
    const char* noises[] = {"white", "pink"};
    const double snrs[]  = {0.0, 10.0};
    for (const char* sg : sigs) for (const char* nt : noises) for (double snr : snrs) {
        std::vector<double> clean = make_signal(sg, N);
        std::vector<double> nz = make_noise(nt, clean, snr, 1);
        std::vector<double> x(N);
        double mxa = 0;
        for (int n = 0; n < N; n++) { x[n] = clean[n] + nz[n]; mxa = std::max(mxa, std::fabs(x[n])); }

        std::vector<double> xq, y_dut = run_dut(x, xq);
        const int NF = NB + (FRAME - HOP) / HOP;
        std::vector<double> y_ref = ref_wiener(xq, NF);

        double sin_ = snr_db(x, clean, i0, N), sd = snr_db(y_dut, clean, i0, N), sr = snr_db(y_ref, clean, i0, N);
        bool ok = std::fabs(sd - sr) <= TOL_DB && (snr < 0.5 || (sd - sin_) >= MIN_GAIN_DB) && mxa < 15.9;
        if (!ok) fails++;
        char name[64]; std::snprintf(name, sizeof name, "%s/%s/%+.0fdB", sg, nt, snr);
        std::printf("%-30s %8.2f %8.2f %8.2f %+8.2f %+8.2f %8.2f  %s\n", name, sin_, sd, sr, sd - sin_, sd - sr, mxa, ok ? "PASS" : "FAIL");
    }
    return fails ? 1 : 0;
}

int main() {
    int rc = 0;
    rc |= test_fft();
    rc |= test_control();
    rc |= test_reconstruction();
    rc |= test_scenarios();
    std::printf("\nOVERALL: %s\n", rc ? "FAIL" : "PASS");
    return rc;
}
