// AI Generated Code: Verified and Commented by the Team

// Explanation: 
// This code is a reference to check if the LDL algorithm is working. THe fixed point helper functions are called to execute the MVDR
// using LDL decomposition. This tests if the LDL decomposition is performing as expected. 
// The metrics on which it is checked are :
//   1) A * X ~ I   (X = mvdr_rdr_invert output with lower triangle rebuilt by conjugation)
//   2) w from solve_mvdr  vs  double-precision reference MVDR weights

#include <cstdio>
#include <cmath>
#include <complex>
#include <random>
#include "mvdr_block.h"

typedef std::complex<double> cd;
const int N = NUM_ANT;

// double-precision reference: solve A u = a by Gaussian elimination, partial pivoting
// This is a reference against which the LDL method's outputs are evaluated. 

static void ref_solve(cd A[N][N], const cd a[N], cd u[N])
{
    cd M[N][N + 1];
    for (int i = 0; i < N; i++) { for (int j = 0; j < N; j++) M[i][j] = A[i][j]; M[i][N] = a[i]; }
    for (int c = 0; c < N; c++) {
        int p = c;
        for (int r = c + 1; r < N; r++) if (std::abs(M[r][c]) > std::abs(M[p][c])) p = r;
        for (int j = 0; j <= N; j++) std::swap(M[c][j], M[p][j]);
        for (int r = c + 1; r < N; r++) {
            cd f = M[r][c] / M[c][c];
            for (int j = c; j <= N; j++) M[r][j] -= f * M[c][j];
        }
    }
    for (int i = N - 1; i >= 0; i--) {
        cd s = M[i][N];
        for (int j = i + 1; j < N; j++) s -= M[i][j] * u[j];
        u[i] = s / M[i][i];
    }
}

int main()
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    const double PI = 3.14159265358979323846;
    const double scales[3] = {1e-4, 1.0, 1e4};

    int fails = 0, cases = 0;
    double worst_inv = 0, worst_w = 0;

    for (int sc = 0; sc < 3; sc++)
    for (int nint = 0; nint <= 3; nint++)
    for (int rep = 0; rep < 25; rep++) {
        // ---- build R = sigma^2 I + sum_k P_k s_k s_k^H (+ small random Hermitian jitter) ----
        cd R[N][N];
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) R[i][j] = (i == j) ? 1.0 : 0.0;
        for (int k = 0; k < nint; k++) {
            double th  = PI * U(rng) * 0.5;
            double pw  = std::pow(10.0, 1.0 + 3.0 * (U(rng) * 0.5 + 0.5));   // 10 .. 1e4
            cd s[N];
            for (int m = 0; m < N; m++) s[m] = std::polar(1.0, -PI * m * std::sin(th));
            for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) R[i][j] += pw * s[i] * std::conj(s[j]);
        }
        for (int i = 0; i < N; i++) for (int j = i; j < N; j++) {
            cd e(0.01 * U(rng), (i == j) ? 0.0 : 0.01 * U(rng));
            R[i][j] += e; if (j != i) R[j][i] += std::conj(e);
        }
        float Rr[N][N], Ri[N][N];
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) {
            Rr[i][j] = (float)(sc == 3 ? 0 : scales[sc] * R[i][j].real());
            Ri[i][j] = (float)(scales[sc] * R[i][j].imag());
        }
        double th0 = PI * U(rng) * 0.4;
        float a_re[N], a_im[N]; cd a[N];
        for (int m = 0; m < N; m++) {
            a[m] = std::polar(1.0, -PI * m * std::sin(th0));
            a_re[m] = (float)a[m].real(); a_im[m] = (float)a[m].imag();
        }

        // ---- loaded matrix in double from the SAME float inputs ----
        double tr = 0; for (int i = 0; i < N; i++) tr += Rr[i][i];
        float loading = (float)tr * (LOAD_FRAC / (float)N);
        cd A[N][N];
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++)
            A[i][j] = cd(Rr[i][j], Ri[i][j]) + ((i == j) ? (double)loading : 0.0);

        // ---- test 1: A * X ~ I ----
        float Xr[N][N], Xi[N][N];
        mvdr_rdr_invert(Rr, Ri, loading, Xr, Xi);
        double einv = 0;
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) {
            cd s = 0;
            for (int k = 0; k < N; k++) {
                cd x = (k <= j) ? cd(Xr[k][j], Xi[k][j]) : std::conj(cd(Xr[j][k], Xi[j][k]));
                s += A[i][k] * x;
            }
            s -= (i == j) ? 1.0 : 0.0;
            einv = std::fmax(einv, std::abs(s));
        }

        // ---- test 2: weights vs reference ----
        float wr[N], wi[N];
        solve_mvdr(Rr, Ri, a_re, a_im, wr, wi);
        cd u[N], wref[N];
        cd af[N]; for (int m = 0; m < N; m++) af[m] = cd(a_re[m], a_im[m]);
        ref_solve(A, af, u);
        cd den = 0; for (int m = 0; m < N; m++) den += std::conj(af[m]) * u[m];
        double wmax = 0, ew = 0;
        for (int m = 0; m < N; m++) { wref[m] = u[m] / den.real(); wmax = std::fmax(wmax, std::abs(wref[m])); }
        for (int m = 0; m < N; m++) ew = std::fmax(ew, std::abs(cd(wr[m], wi[m]) - wref[m]));
        ew /= wmax;

        worst_inv = std::fmax(worst_inv, einv);
        worst_w   = std::fmax(worst_w, ew);
        cases++;
        if (!(einv < 5e-3) || !(ew < 5e-3)) {
            fails++;
            if (fails < 10) std::printf("FAIL sc=%d nint=%d rep=%d  |AX-I|=%.3e  w_err=%.3e\n", sc, nint, rep, einv, ew);
        }
    }

    // ---- all-zero covariance: must stay finite ----
    {
        float Z[N][N] = {}, Zi[N][N] = {}, a_re[N], a_im[N], wr[N], wi[N];
        for (int m = 0; m < N; m++) { a_re[m] = 1.0f; a_im[m] = 0.0f; }
        solve_mvdr(Z, Zi, a_re, a_im, wr, wi);
        bool ok = true;
        for (int m = 0; m < N; m++) ok = ok && std::isfinite(wr[m]) && std::isfinite(wi[m]);
        std::printf("zero-matrix case: %s (w[0]=%.4f, expect 0.25)\n", ok ? "finite" : "NOT FINITE", wr[0]);
        cases++; if (!ok) fails++;
    }

    std::printf("cases=%d  worst |A*X-I|=%.3e  worst rel. w error=%.3e  -> %s\n",
                cases, worst_inv, worst_w, fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
