// ============================================================================
// wiener_block.cpp -- synthesisable implementation (see wiener_block.h for overview)
// ============================================================================
#include <cmath>
#include "wiener_block.h"

// ============================================================================
// Givens-rotation Wiener-Hopf solver (paper, Sec. II steps 2-5)
//   Rxx = Z^T U  with n(n-1)/2 rotations G_k;  rotate [Rxx | Rsx] -> [U | z],  U h = z.
// Runs once per block, NOT pipelined: HLS shares one set of float cores (small area).
// ============================================================================
void solve_wiener(const float rxx[TAPS], const float rdx[TAPS], float h[TAPS])
{
    float A[TAPS][TAPS], b[TAPS];

    const float diag = rxx[0] * (1.0f + LOAD_FRAC);
    for (int i = 0; i < TAPS; i++) {
        for (int j = 0; j < TAPS; j++) {
            int lag = (i > j) ? (i - j) : (j - i);
            A[i][j] = (lag == 0) ? diag : rxx[lag];
        }
        b[i] = rdx[i];
    }

    // ---- Givens QR: zero A[i][j] for i > j, column by column ----
    GIVENS_J:
    for (int j = 0; j < TAPS; j++) {
        GIVENS_I:
        for (int i = 0; i < TAPS; i++) {
            if (i > j) {
                float ajj = A[j][j], aij = A[i][j];
                float r2 = ajj * ajj + aij * aij;       // Uii^2 + Uji^2
                float c = 1.0f, s = 0.0f;
                if (r2 > PIV_MIN) {
                    float inv = 1.0f / sqrtf(r2);        // 1/sqrt(Uii^2+Uji^2)
                    c = ajj * inv;                       // c = Uii / sqrt(...)
                    s = aij * inv;                       // s = Uji / sqrt(...)
                }
                for (int k = 0; k < TAPS; k++) {
                    if (k >= j) {                        // columns < j are already zero
                        float t1 = c * A[j][k] + s * A[i][k];
                        float t2 = c * A[i][k] - s * A[j][k];
                        A[j][k] = t1;
                        A[i][k] = t2;
                    }
                }
                float u1 = c * b[j] + s * b[i];          // same rotation on Rsx  (z = Z.Rsx)
                float u2 = c * b[i] - s * b[j];
                b[j] = u1;
                b[i] = u2;
            }
        }
    }

    // ---- Back-substitution  U h = z ----
    BACKSUB:
    for (int ii = 0; ii < TAPS; ii++) {
        int i = TAPS - 1 - ii;
        float t = b[i];
        for (int k = 0; k < TAPS; k++)
            if (k > i) t -= A[i][k] * h[k];
        float piv = A[i][i];
        if (piv < PIV_MIN) piv = PIV_MIN;               // U_ii = r >= 0 by construction
        h[i] = t / piv;
    }
}

// ============================================================================
// Top function: one call processes one block
// ============================================================================
void wiener_block_top(const sample_t x[BLOCK], const sample_t d[BLOCK], out_t y[BLOCK],
                      int mode, bool reset, h_t h_used[TAPS])
{
    // Same area policy as mvdr_block: statistics/solver run once per block -> share one float core.
#pragma HLS ALLOCATION operation instances=fmul limit=1
#pragma HLS ALLOCATION operation instances=fadd limit=1
#pragma HLS ALLOCATION operation instances=fsub limit=1
#if WIENER_MUL_LIMIT > 0
#pragma HLS ALLOCATION operation instances=mul limit=WIENER_MUL_LIMIT
#endif

    // ---------------- Persistent state ----------------
    static float    rxx[TAPS], rnn[TAPS], rdx[TAPS];   // smoothed autocorr / noise autocorr / cross-corr
    static h_t      h[TAPS];                           // taps for the NEXT block
    static sample_t xline[TAPS];                       // input delay line, continues across blocks
    static bool initialised = false;
    static bool have_cov = false, have_noise = false, have_ref = false;
#pragma HLS ARRAY_PARTITION variable=rxx complete
#pragma HLS ARRAY_PARTITION variable=rnn complete
#pragma HLS ARRAY_PARTITION variable=rdx complete
#pragma HLS ARRAY_PARTITION variable=h complete
#pragma HLS ARRAY_PARTITION variable=xline complete

    if (reset || !initialised) {                       // quiescent start: pass-through
        RESET_LOOP:
        for (int l = 0; l < TAPS; l++) {
#pragma HLS PIPELINE off
            h[l] = (l == 0) ? h_t(1) : h_t(0);
            rxx[l] = 0.0f; rnn[l] = 0.0f; rdx[l] = 0.0f;
            xline[l] = 0;
        }
        have_cov = false; have_noise = false; have_ref = false;
        initialised = true;
    }

    // Latch taps used for THIS block.
    h_t hl[TAPS];
#pragma HLS ARRAY_PARTITION variable=hl complete
    LATCH_LOOP:
    for (int l = 0; l < TAPS; l++) {
#pragma HLS PIPELINE off
        hl[l] = h[l];
        h_used[l] = hl[l];
    }

    cov_acc_t acc_r[TAPS], acc_d[TAPS];
#pragma HLS ARRAY_PARTITION variable=acc_r complete
#pragma HLS ARRAY_PARTITION variable=acc_d complete
    for (int l = 0; l < TAPS; l++) { acc_r[l] = 0; acc_d[l] = 0; }
    const bool use_ref = (mode == WF_MODE_REF);

    // ------------- PASS 1: filter with old taps + accumulate correlations -------------
    // r[l] += x[n] x[n-l],   p[l] += d[n] x[n-l]   (delay line carries over block boundaries)
    SAMPLE_LOOP:
    for (int n = 0; n < BLOCK; n++) {
#pragma HLS PIPELINE II=WIENER_II
        for (int k = TAPS - 1; k > 0; k--) xline[k] = xline[k - 1];
        xline[0] = x[n];

        y_acc_t s = 0;
        for (int k = 0; k < TAPS; k++) s += hl[k] * xline[k];     // y[n] = sum h[k] x[n-k]
        y[n] = out_t(s);

        for (int l = 0; l < TAPS; l++) acc_r[l] += xline[0] * xline[l];
        if (use_ref) {
            sample_t dn = d[n];
            for (int l = 0; l < TAPS; l++) acc_d[l] += dn * xline[l];
        }
    }

    // ------------- PASS 2: smooth statistics, solve, update taps for NEXT block -------------
    const float INV_BLOCK = 1.0f / (float)BLOCK;       // exact power of two
    float br[TAPS], bd[TAPS];
    PASS2_CONV:
    for (int l = 0; l < TAPS; l++) {
#pragma HLS PIPELINE off
        br[l] = WF_F(acc_r[l]) * INV_BLOCK;
        bd[l] = use_ref ? WF_F(acc_d[l]) * INV_BLOCK : 0.0f;
    }

    if (mode == WF_MODE_NOISE) {
        // Noise-only block: learn Rnn, leave the filter untouched.
        for (int l = 0; l < TAPS; l++) {
#pragma HLS PIPELINE off
            rnn[l] = have_noise ? NOISE_FORGET * rnn[l] + (1.0f - NOISE_FORGET) * br[l] : br[l];
        }
        have_noise = true;
    } else {
        for (int l = 0; l < TAPS; l++) {
#pragma HLS PIPELINE off
            rxx[l] = have_cov ? FORGET * rxx[l] + (1.0f - FORGET) * br[l] : br[l];
            if (use_ref)
                rdx[l] = have_ref ? FORGET * rdx[l] + (1.0f - FORGET) * bd[l] : bd[l];
            else
                rdx[l] = rxx[l] - NOISE_OVERSUB * rnn[l];      // Rsx = Rss = Rxx - Rnn
        }
        have_cov = true;
        if (use_ref) have_ref = true;

        const bool can_solve = use_ref || have_noise;
        if (can_solve) {
            if (!use_ref && rdx[0] < 0.0f) {                   // noise >= signal power: mute
                for (int l = 0; l < TAPS; l++) rdx[l] = 0.0f;
            }
            float hf[TAPS];
            solve_wiener(rxx, rdx, hf);
            HCONV_LOOP:
            for (int l = 0; l < TAPS; l++) {
#pragma HLS PIPELINE off     // rolled -> ONE float->h_t converter
                h[l] = h_t(hf[l]);
            }
        }
    }
}
