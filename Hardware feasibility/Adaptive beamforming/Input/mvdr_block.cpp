// AI Generated Hardware synthesisable code

#include "mvdr_block.h"

// ----------------------------------------------------------------------------
// Shared type converters (one instance each, see ALLOCATION in the top function).
// ----------------------------------------------------------------------------
w_t mvdr_f2w(float x)
{
#pragma HLS INLINE off
    return w_t(x);
}

float mvdr_acc2f(cov_acc_t v)
{
#pragma HLS INLINE off
    return v.to_float();
}

// ============================================================================
// Phases 1 + 2 :  X = (A + load*I)^-1   with   A = R^H D R
//
//   R : unit upper triangular (diag implicit 1),  D = diag(d_i) real, no square roots.
//   Only the upper triangle of every matrix is stored / computed.
//
//   Phase 1 (row by row, i = 0..N-1)
//       d_i    = A_ii - sum_{k<i} Re( conj(R_ki) * T_ki )          (= |R_ki|^2 d_k)
//       inv_d  = 1 / d_i                                           (ONE reciprocal per row)
//       T_ij   = A_ij - sum_{k<i} conj(R_ki) * T_kj     (j > i)    T_kj = d_k * R_kj (undivided)
//       R_ij   = T_ij * inv_d_i
//   Phase 2 (direct back-substitution, i = N-1..0, j = N-1..i)
//       X_ij   = [i==j] * inv_d_i  -  sum_{k>i} R_ik * X_kj
//                where X_kj for k>j is read as conj(X_jk)  (X Hermitian, upper stored)
//
// Rolled loops (PIPELINE off): HLS builds ONE sequential datapath that re-uses the float
// cores allowed by the ALLOCATION pragmas of the top function. Roughly 36 multiplications
// + 4 reciprocals, as in the paper's 40-operation signal flow graph.
// ============================================================================
void mvdr_rdr_invert(const float Ar[NUM_ANT][NUM_ANT], const float Ai[NUM_ANT][NUM_ANT],
                     float load,
                     float Xr[NUM_ANT][NUM_ANT], float Xi[NUM_ANT][NUM_ANT])
{
#pragma HLS INLINE
    float RUr[NUM_ANT][NUM_ANT], RUi[NUM_ANT][NUM_ANT];   // R (strict upper part)
    float Tr [NUM_ANT][NUM_ANT], Ti [NUM_ANT][NUM_ANT];   // T = d_k * R_kj (undivided)
    float inv_d[NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=RUr complete dim=0
#pragma HLS ARRAY_PARTITION variable=RUi complete dim=0
#pragma HLS ARRAY_PARTITION variable=Tr  complete dim=0
#pragma HLS ARRAY_PARTITION variable=Ti  complete dim=0
#pragma HLS ARRAY_PARTITION variable=inv_d complete

    ZERO_I:
    for (int i = 0; i < NUM_ANT; i++)
        ZERO_J:
        for (int j = 0; j < NUM_ANT; j++) {
#pragma HLS PIPELINE off
            RUr[i][j] = 0.0f; RUi[i][j] = 0.0f;
            Tr[i][j]  = 0.0f; Ti[i][j]  = 0.0f;
            Xr[i][j]  = 0.0f; Xi[i][j]  = 0.0f;
        }

    // ---------------- Phase 1: factorisation A = R^H D R ----------------
    P1_I:
    for (int i = 0; i < NUM_ANT; i++) {
#pragma HLS PIPELINE off
        // pivot (real): needs only a real MAC per term
        float s = Ar[i][i] + load;
        P1_KD:
        for (int k = 0; k < NUM_ANT; k++) {
#pragma HLS PIPELINE off
            if (k < i) s -= RUr[k][i] * Tr[k][i] + RUi[k][i] * Ti[k][i];
        }
        if (s < PIV_MIN) s = PIV_MIN;
        inv_d[i] = 1.0f / s;

        P1_J:
        for (int j = 0; j < NUM_ANT; j++) {
#pragma HLS PIPELINE off
            if (j > i) {
                float tr = Ar[i][j], ti = Ai[i][j];
                P1_K:
                for (int k = 0; k < NUM_ANT; k++) {
#pragma HLS PIPELINE off
                    if (k < i) {                       // t -= conj(R_ki) * T_kj
                        tr -= RUr[k][i] * Tr[k][j] + RUi[k][i] * Ti[k][j];
                        ti -= RUr[k][i] * Ti[k][j] - RUi[k][i] * Tr[k][j];
                    }
                }
                Tr[i][j]  = tr;            Ti[i][j]  = ti;
                RUr[i][j] = tr * inv_d[i]; RUi[i][j] = ti * inv_d[i];
            }
        }
    }

    // ---------------- Phase 2: direct back-substitution for the upper triangle of X ---------
    P2_I:
    for (int ii = 0; ii < NUM_ANT; ii++) {
#pragma HLS PIPELINE off
        const int i = NUM_ANT - 1 - ii;
        P2_J:
        for (int jj = 0; jj < NUM_ANT; jj++) {
#pragma HLS PIPELINE off
            const int j = NUM_ANT - 1 - jj;
            if (j >= i) {
                float xr = (i == j) ? inv_d[i] : 0.0f;
                float xi = 0.0f;
                P2_K:
                for (int k = 0; k < NUM_ANT; k++) {
#pragma HLS PIPELINE off
                    if (k > i) {
                        float vr, vi;                  // X_kj (upper stored, else conjugate)
                        if (k <= j) { vr = Xr[k][j]; vi =  Xi[k][j]; }
                        else        { vr = Xr[j][k]; vi = -Xi[j][k]; }
                        xr -= RUr[i][k] * vr - RUi[i][k] * vi;     // x -= R_ik * X_kj
                        xi -= RUr[i][k] * vi + RUi[i][k] * vr;
                    }
                }
                if (i == j) xi = 0.0f;                 // diagonal of X is real
                Xr[i][j] = xr;
                Xi[i][j] = xi;
            }
        }
    }
}

// ============================================================================
// Per-block solver (Phase 3): w = X a / Re(a^H X a),  X = (R + loading I)^-1
// ============================================================================
void solve_mvdr(const float Rr[NUM_ANT][NUM_ANT], const float Ri[NUM_ANT][NUM_ANT],
                const float a_re[NUM_ANT], const float a_im[NUM_ANT],
                float w_re[NUM_ANT], float w_im[NUM_ANT])
{
    // Inlined into mvdr_block_top so that the ALLOCATION limits there govern the solver.
#pragma HLS INLINE
    // Diagonal loading, absolute value = LOAD_FRAC * trace(R) / M
    float trace = 0.0f;
    TRACE_LOOP:
    for (int i = 0; i < NUM_ANT; i++) {
#pragma HLS PIPELINE off
        trace += Rr[i][i];
    }
    const float loading = trace * (LOAD_FRAC / (float)NUM_ANT);

    float Xr[NUM_ANT][NUM_ANT], Xi[NUM_ANT][NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=Xr complete dim=0
#pragma HLS ARRAY_PARTITION variable=Xi complete dim=0
    mvdr_rdr_invert(Rr, Ri, loading, Xr, Xi);

    // ---- u = X a   (lower triangle of X read as conjugate of the upper) ----
    float ur[NUM_ANT], ui[NUM_ANT];
    U_I:
    for (int i = 0; i < NUM_ANT; i++) {
#pragma HLS PIPELINE off
        float sr = 0.0f, si = 0.0f;
        U_J:
        for (int j = 0; j < NUM_ANT; j++) {
#pragma HLS PIPELINE off
            float vr, vi;
            if (j >= i) { vr = Xr[i][j]; vi =  Xi[i][j]; }
            else        { vr = Xr[j][i]; vi = -Xi[j][i]; }
            sr += vr * a_re[j] - vi * a_im[j];
            si += vr * a_im[j] + vi * a_re[j];
        }
        ur[i] = sr;
        ui[i] = si;
    }

    // ---- normalise: a^H u is real and positive for Hermitian positive-definite R ----
    float den = 0.0f;
    DEN_LOOP:
    for (int i = 0; i < NUM_ANT; i++) {
#pragma HLS PIPELINE off
        den += a_re[i] * ur[i] + a_im[i] * ui[i];
    }
    if (den < PIV_MIN) den = PIV_MIN;
    const float inv_den = 1.0f / den;
    WOUT_LOOP:
    for (int i = 0; i < NUM_ANT; i++) {
#pragma HLS PIPELINE off
        w_re[i] = ur[i] * inv_den;
        w_im[i] = ui[i] * inv_den;
    }
}

// ============================================================================
// Top function: one call processes one block (unchanged apart from the fdiv limit)
// ============================================================================
void mvdr_block_top(const in_t x_re[BLOCK][NUM_ANT], const in_t x_im[BLOCK][NUM_ANT],
                    out_t y_re[BLOCK], out_t y_im[BLOCK],
                    const float a_re[NUM_ANT], const float a_im[NUM_ANT],
                    bool reset,
                    w_t w_used_re[NUM_ANT], w_t w_used_im[NUM_ANT])
{
    // All four sensors of one sample are read in the same cycle.
#pragma HLS ARRAY_PARTITION variable=x_re complete dim=2
#pragma HLS ARRAY_PARTITION variable=x_im complete dim=2

    // The solver and covariance smoothing run once per block: share ONE float multiplier,
    // adder, subtractor and divider across the whole design.
#pragma HLS ALLOCATION operation instances=fmul limit=1
#pragma HLS ALLOCATION operation instances=fadd limit=1
#pragma HLS ALLOCATION operation instances=fsub limit=1
#pragma HLS ALLOCATION operation instances=fdiv limit=1
#pragma HLS ALLOCATION function instances=mvdr_f2w limit=1
#pragma HLS ALLOCATION function instances=mvdr_acc2f limit=1
#if MVDR_MUL_LIMIT > 0
#pragma HLS ALLOCATION operation instances=mul limit=MVDR_MUL_LIMIT
#endif

    // ---------------- Persistent state (kept between blocks) ----------------
    static float Rr[NUM_ANT][NUM_ANT];      // smoothed covariance (Hermitian, stored in full)
    static float Ri[NUM_ANT][NUM_ANT];
    static w_t   w_re[NUM_ANT];             // weights to apply to the NEXT block
    static w_t   w_im[NUM_ANT];
    static bool  initialised = false;
    static bool  have_cov    = false;
#pragma HLS ARRAY_PARTITION variable=Rr complete dim=0
#pragma HLS ARRAY_PARTITION variable=Ri complete dim=0
#pragma HLS ARRAY_PARTITION variable=w_re complete
#pragma HLS ARRAY_PARTITION variable=w_im complete

    if (reset || !initialised) {            // quiescent start: w = a/M (delay-and-sum)
        RESET_W:
        for (int k = 0; k < 2 * NUM_ANT; k++) {
#pragma HLS PIPELINE off
            const int  m  = k % NUM_ANT;
            const bool im = (k >= NUM_ANT);
            w_t v = mvdr_f2w((im ? a_im[m] : a_re[m]) * (1.0f / (float)NUM_ANT));
            if (im) w_im[m] = v; else w_re[m] = v;
        }
        RESET_R_I:
        for (int m = 0; m < NUM_ANT; m++)
            RESET_R_J:
            for (int n = 0; n < NUM_ANT; n++) {
#pragma HLS PIPELINE off
                Rr[m][n] = 0.0f; Ri[m][n] = 0.0f;
            }
        have_cov = false;
        initialised = true;
    }

    // Latch the weights used for THIS block (the state is overwritten at the end).
    w_t wr[NUM_ANT], wi[NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=wr complete
#pragma HLS ARRAY_PARTITION variable=wi complete
    LATCH_LOOP:
    for (int m = 0; m < NUM_ANT; m++) {
#pragma HLS PIPELINE off
        wr[m] = w_re[m];
        wi[m] = w_im[m];
        w_used_re[m] = wr[m];
        w_used_im[m] = wi[m];
    }

    // Block covariance accumulators. Only the upper triangle is needed (Hermitian).
    cov_acc_t acc_r[NUM_ANT][NUM_ANT], acc_i[NUM_ANT][NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=acc_r complete dim=0
#pragma HLS ARRAY_PARTITION variable=acc_i complete dim=0
    for (int i = 0; i < NUM_ANT; i++)
        for (int j = 0; j < NUM_ANT; j++) { acc_r[i][j] = 0; acc_i[i][j] = 0; }

    // ------------- PASS 1: beamform with old weights + accumulate covariance -------------
    SAMPLE_LOOP:
    for (int n = 0; n < BLOCK; n++) {
#pragma HLS PIPELINE II=MVDR_II
        in_t xr[NUM_ANT], xi[NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=xr complete
#pragma HLS ARRAY_PARTITION variable=xi complete
        for (int m = 0; m < NUM_ANT; m++) { xr[m] = x_re[n][m]; xi[m] = x_im[n][m]; }

        // y = w^H x = sum_m conj(w_m) x_m
        y_acc_t sr = 0, si = 0;
        for (int m = 0; m < NUM_ANT; m++) {
            sr += wr[m] * xr[m] + wi[m] * xi[m];
            si += wr[m] * xi[m] - wi[m] * xr[m];
        }
        y_re[n] = out_t(sr);
        y_im[n] = out_t(si);

        // R_ij += x_i conj(x_j)   for j >= i
        for (int i = 0; i < NUM_ANT; i++) {
            for (int j = 0; j < NUM_ANT; j++) {
                if (j >= i) {
                    acc_r[i][j] += xr[i] * xr[j] + xi[i] * xi[j];
                    if (j > i) acc_i[i][j] += xi[i] * xr[j] - xr[i] * xi[j];
                }
            }
        }
    }

    // ------------- PASS 2: update covariance and compute weights for the NEXT block -------------
    const float INV_BLOCK = 1.0f / (float)BLOCK;      // exact power of two
    PASS2_I:
    for (int i = 0; i < NUM_ANT; i++) {
        PASS2_J:
        for (int j = 0; j < NUM_ANT; j++) {
#pragma HLS PIPELINE off
            if (j >= i) {
                float br = mvdr_acc2f(acc_r[i][j]) * INV_BLOCK;
                float bi = (j > i) ? mvdr_acc2f(acc_i[i][j]) * INV_BLOCK : 0.0f;
                float nr, ni;
                if (have_cov) {
                    nr = FORGET * Rr[i][j] + (1.0f - FORGET) * br;
                    ni = FORGET * Ri[i][j] + (1.0f - FORGET) * bi;
                } else {
                    nr = br;
                    ni = bi;
                }
                Rr[i][j] = nr;
                Ri[i][j] = ni;
                if (j > i) {                          // lower triangle = conjugate of upper
                    Rr[j][i] = nr;
                    Ri[j][i] = -ni;
                }
            }
        }
    }
    have_cov = true;

    float wr_f[NUM_ANT], wi_f[NUM_ANT];
    solve_mvdr(Rr, Ri, a_re, a_im, wr_f, wi_f);
    WCONV_LOOP:
    for (int k = 0; k < 2 * NUM_ANT; k++) {
#pragma HLS PIPELINE off
        const int  m  = k % NUM_ANT;
        const bool im = (k >= NUM_ANT);
        w_t v = mvdr_f2w(im ? wi_f[m] : wr_f[m]);
        if (im) w_im[m] = v; else w_re[m] = v;
    }
}

// ============================================================================
// OPTIONAL top: on-chip signal store.
// Use as synthesis top INSTEAD of mvdr_block_top if you want the Python signal kept in
// BRAM (up to MAX_BLOCKS blocks) and replayed block by block.
//   1) host calls cmd=LOAD for blk = 0..K-1 (one block per call)
//   2) host calls cmd=RUN  for blk = 0..K-1 in order, reset=true for blk 0
// mvdr_block_top is synthesised as a sub-module (called from ONE site only).
// ============================================================================
void mvdr_store_top(int cmd, int blk,
                    const in_t x_re[BLOCK][NUM_ANT], const in_t x_im[BLOCK][NUM_ANT],
                    out_t y_re[BLOCK], out_t y_im[BLOCK],
                    const float a_re[NUM_ANT], const float a_im[NUM_ANT],
                    bool reset,
                    w_t w_used_re[NUM_ANT], w_t w_used_im[NUM_ANT])
{
#pragma HLS ARRAY_PARTITION variable=x_re complete dim=2
#pragma HLS ARRAY_PARTITION variable=x_im complete dim=2

    // Whole-signal storage: one BRAM bank per sensor and per re/im part.
    static in_t sig_re[MAX_BLOCKS * BLOCK][NUM_ANT];
    static in_t sig_im[MAX_BLOCKS * BLOCK][NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=sig_re complete dim=2
#pragma HLS ARRAY_PARTITION variable=sig_im complete dim=2
#pragma HLS BIND_STORAGE variable=sig_re type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=sig_im type=ram_2p impl=bram

    // Working buffer holding the block that is currently processed.
    static in_t buf_re[BLOCK][NUM_ANT];
    static in_t buf_im[BLOCK][NUM_ANT];
#pragma HLS ARRAY_PARTITION variable=buf_re complete dim=2
#pragma HLS ARRAY_PARTITION variable=buf_im complete dim=2

    const int base = (blk & (MAX_BLOCKS - 1)) << LOG2_BLOCK;   // out-of-range blk wraps

    if (cmd == MVDR_CMD_LOAD) {
        LOAD_LOOP:
        for (int n = 0; n < BLOCK; n++) {
#pragma HLS PIPELINE II=1
            for (int m = 0; m < NUM_ANT; m++) {
                sig_re[base + n][m] = x_re[n][m];
                sig_im[base + n][m] = x_im[n][m];
            }
        }
    } else {
        FETCH_LOOP:
        for (int n = 0; n < BLOCK; n++) {
#pragma HLS PIPELINE II=1
            for (int m = 0; m < NUM_ANT; m++) {
                buf_re[n][m] = sig_re[base + n][m];
                buf_im[n][m] = sig_im[base + n][m];
            }
        }
        mvdr_block_top(buf_re, buf_im, y_re, y_im, a_re, a_im, reset, w_used_re, w_used_im);
    }
}
