// ============================================================================
// wiener_block.h -- Causal BLOCK-adaptive Wiener-Hopf FIR filter for Vitis HLS
//                   (post-filter after the MVDR beamformer; target XC7S50)
//
// Based on: M. A. Basiri, "An Efficient FPGA Implementation of Wiener Filter" (CICT 2024)
//   * Wiener-Hopf equation        H = Rxx^-1 . Rsx
//   * Rxx inverted via Givens-rotation QR (n(n-1)/2 rotations, c = Uii/sqrt(Uii^2+Uji^2),
//     s = Uji/sqrt(...)), exactly as in the paper.
//   * Paper's Rxx^-1 = U^-1 Z is NOT formed explicitly: the same rotations are applied to the
//     right-hand side (z = Z.Rsx) and U h = z is solved by back-substitution. Mathematically
//     identical to H = U^-1 Z Rsx, but needs O(n^2) work instead of the O(n^4) adjoint/determinant
//     inverse, and no n x n Z matrix is stored.
//
// Same architecture/conventions as mvdr_block (so the two chain together):
//   * Weights applied to block k are computed ONLY from blocks 0..k-1 (causal).
//   * Block 0 after reset: pass-through h = [1 0 0 ...].
//   * Per-SAMPLE path -> ap_fixed (DSP friendly). Per-BLOCK solver -> 32-bit float Givens.
//   * Statistics smoothed per block: R <- FORGET*R + (1-FORGET)*R_block.
//
// Where does the "desired" signal Rsx come from?  Three block modes (argument `mode`):
//   WF_MODE_REF   : a clean reference d[] is supplied (paper's setup, s known).  Rsx = E[d x].
//   WF_MODE_NOISE : block is NOISE ONLY (initial segment / VAD says speech absent).
//                   Learns the noise autocorrelation Rnn, filter is left unchanged.
//   WF_MODE_RUN   : normal operation. s and noise uncorrelated => Rsx = Rss = Rxx - Rnn.
//                   (Needs a prior WF_MODE_NOISE block, like spectral subtraction needs mu(k).)
// ============================================================================
#ifndef WIENER_BLOCK_H
#define WIENER_BLOCK_H

// Compile with -DWIENER_NO_AP to test on a PC without the Vitis headers (all float).
#ifdef WIENER_NO_AP
typedef float sample_t;  typedef float h_t;  typedef float out_t;
typedef float cov_acc_t; typedef float y_acc_t;
#define WF_F(x) ((float)(x))
#define WF_D(x) ((double)(x))
#else
#include <ap_fixed.h>
// Samples: identical to mvdr in_t (18 bit = DSP48 port, range +/-16, LSB 2^-13).
typedef ap_fixed<18, 5, AP_RND, AP_SAT> sample_t;
// Filter taps: 25 bit = DSP48 port, range +/-16, LSB 2^-20.
typedef ap_fixed<25, 5, AP_RND, AP_SAT> h_t;
// Output: same as mvdr out_t.
typedef ap_fixed<24, 5, AP_RND, AP_SAT> out_t;
// Correlation accumulators, exact: 26 frac bits (13+13), +10 bits for BLOCK=1024, +10 int bits.
typedef ap_fixed<48, 21> cov_acc_t;
// FIR sum: 13 + 20 = 33 frac bits, +3 bits for 8 taps.
typedef ap_fixed<48, 12> y_acc_t;
#define WF_F(x) ((x).to_float())
#define WF_D(x) ((x).to_double())
#endif

// ------------------------------ SIZES ---------------------------------------
const int TAPS       = 8;                  // FIR length N (Rxx is N x N Toeplitz). Paper: n x n.
const int LOG2_BLOCK = 10;
const int BLOCK      = 1 << LOG2_BLOCK;    // 1024 samples; block average = shift

// ------------------------------ ALGORITHM -----------------------------------
const float LOAD_FRAC     = 1.0e-3f;  // diagonal loading: r0 -> r0*(1+LOAD_FRAC) (conditioning)
const float FORGET        = 0.90f;    // Rxx / Rdx smoothing per block
const float NOISE_FORGET  = 0.90f;    // Rnn smoothing per block
const float NOISE_OVERSUB = 1.0f;     // Rsx = Rxx - NOISE_OVERSUB*Rnn  (>1 = more aggressive)
const float PIV_MIN       = 1.0e-12f; // floor for sqrt/divide protection

enum { WF_MODE_REF = 0, WF_MODE_NOISE = 1, WF_MODE_RUN = 2 };

// ------------------------------ TIMING --------------------------------------
// Per-sample loop does 2*TAPS (correlation) + TAPS (FIR) multiplies -> 24 (32 in REF mode).
// At II=8 the sample path needs >= 3-4 multipliers; use -DWIENER_MUL_LIMIT=N to enforce it
// (II alone did not reduce DSPs in the MVDR design either).
#ifndef WIENER_II
#define WIENER_II 8
#endif
#ifndef WIENER_MUL_LIMIT
#define WIENER_MUL_LIMIT 0
#endif

// ------------------------------ TOP -----------------------------------------
//   x       : input block (e.g. beamformer output, real, converted to sample_t)
//   d       : clean reference block (only read when mode == WF_MODE_REF; pass x otherwise)
//   y       : filtered output (uses taps computed from PREVIOUS blocks)
//   mode    : WF_MODE_REF / WF_MODE_NOISE / WF_MODE_RUN
//   reset   : forget all state (pass-through taps, empty statistics, empty delay line)
//   h_used  : taps applied to this block (debug / verification)
void wiener_block_top(const sample_t x[BLOCK], const sample_t d[BLOCK], out_t y[BLOCK],
                      int mode, bool reset, h_t h_used[TAPS]);

// Exposed for unit testing. rxx[l], rdx[l], l = 0..TAPS-1 define the Toeplitz Rxx
// (A[i][j] = rxx[|i-j|]) and Rsx. Solves A h = rdx via Givens QR + back-substitution.
void solve_wiener(const float rxx[TAPS], const float rdx[TAPS], float h[TAPS]);

#endif
