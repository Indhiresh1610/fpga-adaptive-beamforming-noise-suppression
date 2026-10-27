// ============================================================================
// wiener_fd.h -- Causal STFT-domain Wiener post-filter for Vitis HLS (target XC7S50)
//
// Hardware port of the Python "enhance(noisy, 'Wiener')" path:
//   * STFT   : Hann (periodic), NPERSEG = 512, NOVERLAP = 384  (HOP = 128)
//   * noise  : PSD = mean(|Y|^2) over N_INIT_FRAMES = 20 noise-only frames
//   * xi     : decision-directed, alpha = 0.98
//                xi = a * A_prev^2/Pn + (1-a) * max(gamma-1, 0),   xi >= 1e-6
//                gamma = |Y|^2 / Pn,   A_prev = |G_prev * Y_prev|
//   * gain   : Wiener  G = xi / (1 + xi)
//   * ISTFT  : IFFT -> Hann synthesis window -> overlap-add, / sum(w^2) (= 1.5 for 75 % overlap)
//
// Differences to the Python (all forced by streaming/causality):
//   1. Noise PSD is learned ON-LINE from the first N_INIT_FRAMES full frames after reset
//      (Python looks at frames 0..19 of the whole file before processing).
//      During the first N_SKIP_FRAMES + N_INIT_FRAMES = 23 frames the output is muted (zeros).
//   2. Streaming latency = FRAME - HOP = 384 samples (24 ms @ 16 kHz).
//   3. No sqrt / exp needed: A_prev^2 = G^2 |Y|^2, so the only divider is G = xi/(1+xi).
//      (MMSE-STSA / log-MMSE would need Bessel / exp1 -> a different, bigger design.)
//   4. Output is delayed/aligned by the caller (see tb): block f returns samples
//      [HOP*f - 384, HOP*f - 384 + 127] of the input time line.
//
// Architecture: ONE call = ONE hop (128 new samples in, 128 finished samples out).
//   Real-time budget @16 kHz = 128 samples = 8 ms = 800 000 cycles @ 100 MHz.
//   Everything runs sequentially on ONE shared float FFT engine (used twice: FFT + IFFT).
//
// Compile with -DWFD_NO_AP to run on a PC without the Vitis headers (all float).
// ============================================================================
#ifndef WIENER_FD_H
#define WIENER_FD_H

#ifdef WFD_NO_AP
typedef float fd_sample_t;  typedef float fd_out_t;
#define WFD_F(x) ((float)(x))
#define WFD_D(x) ((double)(x))
#else
#include <ap_fixed.h>
// Same I/O types as the time-domain Wiener / MVDR blocks (chainable).
typedef ap_fixed<18, 5, AP_RND, AP_SAT> fd_sample_t;   // +/-16, LSB 2^-13
typedef ap_fixed<24, 5, AP_RND, AP_SAT> fd_out_t;      // +/-16, LSB 2^-19
#define WFD_F(x) ((x).to_float())
#define WFD_D(x) ((x).to_double())
#endif

// ------------------------------ SIZES ---------------------------------------
const int LOG2_FRAME = 9;
const int FRAME      = 1 << LOG2_FRAME;     // 512  (NPERSEG)
const int HOP        = 128;                 // FRAME - NOVERLAP(384)
const int NBINS      = FRAME / 2 + 1;       // 257 one-sided bins
const int N_INIT_FRAMES = 20;               // noise-only frames averaged (Python: N_INIT_FRAMES)
const int N_SKIP_FRAMES = FRAME / HOP - 1;  // 3 start-up frames that are partly zero-filled
const int LATENCY_SAMPLES = FRAME - HOP;    // 384

// ------------------------------ ALGORITHM -----------------------------------
const float DD_ALPHA  = 0.98f;              // decision-directed smoothing (Python dd_alpha)
const float XI_MIN    = 1.0e-6f;            // Python: xi = max(xi, 1e-6)
const float EPS       = 1.0e-12f;           // Python eps in noise_psd + eps
const float OLA_NORM  = 1.0f / 1.5f;        // 1 / sum(hann^2) for 75 % overlap
const float INV_FRAME = 1.0f / (float)FRAME;
const float INV_NINIT = 1.0f / (float)N_INIT_FRAMES;

// ------------------------------ TIMING / AREA -------------------------------
// WFD_SHARE=1 : limit float fmul/fadd/fsub to ONE instance each (same area policy as wiener_block).
//               Set -DWFD_SHARE=0 to let HLS use as many cores as the pipeline wants.
#ifndef WFD_SHARE
#define WFD_SHARE 1
#endif
#ifndef WFD_FFT_II
#define WFD_FFT_II 4        // butterfly loop (4 fmul -> II>=4 with one multiplier)
#endif
#ifndef WFD_GAIN_II
#define WFD_GAIN_II 16      // per-bin gain loop (~11 fmul + 1 fdiv)
#endif

// ------------------------------ TOP -----------------------------------------
//   x           : HOP new input samples (e.g. beamformer output)
//   y           : HOP finished output samples (delayed by LATENCY_SAMPLES, zeros while learning noise)
//   reset       : forget all state (empty buffers, restart noise learning)
//   noise_ready : true once the noise PSD is learned and the gain is active
void wiener_fd_top(const fd_sample_t x[HOP], fd_out_t y[HOP], bool reset, bool *noise_ready);

// Exposed for unit testing: in-place 512-pt radix-2 DIT FFT.
// Input must be in BIT-REVERSED order (use wfd_bitrev), output is natural order. No scaling.
void wfd_fft(float re[FRAME], float im[FRAME]);
int  wfd_bitrev(int n);

#endif
