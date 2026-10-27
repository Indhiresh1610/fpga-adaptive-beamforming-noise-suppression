// AI generated Code: Verified and Commented by the team
// Initialises variables, constants, parameters and functions which are used in the hardware synthesisable code for HLS. 

#ifndef MVDR_BLOCK_H
#define MVDR_BLOCK_H

#include <ap_fixed.h>

// ------------------------------ ULA and Block parameters -------------------------------
const int   NUM_ANT    = 4;                 // M, number of sensors
const int   LOG2_BLOCK = 10;                // represents the base-2 log of the block size 
const int   BLOCK      = 1 << LOG2_BLOCK;   // 1024 samples/block = 0.128 s at 8 kHz
// 1 is shifted by 10 bits in binary, which means it is 2^10 = 1024


// ------------------------------ SIGNAL STORAGE ------------------------------
// Used only by mvdr_store_top (optional wrapper, see below).
// Each stored block costs 1024 x 4 sensors x 2 (re/im) x 18 bit = 144 kbit ~ 4 BRAM36.
// XC7S50 has 75 BRAM36 (2.7 Mbit). MAX_BLOCKS = 8 -> 8192 samples = 1.02 s at 8 kHz,
// ~32 BRAM36 (43 %). Must be a power of two.
const int   LOG2_MAX_BLOCKS = 3;
const int   MAX_BLOCKS      = 1 << LOG2_MAX_BLOCKS;

// ------------------------------ MVDR parameters -----------------------------------
const float LOAD_FRAC = 1.0e-3f;  // diagonal loading fraction
const float FORGET    = 0.5f;    // covariance forgetting per block
const float PIV_MIN   = 1.0e-12f; // pivot floor: protects against divide-by-zero (all-zero input)

// ------------------------------ TIMING / THROUGHPUT -------------------------
#ifndef MVDR_MUL_LIMIT
#define MVDR_MUL_LIMIT 0
#endif

#ifndef MVDR_II
#define MVDR_II 8
#endif

// --------initialising the FIXED POINT TYPES used for different purposes--------- 

typedef ap_fixed<18, 5, AP_RND, AP_SAT> in_t;      // input samples, +/-16, LSB 2^-13: *must be kept in mind while creating inputs*

#ifndef MVDR_W_QMODE
#define MVDR_W_QMODE AP_RND
#endif
#ifndef MVDR_W_OMODE
#define MVDR_W_OMODE AP_SAT
#endif
typedef ap_fixed<25, 5, MVDR_W_QMODE, MVDR_W_OMODE> w_t;   // weights

typedef ap_fixed<24, 5, AP_RND, AP_SAT> out_t;     // output samples
typedef ap_fixed<48, 21> cov_acc_t;                // covariance accumulator (exact)
typedef ap_fixed<48, 12> y_acc_t;                  // beamformer accumulator (exact)

// below are all function declarations, which tell the compiler the function's interface. 

// ------------------------------ TOP FUNCTION --------------------------------
// One call = one block of BLOCK samples (all NUM_ANT sensors per sample).
// Interface identical to the previous version.
void mvdr_block_top(const in_t x_re[BLOCK][NUM_ANT], const in_t x_im[BLOCK][NUM_ANT],
                    out_t y_re[BLOCK], out_t y_im[BLOCK],
                    const float a_re[NUM_ANT], const float a_im[NUM_ANT],
                    bool reset,
                    w_t w_used_re[NUM_ANT], w_t w_used_im[NUM_ANT]);

// Above vars are: x-> input signal, across all sensors (matrix) ; y-> output signal (vector) ; a-> steering vector; w-> weights (vector)

// ------------------------------ OPTIONAL: SIGNAL STORE WRAPPER --------------
// Alternative synthesis top. Holds up to MAX_BLOCKS blocks of the Python-generated
// signal in on-chip BRAM and replays them through mvdr_block_top.
//   cmd = MVDR_CMD_LOAD : copy x_re/x_im into storage slot `blk` (no processing)
//   cmd = MVDR_CMD_RUN  : process stored slot `blk`; y_*, w_used_* are valid afterwards
// Blocks must be RUN in order 0,1,2,... (the covariance state is causal); use reset=true on
// the first one.
const int MVDR_CMD_LOAD = 0;
const int MVDR_CMD_RUN  = 1;
void mvdr_store_top(int cmd, int blk,
                    const in_t x_re[BLOCK][NUM_ANT], const in_t x_im[BLOCK][NUM_ANT],
                    out_t y_re[BLOCK], out_t y_im[BLOCK],
                    const float a_re[NUM_ANT], const float a_im[NUM_ANT],
                    bool reset,
                    w_t w_used_re[NUM_ANT], w_t w_used_im[NUM_ANT]);

// ------------------------------ SOLVER (exposed for unit tests) -------------
// Full inverse of Hermitian positive-definite (A + load*I) via A = R^H D R and direct
// back-substitution. A is given as a full Hermitian matrix (only the upper triangle is read).
// X (= inverse) is returned with its UPPER triangle valid (diagonal real); lower = conj of upper.
void mvdr_rdr_invert(const float Ar[NUM_ANT][NUM_ANT], const float Ai[NUM_ANT][NUM_ANT],
                     float load,
                     float Xr[NUM_ANT][NUM_ANT], float Xi[NUM_ANT][NUM_ANT]);

// Solves (R + loading*I) u = a, returns w = u / (a^H u). R is Hermitian (full matrix given).
void solve_mvdr(const float Rr[NUM_ANT][NUM_ANT], const float Ri[NUM_ANT][NUM_ANT],
                const float a_re[NUM_ANT], const float a_im[NUM_ANT],
                float w_re[NUM_ANT], float w_im[NUM_ANT]);

#endif
