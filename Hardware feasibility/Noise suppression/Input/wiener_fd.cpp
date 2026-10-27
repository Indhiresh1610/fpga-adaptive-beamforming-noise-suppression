// ============================================================================
// wiener_fd.cpp -- synthesisable implementation (see wiener_fd.h for overview)
// ============================================================================
#include "wiener_fd.h"
#include "wiener_fd_tables.h"

// 9-bit bit reversal (pure wiring after unroll)
int wfd_bitrev(int n)
{
    int r = 0;
    BITREV:
    for (int i = 0; i < LOG2_FRAME; i++) {
#pragma HLS UNROLL
        r = (r << 1) | ((n >> i) & 1);
    }
    return r;
}

// ============================================================================
// In-place radix-2 DIT FFT, 512 points, single shared butterfly.
// Not inlined -> forward FFT and inverse FFT share the SAME hardware.
// ============================================================================
void wfd_fft(float re[FRAME], float im[FRAME])
{
#pragma HLS INLINE off
#if WFD_SHARE
#pragma HLS ALLOCATION operation instances=fmul limit=1
#pragma HLS ALLOCATION operation instances=fadd limit=1
#pragma HLS ALLOCATION operation instances=fsub limit=1
#endif
    FFT_STAGE:
    for (int s = 0; s < LOG2_FRAME; s++) {
        const int half = 1 << s;
        FFT_BFLY:
        for (int b = 0; b < FRAME / 2; b++) {
#pragma HLS PIPELINE II=WFD_FFT_II
            // pairs inside one stage are disjoint -> no loop-carried dependence
#pragma HLS DEPENDENCE variable=re type=inter dependent=false
#pragma HLS DEPENDENCE variable=im type=inter dependent=false
            const int j  = b & (half - 1);
            const int i0 = ((b >> s) << (s + 1)) | j;
            const int i1 = i0 + half;
            const int tw = j << (LOG2_FRAME - 1 - s);
            const float cw = WFD_COS[tw];
            const float sw = -WFD_SIN[tw];                 // W = cos - j sin
            const float ar = re[i0], ai = im[i0];
            const float br = re[i1], bi = im[i1];
            const float tr = br * cw - bi * sw;
            const float ti = br * sw + bi * cw;
            re[i0] = ar + tr;  im[i0] = ai + ti;
            re[i1] = ar - tr;  im[i1] = ai - ti;
        }
    }
}

// ============================================================================
// Top function: one call processes one hop
// ============================================================================
void wiener_fd_top(const fd_sample_t x[HOP], fd_out_t y[HOP], bool reset, bool *noise_ready)
{
#if WFD_SHARE
#pragma HLS ALLOCATION operation instances=fmul limit=1
#pragma HLS ALLOCATION operation instances=fadd limit=1
#pragma HLS ALLOCATION operation instances=fsub limit=1
#endif

    // ---------------- Persistent state ----------------
    static fd_sample_t xbuf[FRAME];     // circular input history (last 512 samples)
    static float       ola[FRAME];      // circular overlap-add accumulator
    static float       noise[NBINS];    // noise PSD sum while learning, then 1/(PSD+eps)
    static float       aprev2[NBINS];   // A_prev^2 = (G*|Y|)^2 of the previous frame
    static int         wr = 0;          // circular index of the OLDEST sample of the current frame
    static int         fcnt = 0;        // frames processed since reset (saturating)
    static bool        ready = false, initialised = false;

    if (reset || !initialised) {
        RESET_FRAME:
        for (int i = 0; i < FRAME; i++) {
#pragma HLS PIPELINE II=1
            xbuf[i] = 0;
            ola[i]  = 0.0f;
        }
        RESET_BIN:
        for (int k = 0; k < NBINS; k++) {
#pragma HLS PIPELINE II=1
            noise[k]  = 0.0f;
            aprev2[k] = 0.0f;
        }
        wr = 0; fcnt = 0; ready = false; initialised = true;
    }

    // ---------------- Push HOP new samples (they become the NEWEST 128 of the frame) ----------------
    PUSH:
    for (int i = 0; i < HOP; i++) {
#pragma HLS PIPELINE II=1
        xbuf[(wr + FRAME - HOP + i) & (FRAME - 1)] = x[i];
    }

    const bool learn = (fcnt >= N_SKIP_FRAMES) && (fcnt < N_SKIP_FRAMES + N_INIT_FRAMES);
    const bool enh   = ready && !learn;

    if (learn || enh) {
        float re[FRAME], im[FRAME];     // analysis spectrum   (-> BRAM)
        float re2[FRAME], im2[FRAME];   // gained, conjugated spectrum for the inverse FFT

        // ---- window + load in bit-reversed order ----
        LOAD:
        for (int n = 0; n < FRAME; n++) {
#pragma HLS PIPELINE II=2
#pragma HLS DEPENDENCE variable=re type=inter dependent=false
#pragma HLS DEPENDENCE variable=im type=inter dependent=false
            const float v = WFD_F(xbuf[(wr + n) & (FRAME - 1)]) * WFD_HANN[n];
            const int r = wfd_bitrev(n);
            re[r] = v;
            im[r] = 0.0f;
        }

        wfd_fft(re, im);                                 // Y[k], k = 0..511 (only 0..256 used)

        if (learn) {
            // ---- noise PSD accumulation over N_INIT_FRAMES full frames ----
            NOISE_ACC:
            for (int k = 0; k < NBINS; k++) {
#pragma HLS PIPELINE II=4
                noise[k] += re[k] * re[k] + im[k] * im[k];
            }
            if (fcnt == N_SKIP_FRAMES + N_INIT_FRAMES - 1) {
                NOISE_INV:                               // store 1/(mean + eps): no divider in the main loop
                for (int k = 0; k < NBINS; k++) {
#pragma HLS PIPELINE off
                    noise[k] = 1.0f / (noise[k] * INV_NINIT + EPS);
                }
                ready = true;
            }
        } else {
            // ---- per-bin Wiener gain with decision-directed xi ----
            GAIN_LOOP:
            for (int k = 0; k < NBINS; k++) {
#pragma HLS PIPELINE II=WFD_GAIN_II
#pragma HLS DEPENDENCE variable=re2 type=inter dependent=false
#pragma HLS DEPENDENCE variable=im2 type=inter dependent=false
                const float yr = re[k], yi = im[k];
                const float P   = yr * yr + yi * yi;              // |Y|^2
                const float inv = noise[k];                       // 1/Pn
                float gm1 = P * inv - 1.0f;                       // gamma - 1
                if (gm1 < 0.0f) gm1 = 0.0f;
                float xi = DD_ALPHA * (aprev2[k] * inv) + (1.0f - DD_ALPHA) * gm1;
                if (xi < XI_MIN) xi = XI_MIN;
                const float G = xi / (1.0f + xi);                 // Wiener gain
                aprev2[k] = G * G * P;                            // A_prev^2 for the next frame
                const float gr = G * yr, gi = G * yi;

                // conj(Z[k]) at k, conj(Z[512-k]) = Z[k] at 512-k  -> IFFT(Z) = conj(FFT(conj Z))/N
                const int r0 = wfd_bitrev(k);
                re2[r0] = gr;  im2[r0] = -gi;
                if (k > 0 && k < FRAME / 2) {
                    const int r1 = wfd_bitrev(FRAME - k);
                    re2[r1] = gr;  im2[r1] = gi;
                }
            }

            wfd_fft(re2, im2);                                    // real part = N * time signal

            // ---- synthesis window + overlap-add ----
            OLA_ADD:
            for (int n = 0; n < FRAME; n++) {
#pragma HLS PIPELINE II=4
#pragma HLS DEPENDENCE variable=ola type=inter dependent=false
                const int idx = (wr + n) & (FRAME - 1);
                ola[idx] += re2[n] * INV_FRAME * WFD_HANN[n];
            }
        }
    }

    // ---------------- Emit the oldest HOP samples (now complete), free them ----------------
    OUT:
    for (int i = 0; i < HOP; i++) {
#pragma HLS PIPELINE II=2
        const int idx = (wr + i) & (FRAME - 1);
        y[i] = fd_out_t(ola[idx] * OLA_NORM);
        ola[idx] = 0.0f;
    }

    wr = (wr + HOP) & (FRAME - 1);
    if (fcnt < N_SKIP_FRAMES + N_INIT_FRAMES) fcnt++;
    *noise_ready = ready;
}
