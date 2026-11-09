"""fixed_point_model.py -- bit-accurate integer emulation of the CURRENT RTL (mvdr_top and children).

SEPARATE from the float64 reference.  It mirrors the RTL arithmetic with Python integers:
  sample_engine : exact 16x25-bit products, 44-bit accumulators, (acc+4096)>>13 half-up rounding to Q2.22,
                  24-bit saturation; 16-word covariance bank S (Q9.30, 39-bit) accumulated for full blocks only
  block_solver  : R update  R += (( (S-R)*3355443 + 2^23) >> 24)  (first block: R = S)
                  trace, power-of-two normalisation to Q16.24 (right shifts round half-up), delta = 2.5e-4*Tn,
                  143-entry micro-op ROM (ucode_rom.py, generated from block_solver.v) on 40-bit register file,
                  88-bit accumulator with round/saturate (+2^23 >>24), restoring divider with half-up rounding
                  (q + (2*rem >= d)), pivot clamp D_MIN, fallback w = a/4, weight scaling by two 40x40 products,
                  25-bit saturation to Q5.20
  controller    : block 0 uses w = a/4 (Q5.20: a_raw<<4); weights swap between blocks; tail has no accumulate/solve
Only the arithmetic is emulated, not clocks/handshakes (those do not change values; see README).
"""
import numpy as np
from ucode_rom import UCODE_ROM

K_MAC, K_PIV, K_S, K_WLO, K_WHI, K_END = 0, 1, 2, 3, 4, 7
LAMBDA_ONE_MINUS_COEF = 3355443     # round(0.2 * 2^24)
LOAD_COEF = 1073742                 # round(2.5e-4 * 2^32)
D_MIN = 1024                        # 2^-14 in Q16.24
RF_ONE, RF_RSLO, RF_RSHI = 60, 61, 62
PAIRS = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]


def wrap(v, bits):
    m = 1 << bits
    v &= m - 1
    return v - m if v >> (bits - 1) else v


def round_sat_acc(acc):
    """RTL round_sat_acc: +2^23, >>24, saturate to signed 40 bit. Returns (value, saturated)."""
    sh = (acc + (1 << 23)) >> 24
    if sh > (1 << 39) - 1:
        return (1 << 39) - 1, True
    if sh < -(1 << 39):
        return -(1 << 39), True
    return sh, False


def div_rnd(n, d):
    """serial_divider quotient_rnd: q + (2*rem >= d)."""
    q, r = divmod(n, d)
    return q + (1 if 2 * r >= d else 0)


class Solver:
    def __init__(self, a_re, a_im, block_len=76):
        self.a_re, self.a_im = [int(v) for v in a_re], [int(v) for v in a_im]
        self.R = [0] * 16
        self.have_cov = False
        self.acc_w = 32 + (block_len - 1).bit_length()

    def a_default_w(self):                          # w = a/4 in Q5.20 (a_raw << 4): [re0..re3, im0..im3]
        return [v << 4 for v in self.a_re] + [v << 4 for v in self.a_im]

    def solve(self, S):
        fl = dict(err_pivot=0, err_solver=0, sat_solver=0, sat_w=0, zero_trace=0)
        dg = {}
        # ---- covariance update ----
        if not self.have_cov:
            self.R = [int(s) for s in S]
            self.have_cov = True
        else:
            for k in range(16):
                prod = (int(S[k]) - self.R[k]) * LAMBDA_ONE_MINUS_COEF
                self.R[k] = self.R[k] + ((prod + 8388608) >> 24)
        R = self.R
        for k in range(16):
            if not -(1 << (self.acc_w - 1)) <= R[k] < (1 << (self.acc_w - 1)):
                raise OverflowError("covariance state exceeds ACC_W bits")
        # ---- trace / normalisation ----
        trace = R[0] + R[1] + R[2] + R[3]
        dg["trace_int"] = trace
        if trace == 0:
            fl["zero_trace"] = 1
            return self.a_default_w(), fl, dg
        msb = trace.bit_length() - 1
        dg["msb_p"] = msb

        def norm(r):
            r40 = wrap(r, 40)
            if msb <= 24:
                return wrap(r40 << (24 - msb), 40)
            return wrap((r40 + (1 << (msb - 25))) >> (msb - 24), 40)

        rf = [0] * 64
        for k in range(16):
            rf[k] = norm(R[k])
        tn = wrap(sum(rf[:4]), 40) & ((1 << 40) - 1)
        for k in range(4):
            rf[16 + k] = self.a_re[k] << 10
            rf[20 + k] = self.a_im[k] << 10
        rf[RF_ONE] = 1 << 24
        # ---- diagonal loading ----
        prod = (tn & 0xFFFFFFFF) * LOAD_COEF
        delta = ((prod + 0x80000000) & 0xFFFFFFFFFFFFFFFF) >> 32
        dg["tn"] = tn
        dg["delta"] = delta
        for k in range(4):
            rf[k] = wrap(rf[k] + delta, 40)
        dg["A_loaded"] = list(rf[:16])
        # ---- micro-op execution ----
        acc = 0
        w = [0] * 8
        pc = 0
        pivots = []
        fallback = False
        while True:
            kind, init, sub, last, ra, rb, dst = UCODE_ROM[pc]
            if kind == K_END:
                break
            p = rf[ra] * rf[rb]
            acc_new = wrap((0 if init else acc) + (-p if sub else p), 88)
            val, sat = round_sat_acc(acc_new)
            if kind == K_MAC:
                if last and sat:
                    fl["sat_solver"] = 1
                acc = acc_new
                if last:
                    rf[dst] = val
            elif kind == K_PIV:
                if sat:
                    fl["sat_solver"] = 1
                acc = acc_new
                d = val
                if d < D_MIN:
                    fl["err_pivot"] = 1
                    d = D_MIN
                pivots.append(val)
                rf[dst] = wrap(div_rnd(1 << 48, d), 40)
            elif kind == K_S:
                if sat:
                    fl["sat_solver"] = 1
                acc = acc_new
                dg["s_int"] = val
                if val <= 1024:
                    fl["err_solver"] = 1
                    fallback = True
                    break
                q = div_rnd(1 << 64, val)
                rf[RF_RSLO] = q & ((1 << 39) - 1)
                rf[RF_RSHI] = q >> 39
            elif kind == K_WLO:
                acc = (p + (1 << 43)) >> 39
            elif kind == K_WHI:
                sh = acc_new >> 5
                if sh > 16777215:
                    w[dst], fl["sat_w"] = 16777215, 1
                elif sh < -16777216:
                    w[dst], fl["sat_w"] = -16777216, 1
                else:
                    w[dst] = wrap(sh, 25)
            pc += 1
        dg["pivots"] = pivots
        dg["u_re"] = [rf[46], rf[47], rf[48], rf[55]]
        dg["u_im"] = [rf[49], rf[50], rf[51], rf[59]]
        if fallback:
            return self.a_default_w(), fl, dg
        return w, fl, dg


def beamform_sample(xr, xi, w):
    """sample_engine y path. xr/xi: 4 ints (Q1.15); w: [re0..3, im0..3] ints (Q5.20). Returns (re, im, sat)."""
    yre = sum(w[m] * xr[m] + w[4 + m] * xi[m] for m in range(4))
    yim = sum(w[m] * xi[m] - w[4 + m] * xr[m] for m in range(4))
    yre, yim = wrap(yre, 44), wrap(yim, 44)
    out, sat = [], False
    for v in (yre, yim):
        s = (v + 4096) >> 13
        if s > 8388607:
            s, sat = 8388607, True
        elif s < -8388608:
            s, sat = -8388608, True
        out.append(s)
    return out[0], out[1], sat


def accumulate_cov(S, xr, xi):
    for m in range(4):
        S[m] += xr[m] * xr[m] + xi[m] * xi[m]
    for p, (i, j) in enumerate(PAIRS):
        S[4 + p] += xr[i] * xr[j] + xi[i] * xi[j]          # Re(x_i conj(x_j))
        S[10 + p] += xi[i] * xr[j] - xr[i] * xi[j]         # Im(x_i conj(x_j))


def run_fixed_point(a_re, a_im, xr, xi, block_len=76, num_samples=2047):
    """xr, xi: int arrays (n, 4).  Returns dict with raw outputs, weights per block, S/R per block, flags."""
    sol = Solver(a_re, a_im, block_len)
    nblk, tail = num_samples // block_len, num_samples % block_len
    w_active = sol.a_default_w()
    W_used, W_solved, S_blocks, R_blocks, diag = [], [], [], [], []
    flags = dict(err_pivot=0, err_solver=0, sat_solver=0, sat_w=0, zero_trace=0, sat_y=0)
    y = np.zeros((num_samples, 2), dtype=np.int64)
    blk_of = np.zeros(num_samples, dtype=int)

    def run_block(b, n0, n1, accumulate):
        S = [0] * 16
        for n in range(n0, n1):
            r, i, sat = beamform_sample([int(v) for v in xr[n]], [int(v) for v in xi[n]], w_active)
            y[n] = (r, i)
            blk_of[n] = b
            flags["sat_y"] |= int(sat)
            if accumulate:
                accumulate_cov(S, [int(v) for v in xr[n]], [int(v) for v in xi[n]])
        return S

    for b in range(nblk):
        W_used.append(list(w_active))
        S = run_block(b, b * block_len, (b + 1) * block_len, True)
        w_new, fl, dg = sol.solve(S)
        for k in fl:
            flags[k] |= fl[k]
        S_blocks.append(S)
        R_blocks.append(list(sol.R))
        diag.append(dg)
        W_solved.append(list(w_new))
        w_active = list(w_new)                       # w_swap strictly between blocks
    if tail:
        W_used.append(list(w_active))
        run_block(nblk, nblk * block_len, num_samples, False)   # accum_en = 0, no solve
    return dict(y_raw=y, W_used=np.array(W_used, dtype=np.int64), W_solved=np.array(W_solved, dtype=np.int64),
                S=np.array(S_blocks, dtype=object), R=np.array(R_blocks, dtype=object), diag=diag, flags=flags,
                block_of_sample=blk_of, nblk=nblk, tail=tail)


def weights_to_complex(Wint):
    """[re0..re3, im0..im3] Q5.20 ints -> complex (rows = blocks)."""
    Wint = np.asarray(Wint, dtype=float)
    return (Wint[:, :4] + 1j * Wint[:, 4:]) / 2.0 ** 20


def cov_to_matrix(Rint):
    """16-word upper-triangle storage (Q9.30) -> full Hermitian float matrix."""
    R = np.zeros((4, 4), dtype=complex)
    for m in range(4):
        R[m, m] = Rint[m] / 2.0 ** 30
    for p, (i, j) in enumerate(PAIRS):
        R[i, j] = (Rint[4 + p] + 1j * Rint[10 + p]) / 2.0 ** 30
        R[j, i] = np.conj(R[i, j])
    return R
