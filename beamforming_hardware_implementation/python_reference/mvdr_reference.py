"""mvdr_reference.py -- INDEPENDENT float64 reference of the causal block MVDR beamformer.

Written from the RTL specification (not from the earlier Python scripts, not from RTL arithmetic):

  covariance of block b      S_b = sum_{n in block} x_n x_n^H                 (sum, not mean; scale-free)
  recursive covariance       R_0 = S_0 ;  R_b = lam*R_{b-1} + (1-lam)*S_b      (RTL: R + 0.2*(S-R), lam = 0.8)
  diagonal loading           A_b = R_b + loading*trace(R_b)/M * I              (loading = 1e-3, M = 4)
  MVDR weights               u = solve(A_b, a) ;  w = u / (a^H u)             (a^H u real, > 0)
  beamformer                 y_n = w^H x_n
  causality                  block 0 uses w = a/M; block b>=1 uses the weights solved from R_{b-1}.
                             The solve after block b uses R_b and is first used by block b+1.
  tail (N mod K samples)     no covariance accumulation, no solve; uses the weights of the last full block.
  RTL fallbacks              trace(R) == 0, or s <= 2^-14 on the power-of-two-normalised scale -> w = a/M.
                             (RTL also clamps pivots < 2^-14; the float model only REPORTS them.)

The RTL normalises R by a power of two so that trace in [1,2) before loading/solving. That scaling leaves the
weights unchanged mathematically; it only matters for the fixed thresholds (2^-14), so the fallback tests are
evaluated on the normalised scale exactly as the RTL does.
"""
from dataclasses import dataclass
import numpy as np

D_MIN = 2.0 ** -14


@dataclass
class RefConfig:
    num_ant: int = 4
    block_len: int = 76
    num_samples: int = 2047
    fs_hz: float = 512.0
    forgetting: float = 0.8
    loading: float = 1e-3        # fraction of trace/M
    causal: bool = True          # False is ONLY used for sensitivity experiments


def ldl_pivots(A):
    """Pivots d_k of A = L D L^H (plain Gaussian elimination, no pivoting) -- diagnostic only."""
    A = A.astype(complex).copy()
    d = []
    for k in range(A.shape[0]):
        dk = A[k, k].real
        d.append(dk)
        if k + 1 < A.shape[0]:
            col = A[k + 1:, k].copy()
            A[k + 1:, k + 1:] -= np.outer(col, col.conj()) / dk
    return np.array(d)


def run_float_reference(a, X, cfg=RefConfig()):
    M, K, N = cfg.num_ant, cfg.block_len, cfg.num_samples
    X = X[:, :N]
    nblk, tail = N // K, N % K
    I = np.eye(M)
    W_used = np.zeros((nblk + (1 if tail else 0), M), dtype=complex)   # weights used by block b (b = nblk is the tail)
    W_solved = np.zeros((nblk, M), dtype=complex)                      # weights produced by the solve after block b
    diag = []                                                          # per-solve diagnostics
    R = None
    Rs, Ss = [], []
    flags = dict(fallback_zero_trace=0, fallback_s=0, pivot_below_dmin=0)

    def solve_from(R):
        tr = np.trace(R).real
        info = dict(trace=tr)
        if tr == 0:
            flags["fallback_zero_trace"] += 1
            info.update(fallback=True, min_eig=np.nan, cond=np.nan, s_norm=np.nan, pivots=np.full(M, np.nan))
            return a / M, info
        scale = 2.0 ** (-np.floor(np.log2(tr)))                         # RTL: power-of-two so trace_norm in [1,2)
        A = (R + cfg.loading * tr / M * I) * scale                      # loaded, normalised
        ev = np.linalg.eigvalsh(A)
        piv = ldl_pivots(A)
        if np.any(piv < D_MIN):
            flags["pivot_below_dmin"] += 1
        u = np.linalg.solve(A, a)
        s = float(np.real(np.vdot(a, u)))                               # a^H u
        info.update(min_eig=ev[0], max_eig=ev[-1], cond=ev[-1] / ev[0], s_norm=s, pivots=piv, fallback=False,
                    imag_s=float(np.imag(np.vdot(a, u))), scale=scale)
        if s <= D_MIN:
            flags["fallback_s"] += 1
            info["fallback"] = True
            return a / M, info
        return u / s, info

    w_prev = a / M
    for b in range(nblk):
        Xb = X[:, b * K:(b + 1) * K]
        S = Xb @ Xb.conj().T
        R = S.copy() if R is None else cfg.forgetting * R + (1.0 - cfg.forgetting) * S
        Ss.append(S)
        Rs.append(R.copy())
        w_new, info = solve_from(R)
        diag.append(info)
        W_solved[b] = w_new
        W_used[b] = w_prev                                              # causal: weights available BEFORE block b
        w_prev = w_new
    if tail:
        W_used[nblk] = w_prev                                           # tail: weights of the last full block
    # non-causal experiment: block b uses the weights solved from its own data
    if not cfg.causal:
        W_used[:nblk] = W_solved

    Wn = np.empty((M, N), dtype=complex)
    blk_of = np.empty(N, dtype=int)
    for n in range(N):
        b = n // K                                                      # n//K == nblk for tail samples
        blk_of[n] = b
        Wn[:, n] = W_used[b]
    Y = np.einsum("mn,mn->n", Wn.conj(), X)                             # y = w^H x
    return dict(Y=Y, W_used=W_used, W_solved=W_solved, S=Ss, R=Rs, diag=diag, flags=flags,
                block_of_sample=blk_of, nblk=nblk, tail=tail, cfg=cfg)


def write_outputs(res, y_path, w_path):
    import csv
    with open(y_path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["idx", "block", "re_value", "im_value"])
        for n, (y, b) in enumerate(zip(res["Y"], res["block_of_sample"])):
            wr.writerow([n, int(b), f"{y.real:.12e}", f"{y.imag:.12e}"])
    with open(w_path, "w", newline="") as f:
        wr = csv.writer(f)
        hdr = ["block", "used_by_samples"] + [f"w{m}_{c}" for m in range(res["cfg"].num_ant) for c in ("re", "im")]
        wr.writerow(hdr)
        for b, w in enumerate(res["W_used"]):
            lab = "tail" if b == res["nblk"] else "block"
            row = [b, lab] + [f"{v:.12e}" for m in range(len(w)) for v in (w[m].real, w[m].imag)]
            wr.writerow(row)
