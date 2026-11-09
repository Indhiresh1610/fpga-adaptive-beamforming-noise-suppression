"""compare_rtl.py -- compare Python models against the RTL simulation output (y_out.csv from XSim).

NOTE: y_out.csv is a SIMULATION capture (tb_save_output.v), not data captured from the FPGA board.

Tolerance derivation (float64 reference vs RTL), see derive_tolerance():
  y_re = sum_m (wr_m xr_m + wi_m xi_m),  y_im = sum_m (wr_m xi_m - wi_m xr_m)  ->  |dy_re|,|dy_im| <= ||dw||_2 * ||x||_2
  (Cauchy-Schwarz on the 8-real-vector (wr,wi) against (xr,xi)), plus 0.5 LSB of Q2.22 from the output rounding.
  ||dw||_2 <= eps_solve + eps_q where
    eps_q     = sqrt(8) * 2^-21          Q5.20 final rounding, half-LSB (2^-21) on each of 8 real components
    eps_solve = 2 * kappa(A) * c * u * ||w||_2
        A      = normalised, loaded covariance (trace in [1,2)) used to produce the weights of that block
        kappa  = cond_2(A) of that matrix (measured from the float model, per block)
        u      = 2^-25                   half-ulp of the Q16.24 solver words (all stored entries/products round to Q.24)
        c      = 16                      allowance for the ~n^2 sequential roundings of a 4x4 LDL^H solve (backward-error
                                          constant of a Cholesky-type factorisation, ||E||_2 <= c*u*||A||_2)
        factor 2 : the relative perturbation enters both u = A^-1 a and the normaliser s = a^H u.
  Block 0 uses w = a/4, exactly representable in Q5.20 (a_raw << 4) -> eps = 0.
  This is a first-order WORST-CASE bound: it is deliberately not tuned to the observed error.
"""
import csv
import numpy as np

LSB_Y = 2.0 ** -22
U_SOLVER = 2.0 ** -25
C_CHAIN = 16.0


def load_rtl_csv(path):
    idx, blk, re, im = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            idx.append(int(row["idx"])); blk.append(int(row["block"]))
            re.append(int(row["re_raw"])); im.append(int(row["im_raw"]))
    d = dict(idx=np.array(idx), block=np.array(blk), re_raw=np.array(re), im_raw=np.array(im))
    d["y"] = (d["re_raw"] + 1j * d["im_raw"]) / 2.0 ** 22
    return d


def structural_checks(rtl, num_samples, block_len):
    nblk = num_samples // block_len
    exp_blk = np.minimum(np.arange(num_samples) // block_len, nblk)
    return dict(
        n_rows=len(rtl["idx"]), n_rows_ok=len(rtl["idx"]) == num_samples,
        idx_contiguous=bool(np.array_equal(rtl["idx"], np.arange(len(rtl["idx"])))),
        block_labels_ok=bool(np.array_equal(rtl["block"], exp_blk[:len(rtl["block"])])),
        max_abs_raw=int(max(np.abs(rtl["re_raw"]).max(), np.abs(rtl["im_raw"]).max())),
    )


def derive_tolerance(ref, X):
    """Per-sample tolerance array (same bound for re and im) following the derivation in the module docstring."""
    K, N = ref["cfg"].block_len, ref["cfg"].num_samples
    nblk = ref["nblk"]
    xn = np.linalg.norm(X[:, :N], axis=0)
    eps_q = np.sqrt(8.0) * 2.0 ** -21
    eps_w = np.zeros(len(ref["W_used"]))
    for b in range(1, len(ref["W_used"])):
        src = min(b - 1, nblk - 1)                      # weights of block b come from the solve after block b-1 (tail: last)
        dg = ref["diag"][src]
        wn = np.linalg.norm(ref["W_used"][b])
        eps_w[b] = 2.0 * dg["cond"] * C_CHAIN * U_SOLVER * wn + eps_q
    blk = ref["block_of_sample"]
    tol = eps_w[blk] * xn + 0.5 * LSB_Y
    return tol, eps_w


def err_stats(y_ref, y_rtl, floor=None):
    d = y_ref - y_rtl
    if floor is None:
        floor = 0.01 * np.sqrt(np.mean(np.abs(y_rtl) ** 2))
    out = {}
    for name, e, r in (("re", d.real, y_rtl.real), ("im", d.imag, y_rtl.imag), ("complex", np.abs(d), np.abs(y_rtl))):
        ae = np.abs(e)
        out[name] = dict(max_abs=float(ae.max()), mean_abs=float(ae.mean()), rmse=float(np.sqrt(np.mean(ae ** 2))),
                         max_rel=float((ae / np.maximum(np.abs(r), floor)).max()),
                         mean_rel=float((ae / np.maximum(np.abs(r), floor)).mean()),
                         max_abs_lsb=float(ae.max() / LSB_Y))
    out["rel_floor"] = float(floor)
    return out


def per_block(y_ref, y_rtl, blk_of, tol=None):
    rows = []
    for b in np.unique(blk_of):
        m = blk_of == b
        d = y_ref[m] - y_rtl[m]
        r = dict(block=int(b), n=int(m.sum()), max_abs_re=float(np.abs(d.real).max()), max_abs_im=float(np.abs(d.imag).max()),
                 max_abs=float(np.abs(d).max()), mean_abs=float(np.abs(d).mean()), rmse=float(np.sqrt(np.mean(np.abs(d) ** 2))))
        if tol is not None:
            r["tol_max"] = float(tol[m].max())
            r["n_outside"] = int(np.sum((np.abs(d.real) > tol[m]) | (np.abs(d.imag) > tol[m])))
        rows.append(r)
    return rows


def implied_weights(X, y_rtl, blk_of):
    """Weights the RTL must have used in each block, recovered from its output: y_n = w^H x_n  <=>  y = X^T conj(w).
    Least squares per block. Also returns the rigorous recovery-uncertainty bound caused by the 0.5-LSB output rounding."""
    W, eps = [], []
    for b in np.unique(blk_of):
        m = blk_of == b
        A = X[:, m].T                                           # (L, 4)
        cw, *_ = np.linalg.lstsq(A, y_rtl[m], rcond=None)
        W.append(np.conj(cw))
        smin = np.linalg.svd(A, compute_uv=False)[-1]
        eps.append(np.sqrt(m.sum()) * np.sqrt(2.0) * 0.5 * LSB_Y / smin)
    return np.array(W), np.array(eps)


def run_comparison(ref, fx, rtl, X, a):
    """Return full comparison dict (float reference and bit-accurate emulator vs RTL)."""
    N = ref["cfg"].num_samples
    y_rtl = rtl["y"][:N]
    y_ref = ref["Y"]
    y_fx = (fx["y_raw"][:, 0] + 1j * fx["y_raw"][:, 1]) / 2.0 ** 22
    tol, eps_w = derive_tolerance(ref, X)
    d = y_ref - y_rtl
    outside = (np.abs(d.real) > tol) | (np.abs(d.imag) > tol)
    res = dict(
        tol=tol, eps_w=eps_w,
        float_vs_rtl=err_stats(y_ref, y_rtl),
        fixed_vs_rtl=err_stats(y_fx, y_rtl),
        fixed_vs_float=err_stats(y_ref, y_fx),
        n_within_tol=int(N - outside.sum()), n_outside_tol=int(outside.sum()),
        worst_ratio=float(np.max(np.maximum(np.abs(d.real), np.abs(d.imag)) / tol)),
        block_float=per_block(y_ref, y_rtl, ref["block_of_sample"], tol),
        block_fixed=per_block(y_fx, y_rtl, ref["block_of_sample"]),
        raw_diff=fx["y_raw"] - np.stack([rtl["re_raw"][:N], rtl["im_raw"][:N]], axis=1),
    )
    res["fixed_mismatch_samples"] = int(np.count_nonzero(np.any(res["raw_diff"] != 0, axis=1)))
    res["fixed_max_lsb_diff"] = int(np.abs(res["raw_diff"]).max())
    W_imp, eps_ls = implied_weights(X[:, :N], y_rtl, ref["block_of_sample"])
    res["W_implied"], res["eps_ls"] = W_imp, eps_ls
    res["y_ref"], res["y_fx"], res["y_rtl"] = y_ref, y_fx, y_rtl
    return res


def write_per_block_csv(res, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["block", "n_samples", "float_vs_rtl_max_re", "float_vs_rtl_max_im", "float_vs_rtl_max_abs",
                    "float_vs_rtl_mean_abs", "float_vs_rtl_rmse", "tolerance_max", "n_outside_tol",
                    "fixed_vs_rtl_max_abs"])
        for bf, bx in zip(res["block_float"], res["block_fixed"]):
            w.writerow([bf["block"], bf["n"], f"{bf['max_abs_re']:.6e}", f"{bf['max_abs_im']:.6e}", f"{bf['max_abs']:.6e}",
                        f"{bf['mean_abs']:.6e}", f"{bf['rmse']:.6e}", f"{bf['tol_max']:.6e}", bf["n_outside"],
                        f"{bx['max_abs']:.6e}"])
