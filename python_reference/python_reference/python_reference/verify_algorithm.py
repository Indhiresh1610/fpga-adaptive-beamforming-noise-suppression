"""verify_algorithm.py -- mathematical / structural verification of the MVDR algorithm and of its RTL implementation.

Every test returns: id, name, status (PASS / FAIL / INFO), value, establishes, does_not_establish.
Thresholds are derived from the fixed-point formats (see compare_rtl.py), not fitted to the observed errors.
"""
import numpy as np
from fixed_point_model import (run_fixed_point, weights_to_complex, cov_to_matrix, beamform_sample, LAMBDA_ONE_MINUS_COEF,
                               D_MIN)
from mvdr_reference import run_float_reference, RefConfig


def _t(id_, name, ok, value, est, notest):
    return dict(id=id_, name=name, status=("INFO" if ok is None else "PASS" if ok else "FAIL"),
                value=value, establishes=est, does_not_establish=notest)


def run_all_tests(ctx):
    ref, fx, rtl, cmp_, X, a = ctx["ref"], ctx["fx"], ctx["rtl"], ctx["cmp"], ctx["X"], ctx["a"]
    K, N = ctx["block_len"], ctx["num_samples"]
    nblk = ref["nblk"]
    T = []
    eps_w = cmp_["eps_w"]                                   # per used-block weight-error bound (index b = block using them)
    na = np.linalg.norm(a)

    # V1 covariance accumulation exactness (sum of x x^H, upper-triangle word map) ----------------------------------
    d1 = max(np.abs(cov_to_matrix([int(v) for v in fx["S"][b]]) - ref["S"][b]).max() for b in range(nblk))
    T.append(_t("V1", "Per-block covariance S: integer emulation of the RTL accumulator == float sum x x^H",
                d1 <= 1e-12, f"max|S_fixed - S_float| = {d1:.3e}",
                "The 16-word S map (4 diagonal, 6 Re, 6 Im upper-triangle entries, S[i,j] = x_i conj(x_j)) and the Q9.30 "
                "accumulation reproduce sum x x^H exactly (products are exact in 32 bits).",
                "S inside the RTL is not exposed in y_out.csv; it is verified indirectly because every later weight depends on it "
                "(see V7 and the bit-exact output match)."))

    # V2 covariance recursion -------------------------------------------------------------------------------------
    coef_err = abs(LAMBDA_ONE_MINUS_COEF / 2.0 ** 24 - 0.2)
    smax = max(np.abs(S).max() for S in ref["S"])
    bound = 5.0 * (2.0 * coef_err * smax + 2.0 ** -31)      # per-update error, geometric sum 1/(1-0.8) = 5
    d2 = max(np.abs(cov_to_matrix([int(v) for v in fx["R"][b]]) - ref["R"][b]).max() for b in range(nblk))
    T.append(_t("V2", "Recursive covariance R_b = 0.8 R_{b-1} + 0.2 S_b (R_0 = S_0): emulator vs float",
                d2 <= bound, f"max|R_fixed - R_float| = {d2:.3e}  (bound {bound:.3e}; coefficient 3355443/2^24 = "
                             f"{LAMBDA_ONE_MINUS_COEF/2**24:.9f}, lambda_eff - 0.8 = {-(LAMBDA_ONE_MINUS_COEF/2**24-0.2):.2e})",
                "The RTL update rule/constant implement the 0.8/0.2 exponential forgetting and the R = S start.",
                "Does not show that 0.8 is a good design value, only that it is what the RTL implements."))

    # V3 Hermitian / diagonal ---------------------------------------------------------------------------------------
    herm = max(max(np.abs(S - S.conj().T).max() for S in ref["S"]), max(np.abs(R - R.conj().T).max() for R in ref["R"]))
    dmin = min(int(fx["R"][b][k]) for b in range(nblk) for k in range(4))
    T.append(_t("V3", "Covariance Hermitian symmetry and non-negative real diagonal",
                herm <= 1e-12 * smax and dmin >= 0,
                f"float max|R-R^H| = {herm:.2e}; min fixed diagonal word = {dmin} (>= 0)",
                "Float covariances are Hermitian; the RTL stores only the upper triangle (+4 real diagonal words), so its "
                "matrix is Hermitian by construction and its diagonal words are non-negative.",
                "Symmetry of the stored form is structural, not an independent measurement of the RTL registers."))

    # V4 positive definiteness / pivots -----------------------------------------------------------------------------
    mins = [d["min_eig"] for d in ref["diag"]]
    conds = [d["cond"] for d in ref["diag"]]
    pf = min(min(dg["pivots"]) for dg in fx["diag"]) / 2.0 ** 24
    T.append(_t("V4", "Positive definiteness of the loaded, normalised covariance; LDL^H pivots above D_MIN = 2^-14",
                min(mins) > 0 and pf >= D_MIN / 2.0 ** 24 and fx["flags"]["err_pivot"] == 0,
                f"min eigenvalue = {min(mins):.4f}; cond range {min(conds):.1f}..{max(conds):.1f}; "
                f"min fixed-point pivot = {pf:.4f} (floor {D_MIN/2**24:.2e}); err_pivot = {fx['flags']['err_pivot']}",
                "Every solve was well-posed: all 4x4 loaded matrices are positive definite and the pivot clamp never engaged.",
                "Positive definiteness is a property of this data record; it says nothing about worst-case inputs."))

    # V5 distortionless response ------------------------------------------------------------------------------------
    Wsol_f = ref["W_solved"]
    r_f = max(abs(np.vdot(w, a) - 1.0) for w in Wsol_f)                  # w^H a
    Wsol_x = weights_to_complex(fx["W_solved"])
    r_x = [abs(np.vdot(Wsol_x[b], a) - 1.0) for b in range(nblk)]
    ok_x = all(r_x[b] <= eps_w[b + 1] * na for b in range(nblk))
    W_imp = cmp_["W_implied"]
    r_i = [abs(np.vdot(W_imp[b], a) - 1.0) for b in range(len(W_imp))]
    ok_i = all(r_i[b] <= (eps_w[b] + cmp_["eps_ls"][b]) * na for b in range(len(W_imp)))
    T.append(_t("V5", "Distortionless constraint w^H a = 1 (float, quantised RTL-model weights, weights recovered from RTL output)",
                (r_f <= 1e-12) and ok_x and ok_i,
                f"float max|w^H a - 1| = {r_f:.2e}; quantised-weights max = {max(r_x):.2e}; RTL-implied max = {max(r_i):.2e} "
                f"(allowed ~ (eps_w+eps_LS)*||a||, e.g. block-1 bound {(eps_w[1]+cmp_['eps_ls'][1])*na:.1e})",
                "Weights are correctly normalised (division by a^H u, Q5.20 rounding) and the weights the RTL actually applied "
                "satisfy the constraint to the quantisation limit.",
                "A distortionless response toward 0 deg is meaningful as a normalisation check only; it is not evidence of "
                "interference rejection."))

    # V6 solver vs np.linalg.solve MVDR formula -----------------------------------------------------------------------
    dws = [np.linalg.norm(Wsol_x[b] - Wsol_f[b]) for b in range(nblk)]
    ok6 = all(dws[b] <= eps_w[b + 1] for b in range(nblk))
    T.append(_t("V6", "Solver weights (fixed-point LDL^H model) vs float w = R^-1 a / (a^H R^-1 a) from numpy.linalg.solve",
                ok6, f"max ||dw||_2 = {max(dws):.3e}; bound (per block, kappa-scaled) min {min(eps_w[1:nblk+1]):.2e} .. max "
                     f"{max(eps_w[1:nblk+1]):.2e}",
                "The fixed-point LDL^H / forward / diagonal / backward / normalisation chain computes the MVDR solution of the "
                "stated formula to within the derived numerical-error bound.",
                "Bound is worst-case; agreement much tighter than the bound does not prove bit-exactness of the actual RTL "
                "(see the bit-exact emulator comparison)."))

    # V7 weights implied by RTL output ----------------------------------------------------------------------------------
    Wused_x = weights_to_complex(fx["W_used"])
    d7 = np.array([np.linalg.norm(W_imp[b] - Wused_x[b]) for b in range(len(W_imp))])
    ok7 = bool(np.all(d7 <= cmp_["eps_ls"]))
    d7f = np.array([np.linalg.norm(W_imp[b] - ref["W_used"][b]) for b in range(len(W_imp))])
    # contrast: distance to the NON-causal weights (solved from the same block's own data)
    d7nc = np.array([np.linalg.norm(W_imp[b] - ref["W_solved"][b]) for b in range(nblk)])
    T.append(_t("V7", "Blockwise weights recovered from the RTL output (least squares on y = w^H x) vs model weights",
                ok7 and d7nc.min() > 100 * cmp_["eps_ls"][:nblk].max(),
                f"max||w_RTL-implied - w_fixed-model|| = {d7.max():.2e} (LS recovery bound max {cmp_['eps_ls'].max():.2e}); "
                f"vs float reference {d7f.max():.2e}; vs same-block (non-causal) weights min {d7nc.min():.2e}",
                "Block b of the RTL output was produced with exactly the weights the causal model derives from blocks 0..b-1, not "
                "with weights from block b itself. This is a direct, weight-level causality check on the RTL output.",
                "RTL weight registers were not dumped; weights are inferred from y (valid because x has full column rank per "
                "block -- sigma_min is reported in the LS bound)."))

    # V8 causality perturbation experiments -----------------------------------------------------------------------------
    rng = np.random.default_rng(0)
    pb = 7
    sl = slice(pb * K, (pb + 1) * K)
    xr2, xi2 = ctx["xr"].copy(), ctx["xi"].copy()
    xr2[sl] = np.clip(xr2[sl] + rng.integers(-3000, 3000, xr2[sl].shape), -32768, 32767)
    xi2[sl] = np.clip(xi2[sl] + rng.integers(-3000, 3000, xi2[sl].shape), -32768, 32767)
    fx2 = run_fixed_point(ctx["a_re"], ctx["a_im"], xr2, xi2, K, N)
    same_pre = np.array_equal(fx2["W_used"][:pb + 1], fx["W_used"][:pb + 1])
    diff_post = not np.array_equal(fx2["W_used"][pb + 1], fx["W_used"][pb + 1])
    y_pre = np.array_equal(fx2["y_raw"][:pb * K], fx["y_raw"][:pb * K])
    wA = [int(v) for v in fx2["W_used"][pb]]
    exp = [beamform_sample([int(v) for v in xr2[n]], [int(v) for v in xi2[n]], wA)[:2] for n in range(pb * K, (pb + 1) * K)]
    y_self = np.array_equal(np.array(exp), fx2["y_raw"][sl])
    X2 = (xr2 + 1j * xi2).T / 2.0 ** 15
    rf2 = run_float_reference(a, X2, ref["cfg"])
    f_pre = np.allclose(rf2["W_used"][:pb + 1], ref["W_used"][:pb + 1], atol=0, rtol=0)
    f_post = not np.allclose(rf2["W_used"][pb + 1], ref["W_used"][pb + 1], atol=1e-6, rtol=0)
    T.append(_t("V8", f"Causal weight update (perturb only block {pb} of the input)",
                same_pre and diff_post and y_pre and y_self and f_pre and f_post,
                f"fixed model: weights of blocks 0..{pb} unchanged={same_pre}, block {pb+1} changed={diff_post}, outputs of blocks "
                f"0..{pb-1} unchanged={y_pre}, block-{pb} outputs use the unperturbed weights={y_self}; float model: "
                f"blocks 0..{pb} unchanged={f_pre}, block {pb+1} changed={f_post}",
                "In both models, weights applied in a block depend only on earlier blocks and the first use of a block's data is "
                "in the next block (no retroactive update).",
                "This is a property of the models. The RTL itself is covered by V7 and the bit-exact output match."))

    # V9 tail handling -----------------------------------------------------------------------------------------------------
    tail = ref["tail"]
    if tail:
        eq_fx = np.array_equal(fx["W_used"][nblk], fx["W_solved"][nblk - 1])
        lab = bool(np.all(rtl["block"][nblk * K:N] == nblk)) and len(rtl["block"][nblk * K:N]) == tail
        d_tail = np.linalg.norm(W_imp[nblk] - Wused_x[nblk - 1 + 1])
        ok9 = eq_fx and lab and d_tail <= cmp_["eps_ls"][nblk]
        T.append(_t("V9", f"Tail handling ({tail} samples after {nblk} full blocks)", ok9,
                    f"tail uses weights of the last full block (model)={eq_fx}; RTL block label {nblk} on {tail} rows={lab}; "
                    f"RTL-implied tail weights vs model: {d_tail:.2e} (bound {cmp_['eps_ls'][nblk]:.2e})",
                    "The RTL beamforms the tail with the weights solved after the last full block, with correct indexing/labelling.",
                    "'No covariance update/solve in the tail' is not observable from y_out.csv (nothing is output afterwards); it "
                    "is read from the RTL source and modelled (accum_en = 0, solver bypassed)."))

    # V10 saturation / solver status ----------------------------------------------------------------------------------------
    fl = fx["flags"]
    hr_y = max(np.abs(fx["y_raw"]).max() / 2.0 ** 23, 0)
    hr_w = np.abs(fx["W_solved"]).max() / 2.0 ** 24
    hr_S = max(abs(int(v)) for row in fx["S"] for v in row) / 2.0 ** 38
    rail = int(np.sum((np.abs(rtl["re_raw"]) >= 2 ** 23 - 1) | (np.abs(rtl["im_raw"]) >= 2 ** 23 - 1)))
    ok10 = all(v == 0 for v in fl.values()) and sum(ref["flags"].values()) == 0 and rail == 0
    T.append(_t("V10", "No saturation, pivot clamp, solver fallback or zero-trace event", ok10,
                f"emulator flags {fl}; float fallbacks {ref['flags']}; RTL rows at Q2.22 rails: {rail}; "
                f"headroom used: |y| {hr_y:.1%} of 2^23, |w| {hr_w:.1%} of 2^24, S {hr_S:.1%} of 2^38",
                "On this record no arithmetic saturated and no fallback path was taken in the model; RTL outputs are not "
                "railed.",
                "The RTL status register is not in y_out.csv (the tb prints status at the end of the run; please confirm it "
                "printed status=0x00). Fallback and saturation paths are therefore NOT exercised/tested by this record."))

    # V11 initial weights ---------------------------------------------------------------------------------------------------
    init_ok = np.array_equal(fx["W_used"][0], np.concatenate([ctx["a_re"], ctx["a_im"]]) * 16)
    b0 = np.all(cmp_["raw_diff"][:K] == 0)
    T.append(_t("V11", "Block 0 uses quiescent weights w = a/4", bool(init_ok and b0),
                f"model block-0 weights == a_raw<<4 (Q5.20): {init_ok}; RTL block-0 output == bit-exact a/4 beamformer: {bool(b0)}",
                "First block is processed with w = a/4 exactly as specified.", "Nothing about later blocks."))

    # V12 discriminating power of this input ---------------------------------------------------------------------------------
    y_rtl = cmp_["y_rtl"]
    tolmax = cmp_["tol"].max()
    variants = [("forgetting 0.95 (HLS-design value)", RefConfig(forgetting=0.95)), ("forgetting 0.5", RefConfig(forgetting=0.5)),
                ("loading 1e-2", RefConfig(loading=1e-2)), ("loading 1e-4", RefConfig(loading=1e-4)),
                ("no diagonal loading", RefConfig(loading=0.0)), ("NON-causal (block uses own weights)", RefConfig(causal=False))]
    rows, allsep = [], True
    base_err = float(np.abs(cmp_["y_ref"] - y_rtl).max())
    for nm, c in variants:
        c.block_len, c.num_samples = K, N
        e = float(np.abs(run_float_reference(a, X, c)["Y"] - y_rtl).max())
        rows.append((nm, e, e > tolmax))
        allsep &= e > tolmax
    T.append(_t("V12", "Discriminating power: do wrong algorithm variants fail the same comparison?", None,
                "; ".join(f"{n}: {e:.2e} ({'rejected' if r else 'NOT distinguishable'})" for n, e, r in rows)
                + f"   [baseline {base_err:.2e}, largest tolerance {tolmax:.2e}]",
                "Which parameter/ordering errors this particular input would have caught.",
                "Variants marked NOT distinguishable cannot be confirmed by this noise-only record; they need other test inputs."))
    ctx["variants"] = rows
    return T


def format_tests(T):
    lines = []
    for t in T:
        lines += [f"[{t['status']}] {t['id']}  {t['name']}", f"      result : {t['value']}",
                  f"      shows  : {t['establishes']}", f"      limits : {t['does_not_establish']}", ""]
    return "\n".join(lines)
