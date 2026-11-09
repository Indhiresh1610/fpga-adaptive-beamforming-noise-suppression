"""plot_results.py -- PNG plots of the Python-vs-RTL comparison."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from fixed_point_model import weights_to_complex

LSB = 2.0 ** -22


def _save(fig, outdir, name):
    p = os.path.join(outdir, name)
    fig.tight_layout()
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_all(ctx, outdir):
    os.makedirs(outdir, exist_ok=True)
    cmp_, ref, fx = ctx["cmp"], ctx["ref"], ctx["fx"]
    K, N, nblk = ctx["block_len"], ctx["num_samples"], ref["nblk"]
    n = np.arange(N)
    y_rtl, y_ref, y_fx, tol = cmp_["y_rtl"], cmp_["y_ref"], cmp_["y_fx"], cmp_["tol"]
    paths = []

    for comp, fn, lab in ((np.real, "01_real_python_vs_rtl.png", "Real"), (np.imag, "02_imag_python_vs_rtl.png", "Imaginary")):
        fig, ax = plt.subplots(2, 1, figsize=(11, 7))
        ax[0].plot(n, comp(y_rtl), lw=1.4, label="RTL (XSim y_out.csv)")
        ax[0].plot(n, comp(y_ref), "--", lw=1.0, label="Python float64 reference")
        for b in range(nblk + 1):
            ax[0].axvline(b * K, color="grey", lw=0.3)
        ax[0].set_xlabel("sample index"); ax[0].set_ylabel(f"{lab}(y)  [scaled by input gain g]")
        ax[0].set_title(f"{lab} part of beamformed output: Python reference vs RTL (grey lines = block boundaries)")
        ax[0].legend(loc="upper right")
        z = slice(N - 40, N)
        ax[1].plot(n[z], comp(y_rtl)[z], "o-", ms=3.5, label="RTL")
        ax[1].plot(n[z], comp(y_ref)[z], "x--", ms=4, label="Python float64")
        ax[1].axvline((nblk) * K - 0.5, color="r", lw=0.8, label=f"start of tail (block {nblk})")
        ax[1].set_xlabel("sample index"); ax[1].set_ylabel(f"{lab}(y)"); ax[1].set_title(f"Zoom: last 40 samples (block {nblk - 1} -> tail)")
        ax[1].legend(loc="best")
        paths.append(_save(fig, outdir, fn))

    # 3 absolute error vs sample
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    d = y_ref - y_rtl
    ax[0].semilogy(n, np.abs(d.real) + 1e-12, lw=0.8, label="|Re error|")
    ax[0].semilogy(n, np.abs(d.imag) + 1e-12, lw=0.8, label="|Im error|")
    ax[0].semilogy(n, tol, "k-", lw=1.0, label="derived tolerance")
    ax[0].axhline(LSB, color="g", ls=":", label="1 output LSB (2^-22)")
    ax[0].set_ylabel("absolute error"); ax[0].set_title("Float64 reference vs RTL: absolute error per sample")
    ax[0].legend(ncol=4, fontsize=8)
    raw = cmp_["raw_diff"]
    ax[1].plot(n, raw[:, 0], lw=1.0, label="Re  (emulator - RTL)")
    ax[1].plot(n, raw[:, 1], lw=1.0, label="Im  (emulator - RTL)")
    ax[1].set_ylim(-1, 1); ax[1].set_xlabel("sample index"); ax[1].set_ylabel("difference [LSB]")
    ax[1].set_title(f"Bit-accurate fixed-point emulator vs RTL (raw integers): {cmp_['fixed_mismatch_samples']} mismatching samples")
    ax[1].legend()
    paths.append(_save(fig, outdir, "03_abs_error_vs_sample.png"))

    # 4 error per block
    fig, ax = plt.subplots(figsize=(11, 5))
    bl = cmp_["block_float"]
    x = np.arange(len(bl))
    ax.bar(x - 0.2, [r["max_abs"] for r in bl], 0.4, label="max |error|")
    ax.bar(x + 0.2, [r["rmse"] for r in bl], 0.4, label="RMSE")
    ax.plot(x, [r["tol_max"] for r in bl], "k_", ms=18, mew=2, label="tolerance (max in block)")
    ax.axhline(LSB, color="g", ls=":", label="1 output LSB")
    ax.set_yscale("log"); ax.set_xlabel(f"block ({nblk} = tail)"); ax.set_ylabel("|y_python - y_RTL|")
    ax.set_title("Float64 reference vs RTL: error per block"); ax.legend(ncol=4, fontsize=8)
    paths.append(_save(fig, outdir, "04_error_per_block.png"))

    # 5 weight evolution
    Wf, Wx, Wi = ref["W_used"], weights_to_complex(fx["W_used"]), cmp_["W_implied"]
    fig, ax = plt.subplots(2, 4, figsize=(15, 6), sharex=True)
    b = np.arange(len(Wf))
    for m in range(4):
        for r, (fn, lab) in enumerate(((np.real, "Re"), (np.imag, "Im"))):
            ax[r, m].plot(b, fn(Wf[:, m]), "o-", ms=4, label="float64 reference")
            ax[r, m].plot(b, fn(Wx[:, m]), "s", ms=6, mfc="none", label="fixed-point emulator")
            ax[r, m].plot(b, fn(Wi[:, m]), "x", ms=7, label="recovered from RTL output")
            ax[r, m].set_title(f"{lab}(w{m}) used in block")
            ax[r, m].grid(alpha=0.3)
            if r == 1:
                ax[r, m].set_xlabel(f"block ({nblk} = tail)")
    ax[0, 0].legend(fontsize=7)
    fig.suptitle("Weight evolution across blocks (block 0 = a/4; block b uses the solve of block b-1)")
    paths.append(_save(fig, outdir, "05_weight_evolution.png"))

    # 6 error histogram + conditioning
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    ax[0].hist(np.concatenate([d.real, d.imag]) / LSB, bins=41)
    ax[0].set_xlabel("float64 - RTL error [output LSB]"); ax[0].set_ylabel("count (re and im)")
    ax[0].set_title("Error distribution")
    ax[1].semilogy([dg["cond"] for dg in ref["diag"]], "o-", label="cond(A_loaded)")
    ax[1].set_xlabel("solve after block"); ax[1].set_title("Conditioning of the loaded covariance"); ax[1].legend()
    ax[2].plot([min(dg["pivots"]) / 2 ** 24 for dg in fx["diag"]], "o-")
    ax[2].axhline(2 ** -14, color="r", ls="--", label="D_MIN = 2^-14")
    ax[2].set_yscale("log"); ax[2].set_xlabel("solve after block"); ax[2].set_title("Smallest LDL^H pivot (Q16.24 model)")
    ax[2].legend()
    paths.append(_save(fig, outdir, "06_error_hist_and_conditioning.png"))

    # 7 sensitivity
    if ctx.get("variants"):
        fig, ax = plt.subplots(figsize=(10, 4.8))
        names = ["baseline (spec)"] + [v[0] for v in ctx["variants"]]
        vals = [float(np.abs(y_ref - y_rtl).max())] + [v[1] for v in ctx["variants"]]
        ax.barh(names, vals)
        ax.axvline(tol.max(), color="k", ls="--", label="largest derived tolerance")
        ax.set_xscale("log"); ax.set_xlabel("max |y_variant - y_RTL|"); ax.invert_yaxis(); ax.legend()
        ax.set_title("What would this input have caught? (variant algorithms vs the same RTL output)")
        paths.append(_save(fig, outdir, "07_variant_sensitivity.png"))
    return paths
