"""run_all.py -- MVDR Beamformer Python Reference & Verification Runner.

Canonical Case 2 Configuration:
  - 2047 samples (26 full blocks of 76 samples + 1 tail block of 71 samples)
  - 4 antennas, element spacing 0.5 lambda
  - Forgetting factor 0.8, diagonal loading 1e-3, pivot clamp floor 2^-14
  - 27 adaptive weight sets (block 0 quiescent a/4; blocks 1..25 solved; tail block 26 inherits)

Usage:
    # Python-only reference generation mode (from project root):
    python python_reference/run_all.py --mem inputs/case_2.mem

    # Python-only reference generation mode (from within python_reference/):
    python run_all.py --mem ../inputs/case_2.mem

    # Full hardware/RTL comparison mode:
    python python_reference/run_all.py --mem inputs/case_2.mem --rtl-csv fpga_outputs/case_2/fpga_case2_y.csv

Writes all generated CSVs and reports to results/ and plots to plots/.
"""
import argparse
import csv
import datetime
import glob
import os
import re
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from parse_mem import load_mem, samples_used
from mvdr_reference import run_float_reference, write_outputs, RefConfig
from fixed_point_model import (run_fixed_point, weights_to_complex, LAMBDA_ONE_MINUS_COEF, LOAD_COEF, D_MIN)
from ucode_rom import UCODE_ROM
import compare_rtl
import verify_algorithm
import plot_results

# Parameters established from the CURRENT RTL (mvdr_top.v / block_controller.v / block_solver.v)
BLOCK_LEN, NUM_SAMPLES, MEM_DEPTH, FS_HZ, M_ANT = 76, 2047, 2048, 512, 4   # 2048-line .mem: line 0 = a, 2047 samples
FORGET, LOADING = 0.8, 1e-3


def find(d, name):
    c = [p for p in glob.glob(os.path.join(d, "*")) if os.path.basename(p) == name or os.path.basename(p).endswith("_" + name)]
    return c[0] if c else None


def check_rtl_source(rtl_dir):
    """Re-extract constants and the solver micro-code from the Verilog and compare with what the models use."""
    rows = []
    top, sol = find(rtl_dir, "mvdr_top.v"), find(rtl_dir, "block_solver.v")
    if not top or not sol:
        return [("RTL source check", "mvdr_top.v / block_solver.v not found", "", False)]
    t = open(top).read()
    for nm, mv in (("BLOCK_LEN", BLOCK_LEN), ("NUM_SAMPLES", NUM_SAMPLES), ("MEM_DEPTH", MEM_DEPTH), ("PACED", 0),
                   ("LAMBDA_ONE_MINUS_COEF", LAMBDA_ONE_MINUS_COEF), ("LOAD_COEF", LOAD_COEF), ("D_MIN", D_MIN)):
        m = re.search(r"parameter\s+%s\s*=\s*(\d+)" % nm, t)
        rows.append((f"mvdr_top.v parameter {nm}", int(m.group(1)) if m else None, mv, bool(m) and int(m.group(1)) == mv))
    s = open(sol).read()
    K = {"K_MAC": 0, "K_PIV": 1, "K_S": 2, "K_WLO": 3, "K_WHI": 4, "K_END": 7}
    rom = {}
    for m in re.finditer(r"8'd(\d+)\s*:\s*ucode = \{(K_\w+),\s*1'b(\d),\s*1'b(\d),\s*1'b(\d),\s*6'd(\d+)\s*,\s*6'd(\d+)\s*,\s*6'd(\d+)\s*\};", s):
        rom[int(m.group(1))] = (K[m.group(2)],) + tuple(int(m.group(j)) for j in range(3, 9))
    same = len(rom) == len(UCODE_ROM) and all(rom.get(i) == UCODE_ROM[i] for i in range(len(UCODE_ROM)))
    rows.append(("block_solver.v micro-code ROM (143 entries)", f"{len(rom)} entries", f"{len(UCODE_ROM)} entries", same))
    return rows


def main():
    ap = argparse.ArgumentParser(description="MVDR Reference & Verification Runner")
    ap.add_argument("--mem", required=True, help="Input .mem file (e.g. case_2.mem)")
    ap.add_argument("--rtl-csv", default=None, help="RTL simulation or FPGA hardware output CSV (optional)")
    ap.add_argument("--rtl-dir", default=None, help="Folder with the Verilog sources (optional cross-check)")
    ap.add_argument("--out", default=HERE, help="Base directory for python_reference")
    args = ap.parse_args()

    # Determine directories: results/ and plots/
    base_dir = args.out
    if os.path.basename(base_dir).lower() == "results":
        res_dir = base_dir
        plot_dir = os.path.join(os.path.dirname(base_dir), "plots")
    else:
        res_dir = os.path.join(base_dir, "results")
        plot_dir = os.path.join(base_dir, "plots")

    os.makedirs(res_dir, exist_ok=True)
    os.makedirs(plot_dir, exist_ok=True)

    # 1. Load memory image
    mem = load_mem(args.mem)
    xr, xi, X = samples_used(mem, NUM_SAMPLES)
    a = mem["a"]

    # 2. Float64 reference model
    cfg = RefConfig(num_ant=M_ANT, block_len=BLOCK_LEN, num_samples=NUM_SAMPLES, fs_hz=FS_HZ, forgetting=FORGET, loading=LOADING)
    ref = run_float_reference(a, X, cfg)
    write_outputs(ref, os.path.join(res_dir, "y_python.csv"), os.path.join(res_dir, "python_weights.csv"))

    # 3. Bit-accurate fixed-point emulator
    fx = run_fixed_point(mem["a_re"], mem["a_im"], xr, xi, BLOCK_LEN, NUM_SAMPLES)

    with open(os.path.join(res_dir, "y_fixed_point_model.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "block", "re_raw", "im_raw", "re_value", "im_value"])
        for n in range(NUM_SAMPLES):
            r, i = int(fx["y_raw"][n, 0]), int(fx["y_raw"][n, 1])
            w.writerow([n, int(fx["block_of_sample"][n]), r, i, f"{r/2**22:.9f}", f"{i/2**22:.9f}"])

    with open(os.path.join(res_dir, "fixed_point_weights.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["block", "kind"] + [f"w{m}_{c}_raw_Q5.20" for m in range(4) for c in ("re", "im")])
        for b, row in enumerate(fx["W_used"]):
            w.writerow([b, "tail" if b == ref["nblk"] else "block"] + [int(row[m + 4 * c]) for m in range(4) for c in (0, 1)])

    with open(os.path.join(res_dir, "fpga_y_reference.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "block", "re_raw", "im_raw", "re_value", "im_value", "m_data_hex"])
        for n in range(NUM_SAMPLES):
            r, i = int(fx["y_raw"][n, 0]), int(fx["y_raw"][n, 1])
            r_u = r & 0xFFFFFF
            i_u = i & 0xFFFFFF
            m_data = (i_u << 24) | r_u
            w.writerow([n, int(fx["block_of_sample"][n]), r, i, f"{r/2**22:.9f}", f"{i/2**22:.9f}", f"{m_data:012x}"])

    is_python_only = (args.rtl_csv is None)

    if is_python_only:
        rtl = dict(
            idx=np.arange(NUM_SAMPLES),
            block=fx["block_of_sample"],
            re_raw=fx["y_raw"][:, 0],
            im_raw=fx["y_raw"][:, 1],
            y=(fx["y_raw"][:, 0] + 1j * fx["y_raw"][:, 1]) / 2.0 ** 22
        )
        st = dict(n_rows=NUM_SAMPLES, n_rows_ok=True, idx_contiguous=True, block_labels_ok=True,
                  max_abs_raw=int(max(np.abs(rtl["re_raw"]).max(), np.abs(rtl["im_raw"]).max())))
    else:
        rtl = compare_rtl.load_rtl_csv(args.rtl_csv)
        st = compare_rtl.structural_checks(rtl, NUM_SAMPLES, BLOCK_LEN)

    cmp_ = compare_rtl.run_comparison(ref, fx, rtl, X, a)
    compare_rtl.write_per_block_csv(cmp_, os.path.join(res_dir, "per_block_errors.csv"))

    ctx = dict(mem=mem, xr=xr, xi=xi, X=X, a=a, a_re=mem["a_re"], a_im=mem["a_im"], ref=ref, fx=fx, rtl=rtl, cmp=cmp_,
               block_len=BLOCK_LEN, num_samples=NUM_SAMPLES)
    tests = verify_algorithm.run_all_tests(ctx)
    plots = plot_results.plot_all(ctx, plot_dir)
    src_rows = check_rtl_source(args.rtl_dir) if args.rtl_dir else []

    # 4. Save verification tests
    with open(os.path.join(res_dir, "verification_tests.txt"), "w") as f:
        f.write(f"MVDR Algorithmic Verification Tests (Case: {os.path.basename(args.mem)})\n")
        f.write("=" * 80 + "\n")
        for t in tests:
            f.write(f"[{t['status']:4s}] {t['id']}: {t['name']}\n")
            f.write(f"       Value: {t['value']}\n")
            f.write(f"       Establishes: {t['establishes']}\n\n")

    # 5. Verdicts
    fl = cmp_["float_vs_rtl"]
    A_ok = cmp_["n_outside_tol"] == 0 and st["n_rows_ok"] and st["idx_contiguous"] and st["block_labels_ok"]
    B_ok = cmp_["fixed_mismatch_samples"] == 0
    C_ok = all(t["status"] != "FAIL" for t in tests)
    S_ok = all(r[3] for r in src_rows) if src_rows else True
    overall = A_ok and C_ok
    tol = cmp_["tol"]

    # 6. Detailed comparison report
    L = []
    P = L.append
    P("MVDR BEAMFORMER REFERENCE & VERIFICATION REPORT")
    P("=" * 78)
    mode_str = "PYTHON REFERENCE ONLY (Bit-accurate model baseline)" if is_python_only else f"RTL/FPGA COMPARISON ({os.path.basename(args.rtl_csv)})"
    P(f"Execution Mode: {mode_str}")
    P(f"Generated     : {datetime.datetime.now():%Y-%m-%d %H:%M:%S}")
    P(f"Input Memory  : {os.path.basename(args.mem)}")
    P("")
    P("SCOPE / DESCRIPTION")
    if is_python_only:
        P("  * Python-only reference generation mode: generated independent float64 reference and")
        P("    bit-accurate fixed-point C/RTL model outputs for physical FPGA hardware validation.")
    else:
        P(f"  * Compared against target output: {os.path.basename(args.rtl_csv)}")
    P("  * Samples processed: 2047 (26 full blocks of 76 samples + 1 tail block of 71 samples).")
    P("  * Outputs are scaled by global input gain g applied during memory creation; weights are unaffected.")
    P("")
    P("MVDR ALGORITHM PARAMETERS")
    P(f"  Antennas             : {M_ANT} (uniform linear array, 0.5 lambda spacing)")
    P(f"  Block Length         : {BLOCK_LEN}")
    P(f"  Total Samples        : {NUM_SAMPLES} ({ref['nblk']} full blocks + tail {ref['tail']} samples)")
    P(f"  Sampling Frequency   : {FS_HZ} Hz")
    P(f"  Memory Depth         : {MEM_DEPTH} words")
    P(f"  Forgetting Factor    : lambda = {1 - LAMBDA_ONE_MINUS_COEF/2**24:.9f} (spec: 0.8)")
    P(f"  Diagonal Loading     : delta = {LOAD_COEF/2**32*4:.9e} * trace / 4 (spec: 1e-3 * trace / 4)")
    P(f"  Pivot Threshold D_MIN: 2^-14 ({D_MIN/2**24:.6e})")
    P("  Fixed-Point Formats  : x Q1.15, a Q2.14, S/R Q9.30 (39 b), solver Q16.24 (40 b), w Q5.20 (25 b), y Q2.22 (24 b)")
    P("")
    if src_rows:
        P("RTL SOURCE CROSS-CHECK")
        for nm, rv, mv, ok in src_rows:
            P(f"  [{'OK ' if ok else 'BAD'}] {nm}: RTL={rv}  model={mv}")
        P("")
    P("SIGNAL & CONDITIONING PROPERTIES")
    conds = [d["cond"] for d in ref["diag"]]
    mins = [d["min_eig"] for d in ref["diag"]]
    P(f"  Condition Number Range (loaded cov): {min(conds):.1f} to {max(conds):.1f}")
    P(f"  Minimum Eigenvalue Range           : {min(mins):.4f} to {max(mins):.4f}")
    P(f"  Steering Vector a (float)          : {[complex(round(x.real, 4), round(x.imag, 4)) for x in a]}")
    P("")
    P("ACCURACY METRICS: FLOAT64 vs FIXED-POINT MODEL")
    for k in ("re", "im", "complex"):
        s = fl[k]
        P(f"  {k:8s} max abs: {s['max_abs']:.4e} ({s['max_abs_lsb']:.2f} LSB)  RMSE: {s['rmse']:.4e}  mean abs: {s['mean_abs']:.4e}")
    P(f"  Tolerance bounds per sample        : min {tol.min():.3e}, median {np.median(tol):.3e}, max {tol.max():.3e}")
    P(f"  Samples within derived tolerance   : {cmp_['n_within_tol']}/{NUM_SAMPLES} ({cmp_['n_outside_tol']} outside)")
    P(f"  Worst error-to-tolerance ratio     : {cmp_['worst_ratio']:.3f} (< 1.0 = PASS)")
    P("")
    P("VERIFICATION TESTS SUMMARY")
    for t in tests:
        P(f"  [{t['status']:4s}] {t['id']}: {t['name']}")
    P("")
    P(f"OVERALL VERDICT: {'PASS' if overall else 'FAIL'}")
    P("=" * 78)

    report_text = "\n".join(L)
    with open(os.path.join(res_dir, "comparison_report.txt"), "w") as f:
        f.write(report_text)
    print(report_text)

    # 7. Write summary CSV
    with open(os.path.join(res_dir, "summary.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["metric", "value"])
        wr.writerow(["input_case", os.path.basename(args.mem)])
        wr.writerow(["num_samples", NUM_SAMPLES])
        wr.writerow(["num_full_blocks", ref["nblk"]])
        wr.writerow(["tail_samples", ref["tail"]])
        wr.writerow(["overall_verdict", "PASS" if overall else "FAIL"])
        wr.writerow(["cond_min", f"{min(conds):.2f}"])
        wr.writerow(["cond_max", f"{max(conds):.2f}"])
        wr.writerow(["max_abs_err_lsb", f"{fl['complex']['max_abs_lsb']:.2f}"])
        wr.writerow(["rmse_lsb", f"{fl['complex']['rmse']/(2**-22):.2f}"])
        wr.writerow(["samples_within_tol", cmp_["n_within_tol"]])
        wr.writerow(["samples_outside_tol", cmp_["n_outside_tol"]])

    print(f"\nOutputs written to:")
    print(f"  Results directory: {res_dir}")
    print(f"  Plots directory  : {plot_dir}")


if __name__ == "__main__":
    main()
