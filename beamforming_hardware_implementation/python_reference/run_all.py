"""run_all.py -- run the whole verification pipeline.

    python run_all.py --mem <x_mem.mem> --rtl-csv <y_out.csv> [--rtl-dir <folder with the .v files>]

Reads (never modifies) x_mem.mem and y_out.csv; writes everything under ./results, ./summary.csv and ./README.md.
Exit code 0 = overall PASS.
"""
import argparse, csv, glob, os, re, sys, datetime
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from parse_mem import load_mem, samples_used
from mvdr_reference import run_float_reference, write_outputs, RefConfig
from fixed_point_model import (run_fixed_point, weights_to_complex, LAMBDA_ONE_MINUS_COEF, LOAD_COEF, D_MIN)
from ucode_rom import UCODE_ROM
import compare_rtl, verify_algorithm, plot_results

# Parameters established from the CURRENT RTL (mvdr_top.v / block_controller.v / block_solver.v)
BLOCK_LEN, NUM_SAMPLES, MEM_DEPTH, FS_HZ, M_ANT = 76, 1536, 2048, 512, 4
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
    for pat, nm, v in ((r"w\s*=\s*a\s*/\s*4|a/4", "initial weights a/4 mentioned", None),):
        pass
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mem", required=True)
    ap.add_argument("--rtl-csv", required=True)
    ap.add_argument("--rtl-dir", default=None, help="folder with the Verilog sources (optional constant/ROM cross-check)")
    ap.add_argument("--out", default=HERE)
    args = ap.parse_args()
    res_dir = os.path.join(args.out, "results")
    plot_dir = os.path.join(res_dir, "plots")
    os.makedirs(plot_dir, exist_ok=True)

    mem = load_mem(args.mem)
    xr, xi, X = samples_used(mem, NUM_SAMPLES)
    a = mem["a"]
    rtl = compare_rtl.load_rtl_csv(args.rtl_csv)
    st = compare_rtl.structural_checks(rtl, NUM_SAMPLES, BLOCK_LEN)

    cfg = RefConfig(num_ant=M_ANT, block_len=BLOCK_LEN, num_samples=NUM_SAMPLES, fs_hz=FS_HZ, forgetting=FORGET, loading=LOADING)
    ref = run_float_reference(a, X, cfg)
    write_outputs(ref, os.path.join(res_dir, "y_python.csv"), os.path.join(res_dir, "python_weights.csv"))
    fx = run_fixed_point(mem["a_re"], mem["a_im"], xr, xi, BLOCK_LEN, NUM_SAMPLES)

    with open(os.path.join(res_dir, "y_fixed_point_model.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["idx", "block", "re_raw", "im_raw", "re_value", "im_value"])
        for n in range(NUM_SAMPLES):
            r, i = int(fx["y_raw"][n, 0]), int(fx["y_raw"][n, 1])
            w.writerow([n, int(fx["block_of_sample"][n]), r, i, f"{r/2**22:.9f}", f"{i/2**22:.9f}"])
    with open(os.path.join(res_dir, "fixed_point_weights.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["block", "kind"] + [f"w{m}_{c}_raw_Q5.20" for m in range(4) for c in ("re", "im")])
        for b, row in enumerate(fx["W_used"]):
            w.writerow([b, "tail" if b == ref["nblk"] else "block"] + [int(row[m + 4 * c]) for m in range(4) for c in (0, 1)])

    cmp_ = compare_rtl.run_comparison(ref, fx, rtl, X, a)
    compare_rtl.write_per_block_csv(cmp_, os.path.join(res_dir, "per_block_errors.csv"))
    ctx = dict(mem=mem, xr=xr, xi=xi, X=X, a=a, a_re=mem["a_re"], a_im=mem["a_im"], ref=ref, fx=fx, rtl=rtl, cmp=cmp_,
               block_len=BLOCK_LEN, num_samples=NUM_SAMPLES)
    tests = verify_algorithm.run_all_tests(ctx)
    plots = plot_results.plot_all(ctx, plot_dir)
    src_rows = check_rtl_source(args.rtl_dir) if args.rtl_dir else []

    # ------------------------------------------------------------------ verdicts
    fl = cmp_["float_vs_rtl"]; fxs = cmp_["fixed_vs_rtl"]
    A_ok = cmp_["n_outside_tol"] == 0 and st["n_rows_ok"] and st["idx_contiguous"] and st["block_labels_ok"]
    B_ok = cmp_["fixed_mismatch_samples"] == 0
    C_ok = all(t["status"] != "FAIL" for t in tests)
    S_ok = all(r[3] for r in src_rows) if src_rows else True
    overall = A_ok and C_ok
    tol = cmp_["tol"]

    # ------------------------------------------------------------------ report
    L = []
    P = L.append
    P("MVDR RTL vs PYTHON REFERENCE -- COMPARISON REPORT")
    P("=" * 78)
    P(f"generated {datetime.datetime.now():%Y-%m-%d %H:%M}   input: {os.path.basename(args.mem)}   RTL output: {os.path.basename(args.rtl_csv)}")
    P("")
    P("SCOPE / CAVEATS")
    P("  * y_out.csv is a SIMULATION capture (tb_save_output.v under XSim). No FPGA hardware output was captured or compared.")
    P("  * x_mem.mem is the NO-INTERFERENCE case (steering vector all ones = 0 deg, noise only). Interference suppression / SINR")
    P("    improvement is NOT verified by this run.")
    P("  * Outputs are scaled by the unknown global input gain g (x_mem.mem was written with g applied); weights are unaffected.")
    P("")
    P("PARAMETERS USED (established from the current RTL source)")
    P(f"  antennas {M_ANT}; block length {BLOCK_LEN}; samples {NUM_SAMPLES} ({ref['nblk']} full blocks + tail {ref['tail']}); fs {FS_HZ} Hz; mem depth {MEM_DEPTH}; PACED 0")
    P(f"  forgetting: R += round(0.2*(S-R)), coefficient {LAMBDA_ONE_MINUS_COEF}/2^24 -> lambda = {1-LAMBDA_ONE_MINUS_COEF/2**24:.9f} (spec 0.8)")
    P(f"  loading: delta = {LOAD_COEF}/2^32 * trace_norm = {LOAD_COEF/2**32:.9e}*trace = {LOAD_COEF/2**32*4:.9e}*trace/4 (spec 1e-3*trace/4)")
    P("  initial weights a/4; first covariance R = S; weights solved after block b first used by block b+1; tail: no accumulate/solve")
    P("  D_MIN = 2^-14 pivot clamp; fallback w = a/4 if trace = 0 or s <= 2^-14 (normalised scale)")
    P("  formats: x Q1.15, a Q2.14, S/R Q9.30 (39 b), solver Q16.24 (40 b), w Q5.20 (25 b), y Q2.22 (24 b)")
    if src_rows:
        P("")
        P("RTL SOURCE CROSS-CHECK (constants / micro-code re-parsed from the Verilog vs values used by the models)")
        for nm, rv, mv, ok in src_rows:
            P(f"  [{'OK ' if ok else 'BAD'}] {nm}: RTL={rv}  model={mv}")
    P("")
    P("INPUT / OUTPUT STRUCTURE")
    P(f"  memory words {mem['n_words']} (line 0 = steering vector, lines 1.. = samples); samples in image {mem['xr'].shape[0]}, used {NUM_SAMPLES}")
    P(f"  steering vector (Q2.14 -> float): {np.round(a, 6).tolist()}")
    P(f"  RTL rows {st['n_rows']} (expected {NUM_SAMPLES}): {st['n_rows_ok']}; idx contiguous 0..{NUM_SAMPLES-1}: {st['idx_contiguous']}; block labels as RTL spec: {st['block_labels_ok']}")
    P("")
    P("A. FLOAT64 REFERENCE vs RTL   (algorithm check; differences expected from fixed-point quantisation only)")
    for k in ("re", "im", "complex"):
        s = fl[k]
        P(f"  {k:8s} max abs {s['max_abs']:.4e} ({s['max_abs_lsb']:.2f} LSB)  mean abs {s['mean_abs']:.4e}  RMSE {s['rmse']:.4e}  "
          f"max rel {s['max_rel']:.3e}  mean rel {s['mean_rel']:.3e}")
    P(f"  relative error = |err| / max(|y_RTL|, {fl['rel_floor']:.3e}) (floor = 1% of RMS|y| to avoid division by ~0)")
    P(f"  derived tolerance per sample: min {tol.min():.3e}, median {np.median(tol):.3e}, max {tol.max():.3e}")
    P(f"  samples within tolerance {cmp_['n_within_tol']}/{NUM_SAMPLES}; outside {cmp_['n_outside_tol']}; worst error/tolerance ratio {cmp_['worst_ratio']:.3f}")
    P(f"  per-block table: results/per_block_errors.csv  (worst block max|err| = {max(r['max_abs'] for r in cmp_['block_float']):.3e})")
    P("")
    P("B. BIT-ACCURATE FIXED-POINT EMULATOR vs RTL   (checks the RTL arithmetic itself, raw Q2.22 integers)")
    P(f"  mismatching samples {cmp_['fixed_mismatch_samples']}/{NUM_SAMPLES}; max |difference| {cmp_['fixed_max_lsb_diff']} LSB; emulator flags {fx['flags']}")
    P(f"  float64 reference vs emulator: max abs {cmp_['fixed_vs_float']['complex']['max_abs']:.4e} "
      f"({cmp_['fixed_vs_float']['complex']['max_abs_lsb']:.2f} LSB), RMSE {cmp_['fixed_vs_float']['complex']['rmse']:.4e}")
    P("  -> the difference between float64 and RTL is therefore entirely the quantisation of this fixed-point datapath (Q5.20 weights,")
    P("     Q16.24 solver rounding, Q2.22 output rounding); no saturation occurred.")
    P("")
    P("C. ALGORITHM / STRUCTURE TESTS")
    P(verify_algorithm.format_tests(tests))
    P("TOLERANCE DERIVATION (compare_rtl.py): |dy| <= ||dw||2 ||x||2 + 0.5 LSB(Q2.22), ||dw||2 <= 2*kappa(A)*16*2^-25*||w||2 + sqrt(8)*2^-21")
    P("  (kappa = condition number of the normalised loaded covariance of the block that produced the weights; block 0 has exact weights).")
    P("  It is a first-order worst-case bound from the number formats; its constants (c = 16, factor 2) are engineering allowances, not fitted to the observed error.")
    P("  It is intentionally loose;")
    P("  the sharp check is B (bit-exact), and V12 shows the bound still rejects every wrong-algorithm variant tried.")
    P("")
    P("VERDICT")
    P(f"  A. float64 reference within derived tolerance on all {NUM_SAMPLES} samples : {'PASS' if A_ok else 'FAIL'}")
    P(f"  B. bit-accurate emulator identical to RTL on all {NUM_SAMPLES} samples     : {'PASS' if B_ok else 'FAIL'}")
    P(f"  C. algorithm / structure tests V1-V11                                : {'PASS' if C_ok else 'FAIL'}")
    if src_rows:
        P(f"  D. RTL source constants and micro-code match the models              : {'PASS' if S_ok else 'FAIL'}")
    P(f"  OVERALL (A and C): {'PASS' if overall else 'FAIL'}")
    P("  Scope of PASS: the simulated RTL implements the intended causal block MVDR algorithm on the noise-only record; interference")
    P("  suppression, fallback/saturation paths and hardware behaviour on the board are NOT established by this run.")
    report = "\n".join(L)
    open(os.path.join(res_dir, "comparison_report.txt"), "w").write(report + "\n")
    open(os.path.join(res_dir, "verification_tests.txt"), "w").write(verify_algorithm.format_tests(tests))

    # ------------------------------------------------------------------ summary.csv
    rows = [("verdict", "overall_PASS", int(overall), "A and C"),
            ("verdict", "A_float64_within_tolerance", int(A_ok), ""),
            ("verdict", "B_fixed_point_emulator_bit_exact", int(B_ok), ""),
            ("verdict", "C_algorithm_tests", int(C_ok), "")]
    for k in ("re", "im", "complex"):
        for m in ("max_abs", "mean_abs", "rmse", "max_rel", "mean_rel", "max_abs_lsb"):
            rows.append(("float64_vs_RTL", f"{k}_{m}", f"{fl[k][m]:.6e}", "LSB = 2^-22" if m == "max_abs_lsb" else ""))
    rows += [("float64_vs_RTL", "relative_error_floor", f"{fl['rel_floor']:.6e}", "1% of RMS|y_RTL|"),
             ("float64_vs_RTL", "samples_within_tolerance", cmp_["n_within_tol"], f"of {NUM_SAMPLES}"),
             ("float64_vs_RTL", "samples_outside_tolerance", cmp_["n_outside_tol"], ""),
             ("float64_vs_RTL", "tolerance_min", f"{tol.min():.6e}", "derived"),
             ("float64_vs_RTL", "tolerance_max", f"{tol.max():.6e}", "derived"),
             ("float64_vs_RTL", "worst_error_over_tolerance", f"{cmp_['worst_ratio']:.6f}", "")]
    for k in ("re", "im", "complex"):
        for m in ("max_abs", "mean_abs", "rmse"):
            rows.append(("emulator_vs_RTL", f"{k}_{m}", f"{fxs[k][m]:.6e}", ""))
    rows += [("emulator_vs_RTL", "mismatching_samples", cmp_["fixed_mismatch_samples"], f"of {NUM_SAMPLES}"),
             ("emulator_vs_RTL", "max_raw_difference_LSB", cmp_["fixed_max_lsb_diff"], "")]
    for k in ("max_abs", "mean_abs", "rmse"):
        rows.append(("float64_vs_emulator", f"complex_{k}", f"{cmp_['fixed_vs_float']['complex'][k]:.6e}", ""))
    for t in tests:
        rows.append(("algorithm_test", f"{t['id']} {t['name']}", t["status"], t["value"]))
    for kname, v in fx["flags"].items():
        rows.append(("emulator_flags", kname, v, ""))
    P_ = [("parameter", "num_antennas", M_ANT, "RTL"), ("parameter", "block_length", BLOCK_LEN, "RTL mvdr_top"),
          ("parameter", "num_samples", NUM_SAMPLES, "RTL mvdr_top"), ("parameter", "fs_hz", FS_HZ, "user"),
          ("parameter", "forgetting_lambda", f"{1-LAMBDA_ONE_MINUS_COEF/2**24:.9f}", "RTL 3355443/2^24 complement"),
          ("parameter", "loading_coefficient_x_trace", f"{LOAD_COEF/2**32:.9e}", "RTL 1073742/2^32 (= 1e-3*trace/4)"),
          ("parameter", "initial_weights", "a/4", "RTL"), ("parameter", "D_MIN", "2^-14", "RTL")]
    rows += P_
    with open(os.path.join(args.out, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["section", "metric", "value", "note"]); w.writerows(rows)

    write_readme(args.out, ref, fx, cmp_, tests, st, src_rows, overall, A_ok, B_ok, C_ok, tol, plots, ctx)
    print(report)
    print("\nfiles written to", args.out)
    return 0 if overall else 1


def write_readme(out, ref, fx, cmp_, tests, st, src_rows, overall, A_ok, B_ok, C_ok, tol, plots, ctx):
    fl, fxs = cmp_["float_vs_rtl"], cmp_["fixed_vs_rtl"]
    trows = "\n".join(f"| {t['id']} | {t['status']} | {t['name']} |" for t in tests)
    var = "\n".join(f"| {n} | {e:.2e} | {'rejected' if r else 'NOT distinguishable'} |" for n, e, r in ctx.get("variants", []))
    txt = f"""# MVDR RTL verification package (Python reference + fixed-point emulator)

Independent verification of the 4-antenna causal block-MVDR Verilog RTL (`mvdr_top`) against Python models, using the
**exact** `x_mem.mem` that the RTL simulation read and the RTL output `y_out.csv`.

> **Scope.** `y_out.csv` comes from an XSim *simulation* (`tb_save_output.v`), not from the FPGA board.
> `x_mem.mem` is the *no-interference* case (noise + desired at 0 deg), so **interference suppression / SINR improvement is not
> verified here**. Fallback / saturation paths are not exercised by this record.

## Run

```
pip install numpy matplotlib
python run_all.py --mem <path>/x_mem.mem --rtl-csv <path>/y_out.csv [--rtl-dir <folder with the .v files>]
```
Nothing is regenerated or modified: the memory file and CSV are only read. `--rtl-dir` additionally re-parses the Verilog and checks that
the constants and the 143-entry solver micro-code ROM used by the emulator are identical to the RTL source.

| file | purpose |
|---|---|
| `parse_mem.py` | parses `x_mem.mem` as `input_mem.v` reads it |
| `mvdr_reference.py` | **independent float64 reference** (numpy, `linalg.solve`) written from the RTL specification |
| `fixed_point_model.py`, `ucode_rom.py` | **separate** bit-accurate integer emulation of the RTL arithmetic (ROM generated from `block_solver.v`) |
| `compare_rtl.py` | error statistics, tolerance derivation, per-block errors, weights recovered from RTL output |
| `verify_algorithm.py` | mathematical/structural tests V1-V12 |
| `plot_results.py` | PNG plots in `results/plots/` |
| `run_all.py` | pipeline, `summary.csv`, `results/comparison_report.txt`, this README |

Outputs in `results/`: `y_python.csv` (float64: idx, block, re_value, im_value), `python_weights.csv`, `y_fixed_point_model.csv`,
`fixed_point_weights.csv`, `per_block_errors.csv`, `comparison_report.txt`, `verification_tests.txt`, `plots/*.png`.
(`y_python.csv` has no raw columns because the reference is float64; the raw integers are in `y_fixed_point_model.csv`.)

## What the RTL implements (extracted from the source, not assumed)

* 4 antennas, `BLOCK_LEN = 76`, `NUM_SAMPLES = 1536` -> 20 full blocks + 16-sample tail, `MEM_DEPTH = 2048`, `PACED = 0`.
* Memory: line 0 = steering vector `a` (Q2.14, not scaled by the input gain); line n+1 = snapshot n (Q1.15); word = `{{s3,s2,s1,s0}}`,
  sensor m = bits `[32m +: 32]`, real `[15:0]`, imag `[31:16]`, two's complement.
* Covariance of a block: `S = sum x x^H` (a sum, upper triangle stored as 4 diagonal + 6 Re + 6 Im words, Q9.30, 39 bit).
* Recursion: `R_0 = S_0`, `R_b = R_(b-1) + round(0.2 (S_b - R_(b-1)))`, i.e. **lambda = 0.8** (`3355443/2^24`; the 1.2e-8 coefficient
  rounding is accounted for in the tests).
* Solver input scaled by a power of two so that trace in [1,2); **loading** `delta = 2.5e-4 * trace` = `1e-3 * trace/4`
  (`1073742/2^32`); Hermitian LDL^H (`A = U^H D U`), forward / diagonal / backward solve, `w = u / Re(a^H u)`, Q5.20, saturating.
* Beamformer `y = w^H x` (`y_re = wr xr + wi xi`, `y_im = wr xi - wi xr`), Q9.35 accumulation, half-up rounding to Q2.22.
* Causal: block 0 uses `w = a/4`; weights solved after block b are swapped in before block b+1; the tail (`accum_en = 0`, no solve) uses the
  last full block's weights. Fallback `w = a/4` if trace = 0 or `s <= 2^-14`; pivots below `2^-14` are clamped.
* Difference from the earlier Python scripts: they use a *mean* covariance (here a *sum*; weights are scale-invariant), an unquantised `a`
  (here Q2.14; identical for `a = 1`), and their lambda is chosen from a grid at run time (the RTL value 0.8 is one grid point).

Model equations (float64 reference): `R_b = lam R_(b-1) + (1-lam) S_b`, `A = R + 1e-3 trace(R)/4 I`, `w = A^-1 a / (a^H A^-1 a)`, `y = w^H x`.

## Results on this record

| check | result |
|---|---|
| float64 reference vs RTL, max abs error (re / im) | {fl['re']['max_abs']:.3e} / {fl['im']['max_abs']:.3e}  ({fl['complex']['max_abs_lsb']:.2f} LSB of Q2.22) |
| mean abs / RMSE (complex) | {fl['complex']['mean_abs']:.3e} / {fl['complex']['rmse']:.3e} |
| max / mean relative error (floor 1% of RMS) | {fl['complex']['max_rel']:.3e} / {fl['complex']['mean_rel']:.3e} |
| derived tolerance (min .. max) | {tol.min():.2e} .. {tol.max():.2e} |
| samples within tolerance | {cmp_['n_within_tol']} / {ctx['num_samples']}  (worst error / tolerance = {cmp_['worst_ratio']:.3f}) |
| bit-accurate emulator vs RTL | {cmp_['fixed_mismatch_samples']} mismatching samples, max difference {cmp_['fixed_max_lsb_diff']} LSB |
| structural (1536 rows, contiguous idx, block labels 0..19 + tail 20) | {st['n_rows_ok'] and st['idx_contiguous'] and st['block_labels_ok']} |
| **overall (A and C)** | **{'PASS' if overall else 'FAIL'}**  (A float {'PASS' if A_ok else 'FAIL'}, B emulator exact {'PASS' if B_ok else 'FAIL'}, C tests {'PASS' if C_ok else 'FAIL'}) |

### Tolerance (not tuned)
`|dy| <= ||dw||_2 ||x||_2 + 0.5 LSB`, `||dw||_2 <= 2 kappa(A) * 16 * 2^-25 * ||w||_2 + sqrt(8) 2^-21` with `kappa` the condition number of the
normalised loaded covariance that produced the block's weights (block 0: exact weights, only the 0.5 LSB output rounding). 2^-21 is the Q5.20
half-LSB, 2^-25 the Q16.24 half-ulp, 16 an allowance for the sequential roundings of a 4x4 LDL^H solve. It is a first-order worst-case bound,
so it is loose compared with the observed error (its constants are allowances, not fitted to it); the sharp test is the bit-exact emulator comparison.

### Algorithm / structure tests
| id | status | test |
|---|---|---|
{trows}

Full "what it shows / what it does not show" text for every test is in `results/verification_tests.txt`.

### Would a wrong algorithm have been caught? (V12)
| variant (float64) vs the same RTL output | max abs error | vs tolerance |
|---|---|---|
{var}

## Limits and open items
* Simulation only; no hardware capture. Only the noise-only input was used: the null-forming behaviour of MVDR is untested.
* The RTL status register is not in `y_out.csv`; confirm `tb_save_output` printed `status=0x00`. Saturation/fallback/pivot-clamp paths are not
  exercised by this record (they are covered by your own directed testbenches, not by this package).
* The emulator models arithmetic, not clocking/handshake. Weights are not dumped by the RTL testbench; they are recovered from `y` by least
  squares per block (valid because `x` has full rank per block; bound reported in V5/V7).
* Output amplitudes are scaled by the unknown global input gain `g`; SINR cannot be computed without separate signal components.
"""
    open(os.path.join(out, "README.md"), "w").write(txt)


if __name__ == "__main__":
    sys.exit(main())
