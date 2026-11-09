# MVDR RTL verification package (Python reference + fixed-point emulator)

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
* Memory: line 0 = steering vector `a` (Q2.14, not scaled by the input gain); line n+1 = snapshot n (Q1.15); word = `{s3,s2,s1,s0}`,
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
| float64 reference vs RTL, max abs error (re / im) | 1.157e-06 / 9.948e-07  (4.87 LSB of Q2.22) |
| mean abs / RMSE (complex) | 4.078e-07 / 4.664e-07 |
| max / mean relative error (floor 1% of RMS) | 2.270e-06 / 9.112e-07 |
| derived tolerance (min .. max) | 1.19e-07 .. 4.10e-05 |
| samples within tolerance | 1536 / 1536  (worst error / tolerance = 0.055) |
| bit-accurate emulator vs RTL | 0 mismatching samples, max difference 0 LSB |
| structural (1536 rows, contiguous idx, block labels 0..19 + tail 20) | True |
| **overall (A and C)** | **PASS**  (A float PASS, B emulator exact PASS, C tests PASS) |

### Tolerance (not tuned)
`|dy| <= ||dw||_2 ||x||_2 + 0.5 LSB`, `||dw||_2 <= 2 kappa(A) * 16 * 2^-25 * ||w||_2 + sqrt(8) 2^-21` with `kappa` the condition number of the
normalised loaded covariance that produced the block's weights (block 0: exact weights, only the 0.5 LSB output rounding). 2^-21 is the Q5.20
half-LSB, 2^-25 the Q16.24 half-ulp, 16 an allowance for the sequential roundings of a 4x4 LDL^H solve. It is a first-order worst-case bound,
so it is loose compared with the observed error (its constants are allowances, not fitted to it); the sharp test is the bit-exact emulator comparison.

### Algorithm / structure tests
| id | status | test |
|---|---|---|
| V1 | PASS | Per-block covariance S: integer emulation of the RTL accumulator == float sum x x^H |
| V2 | PASS | Recursive covariance R_b = 0.8 R_{b-1} + 0.2 S_b (R_0 = S_0): emulator vs float |
| V3 | PASS | Covariance Hermitian symmetry and non-negative real diagonal |
| V4 | PASS | Positive definiteness of the loaded, normalised covariance; LDL^H pivots above D_MIN = 2^-14 |
| V5 | PASS | Distortionless constraint w^H a = 1 (float, quantised RTL-model weights, weights recovered from RTL output) |
| V6 | PASS | Solver weights (fixed-point LDL^H model) vs float w = R^-1 a / (a^H R^-1 a) from numpy.linalg.solve |
| V7 | PASS | Blockwise weights recovered from the RTL output (least squares on y = w^H x) vs model weights |
| V8 | PASS | Causal weight update (perturb only block 7 of the input) |
| V9 | PASS | Tail handling (16 samples after 20 full blocks) |
| V10 | PASS | No saturation, pivot clamp, solver fallback or zero-trace event |
| V11 | PASS | Block 0 uses quiescent weights w = a/4 |
| V12 | INFO | Discriminating power: do wrong algorithm variants fail the same comparison? |

Full "what it shows / what it does not show" text for every test is in `results/verification_tests.txt`.

### Would a wrong algorithm have been caught? (V12)
| variant (float64) vs the same RTL output | max abs error | vs tolerance |
|---|---|---|
| forgetting 0.95 (HLS-design value) | 1.11e-01 | rejected |
| forgetting 0.5 | 1.24e-01 | rejected |
| loading 1e-2 | 1.50e-02 | rejected |
| loading 1e-4 | 1.68e-03 | rejected |
| no diagonal loading | 1.87e-03 | rejected |
| NON-causal (block uses own weights) | 1.65e-01 | rejected |

## Limits and open items
* Simulation only; no hardware capture. Only the noise-only input was used: the null-forming behaviour of MVDR is untested.
* The RTL status register is not in `y_out.csv`; confirm `tb_save_output` printed `status=0x00`. Saturation/fallback/pivot-clamp paths are not
  exercised by this record (they are covered by your own directed testbenches, not by this package).
* The emulator models arithmetic, not clocking/handshake. Weights are not dumped by the RTL testbench; they are recovered from `y` by least
  squares per block (valid because `x` has full rank per block; bound reported in V5/V7).
* Output amplitudes are scaled by the unknown global input gain `g`; SINR cannot be computed without separate signal components.
