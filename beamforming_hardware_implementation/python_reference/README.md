# MVDR Beamformer Python Reference Package (Case 2 Canonical)

This package contains the canonical floating-point and bit-accurate fixed-point reference models for the 4-antenna causal block MVDR beamformer implemented on the Spartan-7 FPGA (Real Digital Boolean Board).

## Configuration (Case 2)
- **Input Memory Image**: `inputs/case_2.mem` (2048 words: word 0 = steering vector $a$ in Q2.14; words 1..2047 = data snapshots in Q1.15)
- **Usable Samples**: **2047** ($n = 0 \dots 2046$)
- **Antenna Count**: $M = 4$ uniform linear array ($d = 0.5\lambda$)
- **Desired Steering Angle**: $\theta_{des} = -20^\circ$ ($a = [1+0j, 0.4762+0.8793j, -0.5465+0.8375j, -0.9966-0.0818j]$)
- **Interferers**: 3 strong stationary interferers at $-60^\circ, +15^\circ, +70^\circ$ (10 dB each)
- **Block Organization**:
  - Block length: $K = 76$
  - Full blocks: 26 (Blocks 0..25, samples $0 \dots 1975$)
  - Tail partial-block: 1 (Block 26, 71 samples, samples $1976 \dots 2046$)
  - Total adaptive weight sets: 27 (Block 0 uses quiescent $a/4$; Block $b \ge 1$ uses weights solved from $R_{b-1}$; Block 26 tail inherits weights from Block 25)
- **Algorithm Parameters**:
  - Sample rate $f_s = 512$ Hz
  - Forgetting factor $\lambda = 0.8$
  - Diagonal loading factor $\delta = 10^{-3} \times \frac{\text{trace}(R)}{M} \cdot I$
  - Minimum pivot clamp floor $D_{MIN} = 2^{-14}$

---

## File Contents

```
python_reference/
├── run_all.py                 # Master verification pipeline runner
├── mvdr_reference.py          # Float64 unquantized causal MVDR reference
├── fixed_point_model.py       # Bit-accurate integer C/RTL cycle model
├── parse_mem.py               # Memory image parser (.mem format)
├── compare_rtl.py             # Error statistics & derived mathematical tolerance bounds
├── verify_algorithm.py        # 12 algorithmic verification tests (V1..V12)
├── plot_results.py            # Result visualization & plot generator
├── ucode_rom.py               # 143-entry solver microcode ROM table
├── README.md                  # This file
│
├── results/                   # Canonical Case 2 generated deliverables
│   ├── y_python.csv           # Float64 beamformer output (2047 samples)
│   ├── python_weights.csv     # Float64 weights per block (27 sets)
│   ├── y_fixed_point_model.csv# Bit-accurate fixed-point output (2047 samples)
│   ├── fixed_point_weights.csv# Bit-accurate fixed-point weights (27 sets)
│   ├── fpga_y_reference.csv   # Reference formatted for direct UART diffing
│   ├── summary.csv            # Summary metrics and verdicts
│   ├── comparison_report.txt  # Full textual verification report
│   ├── verification_tests.txt # V1-V12 test logs
│   └── per_block_errors.csv   # Per-block error statistics
│
└── plots/                     # High-resolution PNG plots
    ├── 01_real_python_vs_rtl.png
    ├── 02_imag_python_vs_rtl.png
    ├── 03_abs_error_vs_sample.png
    ├── 04_error_per_block.png
    ├── 05_weight_evolution.png
    ├── 06_error_hist_and_conditioning.png
    └── 07_variant_sensitivity.png
```

---

## How to Run

### 1. Python-Only Reference Mode (from project root)
To run the reference pipeline without needing hardware output:
```powershell
python python_reference/run_all.py --mem inputs/case_2.mem --rtl-dir outputs/main_rtl
```
*(If running from within `python_reference/`, use `python run_all.py --mem ../inputs/case_2.mem --rtl-dir ../outputs/main_rtl`).*

### 2. Full Comparison Mode (with FPGA Hardware Output)
To cross-verify against captured physical FPGA hardware output:
```powershell
python python_reference/run_all.py --mem inputs/case_2.mem --rtl-csv fpga_outputs/case_2/fpga_case2_y.csv --rtl-dir outputs/main_rtl
```
This executes all structural checks, fixed-point bit-exactness checks, and float64 numerical tolerance tests.
