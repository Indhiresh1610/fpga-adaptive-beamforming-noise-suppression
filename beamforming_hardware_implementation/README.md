# 4-Antenna Causal Block MVDR Beamformer on Spartan-7 FPGA

[![Target FPGA: Spartan-7 XC7S50](https://img.shields.io/badge/FPGA-Spartan--7%20XC7S50-blue.svg)](#hardware-platform)
[![Board: Real Digital Boolean](https://img.shields.io/badge/Board-Real%20Digital%20Boolean-orange.svg)](#hardware-platform)
[![Verification: 100% Bit-Exact](https://img.shields.io/badge/Hardware%20Validation-100%25%20Bit--Exact-brightgreen.svg)](#final-hardware-validation-results)
[![Precision: Q2.22 / Q16.24](https://img.shields.io/badge/Fixed--Point-Q2.22%20%7C%20Q16.24%20%7C%20Q5.20-green.svg)](#fixed-point-quantization-architecture)

This repository contains the complete, verified hardware implementation and reference verification suite for a **4-antenna, causal, block-based Minimum Variance Distortionless Response (MVDR) beamformer** synthesized and physically validated on the **Real Digital Boolean Board (AMD/Xilinx Spartan-7 XC7S50-CSGA324 FPGA)**.

The hardware implementation processes 2047 complex baseband snapshots across 4 antenna elements, adaptively estimating sample covariance matrices, performing diagonal loading, solving weights via a custom 143-instruction microcode LDL$^H$ solver, and steering beam patterns in real time at a 50 MHz core clock. Physical hardware output extracted via passive BRAM-buffered UART uplink demonstrates **100% bit-exact arithmetic agreement** with the fixed-point RTL model and strict compliance with the double-precision float64 reference model.

---

## 1. Repository Organization

```text
sp project work/
├── inputs/                            # Memory images (.mem) & signal generator scripts
│   ├── input.py                       # Automated multi-case signal generator
│   ├── case_1.mem ... case_7.mem      # 7 pre-computed input cases (2048 words each)
│   ├── input_mem.v                    # BRAM signal + steering vector loader (selects case_2.mem)
│   ├── x_mem.mem                      # Baseline memory image
│   └── mvdr_block.cpp/.h              # HLS reference kernels
│
├── outputs/                           # Synthesizable RTL, testbenches & uplink tools
│   ├── main_rtl/                      # Complete standalone RTL module suite
│   │   ├── input_mem.v                # Signal BRAM loader
│   │   ├── mvdr_top.v                 # Core beamformer top-level
│   │   ├── block_controller.v         # Main sequencing FSM (26 full blocks + 71-sample tail)
│   │   ├── block_solver.v             # Microcoded LDL^H solver (143 ROM instructions)
│   │   ├── sample_engine.v            # Covariance accumulator & MAC beamformer
│   │   ├── weight_regs.v              # Ping-pong active/next weight registers
│   │   ├── serial_divider.v           # Non-restoring serial integer divider
│   │   ├── output_interface.v         # Output skid buffer & backpressure interface
│   │   └── mvdr_board_top.v           # Board top: MMCM (100->50 MHz), capture BRAM, UART dump
│   ├── test_benches/                  # Self-checking simulation testbenches
│   │   ├── tb_save_output.v           # Core simulation testbench (2047 samples)
│   │   ├── tb_uplink.v                # End-to-end UART capture testbench (2047 samples)
│   │   └── tb_sample_engine.v, ...    # Unit testbenches
│   ├── Verification_infr/             # Testbench common headers
│   │   └── tb_common.vh               # Shared tasks and clock macros
│   └── fpga_uplink_rx.py              # Dual-mode UART receiver & automated verification tool
│
├── python_reference/                  # Canonical Case 2 Python reference package
│   ├── run_all.py                     # Master verification pipeline runner
│   ├── mvdr_reference.py              # Float64 unquantized causal MVDR reference
│   ├── fixed_point_model.py           # Bit-accurate integer C/RTL cycle model
│   ├── parse_mem.py                   # .mem file loader
│   ├── compare_rtl.py                 # Error statistics & derived tolerance bounds
│   ├── verify_algorithm.py            # 12 mathematical verification tests (V1..V12)
│   ├── plot_results.py                # Publication-quality plotting generator
│   ├── ucode_rom.py                   # 143-entry solver microcode table
│   ├── results/                       # Canonical Case 2 CSV deliverables & reports
│   │   ├── y_python.csv               # Float64 beamformer output (2047 samples)
│   │   ├── y_fixed_point_model.csv    # Fixed-point model output (2047 samples)
│   │   ├── python_weights.csv         # Float64 weights (27 sets)
│   │   ├── fixed_point_weights.csv    # Fixed-point weights (27 sets)
│   │   ├── fpga_y_reference.csv       # Reference formatted identically to UART capture
│   │   ├── summary.csv                # Summary metrics
│   │   ├── comparison_report.txt      # Full validation report
│   │   ├── verification_tests.txt     # Test V1-V12 logs
│   │   └── per_block_errors.csv       # Per-block error breakdown
│   └── plots/                         # Reference visualization plots (7 PNGs)
│
├── fpga_outputs/                      # Actual UART captures & hardware validation
│   ├── case_2/                        # Captured physical FPGA hardware outputs
│   │   ├── fpga_case2_y.csv           # 2047 UART-extracted samples (re_raw, im_raw, Q2.22)
│   │   ├── fpga_case2_weights.csv     # 27 UART-extracted weight sets (Q5.20)
│   │   └── fpga_case2_meta.json       # Hardware status, cycle count, checksum telemetry
│   └── final_case2_validation/        # Comprehensive hardware vs. model validation
│       ├── final_case2_comparison_report.txt
│       ├── reports/summary.csv
│       ├── reports/per_block_validation.csv
│       └── plots/                     # 6 comparison plots (FPGA vs Python)
│
├── Beamforming_MVDR/                  # Vivado project (XC7S50-CSGA324)
│   ├── Beamforming_MVDR.xpr           # Vivado project file
│   └── Beamforming_MVDR.srcs/         # Pin constraints (boolean_pins.xdc, timing.xdc)
│
├── vivado_reports/                    # Implementation reports (timing, power, utilization)
├── archive/                           # Preserved historical assets (pre-consolidation backups)
├── .gitignore                         # Clean git-tracking rules
└── README.md                          # This document
```

---

## 2. Canonical Hardware Validation Scenario (Case 2)

The primary validation benchmark is **Case 2 (`case_2.mem`)**, designed to rigorously challenge the beamformer across all degrees of freedom without artificial numerical distortion.

| Parameter | Specification | Hardware Reality |
|---|---|---|
| **Array Geometry** | 4-element Uniform Linear Array (ULA), $d = 0.5\lambda$ | 4 complex antenna snapshot streams |
| **Desired Signal ($\theta_{des}$)** | **$-20^\circ$** ($a = [1, 0.4762+0.8793j, -0.5465+0.8375j, -0.9966-0.0818j]$) | Non-zero complex steering vector |
| **Interferer 1** | $\theta = -60^\circ$, Power = 10.0 (+10 dB relative to signal) | Null placed at $-60^\circ$ |
| **Interferer 2** | $\theta = +15^\circ$, Power = 10.0 (+10 dB relative to signal) | Null placed at $+15^\circ$ |
| **Interferer 3** | $\theta = +70^\circ$, Power = 10.0 (+10 dB relative to signal) | Null placed at $+70^\circ$ |
| **Total Null Capacity** | $M - 1 = 3$ independent spatial nulls | **100% degrees of freedom utilized** |
| **Total Processed Samples** | **2047 usable samples** ($n = 0 \dots 2046$) | 26 full blocks ($K=76$) + 1 tail block ($71$ samples) |
| **Adaptive Weight Updates** | 27 distinct weight sets | Block 0 quiescent ($a/4$), blocks 1..25 solved, tail inherits |
| **Covariance Conditioning** | $\kappa(A) \in [20.1, 24.0]$, $\lambda_{min} \approx 0.0287$ | Stable, well-posed LDL$^H$ pivots ($> 0.075$) |

---

## 3. Fixed-Point Quantization Architecture

The RTL avoids floating-point overhead through a tailored fixed-point numerical scheme:

- **Input Snapshots ($x$)**: **Q1.15** signed 16-bit real and imaginary components, scaled by global input gain $g = 0.082728$ ($PEAK = 0.9$).
- **Steering Vector ($a$)**: **Q2.14** signed 16-bit words (`1.0 = 0x4000 = 16384`).
- **Covariance Accumulator ($S$)**: **Q9.30** accumulation across 76 samples per block (39-bit dynamic range).
- **Recursive Covariance ($R$)**: $R_b = 0.8 R_{b-1} + 0.2 S_b$ implemented via $R + \text{round}(0.2 \cdot (S - R))$ with scaling constant `LAMBDA_ONE_MINUS_COEF = 3355443` ($\approx 0.2 \times 2^{24}$).
- **Diagonal Loading**: $\delta = 10^{-3} \times \frac{\text{trace}(R)}{M} \cdot I$, implemented with `LOAD_COEF = 1073742` ($\approx 2.5 \times 10^{-4} \times 2^{32}$).
- **Microcoded Solver Registers**: **Q16.24** 40-bit internal fixed-point words. Pivot clamp floor $D_{MIN} = 2^{-14}$ (`1024` in Q16.24).
- **Weight Registers ($w$)**: **Q5.20** signed 25-bit words per component (50 bits per antenna, 200 bits total).
- **Beamformer Output ($y$)**: **Q2.22** signed 24-bit real and imaginary components (48 bits total: `{im[23:0], re[23:0]}`). $1\ \text{LSB} = 2^{-22} \approx 2.384 \times 10^{-7}$.

---

## 4. Final Hardware Validation Results

Physical hardware extraction via the Boolean Board's USB-UART bridge at 115200 baud yielded the following results against both reference models:

### 1. Bit-Exact Match against Fixed-Point RTL Model
- **Sample Output (`re_raw`, `im_raw`)**: **2047 / 2047 BIT-EXACT (0 mismatches)** across all real and imaginary integer channels.
- **Weight Registers (`w_raw`)**: **27 / 27 BIT-EXACT (0 mismatches)** across all 4 complex antenna channels.
- **Hardware Telemetry (`fpga_case2_meta.json`)**:
  - Status register: `0x00` (zero saturation, zero pivot clamping, zero configuration or solver error).
  - Solve duration: **1008 clock cycles** ($20.16\ \mu\text{s}$ at 50 MHz).
  - UART integrity: **0 bad checksums, 0 skipped bytes**.
  - Checksum parity: **`1`** (matches LED `LD3` illuminated state on the physical board).

### 2. Numerical Compliance against Double-Precision Float64 Reference
- **Real Component Error**: Max absolute error = $1.1193 \times 10^{-6}$ (**4.69 output LSBs**).
- **Imaginary Component Error**: Max absolute error = $1.0145 \times 10^{-6}$ (**4.26 output LSBs**).
- **Complex Magnitude Error**: Max absolute error = $1.1271 \times 10^{-6}$ (**4.73 output LSBs**).
- **Complex Output RMSE**: $3.8818 \times 10^{-7}$ (**1.63 output LSBs**).
- **Derived Numerical Bound Compliance**: **2047 / 2047 samples (100%) within theoretical tolerance**.

---

## 5. Reproduction & Execution Guide

### Running the Python Reference Model
From the repository root:
```powershell
# Python-only reference generation (Case 2):
python python_reference/run_all.py --mem inputs/case_2.mem --rtl-dir outputs/main_rtl

# Cross-verifying Python models against captured FPGA output:
python python_reference/run_all.py --mem inputs/case_2.mem --rtl-csv fpga_outputs/case_2/fpga_case2_y.csv --rtl-dir outputs/main_rtl
```

### Simulating in Vivado
1. Open `Beamforming_MVDR/Beamforming_MVDR.xpr` in Vivado 2023.x / 2024.x.
2. Set simulation top to `tb_save_output` (for direct CSV generation) or `tb_uplink` (for simulated UART dump).
3. Run simulation (`Run Simulation -> Run Behavioral Simulation`).

### Running Hardware Capture Over UART
With the Boolean board connected to USB and programmed with the generated bitstream:
```powershell
python outputs/fpga_uplink_rx.py --port COM5 --start --compare python_reference/results/y_python.csv --out fpga_outputs/case_2/fpga_case2
```
*(Replace `COM5` with your board's serial port, e.g. `/dev/ttyUSB1` on Linux).*
