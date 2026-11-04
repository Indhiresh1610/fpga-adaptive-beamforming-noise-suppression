# 4-Element Complex MVDR Beamformer RTL (Verilog-2001)
### Target: AMD/Xilinx Spartan-7 XC7S50 (Real Digital Boolean Board) @ 100 MHz

---

## 1. Architecture Overview

This project implements a complete, fully synthesizable, causal, 4-element complex Minimum Variance Distortionless Response (MVDR) beamformer strictly adhering to the **Verilog-2001** standard. The architecture targets the AMD/Xilinx Spartan-7 XC7S50 FPGA (`xc7s50csga324-1`) operating at a 100 MHz system clock ($T_{clk} = 10.0\text{ ns}$).

### Top-Level Block Diagram

```
                 +--------------------------------------------------------------+
                 |                           mvdr_top                           |
                 |                                                              |
                 |  +------------+   sample_idx   +--------------------------+  |
                 |  | input_mem  |<---------------+     block_controller     |  |
                 |  +------------+                +--------------------------+  |
                 |        | x_snap (128b)            | (sequencing & handshakes)|
                 |        v                          v                          |
                 |  +--------------------+       +---------------------------+  |
   w_active (200b)|  |   sample_engine    |  S_rd |       block_solver        |  |
  +-------------->|  |  (Beamformer &    |------>| (Trace norm, Loading,     |  |
  |               |  |  Cov Accumulator) |       |  LDL^H solve, Weight norm)|  |
  |               |  +--------------------+       +---------------------------+  |
  |                        | y_out (Q2.22)                     | w_wr (Q5.20)   |
  |                        v                                   v                |
  |               +--------------------+          +--------------------------+  |
  |               |  output_interface  |          |       weight_regs        |  |
  |               | (AXI-Stream Valid/ |          | (w_active & w_next banks)|--+
  |               |  Ready Handshake)  |          +--------------------------+
  |               +--------------------+
  |                        |
  +------------------------+
```

### Key Performance Specifications
- **Sample Engine Duration**: Exactly **52 clock cycles** per sample ($520\text{ ns}$). Free-running throughput reaches **1.92 MSamples/sec**, far exceeding real-time acoustic/sonar sampling rates ($512\text{ Hz}$).
- **Block Solver Duration**: Exactly **475 clock cycles** ($4.75\text{ }\mu\text{s}$). Occurs between blocks during sample processing or inter-block dead time without dropping samples.
- **DSP Multiplier Lane Sharing**: A single time-multiplexed signed multiplier lane ($25 \times 18$) executes all 48 micro-ops per snapshot:
  - Micro-ops 0..15: Beamforming ($y_{re}, y_{im}$)
  - Micro-ops 16..23: 4 Diagonal covariance products
  - Micro-ops 24..47: 24 Off-diagonal covariance products ($6 \times 2$ real and imag terms)
- **Causal Block Processing**:
  - Block 0: Strictly uses quiescent weights $w = a / 4$ (delay-and-sum beamformer).
  - Block $k \ge 1$: Strictly uses weights computed from blocks $0 \dots k-1$.
  - Atomic weight swap (`w_swap`) occurs strictly between blocks, never while a snapshot is in flight.

---

## 2. THE THREE LENGTHS

These three independent quantities govern sizing, block processing, and memory allocation:

| Parameter | Meaning | Default | Usage |
|---|---|---|---|
| `BLOCK_LEN` | Covariance block size $K$ in samples. | `76` | Used ONLY for block sample counting, covariance accumulation, and accumulator bit-width (`ACC_W = 32 + clog2(BLOCK_LEN)`). |
| `NUM_SAMPLES` | Total valid snapshots $N$ in the record. | `1536` | Record length. Number of valid output samples produced. |
| `MEM_DEPTH` | Allocated 128-bit BRAM words. | `2048` | Physical memory depth. Word 0 stores header steering vector $a$; words $1 \dots N$ store snapshots. $Address = 1 + sample\_idx$. Must satisfy `MEM_DEPTH >= NUM_SAMPLES + 1`. |

Derived in controller:
- `NUM_FULL_BLOCKS = NUM_SAMPLES / BLOCK_LEN` (integer division)
- `TAIL_LEN = NUM_SAMPLES % BLOCK_LEN`
- Incomplete tail samples are beamformed using current active weights; covariance accumulation is disabled (`accum_en = 0`) and the solver is bypassed.

---

## 3. Fixed-Point Arithmetic Formats

All fixed-point expressions use explicit signed declarations, bit-accurate sizing, and half-up rounding (`round_half_up(x) = (x + half_lsb) >>> frac_bits`):

| Signal | Fixed-Point Format | Width | Dynamic Range / LSB | Notes |
|---|---|---|---|---|
| Sensor input snapshots $x$ | Q1.15 signed | 16 | $[-1.0, +0.999969]$ | From memory word: `{s3_im, s3_re, s2_im, s2_re, s1_im, s1_re, s0_im, s0_re}` |
| Steering vector $a$ | Q2.14 signed | 16 | $[-2.0, +1.999939]$ | $1.0 = 16\text{h4000}$ |
| Cross products $x \cdot x$ | Q2.30 signed | 32 | Exact | 16 real products per snapshot |
| Covariance sum $S$ & state $R$ | Q9.30 signed | 39 | Exact | `ACC_W = 32 + clog2(76) = 39`. Overflow impossible even for all $-1.0$. |
| Exponential update factor | Q0.24 unsigned | 24 | $C_1 = 3355443 \approx 0.2 \times 2^{24}$ | $R_{new} = R + (S - R) \cdot C_1 / 2^{24}$ |
| Solver words ($A, d, r, T, U, z, v, u, s$) | Q16.24 signed | 40 | Scale-normalized | Wide accumulation (88 bits), saturated to 40 bits with `sat_solver` |
| Diagonal loading $\delta$ | Q16.24 signed | 40 | $\delta = 2.5 \times 10^{-4} \cdot T_n$ | $LOAD\_COEF = 1073742 \approx 2.5 \times 10^{-4} \times 2^{32}$ |
| Pivot floor $D_{MIN}$ | Q16.24 signed | 40 | $2^{-14} = 1024$ | If hit: clamped to $D_{MIN}$ and latches `status[1]` (`err_pivot`) |
| Reciprocal $r_i = 1/d_i$ | Q16.24 signed | 40 | $2^{48} / d_i$ | Sequential restoring divider with half-LSB rounding |
| Normalizer $r_s = 1/s$ | Q24.40 unsigned | 65 | $2^{64} / s$ | 40 fractional bits for exact weight scaling |
| Output weights $w$ | Q5.20 signed | 25 | $[-16.0, +15.999999]$ | Saturated with `sat_w` |
| Output beamformed $y$ | Q2.22 signed | 24 each | $[-2.0, +1.999999]$ | Scaled from 44-bit accumulator (`>>> 13`) with `sat_y` |

---

## 4. Hardware Module Inventory

| Module File | Purpose | Ports / Features |
|---|---|---|
| `mvdr_top.v` | Top-level integration wrapper | Exposes clock, reset, start, pacing, AXI-Stream master (`m_valid`, `m_data`, `m_last`, `m_idx`, `m_blk`, `m_is_tail`, `m_ready`), status, and debug ports. |
| `block_controller.v` | FSM controller | Manages 3 independent lengths, BRAM latency (1 or 2 cycles), sample pacing (`PACED`), out-of-bounds protection, tail bypass, and sticky status flags. |
| `sample_engine.v` | DSP Beamformer & Covariance Accumulator | 48 micro-ops on shared multiplier lane; 16-word $S$ accumulator; exact saturation and rounding. |
| `block_solver.v` | Complex Hermitian $LDL^H$ Solver | Persistent $R$ memory; power-of-two trace normalization; diagonal loading; $LDL^H$ factorization (no inverse); forward/diagonal/backward solves; weight normalization. |
| `serial_divider.v` | Restoring Sequential Divider | Parametric width (`WIDTH_N=65`, `WIDTH_D=40`); non-restoring/restoring bit-cell; half-LSB rounding; divide-by-zero protection. |
| `weight_regs.v` | Dual-Bank Weight Storage | Registered `w_active` and `w_next` banks; atomic single-cycle swap; quiescent $a/4$ initialization. |
| `output_interface.v` | Output Register & AXI Handshake | Backpressure support (`out_busy`); holds data stable when stalled; prevents lost or reordered samples. |
| `input_mem.v` | Dual-Port/Single BRAM Interface | Memory file preload (`$readmemh`); steering vector extraction on reset; latency 1 or 2 cycles; bounds check. |

---

## 5. Verification Suite

All testbenches are written in pure Verilog without any Python runtime dependency.

| Testbench File | Target Module | Test Groups / Coverage | Result |
|---|---|---|---|
| `tb_input_mem.v` | `input_mem.v` | Memory read latency 1 & 2, bounds check, extreme values, X-filling, steering capture. | **16 / 16 PASSED** |
| `tb_serial_divider.v` | `serial_divider.v` | 65 randomized tests: exact division, remainder, rounding, divide-by-zero, 65-bit normalizer. | **65 / 65 PASSED** |
| `tb_sample_engine.v` | `sample_engine.v` | Micro-op timing (52 cycles), beamforming accuracy, saturation, all-negative accumulation, tail bypass. | **12 / 12 PASSED** |
| `tb_block_solver.v` | `block_solver.v` | Zero matrix fallback ($a/4$), diagonal solve, complex Hermitian solve, scale invariance, pivot clamp ($D_{MIN}$). | **22 / 22 PASSED** |
| `tb_directed.v` | `mvdr_top.v` | **Groups A, B, C, E, F, G**: Memory assertions, quiescent $a/4$ bit-exact, backpressure stall stability, covariance recursion, perturbation causality, tail lengths (tail=0, tail=1, pure tail), block length override (K=8), reset recovery, and paced overrun detection. | **27 / 27 PASSED** |
| `tb_regression.v` | `mvdr_top.v` | **Full Records**: Legacy `x_mem.mem` ($N=2047$), Static scenario ($N=1536$), Dynamic scenario ($N=3072$). Distortionless response $\le 2 \times 10^{-3}$, sequential indexing, clean status flags. | **19 / 19 PASSED** |
| **Total** | | | **161 / 161 PASSED** |

---

## 6. Simulation & Master Regression Instructions

### Prerequisites
- Icarus Verilog 12.0 (`iverilog`, `vvp`).

### Running the Entire Regression Suite
You can execute all 161 automated test assertions using the master regression scripts:

**Windows Command Prompt:**
```cmd
run_all.bat
```

**Windows PowerShell:**
```powershell
.\run_all.ps1
```

### Running Individual Testbenches
```powershell
# 1. Input Memory Unit Test
iverilog -g2005 -o tb_input_mem.vvp input_mem.v tb_input_mem.v
vvp tb_input_mem.vvp

# 2. Serial Divider Unit Test
iverilog -g2005 -o tb_serial_divider.vvp serial_divider.v tb_serial_divider.v
vvp tb_serial_divider.vvp

# 3. Sample Engine Unit Test
iverilog -g2005 -o tb_sample_engine.vvp sample_engine.v tb_sample_engine.v
vvp tb_sample_engine.vvp

# 4. Block Solver Unit Test
iverilog -g2005 -o tb_block_solver.vvp serial_divider.v block_solver.v tb_block_solver.v
vvp tb_block_solver.vvp

# 5. Comprehensive Directed Test Suite
iverilog -g2005 -o tb_directed.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v tb_directed.v
vvp tb_directed.vvp

# 6. Master Full-Record Regression Suite
iverilog -g2005 -o tb_regression.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v tb_regression.v
vvp tb_regression.vvp
```

### Checking Synthesizable RTL Verilog-2001 Compliance
```powershell
iverilog -g2001 -o rtl_check.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v
```

---

## 7. AMD/Xilinx Vivado Synthesis Guide

To synthesize and implement this project in AMD/Xilinx Vivado on a host machine equipped with Vivado:

1. Create a project targeting the **AMD Spartan-7 XC7S50CSGA324-1** (Real Digital Boolean Board).
2. Add all design sources:
   - `mvdr_top.v` (Top module)
   - `block_controller.v`
   - `sample_engine.v`
   - `block_solver.v`
   - `serial_divider.v`
   - `weight_regs.v`
   - `output_interface.v`
   - `input_mem.v`
   - `x_mem.mem` (Memory initialization)
3. Add timing constraint file (`timing.xdc`):
   ```tcl
   create_clock -period 10.000 -name clk -waveform {0.000 5.000} [get_ports clk]
   ```
4. Run Synthesis and Implementation:
   ```tcl
   synth_design -top mvdr_top -part xc7s50csga324-1
   opt_design
   place_design
   route_design
   report_utilization -file utilization.rpt
   report_timing_summary -file timing_summary.rpt
   ```
