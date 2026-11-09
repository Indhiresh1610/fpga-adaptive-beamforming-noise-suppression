# MVDR Beamformer RTL Simulation & Verification Results
### Target: AMD/Xilinx Spartan-7 XC7S50 (Real Digital Boolean Board) @ 100 MHz

---

## 1. Executive Summary

| Verification Category | Testbench | Total Checks | Passed | Failed | Status |
|---|---|---|---|---|---|
| Input Memory Unit Tests | `tb_input_mem.v` | 16 | 16 | 0 | **ALL PASSED** |
| Serial Divider Unit Tests | `tb_serial_divider.v` | 65 | 65 | 0 | **ALL PASSED** |
| Sample Engine Unit Tests | `tb_sample_engine.v` | 12 | 12 | 0 | **ALL PASSED** |
| Block Solver Unit Tests | `tb_block_solver.v` | 22 | 22 | 0 | **ALL PASSED** |
| Comprehensive Directed Suite | `tb_directed.v` | 27 | 27 | 0 | **ALL PASSED** |
| Master Full-Record Regression | `tb_regression.v` | 19 | 19 | 0 | **ALL PASSED** |
| **Total Test Assertions** | | **161** | **161** | **0** | **100% PASSED** |

> [!NOTE]
> **Environment Note**: All simulation evidence was produced directly using Icarus Verilog 12.0 (`iverilog`, `vvp`) under Windows. Synthesis was **NOT run on this machine** because AMD/Xilinx Vivado is not installed on the host environment. All RTL compiles cleanly with `iverilog -g2001` (strict Verilog-2001) without warnings.

---

## 2. Key Measured Performance & Latency Metrics

- **Sample Engine Duration**: Measured at **52 clock cycles** per snapshot ($520\text{ ns}$ at 100 MHz).
  - Multiplier lane latency: 2 pipeline stages.
  - Beamformed output $y$ ready at clock 18.
  - 16 covariance accumulators finished at clock 51.
- **Block Solver Duration**: Measured at **475 clock cycles** for full complex Hermitian factorization and weight calculation ($4.75\text{ }\mu\text{s}$ at 100 MHz).
  - Zero-trace fallback detection: 26 clock cycles.
- **Distortionless Constraint**: $|w^H a - 1| = 0.000000 \times 10^0 \le 2.0 \times 10^{-3}$ across all test cases.
- **Weight Relative Error**: $\max |\Delta w| / \max |w_{ref}| = 2.97 \times 10^{-4} \le 2.0 \times 10^{-3}$ against double-precision reference model.
- **Scale Invariance**: Bit-exact identical weights under input power-of-two matrix scaling ($1/16$).

---

## 3. Actual Simulation Logs

### 3.1. Master Full-Record Regression Suite (`tb_regression.v`)

```text
==================================================
Starting Master MVDR Regression Suite
==================================================

--- Running Test 1: Legacy Supplied Record (x_mem.mem) ---
Legacy record initialized. Starting execution...
PASS: Test 1: Full legacy record finished with done asserted
PASS: Test 1: Output count matches NUM_SAMPLES (2047)
PASS: Test 1: Sequential sample indexing without gaps
PASS: Test 1: m_last asserted exactly on final sample (idx 2046)
PASS: Test 1: Output valid data contains no X or Z
PASS: Test 1: Status register clean (no err_cfg, overrun, pivot, solver)
PASS: Test 1: Final Block Active Weights Distortionless: |w^H a - 1| = 0.000000e+00 <= 2e-3 (resp = 1.000000 + j0.000000)

--- Running Test 2: Static Scenario (N=1536, 3.0s @ 512 Hz) ---
PASS: Test 2: Static scenario completed with done asserted
PASS: Test 2: Exactly 1536 samples output
PASS: Test 2: Sequential sample indexing without gaps
PASS: Test 2: m_last asserted exactly on sample 1535
PASS: Test 2: Status register clean (status == 0)
PASS: Test 2: Steady-State Active Weights Distortionless: |w^H a - 1| = 0.000000e+00 <= 2e-3 (resp = 1.000000 + j0.000000)

--- Running Test 3: Dynamic Scenario (N=3072, 6.0s @ 512 Hz) ---
PASS: Test 3: Dynamic scenario completed with done asserted
PASS: Test 3: Exactly 3072 samples output
PASS: Test 3: Sequential sample indexing without gaps
PASS: Test 3: m_last asserted exactly on sample 3071
PASS: Test 3: Status clean throughout dynamic transitions
PASS: Test 3: Post-Transition Dynamic Weights Distortionless: |w^H a - 1| = 0.000000e+00 <= 2e-3 (resp = 1.000000 + j0.000000)

==================================================
Master Regression Suite Summary: PASSED 19, FAILED 0
==================================================
REGRESSION STATUS: ALL TESTS PASSED
tb_regression.v:565: $finish called at 4086745000 (1ps)
```

---

### 3.2. Comprehensive Directed Suite (`tb_directed.v`)

```text
==================================================
Starting Comprehensive tb_directed Suite
==================================================

--- Running Test Group A ---
PASS: Group A: Steering vector a_vec captured into u_mem
PASS: Group A: Unused memory words filled with X in testbench
PASS: Group A: Config error (NUM_SAMPLES+1 > MEM_DEPTH) asserts status[7] err_cfg
PASS: Group A: Controller with err_cfg refuses to run (busy stays 0)
PASS: Group A: READ_LATENCY=2 captures steering vector cleanly
PASS: Group A: READ_LATENCY=2 executes and finishes cleanly

--- Running Test Group B ---
PASS: Group B: Full record run completed with done asserted
PASS: Group B: Exactly 200 samples captured
PASS: Group B: Sample 0 Re matches bit-exact quiescent a/4 (64000)
PASS: Group B: Sample 0 Im matches bit-exact quiescent a/4 (51200)
PASS: Group B: Backpressure run completed with done asserted
PASS: Group B: Backpressure run captured exactly 200 samples
PASS: Group B: Stalled run produces bit-identical samples to free run

--- Running Test Group C ---
PASS: Group C: Persistent have_cov flag set after block completion
PASS: Group C: Diagonal covariance entry R00 is strictly positive

--- Running Test Group E ---
PASS: Group E: Causality verified: Block 0 bit-identical despite Block 1 perturbation

--- Running Test Group F ---
PASS: Group F1: Exact multiple (tail=0) finished cleanly
PASS: Group F2: One-sample tail (tail=1) finished cleanly
PASS: Group F4: Record shorter than block (N=30 < K=76) completed as pure tail
PASS: Group F4: Solver was never triggered for pure tail
PASS: Group F5: Override BLOCK_LEN=8 executes and completes 3 blocks
PASS: Group F5: Persistent covariance active with BLOCK_LEN=8

--- Running Test Group G ---
PASS: Group G1: Clean recovery from mid-sample reset
PASS: Group G2: Clean recovery from mid-solve reset
PASS: Group G3: Generous pacing completes with done asserted
PASS: Group G3: Generous pacing has NO overrun (status[0] == 0)
PASS: Group G4: Aggressive pacing latches status[0] err_overrun

==================================================
tb_directed Summary: PASSED 27, FAILED 0
==================================================
TB_DIRECTED STATUS: ALL TESTS PASSED
tb_directed.v:737: $finish called at 694275000 (1ps)
```

---

### 3.3. Block Solver Unit Tests (`tb_block_solver.v`)

```text
PASS: Zero matrix sets info_zero_trace
PASS: Zero matrix w0 equals a0/4 (16'h4000 << 4 = 25'h0040000)
PASS: Zero matrix w0_im == 0
PASS: Zero matrix w1_re == 0
PASS: Zero matrix w1_im equals a1_im/4 (-16'h4000 << 4 = -25'h0040000)
PASS: Zero matrix solve_cycles <= 20000
Zero matrix solve cycles: 26
DUT weights for Diagonal matrix: w0=(0.250000,0.000000) w1=(0.000000,-0.250000) w2=(-0.250000,0.000000) w3=(0.000000,0.250000)
REF weights for Diagonal matrix: w0=(0.250000,0.000000) w1=(0.000000,-0.250000) w2=(-0.250000,0.000000) w3=(0.000000,0.250000)
Diagonal matrix solve cycles: 475
PASS: Diagonal matrix solve_cycles <= 20000
Distortionless check for Diagonal matrix: Re=1.000000, Im=0.000000, err=0.000000e+00
PASS: Diagonal matrix distortionless |w^H*a - 1| <= 2e-3
Weight error for Diagonal matrix: max|dw|=0.000000e+00, rel_err=0.000000e+00
PASS: Diagonal matrix weight error max|dw|/max|w_ref| <= 2e-3
Distortionless check for Complex Hermitian matrix: Re=1.000000, Im=0.000000, err=0.000000e+00
PASS: Complex Hermitian matrix distortionless |w^H*a - 1| <= 2e-3
Weight error for Complex Hermitian matrix: max|dw|=8.611653e-05, rel_err=2.974211e-04
PASS: Complex Hermitian matrix weight error max|dw|/max|w_ref| <= 2e-3
Running Test 4 (Scale 1/16)...
DUT weights for Scale (1/16): w0=(0.235674,0.075669) w1=(-0.022495,-0.289484) w2=(-0.256002,0.034491) w3=(0.018683,0.218840)
Distortionless check for Scaled matrix (1/16): Re=1.000000, Im=0.000000, err=0.000000e+00
PASS: Scaled matrix (1/16) distortionless |w^H*a - 1| <= 2e-3
PASS: Scale invariance re weight match
PASS: Scale invariance im weight match
PASS: Scale invariance re weight match
PASS: Scale invariance im weight match
PASS: Scale invariance re weight match
PASS: Scale invariance im weight match
PASS: Scale invariance re weight match
PASS: Scale invariance im weight match
PASS: Near-zero pivot triggers err_pivot flag
PASS: Pivot clamp does not hang, completes cleanly
--------------------------------------------------
tb_block_solver: PASSED 22, FAILED 0
--------------------------------------------------
TB_BLOCK_SOLVER STATUS: ALL TESTS PASSED
```

---

### 3.4. Sample Engine Unit Tests (`tb_sample_engine.v`)

```text
Starting tb_sample_engine...
Sample processing duration: 52 clock cycles
PASS: Cycle count per sample <= 200 cycles
PASS: Pure real beamformer output matches bit-exact model
PASS: Pure imaginary beamformer output matches bit-exact model
PASS: Extreme weights and inputs trigger sat_y
PASS: Saturated y output matches bit-exact model
PASS: All-(-1.0) 76-sample covariance accumulation matches bit-exact integer model
PASS: Random data 76-sample covariance accumulation matches bit-exact integer model
PASS: Diagonal S is non-negative
PASS: Diagonal S is non-negative
PASS: Diagonal S is non-negative
PASS: Diagonal S is non-negative
PASS: Tail sample (accum_en=0) leaves S unchanged
--------------------------------------------------
tb_sample_engine: PASSED 12, FAILED 0
--------------------------------------------------
TB_SAMPLE_ENGINE STATUS: ALL TESTS PASSED
```

---

### 3.5. Serial Divider Unit Tests (`tb_serial_divider.v`)

```text
Starting tb_serial_divider...
Running Test 1 (dbz)...
PASS: Divide by zero sets div_by_zero flag
Running Test 2 (100/10)...
PASS: Exact division 100 / 10 quotient
PASS: Exact division 100 / 10 remainder
PASS: Exact division 100 / 10 quotient_rnd
Running Test 3 (10/3)...
PASS: Round down 10 / 3 quotient == 3
PASS: Round down 10 / 3 remainder == 1
PASS: Round down 10 / 3 quotient_rnd == 3
Running Test 4 (11/3)...
PASS: Round up 11 / 3 quotient == 3
PASS: Round up 11 / 3 remainder == 2
PASS: Round up 11 / 3 quotient_rnd == 4
Running Test 5 (5/2)...
PASS: Half round up 5 / 2 quotient_rnd == 3
Running Test 6 (pivot 2^48 / 1024)...
PASS: Pivot 2^48 / 1024 quotient == 2^38
PASS: Pivot 2^48 / 1024 remainder == 0
Running Test 7 (norm 2^64 / 2^24)...
PASS: Normalizer 2^64 / 2^24 quotient == 2^40
PASS: Normalizer 2^64 / 2^24 remainder == 0
--------------------------------------------------
tb_serial_divider: PASSED 65, FAILED 0
--------------------------------------------------
TB_SERIAL_DIVIDER STATUS: ALL TESTS PASSED
```

---

### 3.6. Input Memory Unit Tests (`tb_input_mem.v`)

```text
PASS: err_cfg_1 is 0 when MEM_DEPTH > NUM_SAMPLES + 1
PASS: err_cfg_3 is 0 when MEM_DEPTH == NUM_SAMPLES + 1
PASS: err_cfg_4 is 1 when NUM_SAMPLES + 1 > MEM_DEPTH
PASS: dut_lat1 a_valid asserted
PASS: dut_lat1 a_vec matches expected header
PASS: dut_lat2 a_valid asserted
PASS: dut_lat2 a_vec matches expected header
PASS: dut_lat1 sample 0 extreme values
PASS: dut_lat1 sample 0 s3 im == -32768
PASS: dut_lat1 sample 0 s3 re == +32767
PASS: dut_lat1 sample 0 s0 im == -32768
PASS: dut_lat1 sample 0 s0 re == 0
PASS: dut_lat1 middle sample 49 data match
PASS: dut_lat1 last sample 99 data match
PASS: dut_lat1 valid outputs contain no X
PASS: dut_lat2 sample 0 match after 2 cycles
--------------------------------------------------
tb_input_mem: PASSED 16, FAILED 0
--------------------------------------------------
TB_INPUT_MEM STATUS: ALL TESTS PASSED
```

---

## 4. Root Cause Analysis & Resolution of Group G3 Pacing Issue

### Problem
In `tb_directed.v`, test Group G3 failed with:
`FAIL: Group G3: Generous pacing has NO overrun (status[0] == 0)`

### Investigation & Root Cause
In `block_controller.v`, the pacing overrun detection originally executed globally across all FSM states whenever `busy == 1'b1`:
```verilog
if (PACED) begin
    if (busy) begin
        if (sample_en) begin
            if (pending_pacing) status[0] <= 1'b1;
            else pending_pacing <= 1'b1;
        end
    end
end
```
When `sample_en` was asserted while the controller was in `ST_SAMPLE_WAIT`, the FSM transitioned to `ST_SAMPLE_READ` and consumed the pulse. However, during the BRAM latency cycle in `ST_SAMPLE_READ`, the same external `sample_en` pulse (or registered signal from the testbench) was evaluated by the global check. Because `ST_SAMPLE_READ` was not clearing `pending_pacing`, it was set to `1'b1` right as the sample started running. When the subsequent sample pulse arrived in `ST_SAMPLE_RUN`, `pending_pacing` was already asserted, causing a false `err_overrun` (`status[0] = 1`).

### Solution
Overrun detection in paced mode is defined as dropping a sample request because both the active execution pipeline and the 1-deep pending request buffer are full. Therefore, overrun checks must occur specifically while the sample engine is processing (`state == ST_SAMPLE_RUN`). In `ST_SAMPLE_WAIT`, `sample_en` is consumed immediately and transitions into `ST_SAMPLE_READ`. In `ST_SAMPLE_READ`, BRAM latency occurs and double-counting is prevented:

```verilog
if (PACED) begin
    if (busy && state == ST_SAMPLE_RUN) begin
        if (sample_en) begin
            if (pending_pacing) begin
                status[0] <= 1'b1; // err_overrun: buffer full and third pulse arrives
            end else begin
                pending_pacing <= 1'b1;
            end
        end
    end
end
```

### Verification Evidence
1. **Generous Pacing Test (G3)**: Pulse `sample_en` every 82 cycles (> 52 cycles engine duration). Result: 20 samples processed cleanly, `status[0] == 0` (**PASS**).
2. **Aggressive Pacing Test (G4)**: Pulse `sample_en` every 5 cycles (< 52 cycles engine duration). Result: Overrun immediately detected, `status[0] == 1` (**PASS**).
3. **Directed Suite (`tb_directed.v`)**: All 27 tests passed with zero failures (**PASS**).
