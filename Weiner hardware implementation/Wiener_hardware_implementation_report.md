# Wiener post-filter: hardware implementation report

Target: Boolean Board, Spartan-7 XC7S50-CSGA324-1, Vivado 2025.2. Input: output of the team's MVDR beamformer (case_2: 4 sensors, desired signal at -20 deg, three interferers, narrowband 30 Hz tone in white noise, fs = 512 Hz).

## 1. Algorithm (what is implemented)
Causal frequency-domain Wiener filter (STFT, overlap-add):
- Frame 64 complex samples, hop 16 (75 % overlap), Hann window (Q1.15), 64-point FFT / IFFT (Xilinx FFT IP v9.1, fixed point, 18-bit input precision, unscaled, radix-2 burst I/O, natural order, truncation).
- Noise level: one white-noise level learned once from the first full frame (rank-16 start, then two trimmed-mean passes), never updated. Block 0 of the beamformer (quiescent weights) is dropped before the Wiener.
- Per bin: xi = 0.98 * (previous cleaned power / noise) + 0.02 * max(gamma - 1, 0), gain G = xi / (1 + xi); overlap-add with a fixed 2/3 normalisation.
- Data format: Q2.22 complex, 48 bits per sample (24-bit real + 24-bit imaginary), in and out. Latency 48 samples (the first 48 outputs are warm-up).
- Valid for: narrowband signal in white noise, no noise-only lead-in. Not valid for coloured noise or wideband signals.

## 2. Procedure and results
| Step | What was done | Result |
|---|---|---|
| 1. Python / HLS check | Python reference vs C++ (HLS) implementation; algorithm adapted to the beamformer (64-pt complex, hop 16, no lead-in noise estimate, block 0 dropped) | C++ matches Python within 0.37 dB; fixed-point rounding <= 0.001 dB |
| 2. Bit-accurate Python model (`wiener_model.py`) on the team's real case_2 beamformer output (FPGA-measured) | Tone-to-residual ratio, blocks >= 1, first 48 samples skipped | beamformer **14.06 dB** -> Wiener **26.18 dB** (gain +12.12 dB) |
| 3. RTL simulation, Wiener only (testbench reads the same 1971 samples) | Stand-in FFT model: compared with the Python model | exact match (0 LSB difference) |
| 4. RTL simulation, Wiener only, real Xilinx FFT IP | Compared with the Python model | 1971 samples, 1766 differ, max 2560 LSB (about 0.0006 of full scale), model-vs-RTL SNR 50.2 dB (PASS, FFT IP truncates); tone-to-residual 14.06 -> model 26.18 / RTL **26.16 dB** (gain +12.10 dB) |
| 5. Simulation of the board design (input ROM + feeder + Wiener + capture) | Same samples, real FFT IP | same numbers as step 4 |
| 6. Synthesis, Wiener alone (50 MHz clock constraint) | Timing | WNS +7.783 ns, WHS +0.057 ns, 0 failing endpoints |
| | Utilization | LUT 2702 (core 1934, FFT 767), registers 2361, block RAM 3, DSP 18 (core 12, FFT 6) |
| 7. Implementation, Wiener alone | Timing | WNS +6.498 ns, WHS +0.018 ns, pulse width +8.750 ns, 0 failing endpoints, constraints met |
| | Utilization | LUT 2462 (7.6 %), registers 2349 (3.6 %), slices 875 (10.7 %), LUT as memory 582 (6.1 %), block RAM 3 (4 %), DSP 18 (15 %) |
| 8. Implementation, board design `wiener_board_top` (adds MMCM 100 -> 50 MHz, input ROM, feeder, UART capture) | Timing | WNS +6.320 ns, WHS +0.016 ns, pulse width +3.000 ns, 0 failing endpoints, constraints met |
| | Utilization | LUT 2912 (8.9 %), registers 2845 (4.4 %), slices 1037, block RAM 11 tiles (14.7 %), DSP 18 (15 %), MMCM 1, 11 I/O pins |
| 9. Run on the FPGA | Button start, UART dump to the PC (115200 baud, COM10) | 27,615 bytes in 2.6 s, 1971 samples, 0 bad checksums, 0 skipped bytes, end record 1971, Wiener status 0x01 (noise learned, no saturation), no capture overflow |
| 10. Board output vs Python model | `wiener_check.py compare` | 1971 samples, 1766 differ, max 2560 LSB, SNR 50.2 dB, **PASS** (identical to the simulation with the real IP: `Outputs/wf_board.csv` and `Outputs/wf_out.csv` are byte-identical, 0 LSB difference) |
| | `wiener_quality.py` | beamformer 14.06 dB -> model 26.18 dB -> **FPGA 26.16 dB** (gain +12.10 dB, 0.02 dB below the model) |

LED behaviour after a start: LED0 flashes during input, LED1 and LED3 stay on (input sent, Wiener finished), LED4 on during the UART dump, LED2 / LED5 (saturation / overflow) off.

## 3. How the hardware test works
`wf_in.mem` (the beamformer output, 1971 samples, block 0 removed) is stored in a block-RAM ROM. A feeder sends the samples into the Wiener, followed by 80 zeros to flush the latency; the first 48 outputs are skipped and the next 1971 are stored in block RAM and sent to the PC over the USB-UART (framed records with checksums). The PC script `wiener_uart_rx.py` decodes them into `wf_board.csv`, which is compared with the Python model's `wf_expected.csv`. Pass criterion: model-vs-RTL SNR > 50 dB, and the tone-to-residual gain within about 0.1 dB of the model.

## 4. Reproduce
See `wiener_fpga/README.md`. In short: new Vivado project (part xc7s50csga324-1) with `rtl/*.v` and `data/wf_in.mem`, constraints from `constraints/`, run `source rtl/create_xfft_ip.tcl`, set top `wiener_board_top`, synthesise, implement, generate bitstream, program, then
`python tools/wiener_uart_rx.py --port COMx --start` and `python tools/wiener_check.py compare data/wf_expected.csv wf_board.csv`.
Simulation testbenches: `tb/tb_wiener_only.v` (Wiener core alone; reads `wf_in.mem`, writes `wf_out.csv`; with the real FFT IP, or with the simulation-only stand-in `tb/xfft_0_behav_SIMULATION_ONLY.v`, which must never be added to the synthesis sources) and `tb/tb_wiener_system_only.v` (whole board logic).

## 5. Status of the full chain (beamformer + Wiener) and limits
- The Wiener alone is verified in Python, simulation, synthesis, implementation and on the board.
- Tested on one input case (case_2). The Wiener gain depends on the signal occupying a small part of the band (about +12 dB for a 30 Hz tone at fs = 512 Hz).
- The FFT IP truncates, so the board output differs from the exact model by up to 2560 LSB (about 0.0006); this costs 0.02 dB of filtering gain.
- Files taken unchanged from the beamforming team: `rtl/serial_divider.v`, `rtl/uart_capture_lib.v`, `constraints/boolean_pins.xdc`, `constraints/timing.xdc`.

## Results folder
`wiener_fpga/Outputs/` holds the evidence: screenshots of the simulation checks, synthesis and implementation utilization/timing, the implemented design and the board checks, plus `wf_out.csv` (simulation), `wf_board.csv` (board) and `wf_expected.csv` (model). A PDF version of this report with the folder guide is `Wiener_hardware_implementation_report.pdf`.
