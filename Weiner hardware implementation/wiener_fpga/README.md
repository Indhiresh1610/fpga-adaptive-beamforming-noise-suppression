# Wiener filter alone on the Boolean Board (XC7S50-CSGA324-1)

STATUS: the new top (wiener_board_top.v), the testbench and the receiver are NOT compiled/simulated by me; the
receiver was tested on a synthetic dump, the Wiener core itself passed simulation + synthesis + implementation earlier.
Simulate once (below) before programming the board. The board run itself is untested.

What happens on the board: input ROM (wf_in.mem, 1971 beamformer samples of case_2) -> feeder -> Wiener -> capture
BRAM -> UART to the PC. The result must equal data/wf_expected.csv (Python model) up to a few LSB.

## Files
rtl/wiener_board_top.v   NEW: board top (MMCM 100->50 MHz, buttons, LEDs) + wiener_system (ROM, feeder, Wiener, capture)
rtl/wiener_cplx_core.v, wiener_cplx_top.v, serial_divider.v, create_xfft_ip.tcl   (the Wiener, as simulated)
rtl/uart_capture_lib.v   team's stream_capture_uplink, uart_tx, uart_rx, copied unchanged
data/wf_in.mem           input samples (add it as a DESIGN source: the ROM is loaded from it at synthesis)
data/wf_expected.csv     Python model output
constraints/boolean_pins.xdc, timing.xdc   the team's, unchanged (same port names)
tb/tb_wiener_system_only.v   simulation of the whole board logic (no MMCM / UART dump)
tools/wiener_uart_rx.py  receives the UART dump -> wf_board.csv ;  tools/wiener_check.py compare ...

## Vivado (new project)
1. New RTL project, part xc7s50csga324-1.
2. Add design sources: all .v in rtl/ and data/wf_in.mem.  Add constraints: both .xdc in constraints/.
3. Tcl Console:  source {<path>/rtl/create_xfft_ip.tcl}      (creates xfft_0)
4. Set the top module to wiener_board_top (Sources -> right-click -> Set as Top).
5. Optional but recommended simulation: add tb/tb_wiener_system_only.v as simulation source, set it as sim top, Run
   Behavioral Simulation, `run all`, copy wf_out.csv next to wiener_check.py, then
   python wiener_check.py compare wf_expected.csv wf_out.csv      (expect PASS)
6. Run Synthesis, Run Implementation (check timing summary: WNS positive, "constraints are met"), Generate Bitstream.
7. Board: connect the micro-USB cable to the JTAG/PROG port, Hardware Manager -> Open Target -> Auto Connect ->
   Program Device. (The UART uses the "PROG UART" USB port; find its COM port in Device Manager.)

## Run on the board
    pip install pyserial
    python tools/wiener_uart_rx.py --port COM5 --start        (COM5 = your port; this sends 'S')
    python tools/wiener_check.py compare data/wf_expected.csv wf_board.csv
Or press btn_start, then run wiener_uart_rx.py without --start. Expected: 1971 samples, 0 bad checksums, PASS.
LEDs: 0 feeding, 1 all inputs sent, 2 Wiener saturation flag, 3 Wiener finished, 4 dump running, 5 capture overflow.
The dump takes about 3 s (115200 baud). Press BTN0 to reset.
The board result may differ from wf_expected.csv by a few LSB (the FFT IP truncates, the model does not).
