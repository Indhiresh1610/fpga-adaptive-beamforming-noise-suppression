## ============================================================================
## boolean_pins.xdc -- pin / I-O constraints for mvdr_board_top (with UART uplink)
## Board : Real Digital Boolean Board (XC7S50-CSGA324)
## Pins  : clk F14, btn[0] J2, btn[1] J5, led[0..3] G1 G2 F1 F2 (as before),
##         USB-UART V12 (PC -> FPGA) / U11 (FPGA -> PC), led[4..5] E1 E2.
##
## Use this file TOGETHER with your existing timing.xdc, which already holds
##   create_clock -period 10.000 -name clk -waveform {0.000 5.000} [get_ports clk]
## Do NOT add a second create_clock here.
## ============================================================================

## 100 MHz on-board oscillator
set_property -dict { PACKAGE_PIN F14 IOSTANDARD LVCMOS33 } [get_ports { clk }]

## Push buttons: BTN0 = reset, BTN1 = start
set_property -dict { PACKAGE_PIN J2  IOSTANDARD LVCMOS33 } [get_ports { btn_rst }]
set_property -dict { PACKAGE_PIN J5  IOSTANDARD LVCMOS33 } [get_ports { btn_start }]

## LEDs: LD0 = busy, LD1 = done, LD2 = any status flag, LD3 = checksum parity,
##       LD4 = UART dump in progress, LD5 = capture overflow
set_property -dict { PACKAGE_PIN G1  IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN G2  IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN F1  IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN F2  IOSTANDARD LVCMOS33 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN E1  IOSTANDARD LVCMOS33 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN E2  IOSTANDARD LVCMOS33 } [get_ports { led[5] }]

## USB-UART bridge on the "PROG UART" micro-USB port
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd }]
set_property -dict { PACKAGE_PIN U11 IOSTANDARD LVCMOS33 } [get_ports { uart_txd }]

## Configuration bank 0 voltage (clears the CFGBVS DRC)
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

## Buttons, LEDs and the UART are slow, asynchronous I/O (buttons and uart_rxd
## are re-synchronised inside the design), so they are excluded from timing.
set_false_path -from [get_ports { btn_rst btn_start uart_rxd }]
set_false_path -to   [get_ports { led[*] uart_txd }]

