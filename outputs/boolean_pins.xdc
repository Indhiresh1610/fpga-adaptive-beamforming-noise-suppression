## ============================================================================
## boolean_pins.xdc -- FINAL pin / I-O constraints for mvdr_board_top
## Board : Real Digital Boolean Board (XC7S50-CSGA324)
## Pins  : verified line-by-line against Real Digital's official Boolean
##         master XDC (clk F14, btn[0] J2, btn[1] J5, led[0..3] G1 G2 F1 F2).
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

## LEDs: LD0 = busy, LD1 = done, LD2 = any status flag, LD3 = checksum bit
set_property -dict { PACKAGE_PIN G1  IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN G2  IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN F1  IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN F2  IOSTANDARD LVCMOS33 } [get_ports { led[3] }]

## Configuration bank 0 voltage (clears the CFGBVS DRC)
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

## Buttons and LEDs are slow, human-facing, asynchronous I/O (the buttons are
## re-synchronised inside the design), so they are excluded from timing.
set_false_path -from [get_ports { btn_rst btn_start }]
set_false_path -to   [get_ports { led[*] }]
