# OPTIONAL. Add this file ONLY if timing fails inside the Wiener at 50 MHz (look at the paths from u_wiener/u_core/mop*).
# The Wiener core captures every multiplier product at least 5 clock cycles after the operand registers (mop*)
# change, so these paths may be declared 5-cycle paths.  Do not add it by default: a wrong exception hides real errors.
set_multicycle_path -setup 5 -from [get_cells -hierarchical -filter {NAME =~ *u_wiener/u_core/mop*}]
set_multicycle_path -hold  4 -from [get_cells -hierarchical -filter {NAME =~ *u_wiener/u_core/mop*}]
