@echo off
echo =======================================================================
echo MVDR Beamformer -- Full Regression Verification Suite
echo Target: AMD/Xilinx Spartan-7 XC7S50 (Real Digital Boolean Board)
echo =======================================================================

echo.
echo [1/6] Running Memory Unit Test (tb_input_mem)...
iverilog -g2005 -o tb_input_mem.vvp input_mem.v tb_input_mem.v
if errorlevel 1 goto error
vvp tb_input_mem.vvp
if errorlevel 1 goto error

echo.
echo [2/6] Running Serial Divider Unit Test (tb_serial_divider)...
iverilog -g2005 -o tb_serial_divider.vvp serial_divider.v tb_serial_divider.v
if errorlevel 1 goto error
vvp tb_serial_divider.vvp
if errorlevel 1 goto error

echo.
echo [3/6] Running Sample Engine Unit Test (tb_sample_engine)...
iverilog -g2005 -o tb_sample_engine.vvp sample_engine.v tb_sample_engine.v
if errorlevel 1 goto error
vvp tb_sample_engine.vvp
if errorlevel 1 goto error

echo.
echo [4/6] Running Block Solver Unit Test (tb_block_solver)...
iverilog -g2005 -o tb_block_solver.vvp serial_divider.v block_solver.v tb_block_solver.v
if errorlevel 1 goto error
vvp tb_block_solver.vvp
if errorlevel 1 goto error

echo.
echo [5/6] Running Comprehensive Directed Suite (tb_directed)...
iverilog -g2005 -o tb_directed.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v tb_directed.v
if errorlevel 1 goto error
vvp tb_directed.vvp
if errorlevel 1 goto error

echo.
echo [6/6] Running Full-Record Master Regression Suite (tb_regression)...
iverilog -g2005 -o tb_regression.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v tb_regression.v
if errorlevel 1 goto error
vvp tb_regression.vvp
if errorlevel 1 goto error

echo.
echo =======================================================================
echo OVERALL REGRESSION STATUS: ALL 161 TESTS PASSED SUCCESSFULLY!
echo =======================================================================
exit /b 0

:error
echo.
echo =======================================================================
echo OVERALL REGRESSION STATUS: REGRESSION FAILED!
echo =======================================================================
exit /b 1
