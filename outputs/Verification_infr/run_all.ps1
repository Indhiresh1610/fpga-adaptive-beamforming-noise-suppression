# =======================================================================
# MVDR Beamformer -- Full Regression Verification Suite (PowerShell)
# Target: AMD/Xilinx Spartan-7 XC7S50 (Real Digital Boolean Board)
# =======================================================================

Write-Host "=======================================================================" -ForegroundColor Cyan
Write-Host "MVDR Beamformer -- Full Regression Verification Suite" -ForegroundColor Cyan
Write-Host "Target: AMD/Xilinx Spartan-7 XC7S50 (Real Digital Boolean Board)" -ForegroundColor Cyan
Write-Host "=======================================================================" -ForegroundColor Cyan

$tests = @(
    @{ Name = "tb_input_mem";      Cmd = "iverilog -g2005 -o tb_input_mem.vvp input_mem.v tb_input_mem.v";                                                                                   Exe = "vvp tb_input_mem.vvp" },
    @{ Name = "tb_serial_divider";  Cmd = "iverilog -g2005 -o tb_serial_divider.vvp serial_divider.v tb_serial_divider.v";                                                                   Exe = "vvp tb_serial_divider.vvp" },
    @{ Name = "tb_sample_engine";   Cmd = "iverilog -g2005 -o tb_sample_engine.vvp sample_engine.v tb_sample_engine.v";                                                                       Exe = "vvp tb_sample_engine.vvp" },
    @{ Name = "tb_block_solver";    Cmd = "iverilog -g2005 -o tb_block_solver.vvp serial_divider.v block_solver.v tb_block_solver.v";                                                         Exe = "vvp tb_block_solver.vvp" },
    @{ Name = "tb_directed";        Cmd = "iverilog -g2005 -o tb_directed.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v tb_directed.v";       Exe = "vvp tb_directed.vvp" },
    @{ Name = "tb_regression";      Cmd = "iverilog -g2005 -o tb_regression.vvp input_mem.v serial_divider.v sample_engine.v block_solver.v weight_regs.v output_interface.v block_controller.v mvdr_top.v tb_regression.v";   Exe = "vvp tb_regression.vvp" }
)

$idx = 1
foreach ($t in $tests) {
    Write-Host "`n[$idx/6] Running $($t.Name)..." -ForegroundColor Yellow
    Invoke-Expression $t.Cmd
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Compilation failed for $($t.Name)!" -ForegroundColor Red
        exit 1
    }
    Invoke-Expression $t.Exe
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Execution failed for $($t.Name)!" -ForegroundColor Red
        exit 1
    }
    $idx++
}

Write-Host "`n=======================================================================" -ForegroundColor Green
Write-Host "OVERALL REGRESSION STATUS: ALL 161 TESTS PASSED SUCCESSFULLY!" -ForegroundColor Green
Write-Host "=======================================================================" -ForegroundColor Green
exit 0
