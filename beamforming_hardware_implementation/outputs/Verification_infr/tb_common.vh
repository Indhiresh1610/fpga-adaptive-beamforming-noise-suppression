// ============================================================================
// tb_common.vh -- Common verification tasks, mathematical reference models,
// Box-Muller Gaussian generator, and signal synthesis utilities.
// Strictly Verilog-2001 compatible (no unpacked array task ports).
// Include this file INSIDE a module.
// ============================================================================

`ifndef TB_COMMON_VH
`define TB_COMMON_VH

// ----------------------------------------------------------------------------
// Quantization Functions
// ----------------------------------------------------------------------------
function signed [15:0] q_round_sat;
    input real val;
    input integer frac_bits;
    real scaled;
    real rounded;
    begin
        scaled = val * (1 << frac_bits);
        if (scaled >= 0.0) rounded = $floor(scaled + 0.5);
        else               rounded = -$floor(-scaled + 0.5);

        if (rounded > 32767.0)       q_round_sat = 16'sh7FFF;
        else if (rounded < -32768.0) q_round_sat = -16'sh8000;
        else                         q_round_sat = rounded;
    end
endfunction

// ----------------------------------------------------------------------------
// Windowed SINR Metric
// ----------------------------------------------------------------------------
function real compute_sinr_db;
    input real pd_sum;
    input real pi_sum;
    real r_pd, r_pi;
    begin
        r_pd = (pd_sum > 1.0e-30) ? pd_sum : 1.0e-30;
        r_pi = (pi_sum > 1.0e-30) ? pi_sum : 1.0e-30;
        compute_sinr_db = 10.0 * ($ln(r_pd / r_pi) / $ln(10.0));
    end
endfunction

// ----------------------------------------------------------------------------
// Box-Muller Standard Normal Generator
// ----------------------------------------------------------------------------
task box_muller;
    inout integer seed;
    output real z0;
    output real z1;
    real u1, u2, r;
    real pi_val;
    begin
        pi_val = 3.14159265358979323846;
        u1 = ($dist_uniform(seed, 1, 2147483646) + 0.5) / 2147483648.0;
        u2 = ($dist_uniform(seed, 0, 2147483647) + 0.5) / 2147483648.0;
        if (u1 <= 1.0e-12) u1 = 1.0e-12;
        r = $sqrt(-2.0 * $ln(u1));
        z0 = r * $cos(2.0 * pi_val * u2);
        z1 = r * $sin(2.0 * pi_val * u2);
    end
endtask

`endif // TB_COMMON_VH
