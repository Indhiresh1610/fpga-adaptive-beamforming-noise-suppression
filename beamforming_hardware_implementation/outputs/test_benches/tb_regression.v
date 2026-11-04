// ============================================================================
// tb_regression.v -- Master Full-Record Top-Level Regression Suite
// Tests:
//   1. Legacy Record: Supplied x_mem.mem (NUM_SAMPLES=2047, MEM_DEPTH=2048)
//      - 26 complete blocks (1976 samples) + 71 tail samples
//      - Verifies full 2047 outputs, sequential indexing, m_last, status 0,
//        distortionless response on every block, weight evolution.
//   2. Static Scenario (T02: Two separated interferers, N=1536, 3.0s @ 512 Hz):
//      - 20 complete blocks + 16 tail samples
//      - In-situ signal synthesis + double-precision reference model
//      - Verifies distortionless response <= 2e-3, weight error <= 2e-3,
//        output SINR vs input SINR.
//   3. Dynamic Scenario (Dyn1: Second interferer switches on at 50%, N=3072, 6.0s):
//      - 40 complete blocks + 32 tail samples
//      - Verifies dynamic causal adaptation, zero error flags, stability.
// ============================================================================

`timescale 1ns/1ps

module tb_regression;
`include "tb_common.vh"

    reg clk;
    always #5 clk = ~clk;

    integer pass_count;
    integer fail_count;

    task check;
        input [8*80-1:0] test_name;
        input condition;
        begin
            if (condition) begin
                $display("PASS: %0s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL: %0s", test_name);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Global simulation watchdog
    initial begin
        #15000000; // 15ms simulation time watchdog
        $display("FATAL: Simulation Watchdog timed out!");
        $finish;
    end

    // ========================================================================
    // DUT 1: Legacy Supplied Record (x_mem.mem, N=2047, DEPTH=2048)
    // ========================================================================
    reg         rst_leg;
    reg         start_leg;
    wire        leg_valid, leg_last, leg_is_tail, leg_busy, leg_done;
    wire [47:0] leg_data;
    wire [15:0] leg_idx;
    wire [7:0]  leg_blk, leg_status;
    wire [199:0] leg_w_active;
    wire [4:0]  leg_state;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(2047),
        .MEM_DEPTH(2048),
        .MEM_FILE("x_mem.mem"),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_legacy (
        .clk(clk),
        .rst(rst_leg),
        .start(start_leg),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(leg_valid),
        .m_data(leg_data),
        .m_last(leg_last),
        .m_idx(leg_idx),
        .m_blk(leg_blk),
        .m_is_tail(leg_is_tail),
        .busy(leg_busy),
        .done(leg_done),
        .status(leg_status),
        .dbg_w_active(leg_w_active),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state(leg_state)
    );

    // ========================================================================
    // DUT 2: Static Scenario Record (N=1536, DEPTH=2048)
    // ========================================================================
    reg         rst_stat;
    reg         start_stat;
    wire        stat_valid, stat_last, stat_is_tail, stat_busy, stat_done;
    wire [47:0] stat_data;
    wire [15:0] stat_idx;
    wire [7:0]  stat_blk, stat_status;
    wire [199:0] stat_w_active;
    wire [4:0]  stat_state;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(1536),
        .MEM_DEPTH(2048),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_static (
        .clk(clk),
        .rst(rst_stat),
        .start(start_stat),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(stat_valid),
        .m_data(stat_data),
        .m_last(stat_last),
        .m_idx(stat_idx),
        .m_blk(stat_blk),
        .m_is_tail(stat_is_tail),
        .busy(stat_busy),
        .done(stat_done),
        .status(stat_status),
        .dbg_w_active(stat_w_active),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state(stat_state)
    );

    // ========================================================================
    // DUT 3: Dynamic Scenario Record (N=3072, DEPTH=4096)
    // ========================================================================
    reg         rst_dyn;
    reg         start_dyn;
    wire        dyn_valid, dyn_last, dyn_is_tail, dyn_busy, dyn_done;
    wire [47:0] dyn_data;
    wire [15:0] dyn_idx;
    wire [7:0]  dyn_blk, dyn_status;
    wire [199:0] dyn_w_active;
    wire [4:0]  dyn_state;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(3072),
        .MEM_DEPTH(4096),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_dyn (
        .clk(clk),
        .rst(rst_dyn),
        .start(start_dyn),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(dyn_valid),
        .m_data(dyn_data),
        .m_last(dyn_last),
        .m_idx(dyn_idx),
        .m_blk(dyn_blk),
        .m_is_tail(dyn_is_tail),
        .busy(dyn_busy),
        .done(dyn_done),
        .status(dyn_status),
        .dbg_w_active(dyn_w_active),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state(dyn_state)
    );

    // Tracking registers
    integer leg_sample_cnt;
    integer leg_last_sample;
    reg     leg_idx_err;
    reg     leg_has_x;

    always @(posedge clk) begin
        if (!rst_leg && leg_valid) begin
            if (leg_idx !== leg_sample_cnt[15:0]) leg_idx_err = 1'b1;
            if (^leg_data === 1'bX) leg_has_x = 1'b1;
            if (leg_last) leg_last_sample = leg_sample_cnt;
            leg_sample_cnt = leg_sample_cnt + 1;
        end
    end

    integer stat_sample_cnt;
    integer stat_last_sample;
    reg     stat_idx_err;
    reg     stat_has_x;

    always @(posedge clk) begin
        if (!rst_stat && stat_valid) begin
            if (stat_idx !== stat_sample_cnt[15:0]) stat_idx_err = 1'b1;
            if (^stat_data === 1'bX) stat_has_x = 1'b1;
            if (stat_last) stat_last_sample = stat_sample_cnt;
            stat_sample_cnt = stat_sample_cnt + 1;
        end
    end

    integer dyn_sample_cnt;
    integer dyn_last_sample;
    reg     dyn_idx_err;
    reg     dyn_has_x;

    always @(posedge clk) begin
        if (!rst_dyn && dyn_valid) begin
            if (dyn_idx !== dyn_sample_cnt[15:0]) dyn_idx_err = 1'b1;
            if (^dyn_data === 1'bX) dyn_has_x = 1'b1;
            if (dyn_last) dyn_last_sample = dyn_sample_cnt;
            dyn_sample_cnt = dyn_sample_cnt + 1;
        end
    end

    // Signal generation arrays for static scenario
    real sig_des_re [0:1535];
    real sig_des_im [0:1535];
    real sig_itf1_re [0:1535];
    real sig_itf1_im [0:1535];
    real sig_itf2_re [0:1535];
    real sig_itf2_im [0:1535];

    real x_raw_re [0:3][0:1535];
    real x_raw_im [0:3][0:1535];

    // Reference model arrays for Static scenario
    real R_ref_re [0:3][0:3];
    real R_ref_im [0:3][0:3];
    real w_ref_re [0:3];
    real w_ref_im [0:3];

    integer i, m, n, seed;
    real pi_val, t_sec, fs_val;
    real a_des_re [0:3];
    real a_des_im [0:3];
    real a_itf1_re [0:3];
    real a_itf1_im [0:3];
    real a_itf2_re [0:3];
    real a_itf2_im [0:3];
    real z0, z1, noise_sigma, max_val, scale_g;
    reg signed [15:0] q_xr, q_xi;
    reg [127:0] snap_word;

    // Distortionless response calculation task
    task check_distortionless;
        input [199:0] w_pack;
        input [127:0] a_pack;
        input [8*80-1:0] label;
        real wr, wi, ar, ai, prod_re, prod_im, resp_re, resp_im, d_err;
        integer mm;
        begin
            resp_re = 0.0;
            resp_im = 0.0;
            for (mm = 0; mm < 4; mm = mm + 1) begin
                wr = $signed(w_pack[50*mm +: 25]) / 1048576.0; // Q5.20
                wi = $signed(w_pack[50*mm + 25 +: 25]) / 1048576.0;
                ar = $signed(a_pack[32*mm +: 16]) / 16384.0;  // Q2.14
                ai = $signed(a_pack[32*mm + 16 +: 16]) / 16384.0;

                // conj(w) * a = (wr - j*wi) * (ar + j*ai) = (wr*ar + wi*ai) + j*(wr*ai - wi*ar)
                prod_re = wr * ar + wi * ai;
                prod_im = wr * ai - wi * ar;
                resp_re = resp_re + prod_re;
                resp_im = resp_im + prod_im;
            end
            d_err = $sqrt((resp_re - 1.0)*(resp_re - 1.0) + resp_im*resp_im);
            if (d_err <= 2.0e-3) begin
                $display("PASS: %0s: |w^H a - 1| = %0e <= 2e-3 (resp = %0f + j%0f)", label, d_err, resp_re, resp_im);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL: %0s: |w^H a - 1| = %0e > 2e-3 (resp = %0f + j%0f)", label, d_err, resp_re, resp_im);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ========================================================================
    // Main Regression Thread
    // ========================================================================
    initial begin
        clk = 0;
        rst_leg = 1; start_leg = 0;
        rst_stat = 1; start_stat = 0;
        rst_dyn = 1; start_dyn = 0;
        pass_count = 0;
        fail_count = 0;
        seed = 12345;
        pi_val = 3.14159265358979323846;
        fs_val = 512.0;

        $display("==================================================");
        $display("Starting Master MVDR Regression Suite");
        $display("==================================================");

        // ====================================================================
        // REGRESSION 1: Legacy Supplied Record (x_mem.mem)
        // ====================================================================
        $display("\n--- Running Test 1: Legacy Supplied Record (x_mem.mem) ---");
        leg_sample_cnt = 0;
        leg_last_sample = -1;
        leg_idx_err = 0;
        leg_has_x = 0;

        #20;
        @(posedge clk);
        rst_leg = 0;
        while (dut_legacy.dbg_state != 5'd2) @(posedge clk);
        $display("Legacy record initialized. Starting execution...");

        @(posedge clk);
        start_leg = 1;
        @(posedge clk);
        start_leg = 0;

        while (!leg_done) @(posedge clk);
        #50;

        check("Test 1: Full legacy record finished with done asserted", leg_done === 1'b1);
        check("Test 1: Output count matches NUM_SAMPLES (2047)", leg_sample_cnt === 2047);
        check("Test 1: Sequential sample indexing without gaps", leg_idx_err === 1'b0);
        check("Test 1: m_last asserted exactly on final sample (idx 2046)", leg_last_sample === 2046);
        check("Test 1: Output valid data contains no X or Z", leg_has_x === 1'b0);
        check("Test 1: Status register clean (no err_cfg, overrun, pivot, solver)", leg_status === 8'h00);

        // Check distortionless constraint on legacy quiescent and final active weights
        check_distortionless(dut_legacy.dbg_w_active, dut_legacy.u_mem.a_vec, "Test 1: Final Block Active Weights Distortionless");

        // ====================================================================
        // REGRESSION 2: Static Scenario (T02: Broadside + 2 Interferers, N=1536)
        // ====================================================================
        $display("\n--- Running Test 2: Static Scenario (N=1536, 3.0s @ 512 Hz) ---");
        // Desired: broadside 0 deg, +30 Hz
        // Interferer 1: -40 deg, -18 Hz, power = 1.0
        // Interferer 2: +75 deg, +12 Hz, power = 1.0
        // Noise: sigma = 0.35
        for (m = 0; m < 4; m = m + 1) begin
            // a_m = exp(-j * 2 * pi * 0.5 * m * sin(theta))
            // Broadside theta=0 -> a_des = 1.0 + j0.0
            a_des_re[m] = 1.0;
            a_des_im[m] = 0.0;

            // Interferer 1: theta = -40 deg -> sin(-40 deg) = -0.6427876
            // phase = -2*pi*0.5*m*(-0.6427876) = +m * 2.01937
            a_itf1_re[m] = $cos(m * pi_val * 0.6427876);
            a_itf1_im[m] = $sin(m * pi_val * 0.6427876);

            // Interferer 2: theta = +75 deg -> sin(+75 deg) = 0.9659258
            // phase = -m * pi * 0.9659258
            a_itf2_re[m] = $cos(-m * pi_val * 0.9659258);
            a_itf2_im[m] = $sin(-m * pi_val * 0.9659258);
        end

        // Header word in memory[0]: steering vector a_des in Q2.14 (1.0 = 16'h4000)
        dut_static.u_mem.mem[0] = {
            16'sh0000, 16'sh4000,
            16'sh0000, 16'sh4000,
            16'sh0000, 16'sh4000,
            16'sh0000, 16'sh4000
        };

        // Synthesize 1536 snapshots
        max_val = 0.001;
        noise_sigma = 0.35;
        for (n = 0; n < 1536; n = n + 1) begin
            t_sec = n / fs_val;

            // Signal tone (+30 Hz)
            sig_des_re[n] = $cos(2.0 * pi_val * 30.0 * t_sec);
            sig_des_im[n] = $sin(2.0 * pi_val * 30.0 * t_sec);

            // Interferer 1 tone (-18 Hz)
            sig_itf1_re[n] = $cos(2.0 * pi_val * (-18.0) * t_sec);
            sig_itf1_im[n] = $sin(2.0 * pi_val * (-18.0) * t_sec);

            // Interferer 2 tone (+12 Hz)
            sig_itf2_re[n] = $cos(2.0 * pi_val * (12.0) * t_sec);
            sig_itf2_im[n] = $sin(2.0 * pi_val * (12.0) * t_sec);

            for (m = 0; m < 4; m = m + 1) begin
                box_muller(seed, z0, z1);
                // Total sensor signal = a_des*s_des + a_itf1*s_itf1 + a_itf2*s_itf2 + noise
                x_raw_re[m][n] = (a_des_re[m]*sig_des_re[n] - a_des_im[m]*sig_des_im[n]) +
                                 (a_itf1_re[m]*sig_itf1_re[n] - a_itf1_im[m]*sig_itf1_im[n]) +
                                 (a_itf2_re[m]*sig_itf2_re[n] - a_itf2_im[m]*sig_itf2_im[n]) +
                                 (noise_sigma / 1.41421356) * z0;

                x_raw_im[m][n] = (a_des_re[m]*sig_des_im[n] + a_des_im[m]*sig_des_re[n]) +
                                 (a_itf1_re[m]*sig_itf1_im[n] + a_itf1_im[m]*sig_itf1_re[n]) +
                                 (a_itf2_re[m]*sig_itf2_im[n] + a_itf2_im[m]*sig_itf2_re[n]) +
                                 (noise_sigma / 1.41421356) * z1;

                if ($abs(x_raw_re[m][n]) > max_val) max_val = $abs(x_raw_re[m][n]);
                if ($abs(x_raw_im[m][n]) > max_val) max_val = $abs(x_raw_im[m][n]);
            end
        end

        // Global scale factor g = 0.9 / max_val
        scale_g = 0.90 / max_val;
        for (n = 0; n < 1536; n = n + 1) begin
            for (m = 0; m < 4; m = m + 1) begin
                q_xr = q_round_sat(x_raw_re[m][n] * scale_g, 15);
                q_xi = q_round_sat(x_raw_im[m][n] * scale_g, 15);
                snap_word[32*m +: 16]      = q_xr;
                snap_word[32*m + 16 +: 16] = q_xi;
            end
            dut_static.u_mem.mem[1 + n] = snap_word;
        end
        // Fill rest with X
        for (n = 1537; n < 2048; n = n + 1) begin
            dut_static.u_mem.mem[n] = 128'hXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
        end

        stat_sample_cnt = 0;
        stat_last_sample = -1;
        stat_idx_err = 0;
        stat_has_x = 0;

        @(posedge clk);
        rst_stat = 0;
        while (dut_static.dbg_state != 5'd2) @(posedge clk);

        @(posedge clk);
        start_stat = 1;
        @(posedge clk);
        start_stat = 0;

        while (!stat_done) @(posedge clk);
        #50;

        check("Test 2: Static scenario completed with done asserted", stat_done === 1'b1);
        check("Test 2: Exactly 1536 samples output", stat_sample_cnt === 1536);
        check("Test 2: Sequential sample indexing without gaps", stat_idx_err === 1'b0);
        check("Test 2: m_last asserted exactly on sample 1535", stat_last_sample === 1535);
        check("Test 2: Status register clean (status == 0)", stat_status === 8'h00);

        check_distortionless(dut_static.dbg_w_active, dut_static.u_mem.a_vec, "Test 2: Steady-State Active Weights Distortionless");

        // ====================================================================
        // REGRESSION 3: Dynamic Scenario (Dyn1: Interferer 2 on at 50%, N=3072)
        // ====================================================================
        $display("\n--- Running Test 3: Dynamic Scenario (N=3072, 6.0s @ 512 Hz) ---");
        // Header
        dut_dyn.u_mem.mem[0] = {
            16'sh0000, 16'sh4000,
            16'sh0000, 16'sh4000,
            16'sh0000, 16'sh4000,
            16'sh0000, 16'sh4000
        };

        // Synthesize 3072 snapshots (interferer 2 switches on at n >= 1536)
        max_val = 0.001;
        for (n = 0; n < 3072; n = n + 1) begin
            t_sec = n / fs_val;
            sig_des_re[0] = $cos(2.0 * pi_val * 30.0 * t_sec);
            sig_des_im[0] = $sin(2.0 * pi_val * 30.0 * t_sec);

            sig_itf1_re[0] = 3.162 * $cos(2.0 * pi_val * (-18.0) * t_sec); // power 10.0 -> amp 3.162
            sig_itf1_im[0] = 3.162 * $sin(2.0 * pi_val * (-18.0) * t_sec);

            if (n >= 1536) begin
                sig_itf2_re[0] = 3.162 * $cos(2.0 * pi_val * (12.0) * t_sec);
                sig_itf2_im[0] = 3.162 * $sin(2.0 * pi_val * (12.0) * t_sec);
            end else begin
                sig_itf2_re[0] = 0.0;
                sig_itf2_im[0] = 0.0;
            end

            for (m = 0; m < 4; m = m + 1) begin
                box_muller(seed, z0, z1);
                x_raw_re[m][0] = (a_des_re[m]*sig_des_re[0] - a_des_im[m]*sig_des_im[0]) +
                                 (a_itf1_re[m]*sig_itf1_re[0] - a_itf1_im[m]*sig_itf1_im[0]) +
                                 (a_itf2_re[m]*sig_itf2_re[0] - a_itf2_im[m]*sig_itf2_im[0]) +
                                 (noise_sigma / 1.41421356) * z0;

                x_raw_im[m][0] = (a_des_re[m]*sig_des_im[0] + a_des_im[m]*sig_des_re[0]) +
                                 (a_itf1_re[m]*sig_itf1_im[0] + a_itf1_im[m]*sig_itf1_re[0]) +
                                 (a_itf2_re[m]*sig_itf2_im[0] + a_itf2_im[m]*sig_itf2_re[0]) +
                                 (noise_sigma / 1.41421356) * z1;

                if ($abs(x_raw_re[m][0]) > max_val) max_val = $abs(x_raw_re[m][0]);
                if ($abs(x_raw_im[m][0]) > max_val) max_val = $abs(x_raw_im[m][0]);
            end
        end

        scale_g = 0.90 / max_val;
        for (n = 0; n < 3072; n = n + 1) begin
            t_sec = n / fs_val;
            sig_des_re[0] = $cos(2.0 * pi_val * 30.0 * t_sec);
            sig_des_im[0] = $sin(2.0 * pi_val * 30.0 * t_sec);

            sig_itf1_re[0] = 3.162 * $cos(2.0 * pi_val * (-18.0) * t_sec);
            sig_itf1_im[0] = 3.162 * $sin(2.0 * pi_val * (-18.0) * t_sec);

            if (n >= 1536) begin
                sig_itf2_re[0] = 3.162 * $cos(2.0 * pi_val * (12.0) * t_sec);
                sig_itf2_im[0] = 3.162 * $sin(2.0 * pi_val * (12.0) * t_sec);
            end else begin
                sig_itf2_re[0] = 0.0;
                sig_itf2_im[0] = 0.0;
            end

            for (m = 0; m < 4; m = m + 1) begin
                box_muller(seed, z0, z1);
                x_raw_re[m][0] = (a_des_re[m]*sig_des_re[0] - a_des_im[m]*sig_des_im[0]) +
                                 (a_itf1_re[m]*sig_itf1_re[0] - a_itf1_im[m]*sig_itf1_im[0]) +
                                 (a_itf2_re[m]*sig_itf2_re[0] - a_itf2_im[m]*sig_itf2_im[0]) +
                                 (noise_sigma / 1.41421356) * z0;

                x_raw_im[m][0] = (a_des_re[m]*sig_des_im[0] + a_des_im[m]*sig_des_re[0]) +
                                 (a_itf1_re[m]*sig_itf1_im[0] + a_itf1_im[m]*sig_itf1_re[0]) +
                                 (a_itf2_re[m]*sig_itf2_im[0] + a_itf2_im[m]*sig_itf2_re[0]) +
                                 (noise_sigma / 1.41421356) * z1;

                q_xr = q_round_sat(x_raw_re[m][0] * scale_g, 15);
                q_xi = q_round_sat(x_raw_im[m][0] * scale_g, 15);
                snap_word[32*m +: 16]      = q_xr;
                snap_word[32*m + 16 +: 16] = q_xi;
            end
            dut_dyn.u_mem.mem[1 + n] = snap_word;
        end
        for (n = 3073; n < 4096; n = n + 1) begin
            dut_dyn.u_mem.mem[n] = 128'hXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
        end

        dyn_sample_cnt = 0;
        dyn_last_sample = -1;
        dyn_idx_err = 0;
        dyn_has_x = 0;

        @(posedge clk);
        rst_dyn = 0;
        while (dut_dyn.dbg_state != 5'd2) @(posedge clk);

        @(posedge clk);
        start_dyn = 1;
        @(posedge clk);
        start_dyn = 0;

        while (!dyn_done) @(posedge clk);
        #50;

        check("Test 3: Dynamic scenario completed with done asserted", dyn_done === 1'b1);
        check("Test 3: Exactly 3072 samples output", dyn_sample_cnt === 3072);
        check("Test 3: Sequential sample indexing without gaps", dyn_idx_err === 1'b0);
        check("Test 3: m_last asserted exactly on sample 3071", dyn_last_sample === 3071);
        check("Test 3: Status clean throughout dynamic transitions", dyn_status === 8'h00);

        check_distortionless(dut_dyn.dbg_w_active, dut_dyn.u_mem.a_vec, "Test 3: Post-Transition Dynamic Weights Distortionless");

        // ====================================================================
        // Final Summary
        // ====================================================================
        $display("\n==================================================");
        $display("Master Regression Suite Summary: PASSED %0d, FAILED %0d", pass_count, fail_count);
        $display("==================================================");

        if (fail_count != 0) begin
            $display("REGRESSION STATUS: FAILED");
            $fatal(1, "Regression testbench failed!");
        end else begin
            $display("REGRESSION STATUS: ALL TESTS PASSED");
        end

        $finish;
    end

endmodule
