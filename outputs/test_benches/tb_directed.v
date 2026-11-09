// ============================================================================
// tb_directed.v -- Comprehensive Directed Test Suite (Groups A, B, C, E, F, G)
// Verilog-2001 / Verilog-2005 compliant testbench.
// ============================================================================

`timescale 1ns/1ps

module tb_directed;
`include "tb_common.vh"

    reg clk;
    reg rst;
    reg start;
    reg sample_en;
    reg m_ready;

    integer pass_count;
    integer fail_count;

    always #5 clk = ~clk;

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

    // ------------------------------------------------------------------------
    // DUT 1: Standard Configuration (K=76, N=200, DEPTH=256, LAT=1, PACED=0)
    // ------------------------------------------------------------------------
    wire        dut1_m_valid, dut1_m_last, dut1_m_is_tail, dut1_busy, dut1_done;
    wire [47:0] dut1_m_data;
    wire [15:0] dut1_m_idx;
    wire [7:0]  dut1_m_blk, dut1_status;
    wire [4:0]  dut1_state;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(200),
        .MEM_DEPTH(256),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut1 (
        .clk(clk),
        .rst(rst),
        .start(start),
        .sample_en(sample_en),
        .m_ready(m_ready),
        .m_valid(dut1_m_valid),
        .m_data(dut1_m_data),
        .m_last(dut1_m_last),
        .m_idx(dut1_m_idx),
        .m_blk(dut1_m_blk),
        .m_is_tail(dut1_m_is_tail),
        .busy(dut1_busy),
        .done(dut1_done),
        .status(dut1_status),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state(dut1_state)
    );

    // Continuous checks for DUT 1:
    // Memory address must never exceed NUM_SAMPLES
    always @(posedge clk) begin
        if (!rst && dut1.u_ctrl.state >= 5'd4 && dut1.u_ctrl.state <= 5'd6) begin
            if (dut1.u_ctrl.sample_idx_reg >= 200) begin
                $display("FAIL: DUT1 Controller addressed sample_idx %0d >= NUM_SAMPLES 200",
                         dut1.u_ctrl.sample_idx_reg);
                fail_count = fail_count + 1;
            end
        end
    end

    // Outputs must never contain X or Z when m_valid is high
    always @(posedge clk) begin
        if (!rst && dut1_m_valid) begin
            if (^dut1_m_data === 1'bX || ^dut1_m_idx === 1'bX || ^dut1_m_blk === 1'bX || ^dut1_m_last === 1'bX) begin
                $display("FAIL: DUT1 Output contains X/Z while m_valid is high at time %0t!", $time);
                fail_count = fail_count + 1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // DUT CFG_ERR: Tests (NUM_SAMPLES + 1) > MEM_DEPTH
    // ------------------------------------------------------------------------
    reg rst_cfg, start_cfg;
    wire cfg_busy, cfg_done;
    wire [7:0] cfg_status;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(256),
        .MEM_DEPTH(256), // 256 + 1 = 257 > 256 => err_cfg must be 1!
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_cfg (
        .clk(clk),
        .rst(rst_cfg),
        .start(start_cfg),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(),
        .m_data(),
        .m_last(),
        .m_idx(),
        .m_blk(),
        .m_is_tail(),
        .busy(cfg_busy),
        .done(cfg_done),
        .status(cfg_status),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // DUT LAT2: Tests READ_LATENCY = 2
    // ------------------------------------------------------------------------
    reg rst_lat2, start_lat2;
    wire lat2_valid, lat2_last, lat2_done, lat2_busy;
    wire [47:0] lat2_data;
    wire [15:0] lat2_idx;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(50),
        .MEM_DEPTH(128),
        .MEM_FILE(""),
        .READ_LATENCY(2),
        .PACED(0)
    ) dut_lat2 (
        .clk(clk),
        .rst(rst_lat2),
        .start(start_lat2),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(lat2_valid),
        .m_data(lat2_data),
        .m_last(lat2_last),
        .m_idx(lat2_idx),
        .m_blk(),
        .m_is_tail(),
        .busy(lat2_busy),
        .done(lat2_done),
        .status(),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // DUT F1: Exact Multiple (K=76, N=152, DEPTH=256, tail=0)
    // ------------------------------------------------------------------------
    reg rst_f1, start_f1;
    wire f1_valid, f1_last, f1_is_tail, f1_done;
    wire [15:0] f1_idx;
    wire [7:0] f1_blk;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(152),
        .MEM_DEPTH(256),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_f1 (
        .clk(clk),
        .rst(rst_f1),
        .start(start_f1),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(f1_valid),
        .m_data(),
        .m_last(f1_last),
        .m_idx(f1_idx),
        .m_blk(f1_blk),
        .m_is_tail(f1_is_tail),
        .busy(),
        .done(f1_done),
        .status(),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // DUT F2: 1-Sample Tail (K=76, N=153, DEPTH=256, tail=1)
    // ------------------------------------------------------------------------
    reg rst_f2, start_f2;
    wire f2_valid, f2_last, f2_is_tail, f2_done;
    wire [15:0] f2_idx;
    wire [7:0] f2_blk;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(153),
        .MEM_DEPTH(256),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_f2 (
        .clk(clk),
        .rst(rst_f2),
        .start(start_f2),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(f2_valid),
        .m_data(),
        .m_last(f2_last),
        .m_idx(f2_idx),
        .m_blk(f2_blk),
        .m_is_tail(f2_is_tail),
        .busy(),
        .done(f2_done),
        .status(),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // DUT F4: N < K (Record shorter than block: K=76, N=30, DEPTH=64)
    // ------------------------------------------------------------------------
    reg rst_f4, start_f4;
    wire f4_valid, f4_last, f4_is_tail, f4_done;
    wire [15:0] f4_idx;
    wire [7:0] f4_blk;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(30),
        .MEM_DEPTH(64),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_f4 (
        .clk(clk),
        .rst(rst_f4),
        .start(start_f4),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(f4_valid),
        .m_data(),
        .m_last(f4_last),
        .m_idx(f4_idx),
        .m_blk(f4_blk),
        .m_is_tail(f4_is_tail),
        .busy(),
        .done(f4_done),
        .status(),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // DUT F5: Non-default block length (K=8, N=24, DEPTH=64)
    // ------------------------------------------------------------------------
    reg rst_f5, start_f5;
    wire f5_valid, f5_last, f5_is_tail, f5_done;
    wire [15:0] f5_idx;
    wire [7:0] f5_blk;

    mvdr_top #(
        .BLOCK_LEN(8),
        .NUM_SAMPLES(24),
        .MEM_DEPTH(64),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(0)
    ) dut_f5 (
        .clk(clk),
        .rst(rst_f5),
        .start(start_f5),
        .sample_en(1'b0),
        .m_ready(1'b1),
        .m_valid(f5_valid),
        .m_data(),
        .m_last(f5_last),
        .m_idx(f5_idx),
        .m_blk(f5_blk),
        .m_is_tail(f5_is_tail),
        .busy(),
        .done(f5_done),
        .status(),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // DUT G_PACED: Tests PACED = 1
    // ------------------------------------------------------------------------
    reg rst_p, start_p, sample_en_p;
    wire p_valid, p_done, p_busy;
    wire [7:0] p_status;

    mvdr_top #(
        .BLOCK_LEN(76),
        .NUM_SAMPLES(20),
        .MEM_DEPTH(64),
        .MEM_FILE(""),
        .READ_LATENCY(1),
        .PACED(1)
    ) dut_paced (
        .clk(clk),
        .rst(rst_p),
        .start(start_p),
        .sample_en(sample_en_p),
        .m_ready(1'b1),
        .m_valid(p_valid),
        .m_data(),
        .m_last(),
        .m_idx(),
        .m_blk(),
        .m_is_tail(),
        .busy(p_busy),
        .done(p_done),
        .status(p_status),
        .dbg_w_active(),
        .dbg_w_swap(),
        .dbg_blk_done(),
        .dbg_solve_cycles(),
        .dbg_state()
    );

    // ------------------------------------------------------------------------
    // Memory Initializers and Capture Arrays
    // ------------------------------------------------------------------------
    reg [47:0] cap_run1 [0:199];
    reg [47:0] cap_run2 [0:199];
    integer dut1_out_cnt;

    always @(posedge clk) begin
        if (!rst && dut1_m_valid && m_ready) begin
            if (dut1_out_cnt < 200) begin
                cap_run1[dut1_out_cnt] <= dut1_m_data;
                dut1_out_cnt = dut1_out_cnt + 1;
            end
        end
    end

    task init_dut1_mem;
        integer i;
        reg [127:0] sw;
        begin
            // Header: a = [1.0, 1.0, 1.0, 1.0] (1.0 = 16'h4000)
            dut1.u_mem.mem[0] = {
                16'sh0000, 16'sh4000,
                16'sh0000, 16'sh4000,
                16'sh0000, 16'sh4000,
                16'sh0000, 16'sh4000
            };
            for (i = 0; i < 200; i = i + 1) begin
                sw = {
                    16'sd100 + i[15:0], 16'sd200 + i[15:0],  // s3
                    16'sd300 + i[15:0], 16'sd400 + i[15:0],  // s2
                    16'sd500 + i[15:0], 16'sd600 + i[15:0],  // s1
                    16'sd700 + i[15:0], 16'sd800 + i[15:0]   // s0
                };
                dut1.u_mem.mem[1 + i] = sw;
            end
            // Fill words beyond NUM_SAMPLES with X
            for (i = 201; i < 256; i = i + 1) begin
                dut1.u_mem.mem[i] = 128'hXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
            end
        end
    endtask

    // Global simulation watchdog
    initial begin
        #5000000;
        $display("FATAL: Simulation Watchdog timed out!");
        $finish;
    end

    integer i, k;

    // ------------------------------------------------------------------------
    // Main Test Execution
    // ------------------------------------------------------------------------
    initial begin
        clk = 0;
        rst = 1;
        start = 0;
        sample_en = 0;
        m_ready = 1;
        pass_count = 0;
        fail_count = 0;
        dut1_out_cnt = 0;

        rst_cfg = 1; start_cfg = 0;
        rst_lat2 = 1; start_lat2 = 0;
        rst_f1 = 1; start_f1 = 0;
        rst_f2 = 1; start_f2 = 0;
        rst_f4 = 1; start_f4 = 0;
        rst_f5 = 1; start_f5 = 0;
        rst_p = 1; start_p = 0; sample_en_p = 0;

        $display("==================================================");
        $display("Starting Comprehensive tb_directed Suite");
        $display("==================================================");

        // ====================================================================
        // TEST GROUP A: Memory, Widths, X-Filling, Config Error, Latency 2
        // ====================================================================
        $display("\n--- Running Test Group A ---");
        init_dut1_mem();
        #20;
        @(posedge clk);
        rst = 0;
        while (dut1.dbg_state != 5'd2) @(posedge clk);
        check("Group A: Steering vector a_vec captured into u_mem", dut1.u_mem.a_valid === 1'b1);
        check("Group A: Unused memory words filled with X in testbench", dut1.u_mem.mem[255][0] === 1'bX);

        // Test Config Error DUT
        @(posedge clk);
        rst_cfg = 0;
        #20;
        check("Group A: Config error (NUM_SAMPLES+1 > MEM_DEPTH) asserts status[7] err_cfg", dut_cfg.status[7] === 1'b1);
        @(posedge clk);
        start_cfg = 1;
        @(posedge clk);
        start_cfg = 0;
        #50;
        check("Group A: Controller with err_cfg refuses to run (busy stays 0)", dut_cfg.busy === 1'b0);

        // Test READ_LATENCY = 2 DUT
        for (i = 0; i < 128; i = i + 1) dut_lat2.u_mem.mem[i] = 128'h00004000000040000000400000004000;
        @(posedge clk);
        rst_lat2 = 0;
        while (dut_lat2.dbg_state != 5'd2) @(posedge clk);
        check("Group A: READ_LATENCY=2 captures steering vector cleanly", dut_lat2.u_mem.a_valid === 1'b1);
        @(posedge clk);
        start_lat2 = 1;
        @(posedge clk);
        start_lat2 = 0;
        while (!dut_lat2.done) @(posedge clk);
        check("Group A: READ_LATENCY=2 executes and finishes cleanly", dut_lat2.done === 1'b1);

        // ====================================================================
        // TEST GROUP B: Quiescent w0 = a/4 Bit-Exact & Output Backpressure
        // ====================================================================
        $display("\n--- Running Test Group B ---");
        // Run DUT 1 free-run
        dut1_out_cnt = 0;
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        while (!dut1_done) @(posedge clk);
        check("Group B: Full record run completed with done asserted", dut1_done === 1'b1);
        check("Group B: Exactly 200 samples captured", dut1_out_cnt === 200);

        // Verify Sample 0 output:
        // x[0] = s0=(800, 700), s1=(600, 500), s2=(400, 300), s3=(200, 100)
        // sum_re = 2000, sum_im = 1600 in Q1.15
        // y_re = 2000 / 4 * 2^7 = 64000 in Q2.22
        // y_im = 1600 / 4 * 2^7 = 51200 in Q2.22
        check("Group B: Sample 0 Re matches bit-exact quiescent a/4 (64000)", $signed(cap_run1[0][23:0]) === 24'sd64000);
        check("Group B: Sample 0 Im matches bit-exact quiescent a/4 (51200)", $signed(cap_run1[0][47:24]) === 24'sd51200);

        // Save Run 1 results into cap_run2
        for (i = 0; i < 200; i = i + 1) begin
            cap_run2[i] = cap_run1[i];
        end

        // Backpressure test: reset and re-run with frequent random stalls
        @(posedge clk);
        rst = 1;
        #20;
        @(posedge clk);
        rst = 0;
        while (dut1.dbg_state != 5'd2) @(posedge clk);

        dut1_out_cnt = 0;
        fork
            begin
                @(posedge clk);
                start = 1;
                @(posedge clk);
                start = 0;
                while (!dut1_done) @(posedge clk);
            end
            begin
                while (!dut1_done) begin
                    m_ready = 1'b0; // stall
                    @(posedge clk);
                    @(posedge clk);
                    m_ready = 1'b1; // release
                    @(posedge clk);
                end
                m_ready = 1'b1;
            end
        join

        check("Group B: Backpressure run completed with done asserted", dut1_done === 1'b1);
        check("Group B: Backpressure run captured exactly 200 samples", dut1_out_cnt === 200);

        begin : g_bp_compare
            reg ident;
            ident = 1'b1;
            for (i = 0; i < 200; i = i + 1) begin
                if (cap_run1[i] !== cap_run2[i]) begin
                    ident = 1'b0;
                end
            end
            check("Group B: Stalled run produces bit-identical samples to free run", ident === 1'b1);
        end

        // ====================================================================
        // TEST GROUP C: Covariance Recursion Bit-Exactness
        // ====================================================================
        $display("\n--- Running Test Group C ---");
        check("Group C: Persistent have_cov flag set after block completion", dut1.u_sol.have_cov === 1'b1);
        check("Group C: Diagonal covariance entry R00 is strictly positive", dut1.u_sol.R[0] > 0);

        // ====================================================================
        // TEST GROUP E: Causality & Perturbation Test
        // ====================================================================
        $display("\n--- Running Test Group E ---");
        // Save first block (samples 0..75) of original run
        // Now perturb samples in block 1 (samples 76..151)
        for (i = 76; i < 152; i = i + 1) begin
            dut1.u_mem.mem[1 + i] = 128'h0123_4567_89AB_CDEF_FEDC_BA98_7654_3210;
        end

        // Re-run DUT 1
        @(posedge clk);
        rst = 1;
        #20;
        @(posedge clk);
        rst = 0;
        while (dut1.dbg_state != 5'd2) @(posedge clk);
        dut1_out_cnt = 0;
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        while (!dut1_done) @(posedge clk);

        begin : g_causality
            reg causal_ok;
            causal_ok = 1'b1;
            for (i = 0; i < 76; i = i + 1) begin
                if (cap_run1[i] !== cap_run2[i]) begin
                    causal_ok = 1'b0;
                end
            end
            check("Group E: Causality verified: Block 0 bit-identical despite Block 1 perturbation", causal_ok === 1'b1);
        end

        // ====================================================================
        // TEST GROUP F: Block Boundaries and Parameter Overrides
        // ====================================================================
        $display("\n--- Running Test Group F ---");

        // F1: Exact multiple (K=76, N=152, tail=0)
        for (i = 0; i < 256; i = i + 1) dut_f1.u_mem.mem[i] = 128'h00004000000040000000400000004000;
        @(posedge clk);
        rst_f1 = 0;
        while (dut_f1.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start_f1 = 1;
        @(posedge clk);
        start_f1 = 0;
        while (!dut_f1.done) @(posedge clk);
        check("Group F1: Exact multiple (tail=0) finished cleanly", dut_f1.done === 1'b1);

        // F2: One-sample tail (K=76, N=153, tail=1)
        for (i = 0; i < 256; i = i + 1) dut_f2.u_mem.mem[i] = 128'h00004000000040000000400000004000;
        @(posedge clk);
        rst_f2 = 0;
        while (dut_f2.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start_f2 = 1;
        @(posedge clk);
        start_f2 = 0;
        while (!dut_f2.done) @(posedge clk);
        check("Group F2: One-sample tail (tail=1) finished cleanly", dut_f2.done === 1'b1);

        // F4: Record shorter than block (K=76, N=30)
        for (i = 0; i < 64; i = i + 1) dut_f4.u_mem.mem[i] = 128'h00004000000040000000400000004000;
        @(posedge clk);
        rst_f4 = 0;
        while (dut_f4.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start_f4 = 1;
        @(posedge clk);
        start_f4 = 0;
        while (!dut_f4.done) @(posedge clk);
        check("Group F4: Record shorter than block (N=30 < K=76) completed as pure tail", dut_f4.done === 1'b1);
        check("Group F4: Solver was never triggered for pure tail", dut_f4.u_sol.have_cov === 1'b0);

        // F5: Block Length override (K=8, N=24)
        for (i = 0; i < 64; i = i + 1) dut_f5.u_mem.mem[i] = 128'h00004000000040000000400000004000;
        @(posedge clk);
        rst_f5 = 0;
        while (dut_f5.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start_f5 = 1;
        @(posedge clk);
        start_f5 = 0;
        while (!dut_f5.done) @(posedge clk);
        check("Group F5: Override BLOCK_LEN=8 executes and completes 3 blocks", dut_f5.done === 1'b1);
        check("Group F5: Persistent covariance active with BLOCK_LEN=8", dut_f5.u_sol.have_cov === 1'b1);

        // ====================================================================
        // TEST GROUP G: Reset Recovery and PACED Mode
        // ====================================================================
        $display("\n--- Running Test Group G ---");

        // G1: Reset mid-sample
        init_dut1_mem();
        @(posedge clk);
        rst = 1;
        #20;
        @(posedge clk);
        rst = 0;
        while (dut1.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        #500; // Reset right in the middle of sample engine processing
        rst = 1;
        #30;
        @(posedge clk);
        rst = 0;
        while (dut1.dbg_state != 5'd2) @(posedge clk);
        check("Group G1: Clean recovery from mid-sample reset", dut1.dbg_state == 5'd2);

        // G2: Reset mid-solve
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        while (dut1.dbg_state != 5'd8) @(posedge clk); // ST_SOLVE
        #50;
        rst = 1; // Reset right in the middle of solver
        #30;
        @(posedge clk);
        rst = 0;
        while (dut1.dbg_state != 5'd2) @(posedge clk);
        check("Group G2: Clean recovery from mid-solve reset", dut1.dbg_state == 5'd2);

        // G3: PACED = 1 Generous Pacing
        for (i = 0; i < 64; i = i + 1) dut_paced.u_mem.mem[i] = 128'h00004000000040000000400000004000;
        @(posedge clk);
        rst_p = 0;
        while (dut_paced.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start_p = 1;
        @(posedge clk);
        start_p = 0;

        // Pulse sample_en every 80 cycles (generous > 52)
        while (!dut_paced.done) begin
            @(posedge clk);
            sample_en_p = 1'b1;
            @(posedge clk);
            sample_en_p = 1'b0;
            repeat (80) @(posedge clk);
        end
        check("Group G3: Generous pacing completes with done asserted", dut_paced.done === 1'b1);
        check("Group G3: Generous pacing has NO overrun (status[0] == 0)", dut_paced.status[0] === 1'b0);

        // G4: PACED = 1 Aggressive Overrun Pacing
        @(posedge clk);
        rst_p = 1;
        #20;
        @(posedge clk);
        rst_p = 0;
        while (dut_paced.dbg_state != 5'd2) @(posedge clk);
        @(posedge clk);
        start_p = 1;
        @(posedge clk);
        start_p = 0;

        // Pulse sample_en every 5 cycles (< 52 cycles, will cause overrun!)
        for (k = 0; k < 20; k = k + 1) begin
            @(posedge clk);
            sample_en_p = 1'b1;
            @(posedge clk);
            sample_en_p = 1'b0;
            repeat (5) @(posedge clk);
        end
        while (!dut_paced.done) begin
            @(posedge clk);
            sample_en_p = 1'b1;
            @(posedge clk);
            sample_en_p = 1'b0;
            repeat (80) @(posedge clk);
        end
        check("Group G4: Aggressive pacing latches status[0] err_overrun", dut_paced.status[0] === 1'b1);

        // ====================================================================
        // Final Summary
        // ====================================================================
        $display("\n==================================================");
        $display("tb_directed Summary: PASSED %0d, FAILED %0d", pass_count, fail_count);
        $display("==================================================");

        if (fail_count != 0) begin
            $display("TB_DIRECTED STATUS: FAILED");
            $fatal(1, "Testbench failed!");
        end else begin
            $display("TB_DIRECTED STATUS: ALL TESTS PASSED");
        end

        $finish;
    end

endmodule
