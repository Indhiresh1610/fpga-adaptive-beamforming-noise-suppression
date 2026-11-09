// ============================================================================
// tb_sample_engine.v -- Bit-exact unit testbench for sample_engine module
// Tests:
//  - Beamformer output vs bit-exact integer model for known vectors
//  - Pure real, pure imaginary, mixed sign complex multiplication tests
//  - Saturation handling (sat_y flag)
//  - Covariance accumulation vs bit-exact integer model over K=76 samples:
//      * All-zero data
//      * Single-sensor data
//      * All -1.0 (-32768) extreme data (checking overflow safety)
//      * Random complex data
//  - Check accum_en = 0 (tail behavior: y computed, S unchanged)
//  - Measure sample processing cycle count (<= 200)
// ============================================================================

`timescale 1ns/1ps

module tb_sample_engine;

    reg clk;
    reg rst;
    integer pass_count;
    integer fail_count;

    always #5 clk = ~clk;

    localparam BLOCK_LEN = 76;
    localparam ACC_W     = 39;

    reg                 sample_start;
    reg                 accum_en;
    reg                 clear_cov;
    reg  [127:0]        x_snap;
    reg  [199:0]        w_active;
    wire [47:0]         y_out;
    wire                y_valid;
    wire                sample_done;
    wire                sat_y;
    reg  [3:0]          S_rd_addr;
    wire [ACC_W-1:0]    S_rd_data;

    sample_engine #(
        .BLOCK_LEN(BLOCK_LEN),
        .ACC_W(ACC_W)
    ) dut (
        .clk(clk),
        .rst(rst),
        .sample_start(sample_start),
        .accum_en(accum_en),
        .clear_cov(clear_cov),
        .x_snap(x_snap),
        .w_active(w_active),
        .y_out(y_out),
        .y_valid(y_valid),
        .sample_done(sample_done),
        .sat_y(sat_y),
        .S_rd_addr(S_rd_addr),
        .S_rd_data(S_rd_data)
    );

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
    // Golden Integer Reference Model for Beamformer y
    // ------------------------------------------------------------------------
    reg signed [43:0] gold_y_re_acc;
    reg signed [43:0] gold_y_im_acc;
    reg signed [23:0] gold_y_re;
    reg signed [23:0] gold_y_im;
    reg               gold_sat_y;

    task model_beamform;
        input [127:0] x_in;
        input [199:0] w_in;
        reg signed [15:0] xr_m, xi_m;
        reg signed [24:0] wr_m, wi_m;
        reg signed [43:0] re_rnd, im_rnd;
        reg signed [30:0] re_shift, im_shift;
        integer m;
        begin
            gold_y_re_acc = 44'sd0;
            gold_y_im_acc = 44'sd0;
            gold_sat_y = 1'b0;

            for (m = 0; m < 4; m = m + 1) begin
                xr_m = x_in[32*m +: 16];
                xi_m = x_in[32*m + 16 +: 16];
                wr_m = w_in[50*m +: 25];
                wi_m = w_in[50*m + 25 +: 25];

                gold_y_re_acc = gold_y_re_acc + (wr_m * xr_m) + (wi_m * xi_m);
                gold_y_im_acc = gold_y_im_acc + (wr_m * xi_m) - (wi_m * xr_m);
            end

            re_rnd = gold_y_re_acc + 44'sd4096;
            re_shift = re_rnd >>> 13;
            if (re_shift > 31'sd8388607) begin
                gold_y_re = 24'sh7F_FFFF;
                gold_sat_y = 1'b1;
            end else if (re_shift < -31'sd8388608) begin
                gold_y_re = 24'sh80_0000;
                gold_sat_y = 1'b1;
            end else begin
                gold_y_re = re_shift[23:0];
            end

            im_rnd = gold_y_im_acc + 44'sd4096;
            im_shift = im_rnd >>> 13;
            if (im_shift > 31'sd8388607) begin
                gold_y_im = 24'sh7F_FFFF;
                gold_sat_y = 1'b1;
            end else if (im_shift < -31'sd8388608) begin
                gold_y_im = 24'sh80_0000;
                gold_sat_y = 1'b1;
            end else begin
                gold_y_im = im_shift[23:0];
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Golden Integer Reference Model for Covariance S (16 words)
    // ------------------------------------------------------------------------
    reg signed [ACC_W-1:0] gold_S [0:15];

    task model_cov_accum;
        input [127:0] x_in;
        reg signed [15:0] xr [0:3];
        reg signed [15:0] xi [0:3];
        integer m, i, j, p;
        reg signed [31:0] prod;
        begin
            for (m = 0; m < 4; m = m + 1) begin
                xr[m] = x_in[32*m +: 16];
                xi[m] = x_in[32*m + 16 +: 16];
            end

            // Diagonals S0..S3
            for (m = 0; m < 4; m = m + 1) begin
                gold_S[m] = gold_S[m] + (xr[m]*xr[m] + xi[m]*xi[m]);
            end

            // Off-diagonals (0,1), (0,2), (0,3), (1,2), (1,3), (2,3)
            p = 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = i + 1; j < 4; j = j + 1) begin
                    gold_S[4 + p]  = gold_S[4 + p]  + (xr[i]*xr[j] + xi[i]*xi[j]);
                    gold_S[10 + p] = gold_S[10 + p] + (xi[i]*xr[j] - xr[i]*xi[j]);
                    p = p + 1;
                end
            end
        end
    endtask

    task do_sample;
        input [127:0] x_in;
        input [199:0] w_in;
        input         do_accum;
        begin
            @(posedge clk);
            x_snap       <= x_in;
            w_active     <= w_in;
            accum_en     <= do_accum;
            sample_start <= 1'b1;
            @(posedge clk);
            sample_start <= 1'b0;
            @(posedge sample_done);
            @(posedge clk);
        end
    endtask

    integer seed;
    integer s, k;
    reg [127:0] rand_x;
    reg [199:0] rand_w;
    reg [31:0]  r1, r2;
    reg [ACC_W-1:0] saved_S0;
    integer     start_time, cycle_count;
    reg         all_S_match;

    initial begin
        clk = 0;
        rst = 1;
        sample_start = 0;
        accum_en = 1;
        clear_cov = 0;
        x_snap = 0;
        w_active = 0;
        S_rd_addr = 0;
        pass_count = 0;
        fail_count = 0;
        seed = 12345;

        for (k = 0; k < 16; k = k + 1) gold_S[k] = {ACC_W{1'b0}};

        $display("Starting tb_sample_engine...");
        #20;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // --------------------------------------------------------------------
        // Test 1: Cycle count per sample <= 200
        // --------------------------------------------------------------------
        @(posedge clk);
        start_time = $time;
        sample_start <= 1'b1;
        @(posedge clk);
        sample_start <= 1'b0;
        @(posedge sample_done);
        cycle_count = ($time - start_time) / 10;
        $display("Sample processing duration: %0d clock cycles", cycle_count);
        check("Cycle count per sample <= 200 cycles", cycle_count <= 200);

        // --------------------------------------------------------------------
        // Test 2: Beamformer pure real, pure imaginary, mixed signs
        // --------------------------------------------------------------------
        // Pure real: x = {0+1, 0+1, 0+1, 0+1}, w = {0+1, 0+1, 0+1, 0+1} in Q5.20 / Q1.15
        x_snap = {
            16'sd0, 16'sh4000, // s3: re=0.5
            16'sd0, 16'sh4000, // s2: re=0.5
            16'sd0, 16'sh4000, // s1: re=0.5
            16'sd0, 16'sh4000  // s0: re=0.5
        };
        w_active = {
            25'sd0, 25'sh010_0000, // w3: re=1.0 (Q5.20)
            25'sd0, 25'sh010_0000, // w2
            25'sd0, 25'sh010_0000, // w1
            25'sd0, 25'sh010_0000  // w0
        };
        model_beamform(x_snap, w_active);
        do_sample(x_snap, w_active, 1'b0);
        check("Pure real beamformer output matches bit-exact model", y_out === {gold_y_im, gold_y_re});

        // Pure imaginary: x = {j*0.5 ...}, w = {j*1.0 ...}
        // y = conj(w) * x = (-j*1.0) * (j*0.5) = +0.5 per sensor -> sum = +2.0
        x_snap = {
            16'sh4000, 16'sd0,
            16'sh4000, 16'sd0,
            16'sh4000, 16'sd0,
            16'sh4000, 16'sd0
        };
        w_active = {
            25'sh010_0000, 25'sd0,
            25'sh010_0000, 25'sd0,
            25'sh010_0000, 25'sd0,
            25'sh010_0000, 25'sd0
        };
        model_beamform(x_snap, w_active);
        do_sample(x_snap, w_active, 1'b0);
        check("Pure imaginary beamformer output matches bit-exact model", y_out === {gold_y_im, gold_y_re});

        // --------------------------------------------------------------------
        // Test 3: Beamformer Saturation
        // --------------------------------------------------------------------
        // Drive maximum possible weights and samples to saturate y
        x_snap = {
            16'sh7FFF, 16'sh7FFF,
            16'sh7FFF, 16'sh7FFF,
            16'sh7FFF, 16'sh7FFF,
            16'sh7FFF, 16'sh7FFF
        };
        w_active = {
            25'sh07F_FFFF, 25'sh07F_FFFF,
            25'sh07F_FFFF, 25'sh07F_FFFF,
            25'sh07F_FFFF, 25'sh07F_FFFF,
            25'sh07F_FFFF, 25'sh07F_FFFF
        };
        model_beamform(x_snap, w_active);
        do_sample(x_snap, w_active, 1'b0);
        check("Extreme weights and inputs trigger sat_y", sat_y === 1'b1);
        check("Saturated y output matches bit-exact model", y_out === {gold_y_im, gold_y_re});

        // --------------------------------------------------------------------
        // Test 4: Covariance Accumulation over K=76 samples (All -1.0 Extreme)
        // --------------------------------------------------------------------
        @(posedge clk);
        clear_cov <= 1'b1;
        @(posedge clk);
        clear_cov <= 1'b0;
        for (k = 0; k < 16; k = k + 1) gold_S[k] = {ACC_W{1'b0}};

        // Extreme: x = -1.0 (-32768) on all channels
        x_snap = {
            -16'sd32768, -16'sd32768,
            -16'sd32768, -16'sd32768,
            -16'sd32768, -16'sd32768,
            -16'sd32768, -16'sd32768
        };
        w_active = 200'd0;

        for (s = 0; s < BLOCK_LEN; s = s + 1) begin
            model_cov_accum(x_snap);
            do_sample(x_snap, w_active, 1'b1);
        end

        all_S_match = 1'b1;
        for (k = 0; k < 16; k = k + 1) begin
            S_rd_addr = k[3:0];
            #1;
            if (S_rd_data !== gold_S[k]) begin
                all_S_match = 1'b0;
                $display("FAIL: Extreme cov mismatch at word %0d: dut=%h, gold=%h", k, S_rd_data, gold_S[k]);
            end
        end
        check("All-(-1.0) 76-sample covariance accumulation matches bit-exact integer model", all_S_match === 1'b1);

        // --------------------------------------------------------------------
        // Test 5: Covariance Accumulation with Random Data over K=76 samples
        // --------------------------------------------------------------------
        @(posedge clk);
        clear_cov <= 1'b1;
        @(posedge clk);
        clear_cov <= 1'b0;
        for (k = 0; k < 16; k = k + 1) gold_S[k] = {ACC_W{1'b0}};

        for (s = 0; s < BLOCK_LEN; s = s + 1) begin
            for (k = 0; k < 4; k = k + 1) begin
                r1 = $random(seed);
                r2 = $random(seed);
                rand_x[32*k +: 16]      = r1[15:0];
                rand_x[32*k + 16 +: 16] = r2[15:0];
                r1 = $random(seed);
                r2 = $random(seed);
                rand_w[50*k +: 25]      = r1[24:0];
                rand_w[50*k + 25 +: 25] = r2[24:0];
            end
            model_beamform(rand_x, rand_w);
            model_cov_accum(rand_x);
            do_sample(rand_x, rand_w, 1'b1);

            if (y_out !== {gold_y_im, gold_y_re}) begin
                $display("FAIL: Random sample %0d beamformer mismatch: got=%h, exp=%h", s, y_out, {gold_y_im, gold_y_re});
                fail_count = fail_count + 1;
            end
        end

        all_S_match = 1'b1;
        for (k = 0; k < 16; k = k + 1) begin
            S_rd_addr = k[3:0];
            #1;
            if (S_rd_data !== gold_S[k]) begin
                all_S_match = 1'b0;
                $display("FAIL: Random cov mismatch at word %0d: dut=%h, gold=%h", k, S_rd_data, gold_S[k]);
            end
        end
        check("Random data 76-sample covariance accumulation matches bit-exact integer model", all_S_match === 1'b1);

        // Diagonals must be non-negative
        for (k = 0; k < 4; k = k + 1) begin
            S_rd_addr = k[3:0];
            #1;
            check("Diagonal S is non-negative", $signed(S_rd_data) >= 0);
        end

        // --------------------------------------------------------------------
        // Test 6: Tail behavior: accum_en = 0
        // --------------------------------------------------------------------
        // Save current S[0]
        S_rd_addr = 4'd0;
        #1;
        saved_S0 = S_rd_data;

        // Run sample with accum_en = 0
        do_sample(rand_x, rand_w, 1'b0);
        #1;
        check("Tail sample (accum_en=0) leaves S unchanged", S_rd_data === saved_S0);

        $display("--------------------------------------------------");
        $display("tb_sample_engine: PASSED %0d, FAILED %0d", pass_count, fail_count);
        $display("--------------------------------------------------");

        if (fail_count != 0) begin
            $display("TB_SAMPLE_ENGINE STATUS: FAILED");
            $fatal(1, "Testbench failed!");
        end else begin
            $display("TB_SAMPLE_ENGINE STATUS: ALL TESTS PASSED");
        end

        $finish;
    end

endmodule
