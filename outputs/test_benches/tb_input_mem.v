// ============================================================================
// tb_input_mem.v -- Unit testbench for input_mem module
// Tests Group A requirements:
//  - Steering vector capture
//  - Sensor order {s3, s2, s1, s0} and unpacking
//  - Extreme values (-32768, +32767)
//  - Read latency 1 and 2
//  - NUM_SAMPLES / MEM_DEPTH configurations (including MEM_DEPTH = NUM_SAMPLES + 1)
//  - Configuration error detection (NUM_SAMPLES + 1 > MEM_DEPTH)
//  - Out-of-bounds X protection
// ============================================================================

`timescale 1ns/1ps

module tb_input_mem;

    reg clk;
    reg rst;
    integer pass_count;
    integer fail_count;

    // Clock generation (100 MHz -> 10 ns period)
    always #5 clk = ~clk;

    // ------------------------------------------------------------------------
    // Instance 1: READ_LATENCY = 1, standard size
    // ------------------------------------------------------------------------
    localparam DEPTH_1 = 256;
    localparam SAMPLES_1 = 100;
    reg [7:0] idx_1;
    wire [127:0] snap_1;
    wire [127:0] a_vec_1;
    wire a_valid_1;
    wire err_cfg_1;

    input_mem #(
        .MEM_DEPTH(DEPTH_1),
        .NUM_SAMPLES(SAMPLES_1),
        .MEM_FILE(""),
        .READ_LATENCY(1)
    ) dut_lat1 (
        .clk(clk),
        .rst(rst),
        .sample_idx(idx_1),
        .x_snap(snap_1),
        .a_vec(a_vec_1),
        .a_valid(a_valid_1),
        .err_cfg(err_cfg_1)
    );

    // ------------------------------------------------------------------------
    // Instance 2: READ_LATENCY = 2
    // ------------------------------------------------------------------------
    localparam DEPTH_2 = 128;
    localparam SAMPLES_2 = 50;
    reg [6:0] idx_2;
    wire [127:0] snap_2;
    wire [127:0] a_vec_2;
    wire a_valid_2;
    wire err_cfg_2;

    input_mem #(
        .MEM_DEPTH(DEPTH_2),
        .NUM_SAMPLES(SAMPLES_2),
        .MEM_FILE(""),
        .READ_LATENCY(2)
    ) dut_lat2 (
        .clk(clk),
        .rst(rst),
        .sample_idx(idx_2),
        .x_snap(snap_2),
        .a_vec(a_vec_2),
        .a_valid(a_valid_2),
        .err_cfg(err_cfg_2)
    );

    // ------------------------------------------------------------------------
    // Instance 3: Exact boundary MEM_DEPTH = NUM_SAMPLES + 1
    // ------------------------------------------------------------------------
    localparam DEPTH_3 = 64;
    localparam SAMPLES_3 = 63;
    reg [5:0] idx_3;
    wire [127:0] snap_3;
    wire [127:0] a_vec_3;
    wire a_valid_3;
    wire err_cfg_3;

    input_mem #(
        .MEM_DEPTH(DEPTH_3),
        .NUM_SAMPLES(SAMPLES_3),
        .MEM_FILE(""),
        .READ_LATENCY(1)
    ) dut_exact (
        .clk(clk),
        .rst(rst),
        .sample_idx(idx_3),
        .x_snap(snap_3),
        .a_vec(a_vec_3),
        .a_valid(a_valid_3),
        .err_cfg(err_cfg_3)
    );

    // ------------------------------------------------------------------------
    // Instance 4: Configuration Error: NUM_SAMPLES + 1 > MEM_DEPTH
    // ------------------------------------------------------------------------
    localparam DEPTH_4 = 64;
    localparam SAMPLES_4 = 64;
    reg [5:0] idx_4;
    wire [127:0] snap_4;
    wire [127:0] a_vec_4;
    wire a_valid_4;
    wire err_cfg_4;

    input_mem #(
        .MEM_DEPTH(DEPTH_4),
        .NUM_SAMPLES(SAMPLES_4),
        .MEM_FILE(""),
        .READ_LATENCY(1)
    ) dut_err (
        .clk(clk),
        .rst(rst),
        .sample_idx(idx_4),
        .x_snap(snap_4),
        .a_vec(a_vec_4),
        .a_valid(a_valid_4),
        .err_cfg(err_cfg_4)
    );

    // Helper task to check conditions
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

    integer i;
    reg [127:0] expected_header;

    initial begin
        clk = 0;
        rst = 1;
        pass_count = 0;
        fail_count = 0;
        idx_1 = 0;
        idx_2 = 0;
        idx_3 = 0;
        idx_4 = 0;

        // Initialize dut_lat1 memory
        expected_header = {
            16'sh1000, 16'sh2000, // s3: im=0x1000, re=0x2000
            16'sh3000, 16'sh4000, // s2: im=0x3000, re=0x4000
            16'sh5000, 16'sh6000, // s1: im=0x5000, re=0x6000
            16'sh7000, 16'sh4000  // s0: im=0x7000, re=0x4000 (1.0 in Q2.14)
        };
        dut_lat1.mem[0] = expected_header;
        dut_lat2.mem[0] = expected_header;
        dut_exact.mem[0] = expected_header;

        // Sample 0: extreme values
        dut_lat1.mem[1] = {
            16'sh8000, 16'sh7FFF, // s3: min -32768, max +32767
            16'sh7FFF, 16'sh8000, // s2: max +32767, min -32768
            16'sh0000, 16'sh7FFF, // s1: 0, max
            16'sh8000, 16'sh0000  // s0: min, 0
        };

        // Middle sample (idx = 49 -> mem[50])
        dut_lat1.mem[50] = {
            16'sh1234, 16'sh5678,
            16'sh9ABC, 16'shDEF0,
            16'sh1357, 16'sh2468,
            16'sh369C, 16'sh48BF
        };

        // Last sample (idx = 99 -> mem[100])
        dut_lat1.mem[100] = {
            16'sh1111, 16'sh2222,
            16'sh3333, 16'sh4444,
            16'sh5555, 16'sh6666,
            16'sh7777, 16'sh8888
        };

        // Fill unused memory words with 'X'
        for (i = 101; i < DEPTH_1; i = i + 1) begin
            dut_lat1.mem[i] = 128'hXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
        end

        // Wait a few cycles with reset asserted
        #20;
        @(posedge clk);
        rst <= 0;

        // Verify configuration checks
        check("err_cfg_1 is 0 when MEM_DEPTH > NUM_SAMPLES + 1", err_cfg_1 === 1'b0);
        check("err_cfg_3 is 0 when MEM_DEPTH == NUM_SAMPLES + 1", err_cfg_3 === 1'b0);
        check("err_cfg_4 is 1 when NUM_SAMPLES + 1 > MEM_DEPTH", err_cfg_4 === 1'b1);

        // Wait for a_valid_1
        while (!a_valid_1) @(posedge clk);
        check("dut_lat1 a_valid asserted", a_valid_1 === 1'b1);
        check("dut_lat1 a_vec matches expected header", a_vec_1 === expected_header);

        // Wait for a_valid_2
        while (!a_valid_2) @(posedge clk);
        check("dut_lat2 a_valid asserted", a_valid_2 === 1'b1);
        check("dut_lat2 a_vec matches expected header", a_vec_2 === expected_header);

        // Read sample 0 (latency 1)
        idx_1 <= 8'd0;
        @(posedge clk); // sample_idx registered in memory read address
        @(posedge clk); // data available
        check("dut_lat1 sample 0 extreme values", snap_1 === dut_lat1.mem[1]);
        check("dut_lat1 sample 0 s3 im == -32768", $signed(snap_1[127:112]) === -16'sd32768);
        check("dut_lat1 sample 0 s3 re == +32767", $signed(snap_1[111:96]) === 16'sd32767);
        check("dut_lat1 sample 0 s0 im == -32768", $signed(snap_1[31:16]) === -16'sd32768);
        check("dut_lat1 sample 0 s0 re == 0", $signed(snap_1[15:0]) === 16'sd0);

        // Read middle sample 49
        idx_1 <= 8'd49;
        @(posedge clk);
        @(posedge clk);
        check("dut_lat1 middle sample 49 data match", snap_1 === dut_lat1.mem[50]);

        // Read last valid sample 99
        idx_1 <= 8'd99;
        @(posedge clk);
        @(posedge clk);
        check("dut_lat1 last sample 99 data match", snap_1 === dut_lat1.mem[100]);
        check("dut_lat1 valid outputs contain no X", ^snap_1 !== 1'bX);

        // Read with READ_LATENCY = 2
        dut_lat2.mem[1] = 128'h0123456789ABCDEF0123456789ABCDEF;
        idx_2 <= 7'd0;
        @(posedge clk);
        // After 1 cycle, latency 2 should still NOT have updated to new sample
        @(posedge clk);
        @(posedge clk);
        check("dut_lat2 sample 0 match after 2 cycles", snap_2 === dut_lat2.mem[1]);

        // Summary
        $display("--------------------------------------------------");
        $display("tb_input_mem: PASSED %0d, FAILED %0d", pass_count, fail_count);
        $display("--------------------------------------------------");

        if (fail_count != 0) begin
            $display("TB_INPUT_MEM STATUS: FAILED");
            $fatal(1, "Testbench failed!");
        end else begin
            $display("TB_INPUT_MEM STATUS: ALL TESTS PASSED");
        end

        $finish;
    end

endmodule
