// ============================================================================
// tb_serial_divider.v -- Unit testbench for serial_divider module
// ============================================================================

`timescale 1ns/1ps

module tb_serial_divider;

    reg clk;
    reg rst;
    integer pass_count;
    integer fail_count;

    always #5 clk = ~clk;

    localparam WN = 65;
    localparam WD = 40;

    reg             start;
    reg  [WN-1:0]   dividend;
    reg  [WD-1:0]   divisor;
    wire [WN-1:0]   quotient;
    wire [WN-1:0]   quotient_rnd;
    wire [WD-1:0]   remainder;
    wire            busy;
    wire            done;
    wire            div_by_zero;

    serial_divider #(
        .WIDTH_N(WN),
        .WIDTH_D(WD)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .dividend(dividend),
        .divisor(divisor),
        .quotient(quotient),
        .quotient_rnd(quotient_rnd),
        .remainder(remainder),
        .busy(busy),
        .done(done),
        .div_by_zero(div_by_zero)
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

    task do_div;
        input [WN-1:0] in_n;
        input [WD-1:0] in_d;
        begin
            @(posedge clk);
            dividend = in_n;
            divisor  = in_d;
            start    = 1'b1;
            @(posedge clk);
            start    = 1'b0;
            @(posedge done);
            @(posedge clk);
        end
    endtask

    reg [WN-1:0] exp_q;
    reg [WD-1:0] exp_rem;
    reg [WN-1:0] exp_rnd;
    integer seed;
    integer iter;
    reg [31:0] r1, r2, r3;
    reg [64:0] test_n;
    reg [39:0] test_d;

    initial begin
        clk = 0;
        rst = 1;
        start = 0;
        dividend = 0;
        divisor = 0;
        pass_count = 0;
        fail_count = 0;
        seed = 42;

        $display("Starting tb_serial_divider...");
        #20;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // Test 1: Divide by zero
        $display("Running Test 1 (dbz)...");
        do_div(65'd1000, 40'd0);
        check("Divide by zero sets div_by_zero flag", div_by_zero === 1'b1);

        // Test 2: Exact division: 100 / 10 = 10, rem = 0
        $display("Running Test 2 (100/10)...");
        do_div(65'd100, 40'd10);
        check("Exact division 100 / 10 quotient", quotient === 65'd10);
        check("Exact division 100 / 10 remainder", remainder === 40'd0);
        check("Exact division 100 / 10 quotient_rnd", quotient_rnd === 65'd10);

        // Test 3: Round down: 10 / 3 = 3.333 -> q=3, rem=1, rnd=3
        $display("Running Test 3 (10/3)...");
        do_div(65'd10, 40'd3);
        check("Round down 10 / 3 quotient == 3", quotient === 65'd3);
        check("Round down 10 / 3 remainder == 1", remainder === 40'd1);
        check("Round down 10 / 3 quotient_rnd == 3", quotient_rnd === 65'd3);

        // Test 4: Round up: 11 / 3 = 3.666 -> q=3, rem=2, rnd=4
        $display("Running Test 4 (11/3)...");
        do_div(65'd11, 40'd3);
        check("Round up 11 / 3 quotient == 3", quotient === 65'd3);
        check("Round up 11 / 3 remainder == 2", remainder === 40'd2);
        check("Round up 11 / 3 quotient_rnd == 4", quotient_rnd === 65'd4);

        // Test 5: Exact half: 5 / 2 = 2.5 -> q=2, rem=1, rnd=3 (round half up)
        $display("Running Test 5 (5/2)...");
        do_div(65'd5, 40'd2);
        check("Half round up 5 / 2 quotient_rnd == 3", quotient_rnd === 65'd3);

        // Test 6: Pivot reciprocal at D_MIN = 1024
        // 2^48 / 1024 = 2^38 = 274877906944
        $display("Running Test 6 (pivot 2^48 / 1024)...");
        do_div(65'h0001_0000_0000_0000, 40'd1024);
        check("Pivot 2^48 / 1024 quotient == 2^38", quotient === 65'd274877906944);
        check("Pivot 2^48 / 1024 remainder == 0", remainder === 40'd0);

        // Test 7: Normalizer reciprocal: 2^64 / 16777216 (1.0 in Q16.24)
        // 2^64 / 2^24 = 2^40 = 1099511627776 (>= 40 fractional bits)
        $display("Running Test 7 (norm 2^64 / 2^24)...");
        do_div({1'b1, 64'd0}, 40'd16777216);
        check("Normalizer 2^64 / 2^24 quotient == 2^40", quotient === 65'd1099511627776);
        check("Normalizer 2^64 / 2^24 remainder == 0", remainder === 40'd0);

        // Test 8: 50 Random stimulus trials vs integer arithmetic (up to 64-bit numbers)
        for (iter = 0; iter < 50; iter = iter + 1) begin
            r2 = $random(seed);
            r3 = $random(seed);
            test_n = {1'b0, r2, r3};
            if (test_n == 0) test_n = 65'd1;

            r1 = $random(seed);
            r2 = $random(seed);
            test_d = {r1[7:0], r2};
            if (test_d == 0) test_d = 40'd1;

            exp_q = test_n / test_d;
            exp_rem = test_n % test_d;
            if (({exp_rem, 1'b0}) >= test_d) begin
                exp_rnd = exp_q + 65'd1;
            end else begin
                exp_rnd = exp_q;
            end

            do_div(test_n, test_d);

            if (quotient !== exp_q || remainder !== exp_rem || quotient_rnd !== exp_rnd) begin
                $display("FAIL: Random division mismatch at iter %0d: N=%h, D=%h | got Q=%h, R=%h, RND=%h | exp Q=%h, R=%h, RND=%h",
                         iter, test_n, test_d, quotient, remainder, quotient_rnd, exp_q, exp_rem, exp_rnd);
                fail_count = fail_count + 1;
            end else begin
                pass_count = pass_count + 1;
            end
        end

        $display("--------------------------------------------------");
        $display("tb_serial_divider: PASSED %0d, FAILED %0d", pass_count, fail_count);
        $display("--------------------------------------------------");

        if (fail_count != 0) begin
            $display("TB_SERIAL_DIVIDER STATUS: FAILED");
            $fatal(1, "Testbench failed!");
        end else begin
            $display("TB_SERIAL_DIVIDER STATUS: ALL TESTS PASSED");
        end

        $finish;
    end

endmodule
