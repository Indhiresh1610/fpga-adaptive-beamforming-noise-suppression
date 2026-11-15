// ============================================================================
// tb_case_loader.v -- tests the UART case loader inside the full chain (fast baud: CLKS_PER_BIT = 8).
// Needs case_2.mem in the simulation folder (input_mem.v reads it; this bench reads it too).
// Set as the simulation top INSTEAD of tb_wiener_system.  Run:  run all   (a few minutes)
// Tests
//   1  run with the case that is built in (case_2)                      -> result R1
//   2  load a DIFFERENT case (case_2 XOR pattern) over the UART          -> memory must equal what was sent, ok = 1
//   3  load case_2 again, run                                            -> result R2 must be IDENTICAL to R1
//   4  load with a wrong checksum                                        -> err = 1, a following 'S' must be refused
//   5  good load again                                                   -> err cleared, run works again (R3 == R1)
// Throughout: no run may start while a load is in progress (payload bytes contain 'S' / 'D' values).
// Ends with "ALL TESTS PASSED" or "FAILED".
// ============================================================================
`timescale 1ns/1ps
module tb_case_loader;
    localparam CPB = 8;
    reg clk = 1'b0, rst = 1'b1, rxd = 1'b1;
    always #10 clk = ~clk;

    wire busy, done, dumping, cap_overflow, wf_done, uart_txd, ld_err;
    wire [7:0] status, wf_status;

    mvdr_wiener_system #(.CLKS_PER_BIT(CPB)) dut (
        .clk(clk), .rst(rst), .start_req(1'b0), .uart_rxd(rxd), .uart_txd(uart_txd),
        .busy(busy), .done(done), .status(status), .wf_status(wf_status), .wf_done(wf_done),
        .dumping(dumping), .cap_overflow(cap_overflow), .ld_err(ld_err));

    reg [127:0] caseA [0:2047];
    reg [127:0] cur   [0:2047];
    reg signed [23:0] r1re [0:2047], r1im [0:2047];
    reg signed [23:0] r2re [0:2047], r2im [0:2047];
    reg signed [23:0] r3re [0:2047], r3im [0:2047];
    integer run_id, nw, i, errors, loading;
    reg [7:0] sum;

    // collect the Wiener output of the current run
    always @(posedge clk) if (dut.w_tap_v) begin
        if (run_id == 1) begin r1re[nw] <= $signed(dut.w_m_re); r1im[nw] <= $signed(dut.w_m_im); end
        if (run_id == 2) begin r2re[nw] <= $signed(dut.w_m_re); r2im[nw] <= $signed(dut.w_m_im); end
        if (run_id == 3) begin r3re[nw] <= $signed(dut.w_m_re); r3im[nw] <= $signed(dut.w_m_im); end
        nw <= nw + 1;
    end
    // no run may start during a load
    always @(posedge clk) if (loading && (busy || dut.q != 3'd0)) begin
        errors = errors + 1; $display("FAIL: a run started during a load (t=%0t)", $time);
    end

    task send_byte(input [7:0] b);
        integer k;
        begin
            rxd = 1'b0; repeat (CPB) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin rxd = b[k]; repeat (CPB) @(posedge clk); end
            rxd = 1'b1; repeat (CPB) @(posedge clk);
        end
    endtask

    // send 'L' + 2048 words (cur[]) + checksum (+1 if bad)
    task send_load(input bad);
        integer w, bb;
        reg [7:0] byte_v;
        begin
            loading = 1;
            sum = 8'd0;
            send_byte(8'h4C);
            for (w = 0; w < 2048; w = w + 1)
                for (bb = 15; bb >= 0; bb = bb - 1) begin
                    byte_v = cur[w] >> (8 * bb);
                    sum = sum + byte_v;
                    send_byte(byte_v);
                end
            send_byte(bad ? (sum + 8'd1) : sum);
            repeat (50) @(posedge clk);
            loading = 0;
        end
    endtask

    task check_mem(input [255:0] name);
        integer w, bad;
        begin
            bad = 0;
            for (w = 0; w < 2048; w = w + 1)
                if (dut.u_core.u_mem.mem[w] !== cur[w]) bad = bad + 1;
            if (bad != 0) begin errors = errors + 1; $display("FAIL: %0s: %0d memory words differ from what was sent", name, bad); end
            else $display("ok  : %0s: all 2048 memory words equal what was sent", name);
        end
    endtask

    // send 'S', wait for the run to finish and for the UART dump to end
    task do_run(input integer id);
        integer t;
        begin
            run_id = id; nw = 0;
            send_byte(8'h53);
            repeat (200) @(posedge clk);
            while (!wf_done) @(posedge clk);
            t = 0; while (!dumping && t < 5000) begin @(posedge clk); t = t + 1; end
            t = 0; while (dumping && t < 6000000) begin @(posedge clk); t = t + 1; end
            repeat (200) @(posedge clk);
            if (nw != 1971) begin errors = errors + 1; $display("FAIL: run %0d captured %0d samples (expected 1971)", id, nw); end
            else $display("ok  : run %0d captured 1971 samples", id);
        end
    endtask

    task compare_runs(input integer a, input integer b);
        integer k, d;
        begin
            d = 0;
            for (k = 0; k < 1971; k = k + 1) begin
                if (a == 1 && b == 2 && (r1re[k] !== r2re[k] || r1im[k] !== r2im[k])) d = d + 1;
                if (a == 1 && b == 3 && (r1re[k] !== r3re[k] || r1im[k] !== r3im[k])) d = d + 1;
            end
            if (d != 0) begin errors = errors + 1; $display("FAIL: result of run %0d differs from run %0d in %0d samples", b, a, d); end
            else $display("ok  : result of run %0d is identical to run %0d", b, a);
        end
    endtask

    initial begin #2000000000; $display("FAIL: watchdog"); $finish; end

    initial begin
        errors = 0; loading = 0; run_id = 0; nw = 0;
        $readmemh("case_2.mem", caseA);
        repeat (20) @(posedge clk); rst = 1'b0; repeat (20) @(posedge clk);

        $display("--- test 1: built-in case");
        do_run(1);

        $display("--- test 2: load a different case");
        for (i = 0; i < 2048; i = i + 1) cur[i] = caseA[i] ^ {16{8'hA5}} ^ i;
        send_load(1'b0);
        if (dut.u_ld.ok !== 1'b1 || ld_err !== 1'b0) begin errors = errors + 1; $display("FAIL: ok/err after a good load: ok=%b err=%b", dut.u_ld.ok, ld_err); end
        else $display("ok  : ok = 1, err = 0 after the load");
        check_mem("different case");

        $display("--- test 3: load case_2 again and run");
        for (i = 0; i < 2048; i = i + 1) cur[i] = caseA[i];
        send_load(1'b0);
        check_mem("case_2 reloaded");
        do_run(2);
        compare_runs(1, 2);

        $display("--- test 4: wrong checksum");
        send_load(1'b1);
        if (ld_err !== 1'b1 || dut.u_ld.ok !== 1'b0) begin errors = errors + 1; $display("FAIL: err = %b, ok = %b after a wrong checksum", ld_err, dut.u_ld.ok); end
        else $display("ok  : err = 1 after a wrong checksum");
        send_byte(8'h53); repeat (5000) @(posedge clk);
        if (busy || dut.q != 3'd0 || dumping) begin errors = errors + 1; $display("FAIL: a run was started after a failed load"); end
        else $display("ok  : 'S' refused after a failed load");

        $display("--- test 5: good load again, run");
        send_load(1'b0);
        if (ld_err !== 1'b0 || dut.u_ld.ok !== 1'b1) begin errors = errors + 1; $display("FAIL: err was not cleared by a good load"); end
        else $display("ok  : err cleared by a good load");
        do_run(3);
        compare_runs(1, 3);

        if (errors == 0) $display("ALL TESTS PASSED");
        else $display("FAILED: %0d error(s)", errors);
        $finish;
    end
endmodule
