// ============================================================================
// tb_wiener_system_only.v -- simulates wiener_system (ROM + feeder + Wiener + capture tap), without MMCM / UART dump.
// Writes wf_out.csv (idx,re_raw,im_raw) from the captured stream, so it can be compared with wf_expected.csv.
// Add as simulation sources: this file, wf_in.mem, and (for the real IP flow) nothing else; the design files are shared.
// ============================================================================
`timescale 1ns/1ps
module tb_wiener_system_only;
    reg clk = 1'b0, rst = 1'b1, start_req = 1'b0;
    always #10 clk = ~clk;                         // 50 MHz
    wire feeding, feed_done, wf_done, dumping, cap_overflow, uart_txd;
    wire [7:0] wf_status;

    wiener_system dut (.clk(clk), .rst(rst), .start_req(start_req), .uart_rxd(1'b1), .uart_txd(uart_txd),
        .feeding(feeding), .feed_done(feed_done), .wf_status(wf_status), .wf_done(wf_done),
        .dumping(dumping), .cap_overflow(cap_overflow));

    integer f, n;
    always @(posedge clk) if (dut.w_tap_v) begin
        $fwrite(f, "%0d,%0d,%0d\n", dut.cnt_o, $signed(dut.w_m_re), $signed(dut.w_m_im));
        n = n + 1;
    end

    initial begin #800000000; $display("FATAL: watchdog, %0d outputs", n); $finish; end
    initial begin
        n = 0;
        f = $fopen("wf_out.csv", "w"); $fwrite(f, "idx,re_raw,im_raw\n");
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);
        start_req = 1'b1; @(posedge clk); start_req = 1'b0;      // same as pressing btn_start
        while (!wf_done) @(posedge clk);
        repeat (50) @(posedge clk);
        $display("done: %0d outputs written to wf_out.csv, wiener status %h", n, wf_status);
        $fclose(f);
        $finish;
    end
endmodule
