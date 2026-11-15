// ============================================================================
// tb_wiener_system.v -- full chain (team MVDR -> Wiener -> capture tap), no MMCM / UART.
// Writes  wf_dump.txt : "idx blk re im" for every captured Wiener sample (Q2.22 ints)
//         bf_dump.txt : "idx blk re im" for every beamformer beat (all blocks)
// Run in a folder that holds case_2.mem (the file input_mem.v reads):
//   iverilog -g2005 -o tb_w tb_wiener_system.v mvdr_wiener_board_top.v mvdr_board_top.v wiener_cplx_top.v \
//     wiener_cplx_core.v xfft_0_behav.v input_mem.v serial_divider.v sample_engine.v block_solver.v \
//     weight_regs.v output_interface.v block_controller.v mvdr_top.v
//   vvp tb_w ; python3 check_wiener_capture.py bf_dump.txt wf_dump.txt
// (mvdr_board_top.v is only needed for its uart_rx / stream_capture_uplink / uart_tx modules.)
// ============================================================================
`timescale 1ns/1ps
module tb_wiener_system;
    reg clk = 1'b0, rst = 1'b1, start_req = 1'b0;
    always #10 clk = ~clk;                       // 50 MHz core clock

    wire busy, done, dumping, cap_overflow, wf_done, uart_txd;
    wire [7:0] status, wf_status;

    mvdr_wiener_system #(.CLKS_PER_BIT(434)) dut (
        .clk(clk), .rst(rst), .start_req(start_req), .uart_rxd(1'b1), .uart_txd(uart_txd),
        .busy(busy), .done(done), .status(status), .wf_status(wf_status), .wf_done(wf_done),
        .dumping(dumping), .cap_overflow(cap_overflow));

    integer f_bf, f_w, n_bf, n_w;
    always @(posedge clk) if (dut.bf_acc) begin
        $fwrite(f_bf, "%0d %0d %0d %0d\n", dut.bf_idx, dut.bf_blk, $signed(dut.bf_data[23:0]), $signed(dut.bf_data[47:24]));
        n_bf = n_bf + 1;
    end
    always @(posedge clk) if (dut.w_tap_v) begin
        $fwrite(f_w, "%0d %0d %0d %0d\n", dut.w_word[63:48], dut.w_word[71:64], $signed(dut.w_m_re), $signed(dut.w_m_im));
        n_w = n_w + 1;
    end

    initial begin #200000000; $display("FATAL: watchdog (200 ms)"); $finish; end

    initial begin
        n_bf = 0; n_w = 0;
        f_bf = $fopen("bf_dump.txt", "w"); f_w = $fopen("wf_dump.txt", "w");
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);
        start_req = 1'b1; @(posedge clk); start_req = 1'b0;     // same as pressing btn_start
        while (!wf_done) @(posedge clk);
        repeat (100) @(posedge clk);
        $display("beamformer beats %0d, captured Wiener samples %0d, bf status %h, wiener status %h, overflow %b",
                 n_bf, n_w, status, wf_status, cap_overflow);
        $fclose(f_bf); $fclose(f_w);
        $finish;
    end
endmodule
