`timescale 1ns/1ps
// ============================================================================
// tb_save_output.v -- runs mvdr_top on x_mem.mem and SAVES every output sample to a CSV file.
// Uses the same parameters as the board build (BLOCK_LEN 76, NUM_SAMPLES 1536, free-run, no
// backpressure), so the file shows what the FPGA computes for the same memory image.
//
// CSV columns:  idx, block, re_raw, im_raw, re_value, im_value, m_data_hex
//   m_data[47:0] = {im[23:0], re[23:0]}, each signed Q2.22  ->  value = raw / 4194304
//   (the values are scaled by the global gain g that was applied when x_mem.mem was written;
//    divide by g to get the original amplitude)
// Also prints the same running checksum the board wrapper computes, so LD3 on the board can be
// compared with the parity printed here.
// Plain Verilog-2001 (the only system tasks used are $fopen/$fwrite/$fclose/$display/$finish).
// ============================================================================
module tb_save_output;
    parameter BLOCK_LEN   = 76;
    parameter NUM_SAMPLES = 1536;
    parameter MEM_DEPTH   = 2048;
    parameter MEM_FILE    = "x_mem.mem";
    parameter OUT_FILE    = "y_out.csv";

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg  rst   = 1'b1;
    reg  start = 1'b0;

    wire         m_valid, m_last, m_is_tail, busy, done, dbg_w_swap, dbg_blk_done;
    wire [47:0]  m_data;
    wire [15:0]  m_idx;
    wire [7:0]   m_blk, status;
    wire [199:0] dbg_w_active;
    wire [31:0]  dbg_solve_cycles;
    wire [4:0]   dbg_state;

    mvdr_top #(
        .BLOCK_LEN   (BLOCK_LEN),
        .NUM_SAMPLES (NUM_SAMPLES),
        .MEM_DEPTH   (MEM_DEPTH),
        .MEM_FILE    (MEM_FILE),
        .PACED       (0)
    ) dut (
        .clk(clk), .rst(rst), .start(start), .sample_en(1'b0), .m_ready(1'b1),
        .m_valid(m_valid), .m_data(m_data), .m_last(m_last), .m_idx(m_idx), .m_blk(m_blk),
        .m_is_tail(m_is_tail), .busy(busy), .done(done), .status(status),
        .dbg_w_active(dbg_w_active), .dbg_w_swap(dbg_w_swap), .dbg_blk_done(dbg_blk_done),
        .dbg_solve_cycles(dbg_solve_cycles), .dbg_state(dbg_state)
    );

    integer fh;
    integer count;
    integer re_s, im_s;
    reg [74:0] checksum;
    wire [74:0] out_word = {m_is_tail, m_last, m_blk, m_idx, m_data, 1'b0};  // same word as the board wrapper

    initial begin
        count    = 0;
        checksum = 75'd0;
        fh = $fopen(OUT_FILE, "w");
        $fwrite(fh, "idx,block,re_raw,im_raw,re_value,im_value,m_data_hex\n");
    end

    always @(posedge clk) begin
        if (rst) begin
            checksum <= 75'd0;
        end else if (m_valid) begin
            re_s = $signed(m_data[23:0]);
            im_s = $signed(m_data[47:24]);
            $fwrite(fh, "%0d,%0d,%0d,%0d,%0.9f,%0.9f,%012h\n",
                    m_idx, m_blk, re_s, im_s, re_s / 4194304.0, im_s / 4194304.0, m_data);
            count    <= count + 1;
            checksum <= {checksum[73:0], checksum[74]} ^ out_word;
        end
    end

    integer guard;
    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;
        guard = 0;
        while (dbg_state != 5'd2 && guard < 1000) begin @(posedge clk); guard = guard + 1; end
        @(posedge clk); start = 1'b1;
        @(posedge clk); start = 1'b0;
        guard = 0;
        while (!done && guard < 10000000) begin @(posedge clk); guard = guard + 1; end
        repeat (10) @(posedge clk);
        $fclose(fh);
        $display("Saved %0d output samples to %0s", count, OUT_FILE);
        $display("done=%b status=0x%02h  (status 0x00 = no saturation / pivot / solver error)", done, status);
        $display("final checksum (75-bit) = %019h   parity = %b   <-- compare this parity with LD3 on the board", checksum, ^checksum);
        $finish;
    end

    // watchdog (simulation time)
    initial begin
        #200000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
