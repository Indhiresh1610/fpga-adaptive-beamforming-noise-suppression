`timescale 1ns/1ps
// ============================================================================
// tb_uplink.v -- end-to-end simulation of the UART uplink for case_2.mem.
//   1. sends 'S' to mvdr_system over a simulated UART line
//   2. decodes every byte the FPGA sends back on uart_txd
//   3. writes them, one hex byte per line, to uart_dump.hex
// Then run:   python fpga_uplink_rx.py --hexfile uart_dump.hex --compare y_out.csv
// The y CSV produced from this dump must be identical to tb_save_output's y_out.csv.
//
// The baud rate is set very high (CLKS_PER_BIT = 8) only to keep the simulation
// short; the board uses 434 (115200 baud at 50 MHz). Nothing else differs.
// Simulate with: tb_uplink.v, mvdr_board_top.v (only its mvdr_system and lower
// modules are used, so no MMCM is simulated), all the core files
// (mvdr_top.v ... serial_divider.v) and case_2.mem. Set tb_uplink as the sim top.
// ============================================================================
module tb_uplink;
    parameter BLOCK_LEN    = 76;
    parameter NUM_SAMPLES  = 2047;
    parameter MEM_DEPTH    = 2048;
    parameter MEM_FILE     = "case_2.mem";
    parameter CLKS_PER_BIT = 8;
    parameter HEX_FILE     = "uart_dump.hex";

    reg clk = 1'b0;
    always #10 clk = ~clk;                // 50 MHz, like the board

    reg rst      = 1'b1;
    reg pc_txd   = 1'b1;                  // PC -> FPGA line
    wire fpga_txd;                        // FPGA -> PC line

    wire        bf_valid, bf_is_tail, bf_last;
    wire [47:0] bf_data;
    wire [15:0] bf_idx;
    wire [7:0]  bf_blk, status;
    wire        busy, done, dumping, cap_overflow, run_arm;

    mvdr_system #(
        .BLOCK_LEN    (BLOCK_LEN),
        .NUM_SAMPLES  (NUM_SAMPLES),
        .MEM_DEPTH    (MEM_DEPTH),
        .MEM_FILE     (MEM_FILE),
        .CLKS_PER_BIT (CLKS_PER_BIT)
    ) dut (
        .clk(clk), .rst(rst), .start_req(1'b0),
        .uart_rxd(pc_txd), .uart_txd(fpga_txd),
        .bf_valid(bf_valid), .bf_data(bf_data), .bf_idx(bf_idx), .bf_blk(bf_blk),
        .bf_is_tail(bf_is_tail), .bf_last(bf_last), .bf_ready(1'b1),
        .busy(busy), .done(done), .status(status),
        .dumping(dumping), .cap_overflow(cap_overflow), .run_arm(run_arm)
    );

    // ---------------- PC side: send one byte ----------------
    integer i;
    task send_byte;
        input [7:0] b;
        begin
            pc_txd = 1'b0; repeat (CLKS_PER_BIT) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                pc_txd = b[i]; repeat (CLKS_PER_BIT) @(posedge clk);
            end
            pc_txd = 1'b1; repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    // ---------------- PC side: receive bytes ----------------
    integer fh;
    integer nbytes;
    integer k;
    reg [7:0] rb;

    initial begin
        nbytes = 0;
        fh = $fopen(HEX_FILE, "w");
        forever begin
            @(negedge fpga_txd);                         // start bit
            repeat (CLKS_PER_BIT / 2) @(posedge clk);    // middle of start bit
            for (k = 0; k < 8; k = k + 1) begin
                repeat (CLKS_PER_BIT) @(posedge clk);
                rb[k] = fpga_txd;
            end
            repeat (CLKS_PER_BIT) @(posedge clk);        // stop bit
            if (fpga_txd !== 1'b1) $display("WARNING: framing error at byte %0d", nbytes);
            $fwrite(fh, "%02x\n", rb);
            nbytes = nbytes + 1;
        end
    end

    // ---------------- sequence ----------------
    integer guard;
    initial begin
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (50) @(posedge clk);
        send_byte(8'h53);                                // 'S'

        guard = 0;
        while (!dumping && guard < 20000000) begin @(posedge clk); guard = guard + 1; end
        $display("run finished: done=%b status=0x%02h overflow=%b", done, status, cap_overflow);
        guard = 0;
        while (dumping && guard < 50000000) begin @(posedge clk); guard = guard + 1; end
        repeat (40 * CLKS_PER_BIT) @(posedge clk);
        $fclose(fh);
        $display("received %0d bytes -> %0s", nbytes, HEX_FILE);
        $finish;
    end
endmodule
