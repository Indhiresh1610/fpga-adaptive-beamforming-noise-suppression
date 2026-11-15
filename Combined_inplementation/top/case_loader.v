// ============================================================================
// case_loader.v -- replace the input case over the UART (no new bitstream per case).
//
// Protocol (PC -> board, 115200 8N1):
//     'L' (0x4C)                                  load command, accepted only while the board is idle (allow = 1)
//     DEPTH * 16 payload bytes                    the lines of case_N.mem in order, every line = 16 bytes, MSB byte first
//                                                 (exactly the value $readmemh would put into one memory word)
//     1 checksum byte                             sum of all payload bytes modulo 256
// Each completed 16-byte word is written to the input memory (ld_we / ld_addr / ld_data), word 0 first
// (word 0 = steering vector, words 1..2047 = snapshots, as in input_mem.v).
//   ok  = 1 : the last load finished and its checksum matched
//   err = 1 : the last load had a wrong checksum or stopped for TIMEOUT clocks (memory content is then NOT trusted;
//             the top refuses to start a run until a good load has been done).  err / ok are only changed by a load
//             attempt, not by reset, so a failed load can never be forgotten by pressing the reset button.
// active = 1 from the 'L' until the checksum byte: the top ignores 'S' / 'D' bytes during that time (payload bytes can
// have any value).
// ============================================================================
`timescale 1ns/1ps
module case_loader #(
    parameter DEPTH   = 2048,
    parameter AW      = 11,
    parameter TIMEOUT = 25000000          // clocks without a byte, inside a load, before it is aborted (0.5 s at 50 MHz)
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          rx_valid,
    input  wire [7:0]    rx_data,
    input  wire          allow,           // board idle: no run, no dump
    output reg           ld_we,
    output reg  [AW-1:0] ld_addr,
    output reg  [127:0]  ld_data,
    output reg           active,
    output reg           ok  = 1'b0,
    output reg           err = 1'b0
);
    localparam [1:0] S_IDLE = 2'd0, S_DATA = 2'd1, S_CHK = 2'd2;

    reg [1:0]    st;
    reg [3:0]    bcnt;                    // byte inside the 16-byte word
    reg [119:0]  sh;                      // the previous 15 bytes of the word
    reg [AW-1:0] waddr;                   // address of the word being collected
    reg [7:0]    sum;
    reg [31:0]   tmo;

    initial begin
        ld_we = 1'b0; ld_addr = {AW{1'b0}}; ld_data = 128'd0; active = 1'b0;
        st = S_IDLE; bcnt = 4'd0; sh = 120'd0; waddr = {AW{1'b0}}; sum = 8'd0; tmo = 32'd0;
    end

    always @(posedge clk) begin
        ld_we <= 1'b0;
        if (rst) begin
            st <= S_IDLE; active <= 1'b0; bcnt <= 4'd0; waddr <= {AW{1'b0}}; sum <= 8'd0; tmo <= 32'd0;
        end else begin
            case (st)
                S_IDLE: begin
                    tmo <= 32'd0;
                    if (rx_valid && (rx_data == 8'h4C) && allow) begin
                        st <= S_DATA; active <= 1'b1; ok <= 1'b0; err <= 1'b0;
                        bcnt <= 4'd0; waddr <= {AW{1'b0}}; sum <= 8'd0;
                    end
                end

                S_DATA: begin
                    if (rx_valid) begin
                        tmo <= 32'd0;
                        sum <= sum + rx_data;
                        sh  <= {sh[111:0], rx_data};
                        if (bcnt == 4'd15) begin
                            bcnt    <= 4'd0;
                            ld_we   <= 1'b1;
                            ld_addr <= waddr;
                            ld_data <= {sh, rx_data};          // 15 earlier bytes + this one = 128 bits, first byte = MSB
                            if (waddr == DEPTH - 1) st <= S_CHK;
                            else                    waddr <= waddr + 1'b1;
                        end else begin
                            bcnt <= bcnt + 4'd1;
                        end
                    end else if (tmo == TIMEOUT - 1) begin
                        err <= 1'b1; active <= 1'b0; st <= S_IDLE;
                    end else begin
                        tmo <= tmo + 32'd1;
                    end
                end

                S_CHK: begin
                    if (rx_valid) begin
                        if (rx_data == sum) ok <= 1'b1; else err <= 1'b1;
                        active <= 1'b0; st <= S_IDLE;
                    end else if (tmo == TIMEOUT - 1) begin
                        err <= 1'b1; active <= 1'b0; st <= S_IDLE;
                    end else begin
                        tmo <= tmo + 32'd1;
                    end
                end

                default: begin st <= S_IDLE; active <= 1'b0; end
            endcase
        end
    end
endmodule
