// ============================================================================
// uart_capture_lib.v -- stream_capture_uplink, uart_tx, uart_rx.
// Copied UNCHANGED from the beamforming team's mvdr_board_top.v (their file also holds the beamformer board
// top, which is not needed for a Wiener-only build).
// ============================================================================
`timescale 1ns/1ps

// ============================================================================
// stream_capture_uplink.v -- PASSIVE capture of the beamformer output stream
// into block RAM, followed by a framed binary dump over UART.
// Strict Verilog-2001.
//
// WHY PASSIVE: the beamformer output is consumed by the downstream Wiener
// filter. This module only *observes* the stream (tap_valid must be driven by
// m_valid & m_ready, i.e. the beats the consumer actually accepted). It has no
// ready output, so it can never stall, drop or reorder the real data path.
// The core runs at full speed; the slow UART dump happens afterwards.
//
// Captured per beat (tap_word, 74 bits):
//   [47:0]  m_data  = {im[23:0], re[23:0]}  (Q2.22)
//   [63:48] m_idx
//   [71:64] m_blk
//   [72]    m_last
//   [73]    m_is_tail
// Captured per weight event (w_word, 200 bits): w_active
//   {w3_im, w3_re, w2_im, w2_re, w1_im, w1_re, w0_im, w0_re}, each Q5.20 (25 b)
//
// Dump is triggered automatically when the beat with m_last=1 is captured,
// or by a dump_req pulse (e.g. a 'D' command from the PC).
//
// ---------------------------------------------------------------------------
// UART FRAME FORMAT (every record):
//   0xA5, 0x5A, TYPE, PAYLOAD[LEN] (little-endian), CHK
//   CHK = XOR of TYPE and all PAYLOAD bytes
//
//   TYPE 'H' (0x48) LEN 10 : version(1)=1, n_y(2), n_w(1), status(1),
//                            solve_cycles(4), flags(1: bit0 overflow)
//   TYPE 'Y' (0x59) LEN 10 : {6'b0, tap_word[73:0]}           (one per beat)
//   TYPE 'W' (0x57) LEN 26 : entry(1), w_word[199:0]           (one per event)
//   TYPE 'E' (0x45) LEN  3 : n_y(2), n_w(1)                    (end marker)
// Order: H, Y x n_y, W x n_w, E
// ============================================================================


module stream_capture_uplink #(
    parameter CLKS_PER_BIT = 434,
    parameter CAP_DEPTH    = 2048,   // max beats stored
    parameter CAP_AW       = 11,     // log2(CAP_DEPTH)
    parameter W_DEPTH      = 32,     // max weight snapshots stored
    parameter W_AW         = 5       // log2(W_DEPTH), must be <= 7
)(
    input  wire              clk,
    input  wire              rst,

    input  wire              arm,            // pulse: clear buffer before a new run
    input  wire              tap_valid,      // = m_valid & m_ready of the observed stream
    input  wire [73:0]       tap_word,
    input  wire              w_cap,          // pulse: snapshot w_word
    input  wire [199:0]      w_word,
    input  wire              dump_req,       // pulse: (re)send the buffer

    input  wire [7:0]        status_in,      // sampled into the header at dump start
    input  wire [31:0]       solve_cycles_in,

    output wire              uart_txd,
    output wire              dumping,
    output reg               overflow,       // a beat or snapshot could not be stored
    output wire [CAP_AW:0]   n_captured
);

    localparam LAST_BIT = 72;

    // ------------------------------------------------------------------------
    // Input register stage (keeps the tap off any core timing path)
    // ------------------------------------------------------------------------
    reg          tap_v_r;
    reg  [73:0]  tap_w_r;
    reg          wcap_r;
    reg  [199:0] w_r;

    always @(posedge clk) begin
        if (rst) begin
            tap_v_r <= 1'b0;
            wcap_r  <= 1'b0;
        end else begin
            tap_v_r <= tap_valid;
            wcap_r  <= w_cap;
        end
        tap_w_r <= tap_word;
        w_r     <= w_word;
    end

    // ------------------------------------------------------------------------
    // Storage
    // ------------------------------------------------------------------------
    (* ram_style = "block" *) reg [79:0]  cap_mem [0:CAP_DEPTH-1];
    reg [199:0] w_mem [0:W_DEPTH-1];

    reg [CAP_AW:0] wr_ptr;
    reg [W_AW:0]   w_ptr;
    reg            pend_dump;

    reg [3:0]      dstate;
    localparam D_IDLE = 4'd0;
    localparam D_SEND = 4'd1;
    localparam D_NEXT = 4'd2;
    localparam D_YRD0 = 4'd3;
    localparam D_YRD1 = 4'd4;
    localparam D_YLD  = 4'd5;
    localparam D_WRD0 = 4'd6;
    localparam D_WRD1 = 4'd7;
    localparam D_WLD  = 4'd8;
    localparam D_END  = 4'd9;

    assign dumping    = (dstate != D_IDLE);
    assign n_captured = wr_ptr;

    wire cap_we = tap_v_r && !arm && !dumping && (wr_ptr < CAP_DEPTH);
    wire w_we   = wcap_r  && !arm && !dumping && (w_ptr  < W_DEPTH);

    always @(posedge clk) begin
        if (cap_we) cap_mem[wr_ptr[CAP_AW-1:0]] <= {6'd0, tap_w_r};
    end

    always @(posedge clk) begin
        if (w_we) w_mem[w_ptr[W_AW-1:0]] <= w_r;
    end

    reg [CAP_AW-1:0] cap_ra;
    reg [79:0]       cap_q;
    reg [W_AW-1:0]   w_ra;
    reg [199:0]      w_q;

    always @(posedge clk) begin
        cap_q <= cap_mem[cap_ra];
        w_q   <= w_mem[w_ra];
    end

    wire dump_start = (dstate == D_IDLE) && (dump_req || pend_dump);

    // ------------------------------------------------------------------------
    // Write pointers and flags
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            wr_ptr    <= {(CAP_AW+1){1'b0}};
            w_ptr     <= {(W_AW+1){1'b0}};
            overflow  <= 1'b0;
            pend_dump <= 1'b0;
        end else if (arm) begin
            wr_ptr    <= {(CAP_AW+1){1'b0}};
            w_ptr     <= {(W_AW+1){1'b0}};
            overflow  <= 1'b0;
            pend_dump <= 1'b0;
        end else begin
            if (tap_v_r) begin
                if (cap_we) begin
                    wr_ptr <= wr_ptr + 1'b1;
                    if (tap_w_r[LAST_BIT]) pend_dump <= 1'b1;
                end else begin
                    overflow <= 1'b1;
                end
            end
            if (wcap_r) begin
                if (w_we) w_ptr    <= w_ptr + 1'b1;
                else      overflow <= 1'b1;
            end
            if (dump_start) pend_dump <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // UART transmitter
    // ------------------------------------------------------------------------
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk      (clk),
        .rst      (rst),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx_busy  (tx_busy),
        .txd      (uart_txd)
    );

    // ------------------------------------------------------------------------
    // Dump FSM
    // ------------------------------------------------------------------------
    localparam [1:0] R_HDR = 2'd0;
    localparam [1:0] R_Y   = 2'd1;
    localparam [1:0] R_W   = 2'd2;
    localparam [1:0] R_END = 2'd3;

    localparam [7:0] T_HDR = 8'h48;   // 'H'
    localparam [7:0] T_Y   = 8'h59;   // 'Y'
    localparam [7:0] T_W   = 8'h57;   // 'W'
    localparam [7:0] T_END = 8'h45;   // 'E'

    reg [1:0]      rkind;
    reg [7:0]      rtype;
    reg [207:0]    pl;          // payload shift register, LSB byte goes first
    reg [5:0]      pl_len;      // payload length in bytes
    reg [5:0]      bpos;        // byte position inside the record
    reg [1:0]      bstate;      // 0 issue, 1 wait busy high, 2 wait busy low
    reg [7:0]      chk;

    reg [CAP_AW:0] n_y;
    reg [CAP_AW:0] y_i;
    reg [W_AW:0]   n_w;
    reg [W_AW:0]   w_i;

    wire [15:0] n_y16 = {{(15-CAP_AW){1'b0}}, n_y};
    wire [7:0]  n_w8  = {{(7-W_AW){1'b0}}, n_w};
    wire [7:0]  w_i8  = {{(7-W_AW){1'b0}}, w_i};
    wire [15:0] wr16  = {{(15-CAP_AW){1'b0}}, wr_ptr};
    wire [7:0]  wp8   = {{(7-W_AW){1'b0}}, w_ptr};

    wire [5:0]  last_pos = pl_len + 6'd3;   // position of the CHK byte

    always @(posedge clk) begin
        if (rst) begin
            dstate   <= D_IDLE;
            rkind    <= R_HDR;
            rtype    <= 8'd0;
            pl       <= 208'd0;
            pl_len   <= 6'd0;
            bpos     <= 6'd0;
            bstate   <= 2'd0;
            chk      <= 8'd0;
            n_y      <= {(CAP_AW+1){1'b0}};
            y_i      <= {(CAP_AW+1){1'b0}};
            n_w      <= {(W_AW+1){1'b0}};
            w_i      <= {(W_AW+1){1'b0}};
            cap_ra   <= {CAP_AW{1'b0}};
            w_ra     <= {W_AW{1'b0}};
            tx_start <= 1'b0;
            tx_data  <= 8'd0;
        end else begin
            case (dstate)
                // ------------------------------------------------------------
                D_IDLE: begin
                    tx_start <= 1'b0;
                    if (dump_start) begin
                        n_y    <= wr_ptr;
                        n_w    <= w_ptr;
                        // header: {flags, solve_cycles, status, n_w, n_y, version}
                        pl     <= {128'd0, {7'd0, overflow}, solve_cycles_in,
                                   status_in, wp8, wr16, 8'h01};
                        pl_len <= 6'd10;
                        rtype  <= T_HDR;
                        rkind  <= R_HDR;
                        bpos   <= 6'd0;
                        bstate <= 2'd0;
                        dstate <= D_SEND;
                    end
                end

                // ------------------------------------------------------------
                // Send one record: A5 5A TYPE PAYLOAD CHK
                // ------------------------------------------------------------
                D_SEND: begin
                    case (bstate)
                        2'd0: begin
                            if (bpos == 6'd0) begin
                                tx_data <= 8'hA5;
                            end else if (bpos == 6'd1) begin
                                tx_data <= 8'h5A;
                            end else if (bpos == 6'd2) begin
                                tx_data <= rtype;
                                chk     <= rtype;
                            end else if (bpos < last_pos) begin
                                tx_data <= pl[7:0];
                                chk     <= chk ^ pl[7:0];
                                pl      <= {8'd0, pl[207:8]};
                            end else begin
                                tx_data <= chk;
                            end
                            tx_start <= 1'b1;
                            bstate   <= 2'd1;
                        end
                        2'd1: begin
                            tx_start <= 1'b0;     // uart_tx latched it this edge
                            bstate   <= 2'd2;
                        end
                        default: begin
                            if (!tx_busy) begin
                                bstate <= 2'd0;
                                if (bpos == last_pos) begin
                                    bpos   <= 6'd0;
                                    dstate <= D_NEXT;
                                end else begin
                                    bpos <= bpos + 6'd1;
                                end
                            end
                        end
                    endcase
                end

                // ------------------------------------------------------------
                D_NEXT: begin
                    case (rkind)
                        R_HDR: begin
                            if (n_y != 0) begin
                                y_i <= {(CAP_AW+1){1'b0}}; dstate <= D_YRD0;
                            end else if (n_w != 0) begin
                                w_i <= {(W_AW+1){1'b0}};   dstate <= D_WRD0;
                            end else begin
                                dstate <= D_END;
                            end
                        end
                        R_Y: begin
                            if (y_i + 1'b1 < n_y) begin
                                y_i <= y_i + 1'b1; dstate <= D_YRD0;
                            end else if (n_w != 0) begin
                                w_i <= {(W_AW+1){1'b0}}; dstate <= D_WRD0;
                            end else begin
                                dstate <= D_END;
                            end
                        end
                        R_W: begin
                            if (w_i + 1'b1 < n_w) begin
                                w_i <= w_i + 1'b1; dstate <= D_WRD0;
                            end else begin
                                dstate <= D_END;
                            end
                        end
                        default: dstate <= D_IDLE;   // R_END finished
                    endcase
                end

                // ------------------------------------------------------------
                // Beat records (2-cycle registered BRAM read)
                // ------------------------------------------------------------
                D_YRD0: begin cap_ra <= y_i[CAP_AW-1:0]; dstate <= D_YRD1; end
                D_YRD1: begin dstate <= D_YLD; end
                D_YLD: begin
                    pl     <= {128'd0, cap_q};
                    pl_len <= 6'd10;
                    rtype  <= T_Y;
                    rkind  <= R_Y;
                    bpos   <= 6'd0;
                    bstate <= 2'd0;
                    dstate <= D_SEND;
                end

                // ------------------------------------------------------------
                // Weight records
                // ------------------------------------------------------------
                D_WRD0: begin w_ra <= w_i[W_AW-1:0]; dstate <= D_WRD1; end
                D_WRD1: begin dstate <= D_WLD; end
                D_WLD: begin
                    pl     <= {w_q, w_i8};
                    pl_len <= 6'd26;
                    rtype  <= T_W;
                    rkind  <= R_W;
                    bpos   <= 6'd0;
                    bstate <= 2'd0;
                    dstate <= D_SEND;
                end

                // ------------------------------------------------------------
                D_END: begin
                    pl     <= {184'd0, n_w8, n_y16};
                    pl_len <= 6'd3;
                    rtype  <= T_END;
                    rkind  <= R_END;
                    bpos   <= 6'd0;
                    bstate <= 2'd0;
                    dstate <= D_SEND;
                end

                default: dstate <= D_IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// uart_tx.v -- 8N1 UART transmitter (LSB first). Strict Verilog-2001.
//
// Handshake: pulse tx_start for one cycle while tx_busy == 0. tx_busy goes high
// on the next clock and stays high until the stop bit has been fully sent.
// CLKS_PER_BIT = round(f_clk / baud), e.g. 50 MHz / 115200 = 434.
// ============================================================================


module uart_tx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_busy,
    output reg        txd          // idle high
);

    reg [9:0]  sh;                 // {stop, data[7:0], start}
    reg [3:0]  bit_cnt;            // number of bits already completed
    reg [15:0] clk_cnt;

    always @(posedge clk) begin
        if (rst) begin
            txd     <= 1'b1;
            tx_busy <= 1'b0;
            sh      <= 10'h3FF;
            bit_cnt <= 4'd0;
            clk_cnt <= 16'd0;
        end else if (!tx_busy) begin
            txd <= 1'b1;
            if (tx_start) begin
                sh      <= {1'b1, tx_data, 1'b0};
                txd     <= 1'b0;           // start bit goes out immediately
                tx_busy <= 1'b1;
                bit_cnt <= 4'd0;
                clk_cnt <= 16'd0;
            end
        end else begin
            if (clk_cnt == CLKS_PER_BIT - 1) begin
                clk_cnt <= 16'd0;
                if (bit_cnt == 4'd9) begin
                    // stop bit has been on the line for a full bit time
                    tx_busy <= 1'b0;
                    txd     <= 1'b1;
                end else begin
                    txd     <= sh[1];
                    sh      <= {1'b1, sh[9:1]};
                    bit_cnt <= bit_cnt + 4'd1;
                end
            end else begin
                clk_cnt <= clk_cnt + 16'd1;
            end
        end
    end

endmodule


// ============================================================================
// uart_rx.v -- 8N1 UART receiver (LSB first). Strict Verilog-2001.
//
// rxd is asynchronous: it passes through a 2-flop synchronizer first.
// Samples each bit in its middle. rx_valid is a 1-cycle pulse with rx_data,
// produced only when the stop bit is high (framing errors are dropped).
// ============================================================================


module uart_rx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rxd,
    output reg        rx_valid,
    output reg  [7:0] rx_data
);

    (* ASYNC_REG = "TRUE" *) reg rx_s1 = 1'b1;
    (* ASYNC_REG = "TRUE" *) reg rx_s2 = 1'b1;

    always @(posedge clk) begin
        rx_s1 <= rxd;
        rx_s2 <= rx_s1;
    end

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  st;
    reg [15:0] cnt;
    reg [2:0]  bi;
    reg [7:0]  sh;

    always @(posedge clk) begin
        if (rst) begin
            st       <= S_IDLE;
            cnt      <= 16'd0;
            bi       <= 3'd0;
            sh       <= 8'd0;
            rx_valid <= 1'b0;
            rx_data  <= 8'd0;
        end else begin
            rx_valid <= 1'b0;
            case (st)
                S_IDLE: begin
                    cnt <= 16'd0;
                    bi  <= 3'd0;
                    if (!rx_s2) st <= S_START;
                end

                // wait to the middle of the start bit and confirm it is still low
                S_START: begin
                    if (cnt == (CLKS_PER_BIT / 2) - 1) begin
                        cnt <= 16'd0;
                        st  <= (!rx_s2) ? S_DATA : S_IDLE;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (cnt == CLKS_PER_BIT - 1) begin
                        cnt <= 16'd0;
                        sh  <= {rx_s2, sh[7:1]};
                        if (bi == 3'd7) st <= S_STOP;
                        else            bi <= bi + 3'd1;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    if (cnt == CLKS_PER_BIT - 1) begin
                        cnt <= 16'd0;
                        st  <= S_IDLE;
                        if (rx_s2) begin
                            rx_valid <= 1'b1;
                            rx_data  <= sh;
                        end
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
