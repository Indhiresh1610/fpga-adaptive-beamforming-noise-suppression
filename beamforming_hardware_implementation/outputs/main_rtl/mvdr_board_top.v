// ============================================================================
// mvdr_board_top.v -- Boolean board top for the MVDR beamformer, with a UART
// link that sends the beamformer output (and per-block weights) to the PC.
// Strict Verilog-2001 (MMCME2_BASE / BUFG are Xilinx primitives).
//
// This ONE file replaces the old mvdr_board_top.v and contains 5 modules:
//   mvdr_board_top        - board top: MMCM 100->50 MHz, reset, buttons, LEDs,
//                           and the spot where the Wiener filter will connect
//   mvdr_system           - MVDR core + run sequencer + capture + UART
//                           (kept separate so tb_uplink.v can simulate it
//                            without the MMCM)
//   stream_capture_uplink - passive BRAM capture of the output stream + UART dump
//   uart_tx, uart_rx      - 8N1 UART
// The core files (mvdr_top.v, block_controller.v, ...) are unchanged.
// ============================================================================

`timescale 1ns/1ps

// ============================================================================
// mvdr_board_top.v -- Board-level top, Real Digital Boolean Board (XC7S50-CSGA324)
// Strict Verilog-2001 (MMCME2_BASE / BUFG are Xilinx primitives).
//
// CHANGES vs. the previous version
//   * The core is now wrapped in mvdr_system, which adds a PASSIVE capture of
//     the beamformer output and a UART link to the PC (USB "PROG UART" port).
//   * New ports: uart_rxd (V12), uart_txd (U11), led[5:4].
//   * The beamformer stream (bf_*) is exposed here with a handshake so the
//     Wiener filter can be dropped in later without touching the capture logic.
//   * Checksum now counts accepted beats (valid & ready), so it stays correct
//     once the Wiener filter applies backpressure.
//   * The core clock is still 50 MHz from the MMCM; the solver timing fix is kept.
//
// Ports : clk (100 MHz), btn_rst, btn_start (active high), uart_rxd, uart_txd, led[5:0]
// LEDs  : led[0] busy, led[1] done, led[2] any status flag,
//         led[3] parity of the running checksum (compare with the simulation),
//         led[4] UART dump in progress, led[5] capture overflow
//
// Run: press btn_start, or send 'S' from the PC. The dump starts on its own when
//      the last sample is captured. Send 'D' to send the same run again.
// ============================================================================


module mvdr_board_top #(
    parameter BLOCK_LEN   = 76,
    parameter NUM_SAMPLES = 2047,
    parameter MEM_DEPTH   = 2048,
    parameter MEM_FILE    = "x_mem.mem",   // UNUSED: the file is set in input_mem.v
    parameter real MMCM_MULT = 10.0,   // VCO = 100 MHz * 10 = 1000 MHz
    parameter real MMCM_DIV  = 20.0,   // 1000 / 20 = 50 MHz
    parameter CORE_HZ     = 50000000,  // MUST match 100 MHz * MMCM_MULT / MMCM_DIV
    parameter BAUD        = 115200     // must match the Python script
)(
    input  wire       clk,         // 100 MHz board clock
    input  wire       btn_rst,     // push button, active high
    input  wire       btn_start,   // push button, active high
    input  wire       uart_rxd,    // from the USB-UART bridge (PC -> FPGA)
    output wire       uart_txd,    // to the USB-UART bridge   (FPGA -> PC)
    output wire [5:0] led
);

    localparam CLKS_PER_BIT = (CORE_HZ + BAUD / 2) / BAUD;   // 434 for 50 MHz / 115200

    // ------------------------------------------------------------------------
    // Clock generation: 100 MHz -> core clock (50 MHz)
    // ------------------------------------------------------------------------
    wire clk_fb;
    wire clk_core_unbuf;
    wire clk_core;
    wire mmcm_locked;

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.0),
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (MMCM_MULT),
        .CLKOUT0_DIVIDE_F   (MMCM_DIV),
        .CLKFBOUT_PHASE     (0.0),
        .CLKOUT0_PHASE      (0.0),
        .CLKOUT0_DUTY_CYCLE (0.5),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKIN1    (clk),
        .CLKFBIN   (clk_fb),
        .CLKFBOUT  (clk_fb),
        .CLKFBOUTB (),
        .CLKOUT0   (clk_core_unbuf),
        .CLKOUT0B  (),
        .CLKOUT1   (),
        .CLKOUT1B  (),
        .CLKOUT2   (),
        .CLKOUT2B  (),
        .CLKOUT3   (),
        .CLKOUT3B  (),
        .CLKOUT4   (),
        .CLKOUT5   (),
        .CLKOUT6   (),
        .LOCKED    (mmcm_locked),
        .PWRDWN    (1'b0),
        .RST       (1'b0)
    );

    BUFG u_bufg_core (
        .I (clk_core_unbuf),
        .O (clk_core)
    );

    // ------------------------------------------------------------------------
    // Reset: held while the MMCM is unlocked, then 16 more cycles, and on button
    // ------------------------------------------------------------------------
    reg [1:0] lock_sync = 2'b00;
    always @(posedge clk_core) begin
        lock_sync <= {lock_sync[0], mmcm_locked};
    end
    wire core_ok = lock_sync[1];

    reg [4:0] por_cnt = 5'd0;
    wire      por     = ~por_cnt[4];

    always @(posedge clk_core) begin
        if (!core_ok)
            por_cnt <= 5'd0;
        else if (por)
            por_cnt <= por_cnt + 5'd1;
    end

    reg [1:0] rst_sync   = 2'b00;
    reg [1:0] start_sync = 2'b00;
    reg       start_d    = 1'b0;

    always @(posedge clk_core) begin
        rst_sync   <= {rst_sync[0],   btn_rst};
        start_sync <= {start_sync[0], btn_start};
        start_d    <= start_sync[1];
    end

    reg rst = 1'b1;
    always @(posedge clk_core) begin
        rst <= por | rst_sync[1] | ~core_ok;
    end

    wire start_pulse = start_sync[1] & ~start_d;

    // ------------------------------------------------------------------------
    // Beamformer output stream
    // ------------------------------------------------------------------------
    wire         bf_valid;
    wire [47:0]  bf_data;
    wire [15:0]  bf_idx;
    wire [7:0]   bf_blk;
    wire         bf_is_tail;
    wire         bf_last;
    wire         bf_ready;

    // ======================== WIENER FILTER GOES HERE ========================
    // Today nothing consumes the stream, so it is always accepted.
    // When the Wiener filter is added, delete the line below and connect:
    //     wiener_top u_wf (
    //         .clk(clk_core), .rst(rst),
    //         .s_valid(bf_valid), .s_data(bf_data), .s_idx(bf_idx), .s_last(bf_last),
    //         .s_ready(bf_ready), ... );
    // The capture inside mvdr_system records exactly the beats the Wiener
    // filter accepts (bf_valid & bf_ready) and never stalls it.
    assign bf_ready = 1'b1;
    // =========================================================================

    wire       busy;
    wire       done;
    wire [7:0] status;
    wire       dumping;
    wire       cap_overflow;
    wire       run_arm;

    mvdr_system #(
        .BLOCK_LEN    (BLOCK_LEN),
        .NUM_SAMPLES  (NUM_SAMPLES),
        .MEM_DEPTH    (MEM_DEPTH),
        .MEM_FILE     (MEM_FILE),
        .CLKS_PER_BIT (CLKS_PER_BIT)
    ) u_sys (
        .clk          (clk_core),
        .rst          (rst),
        .start_req    (start_pulse),
        .uart_rxd     (uart_rxd),
        .uart_txd     (uart_txd),
        .bf_valid     (bf_valid),
        .bf_data      (bf_data),
        .bf_idx       (bf_idx),
        .bf_blk       (bf_blk),
        .bf_is_tail   (bf_is_tail),
        .bf_last      (bf_last),
        .bf_ready     (bf_ready),
        .busy         (busy),
        .done         (done),
        .status       (status),
        .dumping      (dumping),
        .cap_overflow (cap_overflow),
        .run_arm      (run_arm)
    );

    // ------------------------------------------------------------------------
    // Running checksum over every ACCEPTED output beat (same word as before)
    // ------------------------------------------------------------------------
    wire [74:0] out_word = {bf_is_tail, bf_last, bf_blk, bf_idx, bf_data, 1'b0};

    reg [74:0] checksum = 75'd0;
    always @(posedge clk_core) begin
        if (rst || run_arm)
            checksum <= 75'd0;
        else if (bf_valid && bf_ready)
            checksum <= {checksum[73:0], checksum[74]} ^ out_word;
    end

    // ------------------------------------------------------------------------
    // LEDs
    // ------------------------------------------------------------------------
    assign led[0] = busy;
    assign led[1] = done;
    assign led[2] = |status;
    assign led[3] = ^checksum;
    assign led[4] = dumping;
    assign led[5] = cap_overflow;

endmodule


// ============================================================================
// mvdr_system.v -- MVDR core + run sequencer + passive output capture + UART.
// Strict Verilog-2001. Single clock domain (the core clock).
//
//   PC --'S'--> uart_rx --> run sequencer --> (core reset, then start)
//                                                     |
//   mvdr_top m_* ---------------------------------> bf_* ports  --> Wiener filter
//                 \                                   (bf_ready comes back from it)
//                  \--(observe m_valid & bf_ready)--> stream_capture_uplink --> uart_txd --> PC
//
// * The beamformer stream is exported unchanged on bf_*; bf_ready from the
//   consumer (Wiener filter, or 1'b1 today) drives the core's m_ready.
// * The capture only observes accepted beats. It never touches bf_ready, so it
//   cannot affect the Wiener filter or the core.
// * Every run (button or 'S') first resets the core for 16 cycles. This matters:
//   without a reset, a second 'start' would reuse the previous run's covariance
//   R and the last weights instead of a/4, and the output would no longer match
//   the simulation / Python reference.
//
// UART commands (PC -> FPGA, single ASCII byte):
//   'S' / 's' : reset core + start a run (dump follows automatically)
//   'D' / 'd' : re-send the last captured run
// ============================================================================


module mvdr_system #(
    parameter BLOCK_LEN    = 76,
    parameter NUM_SAMPLES  = 2047,
    parameter MEM_DEPTH    = 2048,
    parameter MEM_FILE     = "x_mem.mem",   // UNUSED: the file is set in input_mem.v
    parameter CLKS_PER_BIT = 434         // core_clk / baud
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start_req,       // 1-cycle pulse (push button)

    // UART to the PC
    input  wire         uart_rxd,
    output wire         uart_txd,

    // Beamformer output stream (to the Wiener filter)
    output wire         bf_valid,
    output wire [47:0]  bf_data,         // {im[23:0], re[23:0]} Q2.22
    output wire [15:0]  bf_idx,
    output wire [7:0]   bf_blk,
    output wire         bf_is_tail,
    output wire         bf_last,
    input  wire         bf_ready,

    // Status
    output wire         busy,
    output wire         done,
    output wire [7:0]   status,
    output wire         dumping,
    output wire         cap_overflow,
    output wire         run_arm          // 1-cycle pulse when a new run begins
);

    // ------------------------------------------------------------------------
    // UART receiver + command decode
    // ------------------------------------------------------------------------
    wire       rx_valid;
    wire [7:0] rx_data;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk      (clk),
        .rst      (rst),
        .rxd      (uart_rxd),
        .rx_valid (rx_valid),
        .rx_data  (rx_data)
    );

    wire cmd_start = rx_valid && ((rx_data == 8'h53) || (rx_data == 8'h73)); // 'S' 's'
    wire cmd_dump  = rx_valid && ((rx_data == 8'h44) || (rx_data == 8'h64)); // 'D' 'd'

    // ------------------------------------------------------------------------
    // Run sequencer: soft reset (16 cycles) -> wait for core ready -> start
    // ------------------------------------------------------------------------
    wire [4:0]   dbg_state;
    wire [199:0] dbg_w_active;
    wire         dbg_w_swap;
    wire [31:0]  dbg_solve_cycles;

    localparam [2:0] Q_IDLE = 3'd0;
    localparam [2:0] Q_RST  = 3'd1;
    localparam [2:0] Q_WAIT = 3'd2;
    localparam [2:0] Q_DLY  = 3'd3;
    localparam [2:0] Q_GO   = 3'd4;

    localparam [4:0] CORE_WAIT_START = 5'd2;   // block_controller ST_WAIT_START

    reg [2:0] q;
    reg [4:0] qcnt;
    reg       soft_rst;
    reg       core_start;
    reg       cap_arm;

    always @(posedge clk) begin
        if (rst) begin
            q          <= Q_IDLE;
            qcnt       <= 5'd0;
            soft_rst   <= 1'b0;
            core_start <= 1'b0;
            cap_arm    <= 1'b0;
        end else begin
            core_start <= 1'b0;
            cap_arm    <= 1'b0;
            case (q)
                Q_IDLE: begin
                    if ((start_req || cmd_start) && !dumping) begin
                        soft_rst <= 1'b1;
                        cap_arm  <= 1'b1;      // clear the capture buffer
                        qcnt     <= 5'd0;
                        q        <= Q_RST;
                    end
                end
                Q_RST: begin
                    if (qcnt == 5'd15) begin
                        soft_rst <= 1'b0;
                        qcnt     <= 5'd0;
                        q        <= Q_WAIT;
                    end else begin
                        qcnt <= qcnt + 5'd1;
                    end
                end
                Q_WAIT: begin
                    // a_vec captured and quiescent weights loaded
                    if (dbg_state == CORE_WAIT_START) begin
                        qcnt <= 5'd0;
                        q    <= Q_DLY;
                    end
                end
                Q_DLY: begin
                    if (qcnt == 5'd7) q <= Q_GO;
                    else              qcnt <= qcnt + 5'd1;
                end
                Q_GO: begin
                    core_start <= 1'b1;
                    q          <= Q_IDLE;
                end
                default: q <= Q_IDLE;
            endcase
        end
    end

    wire core_rst = rst | soft_rst;
    assign run_arm = cap_arm;

    // ------------------------------------------------------------------------
    // MVDR core (unchanged)
    // ------------------------------------------------------------------------
    (* DONT_TOUCH = "true" *)
    mvdr_top #(
        .BLOCK_LEN   (BLOCK_LEN),
        .NUM_SAMPLES (NUM_SAMPLES),
        .MEM_DEPTH   (MEM_DEPTH),
        .MEM_FILE    (MEM_FILE),
        .PACED       (0)
    ) u_core (
        .clk              (clk),
        .rst              (core_rst),
        .start            (core_start),
        .sample_en        (1'b0),
        .m_ready          (bf_ready),

        .m_valid          (bf_valid),
        .m_data           (bf_data),
        .m_last           (bf_last),
        .m_idx            (bf_idx),
        .m_blk            (bf_blk),
        .m_is_tail        (bf_is_tail),
        .busy             (busy),
        .done             (done),
        .status           (status),

        .dbg_w_active     (dbg_w_active),
        .dbg_w_swap       (dbg_w_swap),
        .dbg_blk_done     (),
        .dbg_solve_cycles (dbg_solve_cycles),
        .dbg_state        (dbg_state)
    );

    // ------------------------------------------------------------------------
    // Passive capture of accepted beats + weight snapshots
    //   weight entry 0      : quiescent a/4, taken at the start pulse (used in block 0)
    //   weight entry k >= 1 : taken on each w_swap (used in block k; the last
    //                         one is also used for the tail)
    // dbg_w_swap is registered on the same edge that loads w_active, so
    // dbg_w_active already holds the new weights while dbg_w_swap is high.
    // ------------------------------------------------------------------------
    wire [73:0] tap_word = {bf_is_tail, bf_last, bf_blk, bf_idx, bf_data};

    stream_capture_uplink #(
        .CLKS_PER_BIT (CLKS_PER_BIT),
        .CAP_DEPTH    (2048),
        .CAP_AW       (11),
        .W_DEPTH      (32),
        .W_AW         (5)
    ) u_cap (
        .clk             (clk),
        .rst             (rst),
        .arm             (cap_arm),
        .tap_valid       (bf_valid & bf_ready),
        .tap_word        (tap_word),
        .w_cap           (core_start | dbg_w_swap),
        .w_word          (dbg_w_active),
        .dump_req        (cmd_dump),
        .status_in       (status),
        .solve_cycles_in (dbg_solve_cycles),
        .uart_txd        (uart_txd),
        .dumping         (dumping),
        .overflow        (cap_overflow),
        .n_captured      ()
    );

endmodule


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
