// ============================================================================
// mvdr_wiener_board_top.v -- Boolean board top:  team MVDR (UNCHANGED)  ->  Wiener post-filter  ->  UART capture.
// Strict Verilog-2001 (MMCME2_BASE / BUFG are Xilinx primitives).
//
// The beamformer files (mvdr_top, block_controller, sample_engine, block_solver, weight_regs, output_interface,
// serial_divider, input_mem) are used exactly as the beamforming team delivered them.  This file also uses two more
// of their modules unchanged: uart_rx and stream_capture_uplink (both live inside their mvdr_board_top.v, which you
// add to the project too; its own top-level module mvdr_board_top is simply not used).
//
// What this file does differently from the team's mvdr_board_top / mvdr_system:
//   * same MMCM (100 -> 50 MHz), reset, button and LED code, same PORT NAMES -> their boolean_pins.xdc works as is
//   * same run sequencer ('S' / button: 16-cycle soft reset, wait for ST_WAIT_START, start)
//   * the beamformer stream goes into the Wiener filter instead of being captured
//   * the UART capture records the WIENER output (or the raw beamformer stream if TAP_RAW = 1)
// ============================================================================
`timescale 1ns/1ps

module mvdr_wiener_board_top #(
    parameter BLOCK_LEN   = 76,
    parameter NUM_SAMPLES = 2047,
    parameter MEM_DEPTH   = 2048,
    parameter MEM_FILE    = "x_mem.mem",   // UNUSED by the core: the file is set in input_mem.v
    parameter TAP_RAW     = 0,             // 0: UART dump = Wiener output, 1: UART dump = raw beamformer output
    parameter real MMCM_MULT = 10.0,
    parameter real MMCM_DIV  = 20.0,
    parameter CORE_HZ     = 50000000,
    parameter BAUD        = 115200
)(
    input  wire       clk,
    input  wire       btn_rst,
    input  wire       btn_start,
    input  wire       uart_rxd,
    output wire       uart_txd,
    output wire [5:0] led
);
    localparam CLKS_PER_BIT = (CORE_HZ + BAUD / 2) / BAUD;

    // ---------------- clock: identical to the team's board top ----------------
    wire clk_fb, clk_core_unbuf, clk_core, mmcm_locked;
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(1),
        .CLKFBOUT_MULT_F(MMCM_MULT), .CLKOUT0_DIVIDE_F(MMCM_DIV),
        .CLKFBOUT_PHASE(0.0), .CLKOUT0_PHASE(0.0), .CLKOUT0_DUTY_CYCLE(0.5), .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKIN1(clk), .CLKFBIN(clk_fb), .CLKFBOUT(clk_fb), .CLKFBOUTB(),
        .CLKOUT0(clk_core_unbuf), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .LOCKED(mmcm_locked), .PWRDWN(1'b0), .RST(1'b0));
    BUFG u_bufg_core (.I(clk_core_unbuf), .O(clk_core));

    // ---------------- reset / start buttons: identical to the team's board top ----------------
    reg [1:0] lock_sync = 2'b00;
    always @(posedge clk_core) lock_sync <= {lock_sync[0], mmcm_locked};
    wire core_ok = lock_sync[1];

    reg [4:0] por_cnt = 5'd0;
    wire      por     = ~por_cnt[4];
    always @(posedge clk_core) begin
        if (!core_ok)  por_cnt <= 5'd0;
        else if (por)  por_cnt <= por_cnt + 5'd1;
    end

    reg [1:0] rst_sync = 2'b00, start_sync = 2'b00;
    reg       start_d  = 1'b0;
    always @(posedge clk_core) begin
        rst_sync   <= {rst_sync[0],   btn_rst};
        start_sync <= {start_sync[0], btn_start};
        start_d    <= start_sync[1];
    end
    reg rst = 1'b1;
    always @(posedge clk_core) rst <= por | rst_sync[1] | ~core_ok;
    wire start_pulse = start_sync[1] & ~start_d;

    wire       busy, done, dumping, cap_overflow, wf_done, ld_err;
    wire [7:0] status, wf_status;

    mvdr_wiener_system #(
        .BLOCK_LEN(BLOCK_LEN), .NUM_SAMPLES(NUM_SAMPLES), .MEM_DEPTH(MEM_DEPTH),
        .MEM_FILE(MEM_FILE), .CLKS_PER_BIT(CLKS_PER_BIT), .TAP_RAW(TAP_RAW)
    ) u_sys (
        .clk(clk_core), .rst(rst), .start_req(start_pulse),
        .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .busy(busy), .done(done), .status(status), .wf_status(wf_status), .wf_done(wf_done),
        .dumping(dumping), .cap_overflow(cap_overflow), .ld_err(ld_err));

    // LEDs: 0 beamformer busy, 1 beamformer done, 2 any beamformer status flag or Wiener saturation flag,
    //       (also: a failed case load), 3 Wiener finished (all samples captured), 4 UART dump in progress, 5 capture overflow
    assign led[0] = busy;
    assign led[1] = done;
    assign led[2] = (|status) | (|wf_status[3:1]) | ld_err;     // ld_err: the last UART case load failed
    assign led[3] = wf_done;
    assign led[4] = dumping;
    assign led[5] = cap_overflow;
endmodule


// ============================================================================
// mvdr_wiener_system -- run sequencer + mvdr_top (unchanged) + Wiener + capture/UART.   Core clock domain only.
// UART commands (same as the team's):  'S'/'s' reset + start a run (dump follows by itself),  'D'/'d' re-send.
// Added: 'L' + 32768 bytes + checksum loads a new input case into input_mem (see case_loader.v).  'S' / 'D' bytes are
// ignored while a load is running, and a run is refused after a failed load.
//
// Wiener stage (between mvdr_top and the capture):
//   * block 0 (m_blk == 0, quiescent weights, interferers not yet nulled) is accepted and discarded
//   * after the LAST beamformer beat, FLUSH_N zero samples are pushed so the Wiener's 48-sample latency drains
//   * the Wiener's first LAT (=48) outputs are warm-up and are not captured; the next NUM_SAMPLES-BLOCK_LEN outputs
//     are captured with idx = BLOCK_LEN, BLOCK_LEN+1, ... and the matching m_blk, so they line up 1:1 with the
//     beamformer samples of blocks >= 1 (same record format as the team's 'Y' records)
// ============================================================================
module mvdr_wiener_system #(
    parameter BLOCK_LEN    = 76,
    parameter NUM_SAMPLES  = 2047,
    parameter MEM_DEPTH    = 2048,
    parameter MEM_FILE     = "x_mem.mem",
    parameter CLKS_PER_BIT = 434,
    parameter TAP_RAW      = 0,
    parameter FLUSH_N      = 80,           // >= 48 latency + one hop of 16
    parameter LAT          = 48            // Wiener latency = FRAME - HOP
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start_req,
    input  wire        uart_rxd,
    output wire        uart_txd,
    output wire        busy,
    output wire        done,
    output wire [7:0]  status,
    output wire [7:0]  wf_status,
    output wire        wf_done,
    output wire        dumping,
    output wire        cap_overflow,
    output wire        ld_err           // last case load failed (checksum / timeout); runs are refused until a good load
);
    localparam NOUT  = NUM_SAMPLES - BLOCK_LEN;          // Wiener samples to capture (1971)
    localparam NFULL = NUM_SAMPLES / BLOCK_LEN;          // block number of the tail block (26)
    localparam HAS_TAIL = ((NUM_SAMPLES % BLOCK_LEN) != 0);

    // ------------------------------------------------------------------------
    // UART receiver + command decode (as in mvdr_system)
    // ------------------------------------------------------------------------
    wire       rx_valid;
    wire [7:0] rx_data;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rxd(uart_rxd), .rx_valid(rx_valid), .rx_data(rx_data));
    wire ld_active, ld_ok, ld_we;
    wire [10:0]  ld_addr;
    wire [127:0] ld_data;
    wire cmd_start = rx_valid && !ld_active && ((rx_data == 8'h53) || (rx_data == 8'h73));
    wire cmd_dump  = rx_valid && !ld_active && ((rx_data == 8'h44) || (rx_data == 8'h64));

    // ------------------------------------------------------------------------
    // Run sequencer (same as mvdr_system): soft reset 16 cycles -> wait for core ready -> start
    // ------------------------------------------------------------------------
    wire [4:0]   dbg_state;
    wire [199:0] dbg_w_active;
    wire         dbg_w_swap;
    wire [31:0]  dbg_solve_cycles;

    localparam [2:0] Q_IDLE = 3'd0, Q_RST = 3'd1, Q_WAIT = 3'd2, Q_DLY = 3'd3, Q_GO = 3'd4;
    localparam [4:0] CORE_WAIT_START = 5'd2;

    reg [2:0] q;
    reg [4:0] qcnt;
    reg       soft_rst, core_start, cap_arm;

    always @(posedge clk) begin
        if (rst) begin
            q <= Q_IDLE; qcnt <= 5'd0; soft_rst <= 1'b0; core_start <= 1'b0; cap_arm <= 1'b0;
        end else begin
            core_start <= 1'b0;
            cap_arm    <= 1'b0;
            case (q)
                Q_IDLE: if ((start_req || cmd_start) && !dumping && !ld_active && !ld_err) begin
                            soft_rst <= 1'b1; cap_arm <= 1'b1; qcnt <= 5'd0; q <= Q_RST;
                        end
                Q_RST:  if (qcnt == 5'd15) begin soft_rst <= 1'b0; qcnt <= 5'd0; q <= Q_WAIT; end
                        else qcnt <= qcnt + 5'd1;
                Q_WAIT: if (dbg_state == CORE_WAIT_START) begin qcnt <= 5'd0; q <= Q_DLY; end
                Q_DLY:  if (qcnt == 5'd7) q <= Q_GO; else qcnt <= qcnt + 5'd1;
                Q_GO:   begin core_start <= 1'b1; q <= Q_IDLE; end
                default: q <= Q_IDLE;
            endcase
        end
    end
    wire core_rst = rst | soft_rst;      // the Wiener is reset by the same signal: every run starts from scratch

    // ------------------------------------------------------------------------
    // Case loader: replaces the content of the beamformer's input memory.  Accepted only while idle.
    // ------------------------------------------------------------------------
    wire ld_allow = (q == Q_IDLE) && !dumping && !busy;
    case_loader #(.DEPTH(MEM_DEPTH), .AW(11)) u_ld (
        .clk(clk), .rst(rst), .rx_valid(rx_valid), .rx_data(rx_data), .allow(ld_allow),
        .ld_we(ld_we), .ld_addr(ld_addr), .ld_data(ld_data), .active(ld_active), .ok(ld_ok), .err(ld_err));

    // ------------------------------------------------------------------------
    // Beamformer (unchanged)
    // ------------------------------------------------------------------------
    wire         bf_valid, bf_last, bf_is_tail, bf_ready;
    wire [47:0]  bf_data;
    wire [15:0]  bf_idx;
    wire [7:0]   bf_blk;

    (* DONT_TOUCH = "true" *)
    mvdr_top #(
        .BLOCK_LEN(BLOCK_LEN), .NUM_SAMPLES(NUM_SAMPLES), .MEM_DEPTH(MEM_DEPTH),
        .MEM_FILE(MEM_FILE), .PACED(0)
    ) u_core (
        .clk(clk), .rst(core_rst), .start(core_start), .sample_en(1'b0), .m_ready(bf_ready),
        .ld_we(ld_we), .ld_addr(ld_addr), .ld_data(ld_data),
        .m_valid(bf_valid), .m_data(bf_data), .m_last(bf_last), .m_idx(bf_idx), .m_blk(bf_blk),
        .m_is_tail(bf_is_tail), .busy(busy), .done(done), .status(status),
        .dbg_w_active(dbg_w_active), .dbg_w_swap(dbg_w_swap), .dbg_blk_done(),
        .dbg_solve_cycles(dbg_solve_cycles), .dbg_state(dbg_state));

    // ------------------------------------------------------------------------
    // Glue: drop block 0, flush after the last beat
    // ------------------------------------------------------------------------
    wire blk0 = (bf_blk == 8'd0);

    reg        flush_act;
    reg [7:0]  flush_cnt;
    wire       w_s_ready;
    wire       w_s_valid = flush_act ? 1'b1 : (bf_valid && !blk0);
    wire signed [23:0] w_s_re = flush_act ? 24'sd0 : $signed(bf_data[23:0]);
    wire signed [23:0] w_s_im = flush_act ? 24'sd0 : $signed(bf_data[47:24]);
    assign bf_ready = blk0 ? 1'b1 : (flush_act ? 1'b0 : w_s_ready);

    wire bf_acc = bf_valid & bf_ready;
    always @(posedge clk) begin
        if (core_rst) begin
            flush_act <= 1'b0;
            flush_cnt <= 8'd0;
        end else if (!flush_act) begin
            if (bf_acc && bf_last) flush_act <= 1'b1;       // zeros start right after the last real beat
        end else if (w_s_valid && w_s_ready) begin
            flush_cnt <= flush_cnt + 8'd1;
            if (flush_cnt == FLUSH_N - 1) flush_act <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // Wiener (FFT IP inside wiener_cplx_top).  m_ready = 1: the capture is passive and never stalls it.
    // ------------------------------------------------------------------------
    wire               w_m_valid;
    wire signed [23:0] w_m_re, w_m_im;

    wiener_cplx_top u_wiener (
        .clk(clk), .rst(core_rst), .restart(1'b0),
        .s_valid(w_s_valid), .s_ready(w_s_ready), .s_re(w_s_re), .s_im(w_s_im),
        .m_valid(w_m_valid), .m_ready(1'b1), .m_re(w_m_re), .m_im(w_m_im), .status(wf_status));

    // ------------------------------------------------------------------------
    // Wiener output bookkeeping: skip LAT warm-up outputs, then label NOUT samples with idx / blk / last / tail
    // ------------------------------------------------------------------------
    reg [6:0]  skip_cnt;
    reg [15:0] cnt_o;
    reg [6:0]  sub;
    reg [7:0]  blkc;
    reg        wf_done_r;
    wire       w_in_win = (skip_cnt >= LAT) && (cnt_o < NOUT);
    wire       w_tap_v  = w_m_valid && w_in_win;

    always @(posedge clk) begin
        if (core_rst) begin
            skip_cnt <= 7'd0; cnt_o <= 16'd0; sub <= 7'd0; blkc <= 8'd1; wf_done_r <= 1'b0;
        end else if (w_m_valid) begin
            if (skip_cnt < LAT) skip_cnt <= skip_cnt + 7'd1;
            else if (cnt_o < NOUT) begin
                cnt_o <= cnt_o + 16'd1;
                if (sub == BLOCK_LEN - 1) begin sub <= 7'd0; blkc <= blkc + 8'd1; end
                else                      sub <= sub + 7'd1;
                if (cnt_o == NOUT - 1) wf_done_r <= 1'b1;
            end
        end
    end
    assign wf_done = wf_done_r;

    wire        w_last = (cnt_o == NOUT - 1);
    wire        w_tail = HAS_TAIL && (blkc == NFULL);
    wire [15:0] w_idx  = BLOCK_LEN + cnt_o;              // aligned with the beamformer sample index
    wire [73:0] w_word = {w_tail, w_last, blkc, w_idx, w_m_im, w_m_re};
    wire [73:0] r_word = {bf_is_tail, bf_last, bf_blk, bf_idx, bf_data};

    wire        tap_v    = (TAP_RAW != 0) ? bf_acc : w_tap_v;
    wire [73:0] tap_word = (TAP_RAW != 0) ? r_word : w_word;

    // ------------------------------------------------------------------------
    // Passive capture + UART dump (team's module, unchanged).  Weight snapshots as in the team's design.
    // ------------------------------------------------------------------------
    stream_capture_uplink #(
        .CLKS_PER_BIT(CLKS_PER_BIT), .CAP_DEPTH(2048), .CAP_AW(11), .W_DEPTH(32), .W_AW(5)
    ) u_cap (
        .clk(clk), .rst(rst), .arm(cap_arm),
        .tap_valid(tap_v), .tap_word(tap_word),
        .w_cap(core_start | dbg_w_swap), .w_word(dbg_w_active),
        .dump_req(cmd_dump), .status_in(status), .solve_cycles_in(dbg_solve_cycles),
        .uart_txd(uart_txd), .dumping(dumping), .overflow(cap_overflow), .n_captured());
endmodule
