// ============================================================================
// wiener_board_top.v -- Wiener filter ALONE on the Boolean Board (XC7S50-CSGA324).
// Test setup: an input ROM holds the beamformer output (wf_in.mem, 1971 samples of case_2, block 0 already dropped).
// On a start (btn_start or UART 'S') the samples are fed through the Wiener, the outputs are captured in block RAM
// and sent to the PC over the USB-UART (team's stream_capture_uplink / uart_tx / uart_rx, unchanged).
// Same port names as the team's board top, so their boolean_pins.xdc and timing.xdc are used unchanged.
//
// LEDs: 0 feeding input, 1 all inputs sent, 2 Wiener saturation flag, 3 Wiener finished (all outputs captured),
//       4 UART dump in progress, 5 capture overflow
// UART (115200 8N1): 'S' = reset + run (dump follows by itself), 'D' = send the last result again.
// Strict Verilog-2001 (MMCME2_BASE / BUFG are Xilinx primitives).
// ============================================================================
`timescale 1ns/1ps

module wiener_board_top #(
    parameter NIN        = 1971,            // samples in wf_in.mem
    parameter MEM_FILE   = "wf_in.mem",
    parameter real MMCM_MULT = 10.0,        // 100 MHz * 10 / 20 = 50 MHz core clock
    parameter real MMCM_DIV  = 20.0,
    parameter CORE_HZ    = 50000000,
    parameter BAUD       = 115200
)(
    input  wire       clk,          // 100 MHz
    input  wire       btn_rst,
    input  wire       btn_start,
    input  wire       uart_rxd,
    output wire       uart_txd,
    output wire [5:0] led
);
    localparam CLKS_PER_BIT = (CORE_HZ + BAUD / 2) / BAUD;

    // ---------------- clock, reset, start button: same code as the team's board top ----------------
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

    wire       feeding, feed_done, wf_done, dumping, cap_overflow;
    wire [7:0] wf_status;

    wiener_system #(.NIN(NIN), .MEM_FILE(MEM_FILE), .CLKS_PER_BIT(CLKS_PER_BIT)) u_sys (
        .clk(clk_core), .rst(rst), .start_req(start_pulse), .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .feeding(feeding), .feed_done(feed_done), .wf_status(wf_status), .wf_done(wf_done),
        .dumping(dumping), .cap_overflow(cap_overflow));

    assign led[0] = feeding;
    assign led[1] = feed_done;
    assign led[2] = |wf_status[3:1];
    assign led[3] = wf_done;
    assign led[4] = dumping;
    assign led[5] = cap_overflow;
endmodule


// ============================================================================
// wiener_system -- run control + input ROM + feeder + Wiener + capture/UART.  One clock (the core clock).
//   input ROM -> feeder -> wiener_cplx_top -> (skip first LAT outputs) -> capture BRAM -> UART
// The Wiener's first LAT (= 48) outputs are warm-up and are not captured, so captured sample k belongs to input k
// (the same alignment as wf_expected.csv).  NFLUSH zero samples after the last input push the latency out.
// Captured record per sample: idx = k, blk = 0, last = 1 on the final sample, data = {im, re} (Q2.22).
// ============================================================================
module wiener_system #(
    parameter NIN          = 1971,
    parameter NFLUSH       = 80,
    parameter LAT          = 48,
    parameter MEM_FILE     = "wf_in.mem",
    parameter CLKS_PER_BIT = 434
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start_req,
    input  wire        uart_rxd,
    output wire        uart_txd,
    output wire        feeding,
    output wire        feed_done,
    output wire [7:0]  wf_status,
    output wire        wf_done,
    output wire        dumping,
    output wire        cap_overflow
);
    // ------------------------------------------------------------------------
    // UART receiver, command decode
    // ------------------------------------------------------------------------
    wire       rx_valid;
    wire [7:0] rx_data;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rxd(uart_rxd), .rx_valid(rx_valid), .rx_data(rx_data));
    wire cmd_start = rx_valid && ((rx_data == 8'h53) || (rx_data == 8'h73));   // 'S' 's'
    wire cmd_dump  = rx_valid && ((rx_data == 8'h44) || (rx_data == 8'h64));   // 'D' 'd'

    // ------------------------------------------------------------------------
    // Input ROM (block RAM, synchronous read)
    // ------------------------------------------------------------------------
    (* ram_style = "block" *) reg [47:0] rom [0:2047];
    initial $readmemh(MEM_FILE, rom);
    reg [10:0] addr;
    reg [47:0] rom_q;
    always @(posedge clk) rom_q <= rom[addr];

    // ------------------------------------------------------------------------
    // Feeder FSM
    // ------------------------------------------------------------------------
    localparam [2:0] F_IDLE = 3'd0, F_RST = 3'd1, F_FETCH = 3'd2, F_SEND = 3'd3, F_ZERO = 3'd4, F_END = 3'd5;
    reg [2:0]  fst;
    reg [4:0]  rcnt;
    reg [15:0] icnt;
    reg [7:0]  zcnt;
    reg        soft_rst, cap_arm, fdone_r;

    wire        w_s_ready;
    wire        s_valid = (fst == F_SEND) || (fst == F_ZERO);
    wire signed [23:0] s_re = (fst == F_ZERO) ? 24'sd0 : $signed(rom_q[23:0]);
    wire signed [23:0] s_im = (fst == F_ZERO) ? 24'sd0 : $signed(rom_q[47:24]);
    wire        s_acc = s_valid && w_s_ready;

    always @(posedge clk) begin
        if (rst) begin
            fst <= F_IDLE; rcnt <= 5'd0; icnt <= 16'd0; zcnt <= 8'd0; addr <= 11'd0;
            soft_rst <= 1'b0; cap_arm <= 1'b0; fdone_r <= 1'b0;
        end else begin
            cap_arm <= 1'b0;
            case (fst)
                F_IDLE:  if ((start_req || cmd_start) && !dumping) begin
                             soft_rst <= 1'b1; cap_arm <= 1'b1; rcnt <= 5'd0; fdone_r <= 1'b0; fst <= F_RST;
                         end
                F_RST:   if (rcnt == 5'd15) begin
                             soft_rst <= 1'b0; addr <= 11'd0; icnt <= 16'd0; zcnt <= 8'd0; fst <= F_FETCH;
                         end else rcnt <= rcnt + 5'd1;
                F_FETCH: fst <= F_SEND;                                   // rom_q now holds rom[addr]
                F_SEND:  if (s_acc) begin
                             icnt <= icnt + 16'd1;
                             addr <= addr + 11'd1;
                             fst  <= (icnt == NIN - 1) ? F_ZERO : F_FETCH;
                         end
                F_ZERO:  if (s_acc) begin
                             zcnt <= zcnt + 8'd1;
                             if (zcnt == NFLUSH - 1) begin fdone_r <= 1'b1; fst <= F_END; end
                         end
                default: fst <= F_IDLE;                                   // F_END
            endcase
        end
    end
    assign feeding   = (fst != F_IDLE) && (fst != F_END);
    assign feed_done = fdone_r;
    wire core_rst = rst | soft_rst;

    // ------------------------------------------------------------------------
    // Wiener (FFT IP inside wiener_cplx_top). m_ready = 1: the capture is passive.
    // ------------------------------------------------------------------------
    wire               w_m_valid;
    wire signed [23:0] w_m_re, w_m_im;
    wiener_cplx_top u_wiener (
        .clk(clk), .rst(core_rst), .restart(1'b0),
        .s_valid(s_valid), .s_ready(w_s_ready), .s_re(s_re), .s_im(s_im),
        .m_valid(w_m_valid), .m_ready(1'b1), .m_re(w_m_re), .m_im(w_m_im), .status(wf_status));

    // ------------------------------------------------------------------------
    // Output bookkeeping: skip LAT warm-up outputs, then capture NIN samples labelled idx = 0, 1, ...
    // ------------------------------------------------------------------------
    reg [6:0]  skip_cnt;
    reg [15:0] cnt_o;
    reg        wf_done_r;
    wire       w_in_win = (skip_cnt >= LAT) && (cnt_o < NIN);
    wire       w_tap_v  = w_m_valid && w_in_win;
    always @(posedge clk) begin
        if (core_rst) begin
            skip_cnt <= 7'd0; cnt_o <= 16'd0; wf_done_r <= 1'b0;
        end else if (w_m_valid) begin
            if (skip_cnt < LAT) skip_cnt <= skip_cnt + 7'd1;
            else if (cnt_o < NIN) begin
                cnt_o <= cnt_o + 16'd1;
                if (cnt_o == NIN - 1) wf_done_r <= 1'b1;
            end
        end
    end
    assign wf_done = wf_done_r;

    wire        w_last   = (cnt_o == NIN - 1);
    wire [73:0] tap_word = {1'b0, w_last, 8'd0, cnt_o, w_m_im, w_m_re};   // {tail, last, blk, idx, {im,re}}

    // ------------------------------------------------------------------------
    // Capture + UART dump (team's module, unchanged). No weight snapshots here.
    // ------------------------------------------------------------------------
    stream_capture_uplink #(
        .CLKS_PER_BIT(CLKS_PER_BIT), .CAP_DEPTH(2048), .CAP_AW(11), .W_DEPTH(32), .W_AW(5)
    ) u_cap (
        .clk(clk), .rst(rst), .arm(cap_arm),
        .tap_valid(w_tap_v), .tap_word(tap_word),
        .w_cap(1'b0), .w_word(200'd0),
        .dump_req(cmd_dump), .status_in(wf_status), .solve_cycles_in(32'd0),
        .uart_txd(uart_txd), .dumping(dumping), .overflow(cap_overflow), .n_captured());
endmodule
