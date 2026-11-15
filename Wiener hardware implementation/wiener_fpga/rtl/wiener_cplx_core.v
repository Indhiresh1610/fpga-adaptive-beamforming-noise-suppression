// ============================================================================
// wiener_cplx_core.v -- Complex STFT Wiener post-filter (narrowband tone in white noise, NO lead-in)
// Verilog-2001.  Bit-accurate to wiener_model.py (the FFT/IFFT is the Xilinx FFT IP, modelled there as exact).
//
//   * input  : complex stream, Q2.22 per component (24 bit) -- directly the beamformer m_data fields
//   * frame  : 64 points, hop 16, periodic Hann, 75 % overlap  (fs = 512 Hz design)
//   * noise  : ONE scalar level from the first full frame (hop 3): 25th-percentile start + 2 trimmed-mean passes
//              (robust to the tone / residual interferers), assumes WHITE noise
//   * xi     : decision-directed, alpha = 0.98; gain G = xi/(1+xi);  q_k = G^2*gamma_k is the per-bin state
//   * output : complex stream Q2.22, latency 48 samples, zeros for the first 3 hops
//   Arithmetic is done sequentially (shared multipliers + one serial divider): ~13k clocks per hop, i.e.
//   far below the 16-sample period (31 ms at 512 Hz).  All multiplier results are captured >= MCP cycles after
//   their operands change -> see wiener_mcp.xdc (multicycle constraint) if you run this at 100 MHz.
// ============================================================================
`timescale 1ns/1ps

module wiener_cplx_core #(
    parameter NITER = 2                 // trimmed-mean refinement passes (model: NITER)
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 restart,        // 1-cycle pulse: forget everything (new record). FFT must be idle.

    // input stream (Q2.22 complex)
    input  wire                 s_valid,
    output wire                 s_ready,
    input  wire signed [23:0]   s_re,
    input  wire signed [23:0]   s_im,

    // output stream (Q2.22 complex)
    output wire                 m_valid,
    input  wire                 m_ready,
    output wire signed [23:0]   m_re,
    output wire signed [23:0]   m_im,

    output reg  [7:0]           status,         // [0] noise level learned, [1] gamma saturated, [3] IFFT-input saturated

    // Xilinx FFT IP (AXI-Stream), 64 pt, 18-bit input (padded to 24), 25-bit output (padded to 32), unscaled
    output wire [7:0]           fft_cfg_tdata,
    output wire                 fft_cfg_tvalid,
    input  wire                 fft_cfg_tready,
    output wire [47:0]          fft_din_tdata,
    output wire                 fft_din_tvalid,
    input  wire                 fft_din_tready,
    output wire                 fft_din_tlast,
    input  wire [63:0]          fft_dout_tdata,
    input  wire                 fft_dout_tvalid,
    output wire                 fft_dout_tready,
    input  wire                 fft_dout_tlast
);
    // ------------------------------------------------------------------ constants (same as wiener_model.py)
    localparam MULW   = 4;                       // wait cycles for multiplier results (see MCP note)
    localparam [15:0] ALPHA  = 16'd64225;        // round(0.98*65536)
    localparam [10:0] BETA   = 11'd1311;         // round(0.02*65536)
    localparam [35:0] GAM_MAX= 36'hFFFFFFFFF;
    localparam [45:0] XI_MIN = 46'd17;
    localparam [15:0] RANK_K = 16'd56952;        // 3.4761 * 2^14 : 25th percentile -> mean for exponential |Y|^2
    localparam [14:0] BIAS_K = 15'd17707;        // 1.0807 * 2^14 : trimmed-mean bias
    localparam [6:0]  RANK   = 7'd16;
    localparam [16:0] OLA_K  = 17'd21845;        // 2/3 in Q1.15  (1 / sum(hann^2))

    // ------------------------------------------------------------------ states
    localparam [5:0]
      ST_CLR=0,  ST_IDLE=1, ST_PUSH=2, ST_DEC=3,  ST_CFGF=4, ST_LDA=5,  ST_LDB=6,  ST_LDS=7,  ST_RXF=8,
      ST_PWA=9,  ST_PWB=10, ST_RKI=11, ST_RKC=12, ST_RKD=13, ST_L0A=14, ST_L0B=15, ST_TI=16,  ST_TS=17,
      ST_TM=18,  ST_TM2=19, ST_TD=20,  ST_BG1=21, ST_BG1D=22,ST_BQ1=23, ST_BQ2=24, ST_BX=25,  ST_BXD=26,
      ST_BZ=27,  ST_BZ2=28, ST_BQ3=29, ST_BQ4=30, ST_CFGI=31,ST_LDI=32, ST_RXI=33, ST_OA=34,  ST_OB=35,
      ST_EA=36,  ST_EB=37,  ST_EP=38,  ST_EEND=39,ST_WAIT=40,ST_DWAIT=41;

    reg [5:0] st, st_ret;
    reg [6:0] k;                 // bin / sample counter 0..63
    reg [5:0] ri;                // outer index of rank search
    reg [4:0] hi;                // hop sample counter
    reg [5:0] wr;                // circular index of the OLDEST sample of the current frame
    reg [1:0] fcnt;              // hops seen since restart, saturates at 3
    reg       noise_done;
    reg [2:0] it;
    reg [3:0] w;

    // ------------------------------------------------------------------ memories (no reset -> distributed RAM)
    reg signed [17:0] ifr [0:31];  reg signed [17:0] ifi [0:31];   // input FIFO (Q5.13)
    reg signed [17:0] xr  [0:63];  reg signed [17:0] xi  [0:63];   // frame buffer
    reg signed [24:0] yr  [0:63];  reg signed [24:0] yi  [0:63];   // spectrum, later IFFT output
    reg        [49:0] pw  [0:63];                                  // |Y|^2
    reg        [44:0] qs  [0:63];                                  // q = G^2 * gamma of the previous frame (Q.24)
    reg signed [17:0] zr  [0:63];  reg signed [17:0] zi  [0:63];   // gained spectrum (/64) for the IFFT
    reg signed [31:0] ola_r [0:63]; reg signed [31:0] ola_i [0:63];
    reg signed [23:0] of_r [0:31]; reg signed [23:0] of_i [0:31];  // output FIFO

    reg [4:0] iwp, irp, owp, orp;
    reg [5:0] icnt, ocnt;

    // ------------------------------------------------------------------ read ports
    wire [5:0] kk    = k[5:0];
    wire [5:0] xa    = wr + kk;
    wire [5:0] oaddr = ((st==ST_EA) || (st==ST_EB) || (st==ST_EP)) ? (wr + {2'b00, hi[3:0]}) : (wr + kk);
    wire signed [17:0] xr_rd = xr[xa];
    wire signed [17:0] xi_rd = xi[xa];
    wire signed [24:0] yr_rd = yr[kk];
    wire signed [24:0] yi_rd = yi[kk];
    wire        [49:0] pw_rd = pw[kk];
    wire        [49:0] pw_ri = pw[ri];
    wire        [44:0] qs_rd = qs[kk];
    wire signed [17:0] zr_rd = zr[kk];
    wire signed [17:0] zi_rd = zi[kk];
    wire signed [31:0] olr_rd = ola_r[oaddr];
    wire signed [31:0] oli_rd = ola_i[oaddr];
    wire signed [17:0] ifr_rd = ifr[irp];
    wire signed [17:0] ifi_rd = ifi[irp];

    // Hann window ROM, Q1.15 (0..32768), periodic, 64 points
    function [15:0] hann_f;
        input [5:0] n;
        begin
            case (n)
                6'd0: hann_f = 16'd0;
                6'd1: hann_f = 16'd79;
                6'd2: hann_f = 16'd315;
                6'd3: hann_f = 16'd705;
                6'd4: hann_f = 16'd1247;
                6'd5: hann_f = 16'd1935;
                6'd6: hann_f = 16'd2761;
                6'd7: hann_f = 16'd3719;
                6'd8: hann_f = 16'd4799;
                6'd9: hann_f = 16'd5990;
                6'd10: hann_f = 16'd7282;
                6'd11: hann_f = 16'd8661;
                6'd12: hann_f = 16'd10114;
                6'd13: hann_f = 16'd11628;
                6'd14: hann_f = 16'd13188;
                6'd15: hann_f = 16'd14778;
                6'd16: hann_f = 16'd16384;
                6'd17: hann_f = 16'd17990;
                6'd18: hann_f = 16'd19580;
                6'd19: hann_f = 16'd21140;
                6'd20: hann_f = 16'd22654;
                6'd21: hann_f = 16'd24107;
                6'd22: hann_f = 16'd25486;
                6'd23: hann_f = 16'd26778;
                6'd24: hann_f = 16'd27969;
                6'd25: hann_f = 16'd29049;
                6'd26: hann_f = 16'd30007;
                6'd27: hann_f = 16'd30833;
                6'd28: hann_f = 16'd31521;
                6'd29: hann_f = 16'd32063;
                6'd30: hann_f = 16'd32453;
                6'd31: hann_f = 16'd32689;
                6'd32: hann_f = 16'd32768;
                6'd33: hann_f = 16'd32689;
                6'd34: hann_f = 16'd32453;
                6'd35: hann_f = 16'd32063;
                6'd36: hann_f = 16'd31521;
                6'd37: hann_f = 16'd30833;
                6'd38: hann_f = 16'd30007;
                6'd39: hann_f = 16'd29049;
                6'd40: hann_f = 16'd27969;
                6'd41: hann_f = 16'd26778;
                6'd42: hann_f = 16'd25486;
                6'd43: hann_f = 16'd24107;
                6'd44: hann_f = 16'd22654;
                6'd45: hann_f = 16'd21140;
                6'd46: hann_f = 16'd19580;
                6'd47: hann_f = 16'd17990;
                6'd48: hann_f = 16'd16384;
                6'd49: hann_f = 16'd14778;
                6'd50: hann_f = 16'd13188;
                6'd51: hann_f = 16'd11628;
                6'd52: hann_f = 16'd10114;
                6'd53: hann_f = 16'd8661;
                6'd54: hann_f = 16'd7282;
                6'd55: hann_f = 16'd5990;
                6'd56: hann_f = 16'd4799;
                6'd57: hann_f = 16'd3719;
                6'd58: hann_f = 16'd2761;
                6'd59: hann_f = 16'd1935;
                6'd60: hann_f = 16'd1247;
                6'd61: hann_f = 16'd705;
                6'd62: hann_f = 16'd315;
                6'd63: hann_f = 16'd79;
                default: hann_f = 16'd0;
            endcase
        end
    endfunction
    wire [15:0] hann_rd = hann_f(kk);

    // ------------------------------------------------------------------ multipliers (operand regs -> product wires)
    (* use_dsp = "yes" *) reg signed [24:0] mopA_a, mopA_b, mopB_a, mopB_b;
    (* use_dsp = "yes" *) reg signed [31:0] mopC_a, mopD_a;
    (* use_dsp = "yes" *) reg signed [16:0] mopC_b, mopD_b;
    reg signed [63:0] mopW_a;
    reg signed [24:0] mopW_b;
    wire signed [49:0] prodA = mopA_a * mopA_b;
    wire signed [49:0] prodB = mopB_a * mopB_b;
    wire signed [48:0] prodC = mopC_a * mopC_b;
    wire signed [48:0] prodD = mopD_a * mopD_b;
    wire signed [88:0] prodW = mopW_a * mopW_b;

    // rounded / shifted views
    wire signed [48:0] prodC_r = prodC + 49'sd16384;                // +0.5 LSB of >>15
    wire signed [48:0] prodD_r = prodD + 49'sd16384;
    wire signed [33:0] c15 = prodC_r[48:15];
    wire signed [33:0] d15 = prodD_r[48:15];
    wire signed [49:0] prodA_r = prodA + 50'sd536870912;            // +0.5 LSB of >>30
    wire signed [49:0] prodB_r = prodB + 50'sd536870912;
    wire signed [19:0] za30 = prodA_r[49:30];
    wire signed [19:0] zb30 = prodB_r[49:30];
    wire signed [88:0] prodW_r16 = prodW + 89'sd32768;

    // ------------------------------------------------------------------ other datapath regs
    reg signed [17:0] fd_re, fd_im;         // windowed sample waiting for the FFT
    reg        [49:0] pi_r;                 // rank search: current candidate
    reg        [6:0]  rcnt;
    reg        [49:0] r16;
    reg        [51:0] L;                    // scalar noise level (|Y|^2 units)
    reg        [53:0] thr;
    reg        [56:0] tsum;
    reg        [6:0]  tcnt;
    reg        [35:0] gam;
    reg        [44:0] t1;
    reg        [45:0] xi_v;
    reg        [23:0] G;
    reg signed [33:0] vr, vi;

    // serial divider
    reg  [69:0] dvd;  reg [51:0] dvs;  reg dv_start;
    wire [69:0] dv_q, dv_qr;  wire [51:0] dv_rem;  wire dv_busy, dv_done, dv_dz;
    serial_divider #(.WIDTH_N(70), .WIDTH_D(52)) u_div (
        .clk(clk), .rst(rst), .start(dv_start), .dividend(dvd), .divisor(dvs),
        .quotient(dv_q), .quotient_rnd(dv_qr), .remainder(dv_rem),
        .busy(dv_busy), .done(dv_done), .div_by_zero(dv_dz));

    // ------------------------------------------------------------------ AXI-stream to the FFT IP
    assign fft_cfg_tvalid  = (st==ST_CFGF) || (st==ST_CFGI);
    assign fft_cfg_tdata   = (st==ST_CFGF) ? 8'h01 : 8'h00;          // FWD_INV: 1 = forward, 0 = inverse
    assign fft_din_tvalid  = (st==ST_LDS) || (st==ST_LDI);
    assign fft_din_tdata   = (st==ST_LDS) ? {{6{fd_im[17]}}, fd_im, {6{fd_re[17]}}, fd_re}
                                          : {{6{zi_rd[17]}}, zi_rd, {6{zr_rd[17]}}, zr_rd};
    assign fft_din_tlast   = (kk == 6'd63);
    assign fft_dout_tready = (st==ST_RXF) || (st==ST_RXI);
    wire signed [24:0] dout_re = fft_dout_tdata[24:0];
    wire signed [24:0] dout_im = fft_dout_tdata[56:32];

    // ------------------------------------------------------------------ streams
    assign s_ready = (icnt < 6'd32);
    assign m_valid = (ocnt != 6'd0);
    assign m_re    = of_r[orp];
    assign m_im    = of_i[orp];

    wire signed [24:0] s_re_r = s_re + 25'sd256;                      // Q2.22 -> Q5.13, round half up
    wire signed [24:0] s_im_r = s_im + 25'sd256;
    wire [35:0] gm1 = (gam > 36'd65536) ? (gam - 36'd65536) : 36'd0;
    wire [45:0] xs   = {1'b0, t1} + {6'b0, prodW[47:8]};
    wire [45:0] xs_c = (xs < XI_MIN) ? XI_MIN : xs;

    // saturating narrowing helpers
    wire signed [17:0] za_s = (za30 >  20'sd131071) ? 18'sd131071 : (za30 < -20'sd131072) ? -18'sd131072 : za30[17:0];
    wire signed [17:0] zb_s = (zb30 >  20'sd131071) ? 18'sd131071 : (zb30 < -20'sd131072) ? -18'sd131072 : zb30[17:0];
    wire z_sat = (za30 > 20'sd131071) || (za30 < -20'sd131072) || (zb30 > 20'sd131071) || (zb30 < -20'sd131072);
    wire signed [23:0] vr_q = (vr >  34'sd16383) ? 24'sh7FFFFF : (vr < -34'sd16384) ? 24'sh800000 : {vr[14:0], 9'b0};
    wire signed [23:0] vi_q = (vi >  34'sd16383) ? 24'sh7FFFFF : (vi < -34'sd16384) ? 24'sh800000 : {vi[14:0], 9'b0};

    reg [5:0] icnt_n, ocnt_n;

    always @(posedge clk) begin
        icnt_n   = icnt;
        ocnt_n   = ocnt;
        dv_start <= 1'b0;

        if (rst || restart) begin
            st <= ST_CLR; k <= 7'd0; wr <= 6'd0; fcnt <= 2'd0; noise_done <= 1'b0; hi <= 5'd0; it <= 3'd0; w <= 4'd0;
            iwp <= 5'd0; irp <= 5'd0; owp <= 5'd0; orp <= 5'd0; icnt_n = 6'd0; ocnt_n = 6'd0;
            status <= 8'd0;
        end else begin
            // ---- input FIFO write (Q2.22 -> Q5.13)
            if (s_valid && s_ready) begin
                ifr[iwp] <= {{2{s_re_r[24]}}, s_re_r[24:9]};
                ifi[iwp] <= {{2{s_im_r[24]}}, s_im_r[24:9]};
                iwp <= iwp + 5'd1;
                icnt_n = icnt_n + 6'd1;
            end
            // ---- output FIFO read
            if (m_valid && m_ready) begin
                orp <= orp + 5'd1;
                ocnt_n = ocnt_n - 6'd1;
            end

            case (st)
            // ---------------------------------------------------------- clear state RAMs
            ST_CLR: begin
                qs[kk] <= 45'd0; ola_r[kk] <= 32'sd0; ola_i[kk] <= 32'sd0;
                if (k == 7'd63) begin k <= 7'd0; st <= ST_IDLE; end else k <= k + 7'd1;
            end
            // ---------------------------------------------------------- hop control
            ST_IDLE: if (icnt >= 6'd16) begin hi <= 5'd0; st <= ST_PUSH; end
            ST_PUSH: begin                      // move 16 new samples to the NEWEST 16 slots of the frame
                xr[wr + 6'd48 + {2'b00, hi[3:0]}] <= ifr_rd;
                xi[wr + 6'd48 + {2'b00, hi[3:0]}] <= ifi_rd;
                irp <= irp + 5'd1; icnt_n = icnt_n - 6'd1;
                if (hi == 5'd15) begin st <= ST_DEC; end
                hi <= hi + 5'd1;
            end
            ST_DEC: begin
                hi <= 5'd0;
                if (fcnt < 2'd3) st <= ST_EA;           // first hops: nothing to filter, emit (zero) samples
                else begin k <= 7'd0; st <= ST_CFGF; end
            end
            // ---------------------------------------------------------- forward FFT: window + load
            ST_CFGF: if (fft_cfg_tready) begin k <= 7'd0; st <= ST_LDA; end
            ST_LDA: begin
                mopC_a <= xr_rd;  mopC_b <= {1'b0, hann_rd};
                mopD_a <= xi_rd;  mopD_b <= {1'b0, hann_rd};
                w <= 4'd0; st_ret <= ST_LDB; st <= ST_WAIT;
            end
            ST_LDB: begin fd_re <= c15[17:0]; fd_im <= d15[17:0]; st <= ST_LDS; end
            ST_LDS: if (fft_din_tready) begin
                if (k == 7'd63) begin k <= 7'd0; st <= ST_RXF; end
                else begin k <= k + 7'd1; st <= ST_LDA; end
            end
            ST_RXF: if (fft_dout_tvalid) begin
                yr[kk] <= dout_re; yi[kk] <= dout_im;
                if (k == 7'd63) begin k <= 7'd0; st <= ST_PWA; end else k <= k + 7'd1;
            end
            // ---------------------------------------------------------- |Y|^2
            ST_PWA: begin
                mopA_a <= yr_rd; mopA_b <= yr_rd; mopB_a <= yi_rd; mopB_b <= yi_rd;
                w <= 4'd0; st_ret <= ST_PWB; st <= ST_WAIT;
            end
            ST_PWB: begin
                pw[kk] <= prodA[49:0] + prodB[49:0];
                if (k == 7'd63) begin
                    k <= 7'd0;
                    if (noise_done) st <= ST_BG1; else begin ri <= 6'd0; st <= ST_RKI; end
                end else begin k <= k + 7'd1; st <= ST_PWA; end
            end
            // ---------------------------------------------------------- noise level (first frame only)
            // rank search: bin with exactly RANK smaller values (ties broken by index) = RANK-th smallest
            ST_RKI: begin pi_r <= pw_ri; k <= 7'd0; rcnt <= 7'd0; st <= ST_RKC; end
            ST_RKC: begin
                if ((pw_rd < pi_r) || ((pw_rd == pi_r) && (kk < ri))) rcnt <= rcnt + 7'd1;
                if (k == 7'd63) st <= ST_RKD; else k <= k + 7'd1;
            end
            ST_RKD: begin
                if (rcnt == RANK) r16 <= pi_r;
                if (ri == 6'd63) st <= ST_L0A; else begin ri <= ri + 6'd1; st <= ST_RKI; end
            end
            ST_L0A: begin
                mopW_a <= {14'b0, r16}; mopW_b <= {9'b0, RANK_K};
                w <= 4'd0; st_ret <= ST_L0B; st <= ST_WAIT;
            end
            ST_L0B: begin
                L  <= (prodW[65:14] == 52'd0) ? 52'd1 : prodW[65:14];
                it <= 3'd0; st <= ST_TI;
            end
            ST_TI: begin k <= 7'd0; tsum <= 57'd0; tcnt <= 7'd0; thr <= {L, 2'b00}; st <= ST_TS; end
            ST_TS: begin
                if ({4'b0, pw_rd} <= thr) begin tsum <= tsum + {7'b0, pw_rd}; tcnt <= tcnt + 7'd1; end
                if (k == 7'd63) st <= ST_TM; else k <= k + 7'd1;
            end
            ST_TM: begin
                mopW_a <= {7'b0, tsum}; mopW_b <= {10'b0, BIAS_K};
                w <= 4'd0; st_ret <= ST_TM2; st <= ST_WAIT;
            end
            ST_TM2: begin
                dvd <= {12'b0, prodW[71:14]}; dvs <= {45'b0, tcnt}; dv_start <= 1'b1;
                st_ret <= ST_TD; st <= ST_DWAIT;
            end
            ST_TD: begin
                L <= (dv_q[51:0] == 52'd0) ? 52'd1 : dv_q[51:0];
                if (it == (NITER-1)) begin noise_done <= 1'b1; status[0] <= 1'b1; k <= 7'd0; st <= ST_BG1; end
                else begin it <= it + 3'd1; st <= ST_TI; end
            end
            // ---------------------------------------------------------- per-bin decision-directed Wiener gain
            ST_BG1: begin                       // gamma = (P << 16) / L
                dvd <= {4'b0, pw_rd, 16'b0}; dvs <= L; dv_start <= 1'b1;
                st_ret <= ST_BG1D; st <= ST_DWAIT;
            end
            ST_BG1D: begin
                if (dv_q[69:36] != 34'd0) begin gam <= GAM_MAX; status[1] <= 1'b1; end
                else gam <= dv_q[35:0];
                st <= ST_BQ1;
            end
            ST_BQ1: begin                       // 0.98 * q
                mopW_a <= {19'b0, qs_rd}; mopW_b <= {9'b0, ALPHA};
                w <= 4'd0; st_ret <= ST_BQ2; st <= ST_WAIT;
            end
            ST_BQ2: begin
                t1 <= prodW_r16[60:16];
                mopW_a <= {28'b0, gm1}; mopW_b <= {14'b0, BETA};      // 0.02 * max(gamma-1,0)
                w <= 4'd0; st_ret <= ST_BX; st <= ST_WAIT;
            end
            ST_BX: begin                        // xi = max(t1 + t2, XI_MIN);  G = xi / (1 + xi)
                xi_v <= xs_c;
                dvd  <= {xs_c, 24'b0}; dvs <= {6'b0, xs_c} + 52'd16777216; dv_start <= 1'b1;
                st_ret <= ST_BXD; st <= ST_DWAIT;
            end
            ST_BXD: begin G <= dv_q[23:0]; st <= ST_BZ; end
            ST_BZ: begin                        // Y * G
                mopA_a <= yr_rd; mopA_b <= {1'b0, G};
                mopB_a <= yi_rd; mopB_b <= {1'b0, G};
                w <= 4'd0; st_ret <= ST_BZ2; st <= ST_WAIT;
            end
            ST_BZ2: begin
                zr[kk] <= za_s; zi[kk] <= zb_s;
                if (z_sat) status[3] <= 1'b1;
                mopA_a <= {1'b0, G}; mopA_b <= {1'b0, G};              // G^2
                w <= 4'd0; st_ret <= ST_BQ3; st <= ST_WAIT;
            end
            ST_BQ3: begin                       // q = G^2 * gamma
                mopW_a <= {28'b0, gam}; mopW_b <= {1'b0, prodA[47:24]};
                w <= 4'd0; st_ret <= ST_BQ4; st <= ST_WAIT;
            end
            ST_BQ4: begin
                qs[kk] <= prodW[60:16];
                if (k == 7'd63) begin k <= 7'd0; st <= ST_CFGI; end
                else begin k <= k + 7'd1; st <= ST_BG1; end
            end
            // ---------------------------------------------------------- inverse FFT + overlap-add
            ST_CFGI: if (fft_cfg_tready) begin k <= 7'd0; st <= ST_LDI; end
            ST_LDI: if (fft_din_tready) begin
                if (k == 7'd63) begin k <= 7'd0; st <= ST_RXI; end else k <= k + 7'd1;
            end
            ST_RXI: if (fft_dout_tvalid) begin
                yr[kk] <= dout_re; yi[kk] <= dout_im;
                if (k == 7'd63) begin k <= 7'd0; st <= ST_OA; end else k <= k + 7'd1;
            end
            ST_OA: begin
                mopC_a <= yr_rd; mopC_b <= {1'b0, hann_rd};
                mopD_a <= yi_rd; mopD_b <= {1'b0, hann_rd};
                w <= 4'd0; st_ret <= ST_OB; st <= ST_WAIT;
            end
            ST_OB: begin
                ola_r[oaddr] <= olr_rd + c15[31:0];
                ola_i[oaddr] <= oli_rd + d15[31:0];
                if (k == 7'd63) begin hi <= 5'd0; st <= ST_EA; end
                else begin k <= k + 7'd1; st <= ST_OA; end
            end
            // ---------------------------------------------------------- emit the oldest 16 (finished) samples
            ST_EA: begin
                mopC_a <= olr_rd; mopC_b <= {1'b0, OLA_K[15:0]};
                mopD_a <= oli_rd; mopD_b <= {1'b0, OLA_K[15:0]};
                w <= 4'd0; st_ret <= ST_EB; st <= ST_WAIT;
            end
            ST_EB: begin vr <= c15; vi <= d15; st <= ST_EP; end
            ST_EP: if (ocnt < 6'd32) begin
                of_r[owp] <= vr_q; of_i[owp] <= vi_q; owp <= owp + 5'd1; ocnt_n = ocnt_n + 6'd1;
                ola_r[oaddr] <= 32'sd0; ola_i[oaddr] <= 32'sd0;
                if (hi == 5'd15) st <= ST_EEND; else begin hi <= hi + 5'd1; st <= ST_EA; end
            end
            ST_EEND: begin
                wr <= wr + 6'd16;
                if (fcnt != 2'd3) fcnt <= fcnt + 2'd1;
                st <= ST_IDLE;
            end
            // ---------------------------------------------------------- generic waits
            ST_WAIT:  begin if (w == MULW-1) st <= st_ret; else w <= w + 4'd1; end
            ST_DWAIT: begin if (dv_done) st <= st_ret; end
            default: st <= ST_CLR;
            endcase
        end
        icnt <= icnt_n;
        ocnt <= ocnt_n;
    end
endmodule
