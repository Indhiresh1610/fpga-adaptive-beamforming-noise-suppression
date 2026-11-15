// ============================================================================
// xfft_0_behav.v -- SIMULATION-ONLY behavioural stand-in for the Xilinx FFT IP (module name xfft_0).
// Use it only when you simulate WITHOUT the real IP (e.g. iverilog/xsim before generating the IP).
// Do NOT add it to a Vivado project that already contains the xfft_0 IP.
// 64-point, unscaled, natural order, AXI-Stream burst behaviour: config -> 64 inputs -> 64 outputs.
// Input tdata = {im[23:0], re[23:0]} (18-bit data sign-extended), output tdata = {im[31:0], re[31:0]} (25-bit data).
// ============================================================================
`timescale 1ns/1ps
module xfft_0 (
    input  wire         aclk,
    input  wire         aresetn,
    input  wire [7:0]   s_axis_config_tdata,
    input  wire         s_axis_config_tvalid,
    output wire         s_axis_config_tready,
    input  wire [47:0]  s_axis_data_tdata,
    input  wire         s_axis_data_tvalid,
    output wire         s_axis_data_tready,
    input  wire         s_axis_data_tlast,
    output wire [63:0]  m_axis_data_tdata,
    output wire         m_axis_data_tvalid,
    input  wire         m_axis_data_tready,
    output wire         m_axis_data_tlast,
    output wire         event_frame_started,
    output wire         event_tlast_unexpected,
    output wire         event_tlast_missing,
    output wire         event_status_channel_halt,
    output wire         event_data_in_channel_halt,
    output wire         event_data_out_channel_halt
);
    localparam [1:0] S_IDLE = 2'd0, S_LOAD = 2'd1, S_OUT = 2'd2;
    reg [1:0] state;
    reg       fwd;
    reg [6:0] cnt;
    integer   xre [0:63];
    integer   xim [0:63];
    integer   ore [0:63];
    integer   oim [0:63];
    integer   n, kk;
    real      ar, ai, ang, c, s, sr, si;
    reg signed [31:0] o_re_s, o_im_s;

    assign s_axis_config_tready = (state == S_IDLE);
    assign s_axis_data_tready   = (state == S_LOAD);
    assign m_axis_data_tvalid   = (state == S_OUT);
    assign m_axis_data_tlast    = (state == S_OUT) && (cnt == 7'd63);
    assign m_axis_data_tdata    = {o_im_s, o_re_s};
    assign event_frame_started = 1'b0, event_tlast_unexpected = 1'b0, event_tlast_missing = 1'b0,
           event_status_channel_halt = 1'b0, event_data_in_channel_halt = 1'b0, event_data_out_channel_halt = 1'b0;

    always @(posedge aclk) begin
        if (!aresetn) begin state <= S_IDLE; cnt <= 7'd0; fwd <= 1'b1; end
        else case (state)
            S_IDLE: if (s_axis_config_tvalid) begin fwd <= s_axis_config_tdata[0]; cnt <= 7'd0; state <= S_LOAD; end
            S_LOAD: if (s_axis_data_tvalid) begin
                xre[cnt[5:0]] = $signed(s_axis_data_tdata[17:0]);
                xim[cnt[5:0]] = $signed(s_axis_data_tdata[41:24]);
                if (cnt == 7'd63) begin
                    for (kk = 0; kk < 64; kk = kk + 1) begin
                        sr = 0.0; si = 0.0;
                        for (n = 0; n < 64; n = n + 1) begin
                            ang = 6.283185307179586 * kk * n / 64.0; if (fwd) ang = -ang;
                            c = $cos(ang); s = $sin(ang);
                            ar = xre[n]; ai = xim[n];
                            sr = sr + ar*c - ai*s;  si = si + ar*s + ai*c;
                        end
                        ore[kk] = $rtoi($floor(sr + 0.5)); oim[kk] = $rtoi($floor(si + 0.5));
                    end
                    o_re_s <= ore[0]; o_im_s <= oim[0];
                    cnt <= 7'd0; state <= S_OUT;
                end else cnt <= cnt + 7'd1;
            end
            S_OUT: if (m_axis_data_tready) begin
                if (cnt == 7'd63) begin cnt <= 7'd0; state <= S_IDLE; end
                else begin cnt <= cnt + 7'd1; o_re_s <= ore[cnt + 7'd1]; o_im_s <= oim[cnt + 7'd1]; end
            end
            default: state <= S_IDLE;
        endcase
    end
endmodule
