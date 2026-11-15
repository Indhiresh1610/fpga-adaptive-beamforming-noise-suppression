// ============================================================================
// wiener_cplx_top.v -- Wiener core + Xilinx FFT IP (instance name xfft_0, see create_xfft_ip.tcl)
// ============================================================================
`timescale 1ns/1ps
module wiener_cplx_top (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 restart,
    input  wire                 s_valid,
    output wire                 s_ready,
    input  wire signed [23:0]   s_re,
    input  wire signed [23:0]   s_im,
    output wire                 m_valid,
    input  wire                 m_ready,
    output wire signed [23:0]   m_re,
    output wire signed [23:0]   m_im,
    output wire [7:0]           status
);
    wire [7:0]  cfg_tdata;  wire cfg_tvalid, cfg_tready;
    wire [47:0] din_tdata;  wire din_tvalid, din_tready, din_tlast;
    wire [63:0] dout_tdata; wire dout_tvalid, dout_tready, dout_tlast;

    wiener_cplx_core #(.NITER(2)) u_core (
        .clk(clk), .rst(rst), .restart(restart),
        .s_valid(s_valid), .s_ready(s_ready), .s_re(s_re), .s_im(s_im),
        .m_valid(m_valid), .m_ready(m_ready), .m_re(m_re), .m_im(m_im), .status(status),
        .fft_cfg_tdata(cfg_tdata), .fft_cfg_tvalid(cfg_tvalid), .fft_cfg_tready(cfg_tready),
        .fft_din_tdata(din_tdata), .fft_din_tvalid(din_tvalid), .fft_din_tready(din_tready), .fft_din_tlast(din_tlast),
        .fft_dout_tdata(dout_tdata), .fft_dout_tvalid(dout_tvalid), .fft_dout_tready(dout_tready), .fft_dout_tlast(dout_tlast));

    xfft_0 u_fft (
        .aclk(clk), .aresetn(~rst),
        .s_axis_config_tdata(cfg_tdata), .s_axis_config_tvalid(cfg_tvalid), .s_axis_config_tready(cfg_tready),
        .s_axis_data_tdata(din_tdata), .s_axis_data_tvalid(din_tvalid), .s_axis_data_tready(din_tready), .s_axis_data_tlast(din_tlast),
        .m_axis_data_tdata(dout_tdata), .m_axis_data_tvalid(dout_tvalid), .m_axis_data_tready(dout_tready), .m_axis_data_tlast(dout_tlast),
        .event_frame_started(), .event_tlast_unexpected(), .event_tlast_missing(),
        .event_status_channel_halt(), .event_data_in_channel_halt(), .event_data_out_channel_halt());
endmodule
