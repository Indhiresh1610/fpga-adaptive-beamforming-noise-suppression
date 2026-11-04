// ============================================================================
// output_interface.v -- Output Register with AXI-Stream Valid/Ready Handshake
// Compatible with Verilog-2001.
// Supports output backpressure: holds data stable while m_valid && !m_ready.
// Exposes out_busy to controller to prevent lost or reordered samples.
// ============================================================================

`timescale 1ns/1ps

module output_interface (
    input  wire                 clk,
    input  wire                 rst,

    // Internal push interface from sample engine & controller
    input  wire                 push,
    input  wire [47:0]          in_data,   // {im[23:0], re[23:0]} Q2.22
    input  wire [15:0]          in_idx,
    input  wire [7:0]           in_blk,
    input  wire                 in_is_tail,
    input  wire                 in_last,
    output wire                 out_busy,  // 1 if occupied and cannot take new push

    // External Top-Level Master Interface
    output reg                  m_valid,
    output reg  [47:0]          m_data,
    output reg  [15:0]          m_idx,
    output reg  [7:0]           m_blk,
    output reg                  m_is_tail,
    output reg                  m_last,
    input  wire                 m_ready
);

    assign out_busy = m_valid && (!m_ready);

    always @(posedge clk) begin
        if (rst) begin
            m_valid   <= 1'b0;
            m_data    <= 48'd0;
            m_idx     <= 16'd0;
            m_blk     <= 8'd0;
            m_is_tail <= 1'b0;
            m_last    <= 1'b0;
        end else begin
            if (m_valid && m_ready) begin
                if (push) begin
                    // Back-to-back accept and push new sample
                    m_valid   <= 1'b1;
                    m_data    <= in_data;
                    m_idx     <= in_idx;
                    m_blk     <= in_blk;
                    m_is_tail <= in_is_tail;
                    m_last    <= in_last;
                end else begin
                    // Accepted, clear valid
                    m_valid <= 1'b0;
                end
            end else if (!m_valid) begin
                if (push) begin
                    m_valid   <= 1'b1;
                    m_data    <= in_data;
                    m_idx     <= in_idx;
                    m_blk     <= in_blk;
                    m_is_tail <= in_is_tail;
                    m_last    <= in_last;
                end
            end
            // When m_valid && !m_ready: all outputs hold perfectly stable
        end
    end

endmodule
