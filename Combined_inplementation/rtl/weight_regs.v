// ============================================================================
// weight_regs.v -- Weight Register Banks (w_active and w_next)
// Compatible with Verilog-2001.
// Strictly enforces causal block processing:
//   - w_active is read by beamformer, constant throughout the block
//   - w_next is written only by the block solver
//   - w_swap copies w_next -> w_active in 1 cycle only between blocks
//   - Initial weights are a/4 (quiescent)
// ============================================================================

`timescale 1ns/1ps

module weight_regs (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 load_quiescent, // pulse to initialize both banks to a/4
    input  wire                 w_swap,         // 1-cycle pulse to copy w_next -> w_active
    input  wire [127:0]         a_vec,          // steering vector Q2.14: {s3, s2, s1, s0}
    input  wire                 w_wr_en,        // write enable from solver
    input  wire [2:0]           w_wr_idx,       // 0..3 re, 4..7 im
    input  wire signed [24:0]   w_wr_data,      // Q5.20 from solver
    output reg  [199:0]         w_active,       // {w3_im, w3_re, ... w0_im, w0_re}
    output reg  [199:0]         w_next,
    output wire [199:0]         dbg_w_active,
    output reg                  dbg_w_swap
);

    assign dbg_w_active = w_active;

    reg signed [24:0] reg_w_active_re [0:3];
    reg signed [24:0] reg_w_active_im [0:3];
    reg signed [24:0] reg_w_next_re   [0:3];
    reg signed [24:0] reg_w_next_im   [0:3];

    integer m;

    // Helper functions to unpack a_vec
    wire signed [15:0] a0_re = a_vec[15:0];
    wire signed [15:0] a0_im = a_vec[31:16];
    wire signed [15:0] a1_re = a_vec[47:32];
    wire signed [15:0] a1_im = a_vec[63:48];
    wire signed [15:0] a2_re = a_vec[79:64];
    wire signed [15:0] a2_im = a_vec[95:80];
    wire signed [15:0] a3_re = a_vec[111:96];
    wire signed [15:0] a3_im = a_vec[127:112];

    wire signed [24:0] q_w0_re = {{5{a0_re[15]}}, a0_re, 4'd0};
    wire signed [24:0] q_w0_im = {{5{a0_im[15]}}, a0_im, 4'd0};
    wire signed [24:0] q_w1_re = {{5{a1_re[15]}}, a1_re, 4'd0};
    wire signed [24:0] q_w1_im = {{5{a1_im[15]}}, a1_im, 4'd0};
    wire signed [24:0] q_w2_re = {{5{a2_re[15]}}, a2_re, 4'd0};
    wire signed [24:0] q_w2_im = {{5{a2_im[15]}}, a2_im, 4'd0};
    wire signed [24:0] q_w3_re = {{5{a3_re[15]}}, a3_re, 4'd0};
    wire signed [24:0] q_w3_im = {{5{a3_im[15]}}, a3_im, 4'd0};

    always @(posedge clk) begin
        if (rst) begin
            dbg_w_swap <= 1'b0;
            for (m = 0; m < 4; m = m + 1) begin
                reg_w_active_re[m] <= 25'sd0;
                reg_w_active_im[m] <= 25'sd0;
                reg_w_next_re[m]   <= 25'sd0;
                reg_w_next_im[m]   <= 25'sd0;
            end
            w_active <= 200'd0;
            w_next   <= 200'd0;
        end else begin
            dbg_w_swap <= 1'b0;

            if (load_quiescent) begin
                reg_w_active_re[0] <= q_w0_re; reg_w_active_im[0] <= q_w0_im;
                reg_w_active_re[1] <= q_w1_re; reg_w_active_im[1] <= q_w1_im;
                reg_w_active_re[2] <= q_w2_re; reg_w_active_im[2] <= q_w2_im;
                reg_w_active_re[3] <= q_w3_re; reg_w_active_im[3] <= q_w3_im;

                reg_w_next_re[0]   <= q_w0_re; reg_w_next_im[0]   <= q_w0_im;
                reg_w_next_re[1]   <= q_w1_re; reg_w_next_im[1]   <= q_w1_im;
                reg_w_next_re[2]   <= q_w2_re; reg_w_next_im[2]   <= q_w2_im;
                reg_w_next_re[3]   <= q_w3_re; reg_w_next_im[3]   <= q_w3_im;

                w_active <= {q_w3_im, q_w3_re, q_w2_im, q_w2_re, q_w1_im, q_w1_re, q_w0_im, q_w0_re};
                w_next   <= {q_w3_im, q_w3_re, q_w2_im, q_w2_re, q_w1_im, q_w1_re, q_w0_im, q_w0_re};
            end else begin
                // Update w_next from solver write interface
                if (w_wr_en) begin
                    case (w_wr_idx)
                        3'd0: begin reg_w_next_re[0] <= w_wr_data; w_next[24:0]    <= w_wr_data; end
                        3'd1: begin reg_w_next_re[1] <= w_wr_data; w_next[74:50]   <= w_wr_data; end
                        3'd2: begin reg_w_next_re[2] <= w_wr_data; w_next[124:100] <= w_wr_data; end
                        3'd3: begin reg_w_next_re[3] <= w_wr_data; w_next[174:150] <= w_wr_data; end
                        3'd4: begin reg_w_next_im[0] <= w_wr_data; w_next[49:25]   <= w_wr_data; end
                        3'd5: begin reg_w_next_im[1] <= w_wr_data; w_next[99:75]   <= w_wr_data; end
                        3'd6: begin reg_w_next_im[2] <= w_wr_data; w_next[149:125] <= w_wr_data; end
                        3'd7: begin reg_w_next_im[3] <= w_wr_data; w_next[199:175] <= w_wr_data; end
                    endcase
                end

                // Weight swap: copy w_next -> w_active in 1 cycle
                if (w_swap) begin
                    reg_w_active_re[0] <= reg_w_next_re[0]; reg_w_active_im[0] <= reg_w_next_im[0];
                    reg_w_active_re[1] <= reg_w_next_re[1]; reg_w_active_im[1] <= reg_w_next_im[1];
                    reg_w_active_re[2] <= reg_w_next_re[2]; reg_w_active_im[2] <= reg_w_next_im[2];
                    reg_w_active_re[3] <= reg_w_next_re[3]; reg_w_active_im[3] <= reg_w_next_im[3];

                    w_active   <= w_next;
                    dbg_w_swap <= 1'b1;
                end
            end
        end
    end

endmodule
