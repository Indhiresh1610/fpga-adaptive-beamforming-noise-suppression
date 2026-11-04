// ============================================================================
// sample_engine.v -- Beamformer + Covariance Accumulator
// Compatible with Verilog-2001.
// Uses a single time-multiplexed signed multiplier lane (DSP-friendly).
// 48 micro-ops per sample:
//   - 0..15 : Beamforming (y_re, y_im)
//   - 16..23: Diagonal covariance (S0..S3)
//   - 24..47: Off-diagonal covariance (S4..S15)
// ============================================================================

`timescale 1ns/1ps

module sample_engine #(
    parameter BLOCK_LEN = 76,
    parameter ACC_W     = 39  // 32 + clog2(BLOCK_LEN) = 39 for BLOCK_LEN=76
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 sample_start,
    input  wire                 accum_en,      // 1 to accumulate covariance, 0 during tail
    input  wire                 clear_cov,     // clear covariance accumulators
    input  wire [127:0]         x_snap,        // {s3, s2, s1, s0}, each {im[15:0], re[15:0]}
    input  wire [199:0]         w_active,      // {w3_im, w3_re, ... w0_im, w0_re}, each 25-bit signed
    output reg  [47:0]          y_out,         // {y_im[23:0], y_re[23:0]} Q2.22
    output reg                  y_valid,
    output reg                  sample_done,
    output reg                  sat_y,         // sticky saturation flag
    input  wire [3:0]           S_rd_addr,
    output wire [ACC_W-1:0]     S_rd_data
);

    // Verilog-2001 constant ceil-log2 function
    function integer clog2;
        input integer value;
        integer temp;
        begin
            temp = value - 1;
            for (clog2 = 0; temp > 0; clog2 = clog2 + 1)
                temp = temp >> 1;
        end
    endfunction

    // ------------------------------------------------------------------------
    // Unpack input sensors x[m] and weights w[m]
    // ------------------------------------------------------------------------
    wire signed [15:0] xr [0:3];
    wire signed [15:0] xi [0:3];
    assign xr[0] = x_snap[15:0];
    assign xi[0] = x_snap[31:16];
    assign xr[1] = x_snap[47:32];
    assign xi[1] = x_snap[63:48];
    assign xr[2] = x_snap[79:64];
    assign xi[2] = x_snap[95:80];
    assign xr[3] = x_snap[111:96];
    assign xi[3] = x_snap[127:112];

    wire signed [24:0] wr [0:3];
    wire signed [24:0] wi [0:3];
    assign wr[0] = w_active[24:0];
    assign wi[0] = w_active[49:25];
    assign wr[1] = w_active[74:50];
    assign wi[1] = w_active[99:75];
    assign wr[2] = w_active[124:100];
    assign wi[2] = w_active[149:125];
    assign wr[3] = w_active[174:150];
    assign wi[3] = w_active[199:175];

    // ------------------------------------------------------------------------
    // Covariance Accumulator Bank (16 words)
    // ------------------------------------------------------------------------
    reg signed [ACC_W-1:0] S [0:15];
    assign S_rd_data = S[S_rd_addr];

    // ------------------------------------------------------------------------
    // Sequencer and Micro-op control
    // ------------------------------------------------------------------------
    localparam S_IDLE = 2'd0;
    localparam S_RUN  = 2'd1;
    localparam S_DONE = 2'd2;

    reg [1:0]  state;
    reg [5:0]  u_cnt; // 0 .. 47
    reg        busy;

    // Registered snapshot and weights for the current sample
    reg signed [15:0] reg_xr [0:3];
    reg signed [15:0] reg_xi [0:3];
    reg signed [24:0] reg_wr [0:3];
    reg signed [24:0] reg_wi [0:3];
    reg               reg_accum_en;

    integer k;

    // Combinational micro-op decode
    reg signed [24:0] op_a_next;
    reg signed [15:0] op_b_next;
    reg               dest_is_y_re_next;
    reg               dest_is_y_im_next;
    reg               dest_is_cov_next;
    reg [3:0]         cov_idx_next;
    reg               sub_next;

    wire [1:0] m_bf = u_cnt[3:2]; // sensor index for beamforming 0..3

    always @(*) begin
        // Defaults
        op_a_next         = 25'sd0;
        op_b_next         = 16'sd0;
        dest_is_y_re_next = 1'b0;
        dest_is_y_im_next = 1'b0;
        dest_is_cov_next  = 1'b0;
        cov_idx_next      = 4'd0;
        sub_next          = 1'b0;

        if (u_cnt < 6'd16) begin
            // Beamformer micro-ops
            case (u_cnt[1:0])
                2'b00: begin // + wr * xr -> y_re
                    op_a_next         = reg_wr[m_bf];
                    op_b_next         = reg_xr[m_bf];
                    dest_is_y_re_next = 1'b1;
                    sub_next          = 1'b0;
                end
                2'b01: begin // + wi * xi -> y_re
                    op_a_next         = reg_wi[m_bf];
                    op_b_next         = reg_xi[m_bf];
                    dest_is_y_re_next = 1'b1;
                    sub_next          = 1'b0;
                end
                2'b10: begin // + wr * xi -> y_im
                    op_a_next         = reg_wr[m_bf];
                    op_b_next         = reg_xi[m_bf];
                    dest_is_y_im_next = 1'b1;
                    sub_next          = 1'b0;
                end
                2'b11: begin // - wi * xr -> y_im
                    op_a_next         = reg_wi[m_bf];
                    op_b_next         = reg_xr[m_bf];
                    dest_is_y_im_next = 1'b1;
                    sub_next          = 1'b1;
                end
            endcase
        end else if (u_cnt < 6'd24) begin
            // Diagonal covariance S0..S3
            // u = 16..23, 2 micro-ops per diagonal
            dest_is_cov_next = 1'b1;
            sub_next         = 1'b0;
            case (u_cnt - 6'd16)
                6'd0: begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xr[0]; cov_idx_next = 4'd0; end
                6'd1: begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xi[0]; cov_idx_next = 4'd0; end
                6'd2: begin op_a_next = {{9{reg_xr[1][15]}}, reg_xr[1]}; op_b_next = reg_xr[1]; cov_idx_next = 4'd1; end
                6'd3: begin op_a_next = {{9{reg_xi[1][15]}}, reg_xi[1]}; op_b_next = reg_xi[1]; cov_idx_next = 4'd1; end
                6'd4: begin op_a_next = {{9{reg_xr[2][15]}}, reg_xr[2]}; op_b_next = reg_xr[2]; cov_idx_next = 4'd2; end
                6'd5: begin op_a_next = {{9{reg_xi[2][15]}}, reg_xi[2]}; op_b_next = reg_xi[2]; cov_idx_next = 4'd2; end
                6'd6: begin op_a_next = {{9{reg_xr[3][15]}}, reg_xr[3]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd3; end
                6'd7: begin op_a_next = {{9{reg_xi[3][15]}}, reg_xi[3]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd3; end
            endcase
        end else begin
            // Off-diagonal covariance S4..S15
            // u = 24..47
            dest_is_cov_next = 1'b1;
            case (u_cnt - 6'd24)
                // Pair (0, 1): S4 (Re), S10 (Im)
                6'd0:  begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xr[1]; cov_idx_next = 4'd4;  sub_next = 1'b0; end
                6'd1:  begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xi[1]; cov_idx_next = 4'd4;  sub_next = 1'b0; end
                6'd2:  begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xr[1]; cov_idx_next = 4'd10; sub_next = 1'b0; end
                6'd3:  begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xi[1]; cov_idx_next = 4'd10; sub_next = 1'b1; end

                // Pair (0, 2): S5 (Re), S11 (Im)
                6'd4:  begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xr[2]; cov_idx_next = 4'd5;  sub_next = 1'b0; end
                6'd5:  begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xi[2]; cov_idx_next = 4'd5;  sub_next = 1'b0; end
                6'd6:  begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xr[2]; cov_idx_next = 4'd11; sub_next = 1'b0; end
                6'd7:  begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xi[2]; cov_idx_next = 4'd11; sub_next = 1'b1; end

                // Pair (0, 3): S6 (Re), S12 (Im)
                6'd8:  begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd6;  sub_next = 1'b0; end
                6'd9:  begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd6;  sub_next = 1'b0; end
                6'd10: begin op_a_next = {{9{reg_xi[0][15]}}, reg_xi[0]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd12; sub_next = 1'b0; end
                6'd11: begin op_a_next = {{9{reg_xr[0][15]}}, reg_xr[0]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd12; sub_next = 1'b1; end

                // Pair (1, 2): S7 (Re), S13 (Im)
                6'd12: begin op_a_next = {{9{reg_xr[1][15]}}, reg_xr[1]}; op_b_next = reg_xr[2]; cov_idx_next = 4'd7;  sub_next = 1'b0; end
                6'd13: begin op_a_next = {{9{reg_xi[1][15]}}, reg_xi[1]}; op_b_next = reg_xi[2]; cov_idx_next = 4'd7;  sub_next = 1'b0; end
                6'd14: begin op_a_next = {{9{reg_xi[1][15]}}, reg_xi[1]}; op_b_next = reg_xr[2]; cov_idx_next = 4'd13; sub_next = 1'b0; end
                6'd15: begin op_a_next = {{9{reg_xr[1][15]}}, reg_xr[1]}; op_b_next = reg_xi[2]; cov_idx_next = 4'd13; sub_next = 1'b1; end

                // Pair (1, 3): S8 (Re), S14 (Im)
                6'd16: begin op_a_next = {{9{reg_xr[1][15]}}, reg_xr[1]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd8;  sub_next = 1'b0; end
                6'd17: begin op_a_next = {{9{reg_xi[1][15]}}, reg_xi[1]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd8;  sub_next = 1'b0; end
                6'd18: begin op_a_next = {{9{reg_xi[1][15]}}, reg_xi[1]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd14; sub_next = 1'b0; end
                6'd19: begin op_a_next = {{9{reg_xr[1][15]}}, reg_xr[1]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd14; sub_next = 1'b1; end

                // Pair (2, 3): S9 (Re), S15 (Im)
                6'd20: begin op_a_next = {{9{reg_xr[2][15]}}, reg_xr[2]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd9;  sub_next = 1'b0; end
                6'd21: begin op_a_next = {{9{reg_xi[2][15]}}, reg_xi[2]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd9;  sub_next = 1'b0; end
                6'd22: begin op_a_next = {{9{reg_xi[2][15]}}, reg_xi[2]}; op_b_next = reg_xr[3]; cov_idx_next = 4'd15; sub_next = 1'b0; end
                6'd23: begin op_a_next = {{9{reg_xr[2][15]}}, reg_xr[2]}; op_b_next = reg_xi[3]; cov_idx_next = 4'd15; sub_next = 1'b1; end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline stage 1: Multiplier operands & control registers
    // ------------------------------------------------------------------------
    reg signed [24:0] op_a_p1;
    reg signed [15:0] op_b_p1;
    reg               dest_is_y_re_p1;
    reg               dest_is_y_im_p1;
    reg               dest_is_cov_p1;
    reg [3:0]         cov_idx_p1;
    reg               sub_p1;
    reg               valid_p1;

    always @(posedge clk) begin
        if (rst) begin
            op_a_p1         <= 25'sd0;
            op_b_p1         <= 16'sd0;
            dest_is_y_re_p1 <= 1'b0;
            dest_is_y_im_p1 <= 1'b0;
            dest_is_cov_p1  <= 1'b0;
            cov_idx_p1      <= 4'd0;
            sub_p1          <= 1'b0;
            valid_p1        <= 1'b0;
        end else if (state == S_RUN) begin
            op_a_p1         <= op_a_next;
            op_b_p1         <= op_b_next;
            dest_is_y_re_p1 <= dest_is_y_re_next;
            dest_is_y_im_p1 <= dest_is_y_im_next;
            dest_is_cov_p1  <= dest_is_cov_next;
            cov_idx_p1      <= cov_idx_next;
            sub_p1          <= sub_next;
            valid_p1        <= 1'b1;
        end else begin
            valid_p1        <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline stage 2: Multiplier product & control
    // ------------------------------------------------------------------------
    reg signed [40:0] mult_prod_p2;
    reg               dest_is_y_re_p2;
    reg               dest_is_y_im_p2;
    reg               dest_is_cov_p2;
    reg [3:0]         cov_idx_p2;
    reg               sub_p2;
    reg               valid_p2;

    always @(posedge clk) begin
        if (rst) begin
            mult_prod_p2    <= 41'sd0;
            dest_is_y_re_p2 <= 1'b0;
            dest_is_y_im_p2 <= 1'b0;
            dest_is_cov_p2  <= 1'b0;
            cov_idx_p2      <= 4'd0;
            sub_p2          <= 1'b0;
            valid_p2        <= 1'b0;
        end else begin
            mult_prod_p2    <= op_a_p1 * op_b_p1;
            dest_is_y_re_p2 <= dest_is_y_re_p1;
            dest_is_y_im_p2 <= dest_is_y_im_p1;
            dest_is_cov_p2  <= dest_is_cov_p1;
            cov_idx_p2      <= cov_idx_p1;
            sub_p2          <= sub_p1;
            valid_p2        <= valid_p1;
        end
    end

    // ------------------------------------------------------------------------
    // Accumulation logic
    // ------------------------------------------------------------------------
    reg signed [43:0] y_re_acc; // Q9.35 signed
    reg signed [43:0] y_im_acc; // Q9.35 signed

    wire signed [ACC_W-1:0] cov_term = {{(ACC_W-32){mult_prod_p2[31]}}, mult_prod_p2[31:0]};

    always @(posedge clk) begin
        if (rst) begin
            y_re_acc <= 44'sd0;
            y_im_acc <= 44'sd0;
            for (k = 0; k < 16; k = k + 1) begin
                S[k] <= {ACC_W{1'b0}};
            end
        end else begin
            if (clear_cov) begin
                for (k = 0; k < 16; k = k + 1) begin
                    S[k] <= {ACC_W{1'b0}};
                end
            end

            if (sample_start) begin
                y_re_acc <= 44'sd0;
                y_im_acc <= 44'sd0;
            end else if (valid_p2) begin
                if (dest_is_y_re_p2) begin
                    y_re_acc <= y_re_acc + (sub_p2 ? -mult_prod_p2 : mult_prod_p2);
                end else if (dest_is_y_im_p2) begin
                    y_im_acc <= y_im_acc + (sub_p2 ? -mult_prod_p2 : mult_prod_p2);
                end else if (dest_is_cov_p2 && reg_accum_en) begin
                    S[cov_idx_p2] <= S[cov_idx_p2] + (sub_p2 ? -cov_term : cov_term);
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Output rounding and saturation: Q9.35 -> Q2.22 (shift 13, round half-up)
    // ------------------------------------------------------------------------
    wire signed [43:0] y_re_plus_half = y_re_acc + 44'sd4096;
    wire signed [30:0] y_re_shifted   = y_re_plus_half >>> 13;

    wire signed [43:0] y_im_plus_half = y_im_acc + 44'sd4096;
    wire signed [30:0] y_im_shifted   = y_im_plus_half >>> 13;

    reg signed [23:0] y_re_sat;
    reg signed [23:0] y_im_sat;
    reg               sat_detected;

    always @(*) begin
        sat_detected = 1'b0;

        // Saturate real part to 24-bit signed [-2^23, 2^23 - 1]
        if (y_re_shifted > 31'sd8388607) begin
            y_re_sat     = 24'sh7F_FFFF;
            sat_detected = 1'b1;
        end else if (y_re_shifted < -31'sd8388608) begin
            y_re_sat     = 24'sh80_0000;
            sat_detected = 1'b1;
        end else begin
            y_re_sat     = y_re_shifted[23:0];
        end

        // Saturate imag part
        if (y_im_shifted > 31'sd8388607) begin
            y_im_sat     = 24'sh7F_FFFF;
            sat_detected = 1'b1;
        end else if (y_im_shifted < -31'sd8388608) begin
            y_im_sat     = 24'sh80_0000;
            sat_detected = 1'b1;
        end else begin
            y_im_sat     = y_im_shifted[23:0];
        end
    end

    // ------------------------------------------------------------------------
    // Engine FSM
    // ------------------------------------------------------------------------
    reg [2:0] post_cnt;

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            u_cnt        <= 6'd0;
            busy         <= 1'b0;
            sample_done  <= 1'b0;
            y_valid      <= 1'b0;
            y_out        <= 48'd0;
            sat_y        <= 1'b0;
            post_cnt     <= 3'd0;
            reg_accum_en <= 1'b0;
            for (k = 0; k < 4; k = k + 1) begin
                reg_xr[k] <= 16'sd0;
                reg_xi[k] <= 16'sd0;
                reg_wr[k] <= 25'sd0;
                reg_wi[k] <= 25'sd0;
            end
        end else begin
            sample_done <= 1'b0;
            y_valid     <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy  <= 1'b0;
                    u_cnt <= 6'd0;
                    if (sample_start) begin
                        busy         <= 1'b1;
                        reg_accum_en <= accum_en;
                        for (k = 0; k < 4; k = k + 1) begin
                            reg_xr[k] <= xr[k];
                            reg_xi[k] <= xi[k];
                            reg_wr[k] <= wr[k];
                            reg_wi[k] <= wi[k];
                        end
                        state <= S_RUN;
                    end
                end

                S_RUN: begin
                    // Beamformer result is available after u_cnt=15 + 2 pipe stages = cycle 18
                    if (u_cnt == 6'd18) begin
                        y_out   <= {y_im_sat, y_re_sat};
                        y_valid <= 1'b1;
                        if (sat_detected) begin
                            sat_y <= 1'b1;
                        end
                    end

                    if (u_cnt == 6'd47) begin
                        state    <= S_DONE;
                        post_cnt <= 3'd2; // wait 2 cycles for pipeline drain
                    end else begin
                        u_cnt <= u_cnt + 6'd1;
                    end
                end

                S_DONE: begin
                    if (post_cnt == 3'd0) begin
                        sample_done <= 1'b1;
                        busy        <= 1'b0;
                        state       <= S_IDLE;
                    end else begin
                        post_cnt <= post_cnt - 3'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
