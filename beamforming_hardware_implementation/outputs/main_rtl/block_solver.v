// ============================================================================
// block_solver.v -- Causal MVDR Block Solver
// Compatible with Verilog-2001.
// Performs:
//   1. Persistent covariance update: R_new = S (block 0) or R + 0.2*(S - R)
//   2. Trace calculation and power-of-two block normalization to Q16.24
//   3. Diagonal loading: delta = 2.5e-4 * Tn
//   4. Complex Hermitian LDL^H factorization: A = U^H * D * U
//   5. Forward solve: U^H * z = a
//   6. Diagonal solve: D * v = z (v = z * r)
//   7. Backward solve: U * u = v
//   8. Normalization: s = Re(a^H * u), rs = 1/s, w = u * rs (Q5.20)
// Exposes w_wr interface to weight_regs.
// ============================================================================

`timescale 1ns/1ps

module block_solver #(
    parameter BLOCK_LEN             = 76,
    parameter ACC_W                 = 39,
    parameter LAMBDA_ONE_MINUS_COEF = 3355443, // round(0.2 * 2^24)
    parameter LOAD_COEF             = 1073742, // round(2.5e-4 * 2^32)
    parameter D_MIN                 = 1024     // 2^-14 in Q16.24
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     solve_start,
    input  wire [127:0]             a_vec,       // steering vector Q2.14: {s3, s2, s1, s0}
    output reg  [3:0]               S_rd_addr,
    input  wire [ACC_W-1:0]         S_rd_data,

    output reg                      w_wr_en,
    output reg  [2:0]               w_wr_idx,    // 0..3 re, 4..7 im
    output reg  signed [24:0]       w_wr_data,   // Q5.20
    output reg                      solve_done,
    output reg  [31:0]              solve_cycles,

    output reg                      err_pivot,
    output reg                      err_solver,
    output reg                      sat_solver,
    output reg                      sat_w,
    output reg                      info_zero_trace
);

    // ------------------------------------------------------------------------
    // Persistent Covariance State (16 words)
    // ------------------------------------------------------------------------
    reg signed [ACC_W-1:0] R [0:15];
    reg                    have_cov;

    // Buffer for S read from sample_engine
    reg signed [ACC_W-1:0] S_buf [0:15];

    // Normalized A matrix in Q16.24: 4 diagonal (real), 6 off-diagonal (complex)
    reg signed [39:0] A_diag [0:3];
    reg signed [39:0] A_off_re [0:5];
    reg signed [39:0] A_off_im [0:5];

    // Steering vector in Q16.24: a_Q24 = a_Q14 << 10
    reg signed [39:0] a_re [0:3];
    reg signed [39:0] a_im [0:3];

    // LDL results
    reg signed [39:0] d [0:3];     // real pivots
    reg signed [39:0] r [0:3];     // pivot reciprocals
    reg signed [39:0] T_re [0:5];  // 6 off-diagonals (01, 02, 03, 12, 13, 23)
    reg signed [39:0] T_im [0:5];
    reg signed [39:0] U_re [0:5];
    reg signed [39:0] U_im [0:5];

    // Linear solve intermediate vectors
    reg signed [39:0] z_re [0:3];
    reg signed [39:0] z_im [0:3];
    reg signed [39:0] v_re [0:3];
    reg signed [39:0] v_im [0:3];
    reg signed [39:0] u_re [0:3];
    reg signed [39:0] u_im [0:3];

    reg signed [39:0] s_scalar;
    reg [64:0]        rs_Q40;

    // ------------------------------------------------------------------------
    // Serial Divider Instance
    // ------------------------------------------------------------------------
    reg         div_start;
    reg  [64:0] div_n;
    reg  [39:0] div_d;
    wire [64:0] div_q;
    wire [64:0] div_q_rnd;
    wire [39:0] div_rem;
    wire        div_busy;
    wire        div_done;
    wire        div_dbz;

    serial_divider #(
        .WIDTH_N(65),
        .WIDTH_D(40)
    ) u_div (
        .clk(clk),
        .rst(rst),
        .start(div_start),
        .dividend(div_n),
        .divisor(div_d),
        .quotient(div_q),
        .quotient_rnd(div_q_rnd),
        .remainder(div_rem),
        .busy(div_busy),
        .done(div_done),
        .div_by_zero(div_dbz)
    );

    // ------------------------------------------------------------------------
    // Wide Accumulator & Round-Saturate logic for solver
    // ------------------------------------------------------------------------
    reg signed [87:0] wide_acc;

    function signed [39:0] round_sat_acc;
        input signed [87:0] acc_in;
        reg signed [87:0] acc_rnd;
        reg signed [63:0] acc_shift;
        begin
            acc_rnd = acc_in + 88'sh0000_0000_0000_0080_0000; // add 2^23
            acc_shift = acc_rnd >>> 24;
            if (acc_shift > 64'sh0000_007F_FFFF_FFFF) begin
                round_sat_acc = 40'sh7F_FFFF_FFFF;
            end else if (acc_shift < -64'sh0000_0080_0000_0000) begin
                round_sat_acc = -40'sh80_0000_0000;
            end else begin
                round_sat_acc = acc_shift[39:0];
            end
        end
    endfunction

    // ------------------------------------------------------------------------
    // FSM States
    // ------------------------------------------------------------------------
    localparam ST_IDLE          = 5'd0;
    localparam ST_READ_S        = 5'd1;
    localparam ST_UPDATE_COV    = 5'd2;
    localparam ST_TRACE         = 5'd3;
    localparam ST_NORM_R        = 5'd4;
    localparam ST_DIAG_LOAD     = 5'd5;
    localparam ST_LDL_PIVOT     = 5'd6;
    localparam ST_WAIT_DIV_PIV  = 5'd7;
    localparam ST_LDL_OFFDIAG   = 5'd8;
    localparam ST_SOLVE_FWD     = 5'd9;
    localparam ST_SOLVE_DIAG    = 5'd10;
    localparam ST_SOLVE_BWD     = 5'd11;
    localparam ST_NORM_S        = 5'd12;
    localparam ST_WAIT_DIV_RS   = 5'd13;
    localparam ST_SCALE_W       = 5'd14;
    localparam ST_FALLBACK_W    = 5'd15;
    localparam ST_DONE          = 5'd16;

    reg [4:0]  state;
    reg [5:0]  step;
    reg [3:0]  idx_cnt;
    reg [31:0] cycle_cnt;

    // Registers for trace and norm
    reg [ACC_W+1:0] trace_int;
    reg [5:0]       msb_p;
    integer         b, k;
    reg signed [64:0] diff_cov;
    reg signed [64:0] prod_cov;

    // Helper wires for proper sign extension from a_vec
    wire signed [15:0] a_re_slice = a_vec[32*step[1:0] +: 16];
    wire signed [15:0] a_im_slice = a_vec[32*step[1:0] + 16 +: 16];

    // Helper function for 40x40 multiply to 80-bit signed
    function signed [79:0] mul40;
        input signed [39:0] op1;
        input signed [39:0] op2;
        begin
            mul40 = op1 * op2;
        end
    endfunction

    // ------------------------------------------------------------------------
    // FSM Implementation
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state           <= ST_IDLE;
            step            <= 6'd0;
            idx_cnt         <= 4'd0;
            cycle_cnt       <= 32'd0;
            solve_cycles    <= 32'd0;
            have_cov        <= 1'b0;
            S_rd_addr       <= 4'd0;
            w_wr_en         <= 1'b0;
            w_wr_idx        <= 3'd0;
            w_wr_data       <= 25'sd0;
            solve_done      <= 1'b0;
            err_pivot       <= 1'b0;
            err_solver      <= 1'b0;
            sat_solver      <= 1'b0;
            sat_w           <= 1'b0;
            info_zero_trace <= 1'b0;
            div_start       <= 1'b0;
            div_n           <= 65'd0;
            div_d           <= 40'd0;
            trace_int       <= {(ACC_W+2){1'b0}};
            msb_p           <= 6'd0;
            wide_acc        <= 88'sd0;
            for (k = 0; k < 16; k = k + 1) begin
                R[k]     <= {ACC_W{1'b0}};
                S_buf[k] <= {ACC_W{1'b0}};
            end
        end else begin
            solve_done <= 1'b0;
            w_wr_en    <= 1'b0;
            div_start  <= 1'b0;

            if (state != ST_IDLE) begin
                cycle_cnt <= cycle_cnt + 32'd1;
            end

            case (state)
                // ------------------------------------------------------------
                // Wait for solve start
                // ------------------------------------------------------------
                ST_IDLE: begin
                    if (solve_start) begin
                        cycle_cnt <= 32'd0;
                        S_rd_addr <= 4'd0;
                        state     <= ST_READ_S;
                        step      <= 6'd0;
                    end
                end

                // ------------------------------------------------------------
                // Read 16 S words from sample_engine
                // ------------------------------------------------------------
                ST_READ_S: begin
                    S_buf[step[3:0]] <= S_rd_data;
                    if (step == 6'd15) begin
                        state <= ST_UPDATE_COV;
                        step  <= 6'd0;
                    end else begin
                        step      <= step + 6'd1;
                        S_rd_addr <= step[3:0] + 4'd1;
                    end
                end

                // ------------------------------------------------------------
                // Covariance update: R_new = S or R + 0.2*(S - R)
                // ------------------------------------------------------------
                ST_UPDATE_COV: begin
                    if (!have_cov) begin
                        // First block: R = S
                        for (k = 0; k < 16; k = k + 1) begin
                            R[k] <= S_buf[k];
                        end
                        have_cov <= 1'b1;
                        state    <= ST_TRACE;
                    end else begin
                        // Later blocks: R = R + round((S - R) * C1 / 2^24)
                        diff_cov = S_buf[step[3:0]] - R[step[3:0]];
                        prod_cov = diff_cov * $signed({1'b0, LAMBDA_ONE_MINUS_COEF});
                        R[step[3:0]] <= R[step[3:0]] + ((prod_cov + 65'sh0000_0000_0080_0000) >>> 24);

                        if (step == 6'd15) begin
                            state <= ST_TRACE;
                            step  <= 6'd0;
                        end else begin
                            step <= step + 6'd1;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Trace calculation: T = sum of 4 diagonal entries
                // ------------------------------------------------------------
                ST_TRACE: begin
                    trace_int = R[0] + R[1] + R[2] + R[3];
                    if (trace_int == 0) begin
                        info_zero_trace <= 1'b1;
                        state           <= ST_FALLBACK_W;
                        step            <= 6'd0;
                    end else begin
                        msb_p = 6'd0;
                        for (b = 0; b < ACC_W + 2; b = b + 1) begin
                            if (trace_int[b]) msb_p = b[5:0];
                        end
                        state <= ST_NORM_R;
                        step  <= 6'd0;
                    end
                end

                // ------------------------------------------------------------
                // Normalize R to Q16.24: shift by (24 - msb_p)
                // ------------------------------------------------------------
                ST_NORM_R: begin
                    for (k = 0; k < 4; k = k + 1) begin
                        if (msb_p <= 6'd24) begin
                            A_diag[k] <= $signed(R[k]) <<< (6'd24 - msb_p);
                        end else begin
                            A_diag[k] <= ($signed(R[k]) + (40'sh1 <<< (msb_p - 6'd25))) >>> (msb_p - 6'd24);
                        end
                    end
                    for (k = 0; k < 6; k = k + 1) begin
                        if (msb_p <= 6'd24) begin
                            A_off_re[k] <= $signed(R[4 + k]) <<< (6'd24 - msb_p);
                            A_off_im[k] <= $signed(R[10 + k]) <<< (6'd24 - msb_p);
                        end else begin
                            A_off_re[k] <= ($signed(R[4 + k]) + (40'sh1 <<< (msb_p - 6'd25))) >>> (msb_p - 6'd24);
                            A_off_im[k] <= ($signed(R[10 + k]) + (40'sh1 <<< (msb_p - 6'd25))) >>> (msb_p - 6'd24);
                        end
                    end

                    // Convert steering vector from Q2.14 to Q16.24 (exact sign-extend and shift left by 10)
                    for (k = 0; k < 4; k = k + 1) begin
                        begin : b_a_slice
                            reg signed [15:0] re_s, im_s;
                            re_s = a_vec[32*k +: 16];
                            im_s = a_vec[32*k + 16 +: 16];
                            a_re[k] <= {{14{re_s[15]}}, re_s, 10'd0};
                            a_im[k] <= {{14{im_s[15]}}, im_s, 10'd0};
                        end
                    end

                    state <= ST_DIAG_LOAD;
                end

                // ------------------------------------------------------------
                // Diagonal Loading: delta = (Tn * LOAD_COEF + 2^31) >> 32
                // ------------------------------------------------------------
                ST_DIAG_LOAD: begin
                    begin : b_load
                        reg [63:0] tn_prod;
                        reg [39:0] delta;
                        reg [39:0] tn_sum;
                        tn_sum = A_diag[0] + A_diag[1] + A_diag[2] + A_diag[3];
                        tn_prod = tn_sum[31:0] * LOAD_COEF + 64'h0000_0000_8000_0000;
                        delta = {8'd0, tn_prod[63:32]};

                        A_diag[0] <= A_diag[0] + delta;
                        A_diag[1] <= A_diag[1] + delta;
                        A_diag[2] <= A_diag[2] + delta;
                        A_diag[3] <= A_diag[3] + delta;
                    end
                    state   <= ST_LDL_PIVOT;
                    idx_cnt <= 4'd0; // i = 0
                    step    <= 6'd0;
                end

                // ------------------------------------------------------------
                // LDL Factorization: Pivot d_i and start reciprocal r_i = 1/d_i
                // ------------------------------------------------------------
                ST_LDL_PIVOT: begin
                    case (idx_cnt[1:0])
                        2'd0: begin
                            // i = 0: d0 = A_diag[0]
                            if (A_diag[0] < D_MIN) begin
                                d[0]      <= D_MIN;
                                err_pivot <= 1'b1;
                                div_d     <= D_MIN;
                            end else begin
                                d[0]  <= A_diag[0];
                                div_d <= A_diag[0];
                            end
                            div_n     <= 65'h0001_0000_0000_0000; // 2^48
                            div_start <= 1'b1;
                            state     <= ST_WAIT_DIV_PIV;
                        end

                        2'd1: begin
                            // i = 1: d1 = A_diag[1] - (U01_r*T01_r + U01_i*T01_i)
                            case (step)
                                6'd0: begin
                                    wide_acc <= (A_diag[1] <<< 24) - mul40(U_re[0], T_re[0]);
                                    step     <= 6'd1;
                                end
                                6'd1: begin
                                    begin : b_piv1
                                        reg signed [39:0] d_val;
                                        d_val = round_sat_acc(wide_acc - mul40(U_im[0], T_im[0]));
                                        if (d_val < D_MIN) begin
                                            d[1]      <= D_MIN;
                                            err_pivot <= 1'b1;
                                            div_d     <= D_MIN;
                                        end else begin
                                            d[1]  <= d_val;
                                            div_d <= d_val;
                                        end
                                    end
                                    div_n     <= 65'h0001_0000_0000_0000;
                                    div_start <= 1'b1;
                                    state     <= ST_WAIT_DIV_PIV;
                                end
                            endcase
                        end

                        2'd2: begin
                            // i = 2: d2 = A_diag[2] - (U02_r*T02_r + U02_i*T02_i) - (U12_r*T12_r + U12_i*T12_i)
                            case (step)
                                6'd0: begin wide_acc <= (A_diag[2] <<< 24) - mul40(U_re[1], T_re[1]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc - mul40(U_im[1], T_im[1]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[3], T_re[3]);           step <= 6'd3; end
                                6'd3: begin
                                    begin : b_piv2
                                        reg signed [39:0] d_val;
                                        d_val = round_sat_acc(wide_acc - mul40(U_im[3], T_im[3]));
                                        if (d_val < D_MIN) begin
                                            d[2]      <= D_MIN;
                                            err_pivot <= 1'b1;
                                            div_d     <= D_MIN;
                                        end else begin
                                            d[2]  <= d_val;
                                            div_d <= d_val;
                                        end
                                    end
                                    div_n     <= 65'h0001_0000_0000_0000;
                                    div_start <= 1'b1;
                                    state     <= ST_WAIT_DIV_PIV;
                                end
                            endcase
                        end

                        2'd3: begin
                            // i = 3: d3 = A_diag[3] - sum (Uki_r*Tki_r + Uki_i*Tki_i) for k=0,1,2
                            case (step)
                                6'd0: begin wide_acc <= (A_diag[3] <<< 24) - mul40(U_re[2], T_re[2]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc - mul40(U_im[2], T_im[2]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[4], T_re[4]);           step <= 6'd3; end
                                6'd3: begin wide_acc <= wide_acc - mul40(U_im[4], T_im[4]);           step <= 6'd4; end
                                6'd4: begin wide_acc <= wide_acc - mul40(U_re[5], T_re[5]);           step <= 6'd5; end
                                6'd5: begin
                                    begin : b_piv3
                                        reg signed [39:0] d_val;
                                        d_val = round_sat_acc(wide_acc - mul40(U_im[5], T_im[5]));
                                        if (d_val < D_MIN) begin
                                            d[3]      <= D_MIN;
                                            err_pivot <= 1'b1;
                                            div_d     <= D_MIN;
                                        end else begin
                                            d[3]  <= d_val;
                                            div_d <= d_val;
                                        end
                                    end
                                    div_n     <= 65'h0001_0000_0000_0000;
                                    div_start <= 1'b1;
                                    state     <= ST_WAIT_DIV_PIV;
                                end
                            endcase
                        end
                    endcase
                end

                // ------------------------------------------------------------
                // Wait for pivot divider to finish r_i = 1/d_i
                // ------------------------------------------------------------
                ST_WAIT_DIV_PIV: begin
                    if (div_done) begin
                        r[idx_cnt[1:0]] <= div_q_rnd[39:0];
                        if (idx_cnt == 4'd3) begin
                            state   <= ST_SOLVE_FWD;
                            idx_cnt <= 4'd0;
                            step    <= 6'd0;
                        end else begin
                            state <= ST_LDL_OFFDIAG;
                            step  <= 6'd0;
                        end
                    end
                end

                // ------------------------------------------------------------
                // LDL Off-diagonal terms: T_ij and U_ij = T_ij * r_i
                // ------------------------------------------------------------
                ST_LDL_OFFDIAG: begin
                    case (idx_cnt[1:0])
                        2'd0: begin
                            // i = 0: for j = 1, 2, 3:
                            // T01 = A01, T02 = A02, T03 = A03
                            // U0j = T0j * r0
                            case (step)
                                6'd0: begin
                                    T_re[0] <= A_off_re[0]; T_im[0] <= A_off_im[0];
                                    T_re[1] <= A_off_re[1]; T_im[1] <= A_off_im[1];
                                    T_re[2] <= A_off_re[2]; T_im[2] <= A_off_im[2];
                                    U_re[0] <= round_sat_acc(mul40(A_off_re[0], r[0]));
                                    step    <= 6'd1;
                                end
                                6'd1: begin U_im[0] <= round_sat_acc(mul40(A_off_im[0], r[0])); step <= 6'd2; end
                                6'd2: begin U_re[1] <= round_sat_acc(mul40(A_off_re[1], r[0])); step <= 6'd3; end
                                6'd3: begin U_im[1] <= round_sat_acc(mul40(A_off_im[1], r[0])); step <= 6'd4; end
                                6'd4: begin U_re[2] <= round_sat_acc(mul40(A_off_re[2], r[0])); step <= 6'd5; end
                                6'd5: begin
                                    U_im[2] <= round_sat_acc(mul40(A_off_im[2], r[0]));
                                    idx_cnt <= 4'd1;
                                    step    <= 6'd0;
                                    state   <= ST_LDL_PIVOT;
                                end
                            endcase
                        end

                        2'd1: begin
                            // i = 1: for j = 2, 3:
                            // T12 = A12 - conj(U01)*T02
                            // T13 = A13 - conj(U01)*T03
                            // conj(U)*T = (Ur*Tr + Ui*Ti) + j(Ur*Ti - Ui*Tr)
                            // pair (1,2)=3, (0,1)=0, (0,2)=1, (1,3)=4, (0,3)=2
                            case (step)
                                // T12 real
                                6'd0: begin wide_acc <= (A_off_re[3] <<< 24) - mul40(U_re[0], T_re[1]); step <= 6'd1; end
                                6'd1: begin T_re[3]  <= round_sat_acc(wide_acc - mul40(U_im[0], T_im[1])); step <= 6'd2; end

                                // T12 imag
                                6'd2: begin wide_acc <= (A_off_im[3] <<< 24) - mul40(U_re[0], T_im[1]); step <= 6'd3; end
                                6'd3: begin T_im[3]  <= round_sat_acc(wide_acc + mul40(U_im[0], T_re[1])); step <= 6'd4; end

                                // T13 real
                                6'd4: begin wide_acc <= (A_off_re[4] <<< 24) - mul40(U_re[0], T_re[2]); step <= 6'd5; end
                                6'd5: begin T_re[4]  <= round_sat_acc(wide_acc - mul40(U_im[0], T_im[2])); step <= 6'd6; end

                                // T13 imag
                                6'd6: begin wide_acc <= (A_off_im[4] <<< 24) - mul40(U_re[0], T_im[2]); step <= 6'd7; end
                                6'd7: begin T_im[4]  <= round_sat_acc(wide_acc + mul40(U_im[0], T_re[2])); step <= 6'd8; end

                                // U12, U13
                                6'd8:  begin U_re[3] <= round_sat_acc(mul40(T_re[3], r[1])); step <= 6'd9; end
                                6'd9:  begin U_im[3] <= round_sat_acc(mul40(T_im[3], r[1])); step <= 6'd10; end
                                6'd10: begin U_re[4] <= round_sat_acc(mul40(T_re[4], r[1])); step <= 6'd11; end
                                6'd11: begin
                                    U_im[4] <= round_sat_acc(mul40(T_im[4], r[1]));
                                    idx_cnt <= 4'd2;
                                    step    <= 6'd0;
                                    state   <= ST_LDL_PIVOT;
                                end
                            endcase
                        end

                        2'd2: begin
                            // i = 2: for j = 3:
                            // T23 = A23 - conj(U02)*T03 - conj(U12)*T13
                            // pair (2,3)=5, (0,2)=1, (0,3)=2, (1,2)=3, (1,3)=4
                            case (step)
                                // T23 real
                                6'd0: begin wide_acc <= (A_off_re[5] <<< 24) - mul40(U_re[1], T_re[2]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc - mul40(U_im[1], T_im[2]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[3], T_re[4]);           step <= 6'd3; end
                                6'd3: begin T_re[5]  <= round_sat_acc(wide_acc - mul40(U_im[3], T_im[4])); step <= 6'd4; end

                                // T23 imag
                                6'd4: begin wide_acc <= (A_off_im[5] <<< 24) - mul40(U_re[1], T_im[2]); step <= 6'd5; end
                                6'd5: begin wide_acc <= wide_acc + mul40(U_im[1], T_re[2]);           step <= 6'd6; end
                                6'd6: begin wide_acc <= wide_acc - mul40(U_re[3], T_im[4]);           step <= 6'd7; end
                                6'd7: begin T_im[5]  <= round_sat_acc(wide_acc + mul40(U_im[3], T_re[4])); step <= 6'd8; end

                                // U23
                                6'd8: begin U_re[5] <= round_sat_acc(mul40(T_re[5], r[2])); step <= 6'd9; end
                                6'd9: begin
                                    U_im[5] <= round_sat_acc(mul40(T_im[5], r[2]));
                                    idx_cnt <= 4'd3;
                                    step    <= 6'd0;
                                    state   <= ST_LDL_PIVOT;
                                end
                            endcase
                        end
                    endcase
                end

                // ------------------------------------------------------------
                // Forward Solve: U^H * z = a
                // z_i = a_i - sum_{k<i} conj(U_ki) * z_k
                // ------------------------------------------------------------
                ST_SOLVE_FWD: begin
                    case (idx_cnt[1:0])
                        2'd0: begin
                            z_re[0] <= a_re[0];
                            z_im[0] <= a_im[0];
                            idx_cnt <= 4'd1;
                            step    <= 6'd0;
                        end

                        2'd1: begin
                            // z1 = a1 - conj(U01)*z0
                            case (step)
                                6'd0: begin wide_acc <= (a_re[1] <<< 24) - mul40(U_re[0], z_re[0]); step <= 6'd1; end
                                6'd1: begin z_re[1]  <= round_sat_acc(wide_acc - mul40(U_im[0], z_im[0])); step <= 6'd2; end
                                6'd2: begin wide_acc <= (a_im[1] <<< 24) - mul40(U_re[0], z_im[0]); step <= 6'd3; end
                                6'd3: begin
                                    z_im[1] <= round_sat_acc(wide_acc + mul40(U_im[0], z_re[0]));
                                    idx_cnt <= 4'd2;
                                    step    <= 6'd0;
                                end
                            endcase
                        end

                        2'd2: begin
                            // z2 = a2 - conj(U02)*z0 - conj(U12)*z1
                            case (step)
                                6'd0: begin wide_acc <= (a_re[2] <<< 24) - mul40(U_re[1], z_re[0]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc - mul40(U_im[1], z_im[0]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[3], z_re[1]);           step <= 6'd3; end
                                6'd3: begin z_re[2]  <= round_sat_acc(wide_acc - mul40(U_im[3], z_im[1])); step <= 6'd4; end

                                6'd4: begin wide_acc <= (a_im[2] <<< 24) - mul40(U_re[1], z_im[0]); step <= 6'd5; end
                                6'd5: begin wide_acc <= wide_acc + mul40(U_im[1], z_re[0]);           step <= 6'd6; end
                                6'd6: begin wide_acc <= wide_acc - mul40(U_re[3], z_im[1]);           step <= 6'd7; end
                                6'd7: begin
                                    z_im[2] <= round_sat_acc(wide_acc + mul40(U_im[3], z_re[1]));
                                    idx_cnt <= 4'd3;
                                    step    <= 6'd0;
                                end
                            endcase
                        end

                        2'd3: begin
                            // z3 = a3 - conj(U03)*z0 - conj(U13)*z1 - conj(U23)*z2
                            case (step)
                                6'd0: begin wide_acc <= (a_re[3] <<< 24) - mul40(U_re[2], z_re[0]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc - mul40(U_im[2], z_im[0]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[4], z_re[1]);           step <= 6'd3; end
                                6'd3: begin wide_acc <= wide_acc - mul40(U_im[4], z_im[1]);           step <= 6'd4; end
                                6'd4: begin wide_acc <= wide_acc - mul40(U_re[5], z_re[2]);           step <= 6'd5; end
                                6'd5: begin z_re[3]  <= round_sat_acc(wide_acc - mul40(U_im[5], z_im[2])); step <= 6'd6; end

                                6'd6:  begin wide_acc <= (a_im[3] <<< 24) - mul40(U_re[2], z_im[0]); step <= 6'd7; end
                                6'd7:  begin wide_acc <= wide_acc + mul40(U_im[2], z_re[0]);           step <= 6'd8; end
                                6'd8:  begin wide_acc <= wide_acc - mul40(U_re[4], z_im[1]);           step <= 6'd9; end
                                6'd9:  begin wide_acc <= wide_acc + mul40(U_im[4], z_re[1]);           step <= 6'd10; end
                                6'd10: begin wide_acc <= wide_acc - mul40(U_re[5], z_im[2]);           step <= 6'd11; end
                                6'd11: begin
                                    z_im[3] <= round_sat_acc(wide_acc + mul40(U_im[5], z_re[2]));
                                    state   <= ST_SOLVE_DIAG;
                                    step    <= 6'd0;
                                end
                            endcase
                        end
                    endcase
                end

                // ------------------------------------------------------------
                // Diagonal Solve: v_i = z_i * r_i
                // ------------------------------------------------------------
                ST_SOLVE_DIAG: begin
                    case (step)
                        6'd0: begin v_re[0] <= round_sat_acc(mul40(z_re[0], r[0])); step <= 6'd1; end
                        6'd1: begin v_im[0] <= round_sat_acc(mul40(z_im[0], r[0])); step <= 6'd2; end
                        6'd2: begin v_re[1] <= round_sat_acc(mul40(z_re[1], r[1])); step <= 6'd3; end
                        6'd3: begin v_im[1] <= round_sat_acc(mul40(z_im[1], r[1])); step <= 6'd4; end
                        6'd4: begin v_re[2] <= round_sat_acc(mul40(z_re[2], r[2])); step <= 6'd5; end
                        6'd5: begin v_im[2] <= round_sat_acc(mul40(z_im[2], r[2])); step <= 6'd6; end
                        6'd6: begin v_re[3] <= round_sat_acc(mul40(z_re[3], r[3])); step <= 6'd7; end
                        6'd7: begin
                            v_im[3] <= round_sat_acc(mul40(z_im[3], r[3]));
                            state   <= ST_SOLVE_BWD;
                            idx_cnt <= 4'd3;
                            step    <= 6'd0;
                        end
                    endcase
                end

                // ------------------------------------------------------------
                // Backward Solve: U * u = v
                // u_i = v_i - sum_{k>i} U_ik * u_k (i = 3 down to 0)
                // U*u = (Ur*ur - Ui*ui) + j(Ur*ui + Ui*ur)
                // ------------------------------------------------------------
                ST_SOLVE_BWD: begin
                    case (idx_cnt[1:0])
                        2'd3: begin
                            u_re[3] <= v_re[3];
                            u_im[3] <= v_im[3];
                            idx_cnt <= 4'd2;
                            step    <= 6'd0;
                        end

                        2'd2: begin
                            // u2 = v2 - U23*u3
                            case (step)
                                6'd0: begin wide_acc <= (v_re[2] <<< 24) - mul40(U_re[5], u_re[3]); step <= 6'd1; end
                                6'd1: begin u_re[2]  <= round_sat_acc(wide_acc + mul40(U_im[5], u_im[3])); step <= 6'd2; end
                                6'd2: begin wide_acc <= (v_im[2] <<< 24) - mul40(U_re[5], u_im[3]); step <= 6'd3; end
                                6'd3: begin
                                    u_im[2] <= round_sat_acc(wide_acc - mul40(U_im[5], u_re[3]));
                                    idx_cnt <= 4'd1;
                                    step    <= 6'd0;
                                end
                            endcase
                        end

                        2'd1: begin
                            // u1 = v1 - U12*u2 - U13*u3
                            case (step)
                                6'd0: begin wide_acc <= (v_re[1] <<< 24) - mul40(U_re[3], u_re[2]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc + mul40(U_im[3], u_im[2]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[4], u_re[3]);           step <= 6'd3; end
                                6'd3: begin u_re[1]  <= round_sat_acc(wide_acc + mul40(U_im[4], u_im[3])); step <= 6'd4; end

                                6'd4: begin wide_acc <= (v_im[1] <<< 24) - mul40(U_re[3], u_im[2]); step <= 6'd5; end
                                6'd5: begin wide_acc <= wide_acc - mul40(U_im[3], u_re[2]);           step <= 6'd6; end
                                6'd6: begin wide_acc <= wide_acc - mul40(U_re[4], u_im[3]);           step <= 6'd7; end
                                6'd7: begin
                                    u_im[1] <= round_sat_acc(wide_acc - mul40(U_im[4], u_re[3]));
                                    idx_cnt <= 4'd0;
                                    step    <= 6'd0;
                                end
                            endcase
                        end

                        2'd0: begin
                            // u0 = v0 - U01*u1 - U02*u2 - U03*u3
                            case (step)
                                6'd0: begin wide_acc <= (v_re[0] <<< 24) - mul40(U_re[0], u_re[1]); step <= 6'd1; end
                                6'd1: begin wide_acc <= wide_acc + mul40(U_im[0], u_im[1]);           step <= 6'd2; end
                                6'd2: begin wide_acc <= wide_acc - mul40(U_re[1], u_re[2]);           step <= 6'd3; end
                                6'd3: begin wide_acc <= wide_acc + mul40(U_im[1], u_im[2]);           step <= 6'd4; end
                                6'd4: begin wide_acc <= wide_acc - mul40(U_re[2], u_re[3]);           step <= 6'd5; end
                                6'd5: begin u_re[0]  <= round_sat_acc(wide_acc + mul40(U_im[2], u_im[3])); step <= 6'd6; end

                                6'd6:  begin wide_acc <= (v_im[0] <<< 24) - mul40(U_re[0], u_im[1]); step <= 6'd7; end
                                6'd7:  begin wide_acc <= wide_acc - mul40(U_im[0], u_re[1]);           step <= 6'd8; end
                                6'd8:  begin wide_acc <= wide_acc - mul40(U_re[1], u_im[2]);           step <= 6'd9; end
                                6'd9:  begin wide_acc <= wide_acc - mul40(U_im[1], u_re[2]);           step <= 6'd10; end
                                6'd10: begin wide_acc <= wide_acc - mul40(U_re[2], u_im[3]);           step <= 6'd11; end
                                6'd11: begin
                                    u_im[0] <= round_sat_acc(wide_acc - mul40(U_im[2], u_re[3]));
                                    state   <= ST_NORM_S;
                                    step    <= 6'd0;
                                end
                            endcase
                        end
                    endcase
                end

                // ------------------------------------------------------------
                // Normalization: s = Re(sum_i conj(a_i)*u_i) = sum (ar*ur + ai*ui)
                // ------------------------------------------------------------
                ST_NORM_S: begin
                    case (step)
                        6'd0: begin wide_acc <= mul40(a_re[0], u_re[0]); step <= 6'd1; end
                        6'd1: begin wide_acc <= wide_acc + mul40(a_im[0], u_im[0]); step <= 6'd2; end
                        6'd2: begin wide_acc <= wide_acc + mul40(a_re[1], u_re[1]); step <= 6'd3; end
                        6'd3: begin wide_acc <= wide_acc + mul40(a_im[1], u_im[1]); step <= 6'd4; end
                        6'd4: begin wide_acc <= wide_acc + mul40(a_re[2], u_re[2]); step <= 6'd5; end
                        6'd5: begin wide_acc <= wide_acc + mul40(a_im[2], u_im[2]); step <= 6'd6; end
                        6'd6: begin wide_acc <= wide_acc + mul40(a_re[3], u_re[3]); step <= 6'd7; end
                        6'd7: begin
                            begin : b_s_calc
                                reg signed [39:0] s_val;
                                s_val = round_sat_acc(wide_acc + mul40(a_im[3], u_im[3]));
                                s_scalar <= s_val;

                                if (s_val <= 40'sd1024) begin
                                    err_solver <= 1'b1;
                                    state      <= ST_FALLBACK_W;
                                    step       <= 6'd0;
                                end else begin
                                    div_n     <= {1'b1, 64'd0}; // 2^64
                                    div_d     <= s_val;
                                    div_start <= 1'b1;
                                    state     <= ST_WAIT_DIV_RS;
                                end
                            end
                        end
                    endcase
                end

                // ------------------------------------------------------------
                // Wait for divider to finish rs = 2^64 / s
                // ------------------------------------------------------------
                ST_WAIT_DIV_RS: begin
                    if (div_done) begin
                        rs_Q40 <= div_q_rnd;
                        state  <= ST_SCALE_W;
                        step   <= 6'd0;
                    end
                end

                // ------------------------------------------------------------
                // Weight calculation: w_i = round_half_up(u_i * rs_Q40 / 2^44)
                // ------------------------------------------------------------
                ST_SCALE_W: begin
                    begin : b_scale
                        reg signed [104:0] full_p;
                        reg signed [104:0] rnd_p;
                        reg signed [60:0]  shift_p;
                        reg signed [24:0]  sat_result;

                        if (step < 6'd4) begin
                            full_p = u_re[step[1:0]] * $signed({1'b0, rs_Q40});
                        end else begin
                            full_p = u_im[step[1:0]] * $signed({1'b0, rs_Q40});
                        end

                        rnd_p = full_p + (105'sh1 <<< 43); // add 2^43
                        shift_p = rnd_p >>> 44;

                        // Saturate to 25-bit signed (Q5.20)
                        if (shift_p > 61'sh0000_0000_000F_FFFF) begin
                            sat_result = 25'sh0F_FFFF;
                            sat_w      <= 1'b1;
                        end else if (shift_p < -61'sh0000_0000_0010_0000) begin
                            sat_result = -25'sh10_0000;
                            sat_w      <= 1'b1;
                        end else begin
                            sat_result = shift_p[24:0];
                        end

                        w_wr_en   <= 1'b1;
                        w_wr_idx  <= step[2:0];
                        w_wr_data <= sat_result;

                        if (step == 6'd7) begin
                            state <= ST_DONE;
                        end else begin
                            step <= step + 6'd1;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Fallback: w = a / 4
                // ------------------------------------------------------------
                ST_FALLBACK_W: begin
                    w_wr_en  <= 1'b1;
                    w_wr_idx <= step[2:0];
                    if (step < 6'd4) begin
                        w_wr_data <= {{5{a_re_slice[15]}}, a_re_slice, 4'd0};
                    end else begin
                        w_wr_data <= {{5{a_im_slice[15]}}, a_im_slice, 4'd0};
                    end

                    if (step == 6'd7) begin
                        state <= ST_DONE;
                    end else begin
                        step <= step + 6'd1;
                    end
                end

                // ------------------------------------------------------------
                // Completion
                // ------------------------------------------------------------
                ST_DONE: begin
                    solve_done   <= 1'b1;
                    solve_cycles <= cycle_cnt;
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
