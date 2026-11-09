// ============================================================================
// block_solver.v -- Causal MVDR Block Solver (resource-shared / microcoded)
// Compatible with Verilog-2001.
//
// Same algorithm, same fixed-point formats, same rounding points and the same
// interface as the previous block_solver.v. The weights it writes are
// bit-identical; only the hardware schedule changed:
//
//   * ONE pipelined 40x40 signed multiplier (mul_a/mul_b -> prod_m -> prod_p)
//     is shared by every product in the solver (covariance update, diagonal
//     loading, LDL^H, forward/diagonal/backward solve, a^H u, weight scaling).
//   * ONE 88-bit accumulator and ONE round/saturate unit (round_sat_acc).
//   * All solver variables live in a 64 x 40-bit register file (rf) with two
//     read ports and one write port, instead of ~100 individually wired regs.
//   * The LDL^H factorisation and the solves are a 143-entry micro-op ROM
//     (ucode) executed one op every 4 cycles - no pipeline hazards to manage.
//   * Requires ACC_W <= 39 (BLOCK_LEN <= 128), same as the S/R formats imply;
//     do not use a larger BLOCK_LEN without widening the multiplier operands.
//   * Normalisation uses one barrel shifter, 16 entries one per cycle.
//   * S is read directly from the sample engine (no S_buf copy).
//   * w = (u * rs + 2^43) >>> 44 is computed exactly with two 40x40 products
//     (rs split into a 39-bit low half and a 26-bit high half) instead of a
//     separate 40 x 66 multiplier. Proof of exactness:
//         L  = u*rs_lo + 2^43,   X = u*rs_hi + (L >>> 39)
//         (u*rs + 2^43) >>> 44  ==  X >>> 5
//
// Steps (unchanged):
//   1. R_new = S (block 0) or R + round(0.2*(S - R))
//   2. Trace, power-of-two normalisation to Q16.24
//   3. Diagonal loading: delta = 2.5e-4 * Tn
//   4. Complex Hermitian LDL^H: A = U^H D U, r_i = 1/d_i
//   5. Forward solve  U^H z = a
//   6. Diagonal solve v = z * r
//   7. Backward solve U u = v
//   8. s = Re(a^H u), rs = 2^64/s, w = u * rs (Q5.20)
//
// Register-file map (40-bit signed, Q16.24 unless noted):
//    0..3   A_diag[0..3]             4..9   A_off_re[0..5] (= T_re[0..2])
//   10..15  A_off_im[0..5] (= T_im[0..2])
//   16..19  a_re[0..3] (= z_re[0])   20..23 a_im[0..3] (= z_im[0])
//   24..27  r[0..3]                  28..30 T_re[3..5]   31..33 T_im[3..5]
//   34..39  U_re[0..5]               40..45 U_im[0..5]
//   46..48  z_re[1..3], later u_re[0..2]
//   49..51  z_im[1..3], later u_im[0..2]
//   52..55  v_re[0..3] (v_re[3] = u_re[3])
//   56..59  v_im[0..3] (v_im[3] = u_im[3])
//   60 ONE = 2^24      61 rs[38:0]      62 rs[64:39]
// Off-diagonal pair index: 0=(0,1) 1=(0,2) 2=(0,3) 3=(1,2) 4=(1,3) 5=(2,3)
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
    // NOTE: requires ACC_W <= 39 (BLOCK_LEN <= 128) so that S - R fits the
    // 40-bit multiplier operand. Default BLOCK_LEN = 76 gives ACC_W = 39.
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // Micro-op kinds and fixed register-file addresses
    // ------------------------------------------------------------------------
    localparam [2:0] K_MAC = 3'd0;   // acc = (init ? 0 : acc) +/- a*b ; if last: rf[dst] = round_sat(acc)
    localparam [2:0] K_PIV = 3'd1;   // last op of a pivot chain: clamp d, rf[dst] = 2^48 / d
    localparam [2:0] K_S   = 3'd2;   // last op of a^H u: check s, rs = 2^64 / s -> rf[61], rf[62]
    localparam [2:0] K_WLO = 3'd3;   // acc = (u*rs_lo + 2^43) >>> 39
    localparam [2:0] K_WHI = 3'd4;   // w[dst] = sat((acc + u*rs_hi) >>> 5)
    localparam [2:0] K_END = 3'd7;

    localparam [5:0] RF_ONE  = 6'd60;
    localparam [5:0] RF_RSLO = 6'd61;
    localparam [5:0] RF_RSHI = 6'd62;

    // ------------------------------------------------------------------------
    // Micro-op ROM: {kind[2:0], init, sub, last, ra[5:0], rb[5:0], dst[5:0]}
    // Generated schedule; every chain matches the original state-by-state
    // computation term for term (exact integer accumulation, one rounding).
    // ------------------------------------------------------------------------
    function [23:0] ucode;
        input [7:0] a;
        begin
            case (a)
            8'd0  : ucode = {K_PIV, 1'b1, 1'b0, 1'b1, 6'd0 , 6'd60, 6'd24};  // pivot d0, r0 = 1/d0
            8'd1  : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd4 , 6'd24, 6'd34};  // U01 re
            8'd2  : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd10, 6'd24, 6'd40};  // U01 im
            8'd3  : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd5 , 6'd24, 6'd35};  // U02 re
            8'd4  : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd11, 6'd24, 6'd41};  // U02 im
            8'd5  : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd6 , 6'd24, 6'd36};  // U03 re
            8'd6  : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd12, 6'd24, 6'd42};  // U03 im
            8'd7  : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd1 , 6'd60, 6'd0 };  // pivot d1, r1 = 1/d1
            8'd8  : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd4 , 6'd0 };
            8'd9  : ucode = {K_PIV, 1'b0, 1'b1, 1'b1, 6'd40, 6'd10, 6'd25};
            8'd10 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd7 , 6'd60, 6'd0 };  // T12 re
            8'd11 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd5 , 6'd0 };
            8'd12 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd40, 6'd11, 6'd28};
            8'd13 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd13, 6'd60, 6'd0 };  // T12 im
            8'd14 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd11, 6'd0 };
            8'd15 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd40, 6'd5 , 6'd31};
            8'd16 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd8 , 6'd60, 6'd0 };  // T13 re
            8'd17 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd6 , 6'd0 };
            8'd18 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd40, 6'd12, 6'd29};
            8'd19 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd14, 6'd60, 6'd0 };  // T13 im
            8'd20 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd12, 6'd0 };
            8'd21 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd40, 6'd6 , 6'd32};
            8'd22 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd28, 6'd25, 6'd37};  // U12 re
            8'd23 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd31, 6'd25, 6'd43};  // U12 im
            8'd24 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd29, 6'd25, 6'd38};  // U13 re
            8'd25 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd32, 6'd25, 6'd44};  // U13 im
            8'd26 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd2 , 6'd60, 6'd0 };  // pivot d2, r2 = 1/d2
            8'd27 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd5 , 6'd0 };
            8'd28 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd41, 6'd11, 6'd0 };
            8'd29 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd28, 6'd0 };
            8'd30 : ucode = {K_PIV, 1'b0, 1'b1, 1'b1, 6'd43, 6'd31, 6'd26};
            8'd31 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd9 , 6'd60, 6'd0 };  // T23 re
            8'd32 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd6 , 6'd0 };
            8'd33 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd41, 6'd12, 6'd0 };
            8'd34 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd29, 6'd0 };
            8'd35 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd43, 6'd32, 6'd30};
            8'd36 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd15, 6'd60, 6'd0 };  // T23 im
            8'd37 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd12, 6'd0 };
            8'd38 : ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd41, 6'd6 , 6'd0 };
            8'd39 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd32, 6'd0 };
            8'd40 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd43, 6'd29, 6'd33};
            8'd41 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd30, 6'd26, 6'd39};  // U23 re
            8'd42 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd33, 6'd26, 6'd45};  // U23 im
            8'd43 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd3 , 6'd60, 6'd0 };  // pivot d3, r3 = 1/d3
            8'd44 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd36, 6'd6 , 6'd0 };
            8'd45 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd42, 6'd12, 6'd0 };
            8'd46 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd38, 6'd29, 6'd0 };
            8'd47 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd44, 6'd32, 6'd0 };
            8'd48 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd39, 6'd30, 6'd0 };
            8'd49 : ucode = {K_PIV, 1'b0, 1'b1, 1'b1, 6'd45, 6'd33, 6'd27};
            8'd50 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd17, 6'd60, 6'd0 };  // z1 re
            8'd51 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd16, 6'd0 };
            8'd52 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd40, 6'd20, 6'd46};
            8'd53 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd21, 6'd60, 6'd0 };  // z1 im
            8'd54 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd20, 6'd0 };
            8'd55 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd40, 6'd16, 6'd49};
            8'd56 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd18, 6'd60, 6'd0 };  // z2 re
            8'd57 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd16, 6'd0 };
            8'd58 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd41, 6'd20, 6'd0 };
            8'd59 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd46, 6'd0 };
            8'd60 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd43, 6'd49, 6'd47};
            8'd61 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd22, 6'd60, 6'd0 };  // z2 im
            8'd62 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd20, 6'd0 };
            8'd63 : ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd41, 6'd16, 6'd0 };
            8'd64 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd49, 6'd0 };
            8'd65 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd43, 6'd46, 6'd50};
            8'd66 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd19, 6'd60, 6'd0 };  // z3 re
            8'd67 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd36, 6'd16, 6'd0 };
            8'd68 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd42, 6'd20, 6'd0 };
            8'd69 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd38, 6'd46, 6'd0 };
            8'd70 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd44, 6'd49, 6'd0 };
            8'd71 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd39, 6'd47, 6'd0 };
            8'd72 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd45, 6'd50, 6'd48};
            8'd73 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd23, 6'd60, 6'd0 };  // z3 im
            8'd74 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd36, 6'd20, 6'd0 };
            8'd75 : ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd42, 6'd16, 6'd0 };
            8'd76 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd38, 6'd49, 6'd0 };
            8'd77 : ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd44, 6'd46, 6'd0 };
            8'd78 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd39, 6'd50, 6'd0 };
            8'd79 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd45, 6'd47, 6'd51};
            8'd80 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd16, 6'd24, 6'd52};  // v0 re
            8'd81 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd20, 6'd24, 6'd56};  // v0 im
            8'd82 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd46, 6'd25, 6'd53};  // v1 re
            8'd83 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd49, 6'd25, 6'd57};  // v1 im
            8'd84 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd47, 6'd26, 6'd54};  // v2 re
            8'd85 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd50, 6'd26, 6'd58};  // v2 im
            8'd86 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd48, 6'd27, 6'd55};  // v3 re
            8'd87 : ucode = {K_MAC, 1'b1, 1'b0, 1'b1, 6'd51, 6'd27, 6'd59};  // v3 im
            8'd88 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd54, 6'd60, 6'd0 };  // u2 re
            8'd89 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd39, 6'd55, 6'd0 };
            8'd90 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd45, 6'd59, 6'd48};
            8'd91 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd58, 6'd60, 6'd0 };  // u2 im
            8'd92 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd39, 6'd59, 6'd0 };
            8'd93 : ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd45, 6'd55, 6'd51};
            8'd94 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd53, 6'd60, 6'd0 };  // u1 re
            8'd95 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd48, 6'd0 };
            8'd96 : ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd43, 6'd51, 6'd0 };
            8'd97 : ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd38, 6'd55, 6'd0 };
            8'd98 : ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd44, 6'd59, 6'd47};
            8'd99 : ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd57, 6'd60, 6'd0 };  // u1 im
            8'd100: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd37, 6'd51, 6'd0 };
            8'd101: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd43, 6'd48, 6'd0 };
            8'd102: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd38, 6'd59, 6'd0 };
            8'd103: ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd44, 6'd55, 6'd50};
            8'd104: ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd52, 6'd60, 6'd0 };  // u0 re
            8'd105: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd47, 6'd0 };
            8'd106: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd40, 6'd50, 6'd0 };
            8'd107: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd48, 6'd0 };
            8'd108: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd41, 6'd51, 6'd0 };
            8'd109: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd36, 6'd55, 6'd0 };
            8'd110: ucode = {K_MAC, 1'b0, 1'b0, 1'b1, 6'd42, 6'd59, 6'd46};
            8'd111: ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd56, 6'd60, 6'd0 };  // u0 im
            8'd112: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd34, 6'd50, 6'd0 };
            8'd113: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd40, 6'd47, 6'd0 };
            8'd114: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd35, 6'd51, 6'd0 };
            8'd115: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd41, 6'd48, 6'd0 };
            8'd116: ucode = {K_MAC, 1'b0, 1'b1, 1'b0, 6'd36, 6'd59, 6'd0 };
            8'd117: ucode = {K_MAC, 1'b0, 1'b1, 1'b1, 6'd42, 6'd55, 6'd49};
            8'd118: ucode = {K_MAC, 1'b1, 1'b0, 1'b0, 6'd16, 6'd46, 6'd0 };  // s = Re(a^H u), rs = 2^64/s
            8'd119: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd20, 6'd49, 6'd0 };
            8'd120: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd17, 6'd47, 6'd0 };
            8'd121: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd21, 6'd50, 6'd0 };
            8'd122: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd18, 6'd48, 6'd0 };
            8'd123: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd22, 6'd51, 6'd0 };
            8'd124: ucode = {K_MAC, 1'b0, 1'b0, 1'b0, 6'd19, 6'd55, 6'd0 };
            8'd125: ucode = {K_S, 1'b0, 1'b0, 1'b1, 6'd23, 6'd59, 6'd0 };
            8'd126: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd46, 6'd61, 6'd0 };  // w[0] low half
            8'd127: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd46, 6'd62, 6'd0 };  // w[0] high half -> w_wr_idx 0
            8'd128: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd47, 6'd61, 6'd0 };  // w[1] low half
            8'd129: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd47, 6'd62, 6'd1 };  // w[1] high half -> w_wr_idx 1
            8'd130: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd48, 6'd61, 6'd0 };  // w[2] low half
            8'd131: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd48, 6'd62, 6'd2 };  // w[2] high half -> w_wr_idx 2
            8'd132: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd55, 6'd61, 6'd0 };  // w[3] low half
            8'd133: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd55, 6'd62, 6'd3 };  // w[3] high half -> w_wr_idx 3
            8'd134: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd49, 6'd61, 6'd0 };  // w[4] low half
            8'd135: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd49, 6'd62, 6'd4 };  // w[4] high half -> w_wr_idx 4
            8'd136: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd50, 6'd61, 6'd0 };  // w[5] low half
            8'd137: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd50, 6'd62, 6'd5 };  // w[5] high half -> w_wr_idx 5
            8'd138: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd51, 6'd61, 6'd0 };  // w[6] low half
            8'd139: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd51, 6'd62, 6'd6 };  // w[6] high half -> w_wr_idx 6
            8'd140: ucode = {K_WLO, 1'b0, 1'b0, 1'b0, 6'd59, 6'd61, 6'd0 };  // w[7] low half
            8'd141: ucode = {K_WHI, 1'b0, 1'b0, 1'b1, 6'd59, 6'd62, 6'd7 };  // w[7] high half -> w_wr_idx 7
            8'd142: ucode = {K_END, 1'b0, 1'b0, 1'b0, 6'd0 , 6'd0 , 6'd0 };  // end
            default: ucode = {K_END, 1'b0, 1'b0, 1'b0, 6'd0, 6'd0, 6'd0};
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // Round-and-saturate (unchanged from the original; one instance now)
    // ------------------------------------------------------------------------
    function [40:0] round_sat_acc;   // returns {saturated, value[39:0]}
        input signed [87:0] acc_in;
        reg signed [87:0] acc_rnd;
        reg signed [63:0] acc_shift;
        begin
            acc_rnd = acc_in + 88'sh0000_0000_0000_0080_0000; // add 2^23
            acc_shift = acc_rnd >>> 24;
            if (acc_shift > 64'sh0000_007F_FFFF_FFFF) begin
                round_sat_acc = {1'b1, 40'sh7F_FFFF_FFFF};
            end else if (acc_shift < -64'sh0000_0080_0000_0000) begin
                round_sat_acc = {1'b1, -40'sh80_0000_0000};
            end else begin
                round_sat_acc = {1'b0, acc_shift[39:0]};
            end
        end
    endfunction

    // ------------------------------------------------------------------------
    // FSM states
    // ------------------------------------------------------------------------
    localparam [4:0] ST_IDLE       = 5'd0;
    localparam [4:0] ST_COV_RD     = 5'd1;
    localparam [4:0] ST_COV_W1     = 5'd2;
    localparam [4:0] ST_COV_W2     = 5'd3;
    localparam [4:0] ST_COV_UPD    = 5'd4;
    localparam [4:0] ST_TRACE      = 5'd5;
    localparam [4:0] ST_NORM       = 5'd6;
    localparam [4:0] ST_DL_MUL     = 5'd7;
    localparam [4:0] ST_DL_W1      = 5'd8;
    localparam [4:0] ST_DL_W2      = 5'd9;
    localparam [4:0] ST_DL_DELTA   = 5'd10;
    localparam [4:0] ST_DL_ADD     = 5'd11;
    localparam [4:0] ST_UC_ISSUE   = 5'd12;
    localparam [4:0] ST_UC_W1      = 5'd13;
    localparam [4:0] ST_UC_W2      = 5'd14;
    localparam [4:0] ST_UC_ACC     = 5'd15;
    localparam [4:0] ST_UC_DIV     = 5'd16;
    localparam [4:0] ST_UC_RSHI    = 5'd17;
    localparam [4:0] ST_FALLBACK_W = 5'd18;
    localparam [4:0] ST_DONE       = 5'd19;

    reg [4:0]  state;
    reg [7:0]  pc;          // micro-op program counter
    reg [4:0]  cnt;         // element counter for COV / NORM / DL_ADD / FALLBACK
    reg [31:0] cycle_cnt;

    // ------------------------------------------------------------------------
    // Persistent covariance state (16 words) -- names kept for the testbenches
    // ------------------------------------------------------------------------
    reg signed [ACC_W-1:0] R [0:15];
    reg                    have_cov;

    reg [5:0]  msb_p;
    reg [39:0] tn_sum;      // sum of normalised diagonal (40-bit wrap, as before)
    reg [39:0] delta;       // diagonal-loading value
    reg signed [87:0] acc;  // the single wide accumulator

    integer b, k;

    // ------------------------------------------------------------------------
    // Register file: 64 x 40, 2 async read ports, 1 sync write port
    // ------------------------------------------------------------------------
    (* ram_style = "distributed" *) reg signed [39:0] rf [0:63];

    wire [23:0] uc      = ucode(pc);
    wire [2:0]  uc_kind = uc[23:21];
    wire        uc_init = uc[20];
    wire        uc_sub  = uc[19];
    wire        uc_last = uc[18];
    wire [5:0]  uc_ra   = uc[17:12];
    wire [5:0]  uc_rb   = uc[11:6];
    wire [5:0]  uc_dst  = uc[5:0];

    wire [5:0]  rf_ra = (state == ST_DL_ADD) ? {4'd0, cnt[1:0]} : uc_ra;
    wire signed [39:0] rd_a = rf[rf_ra];
    wire signed [39:0] rd_b = rf[uc_rb];

    // ------------------------------------------------------------------------
    // The shared multiplier: 2 pipeline registers after the operand registers
    // (no reset, so Vivado can absorb them into the DSP48 cascade)
    // ------------------------------------------------------------------------
    reg signed [39:0] mul_a;
    reg signed [39:0] mul_b;
    reg signed [79:0] prod_m;
    reg signed [79:0] prod_p;

    always @(posedge clk) begin
        prod_m <= mul_a * mul_b;
        prod_p <= prod_m;
    end

    // ------------------------------------------------------------------------
    // Serial divider (unchanged module)
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
    // Shared combinational datapath
    // ------------------------------------------------------------------------
    // Accumulator: one adder, one round/saturate
    wire signed [87:0] p88     = prod_p;   // sign-extended product
    wire signed [87:0] acc_new = (uc_init ? 88'sd0 : acc) + (uc_sub ? -p88 : p88);
    wire [40:0]        acc_rs  = round_sat_acc(acc_new);
    wire signed [39:0] acc_rnd = acc_rs[39:0];
    wire               acc_sat = acc_rs[40];   // saturation of the rounded result

    // Weight scaling helpers (exact split of u * rs)
    wire signed [87:0] wlo_val = (p88 + (88'sd1 <<< 43)) >>> 39;
    wire signed [87:0] whi_sh  = acc_new >>> 5;    // WHI ops have init=0, sub=0

    // Covariance-update operands: S read straight from the sample engine
    wire signed [ACC_W-1:0] r_sel = R[cnt[3:0]];
    wire signed [ACC_W-1:0] s_rd  = S_rd_data;
    wire signed [39:0]      r_ext = r_sel;     // sign-extended to 40 bits
    wire signed [39:0]      s_ext = s_rd;

    // Normalisation: ONE barrel shifter (same expressions as before)
    reg signed [39:0] norm_val;
    always @(*) begin
        if (msb_p <= 6'd24) begin
            norm_val = r_ext <<< (6'd24 - msb_p);
        end else begin
            norm_val = (r_ext + (40'sh1 <<< (msb_p - 6'd25))) >>> (msb_p - 6'd24);
        end
    end

    // Steering-vector slice: sensor cnt[1:0], imag when cnt[2]=1
    // (used for a in Q16.24 during NORM and for the a/4 fallback)
    wire signed [15:0] a_slice = a_vec[{cnt[1:0], cnt[2], 4'b0000} +: 16];

    // ------------------------------------------------------------------------
    // Register-file write port (single)
    // ------------------------------------------------------------------------
    reg               rf_we;
    reg [5:0]         rf_wa;
    reg signed [39:0] rf_wd;

    always @(*) begin
        rf_we = 1'b0;
        rf_wa = 6'd0;
        rf_wd = 40'sd0;
        case (state)
            ST_NORM: begin
                rf_we = 1'b1;
                if (cnt < 5'd16) begin
                    rf_wa = {1'b0, cnt};
                    rf_wd = norm_val;                              // A in Q16.24
                end else if (cnt < 5'd24) begin
                    rf_wa = {1'b0, cnt};
                    rf_wd = {{14{a_slice[15]}}, a_slice, 10'd0};   // a: Q2.14 -> Q16.24
                end else begin
                    rf_wa = RF_ONE;
                    rf_wd = 40'sd16777216;                         // 1.0 in Q16.24
                end
            end
            ST_DL_ADD: begin
                rf_we = 1'b1;
                rf_wa = {4'd0, cnt[1:0]};
                rf_wd = rd_a + delta;
            end
            ST_UC_ACC: begin
                if (uc_kind == K_MAC && uc_last) begin
                    rf_we = 1'b1;
                    rf_wa = uc_dst;
                    rf_wd = acc_rnd;
                end
            end
            ST_UC_DIV: begin
                if (div_done) begin
                    rf_we = 1'b1;
                    if (uc_kind == K_PIV) begin
                        rf_wa = uc_dst;
                        rf_wd = div_q_rnd[39:0];                   // r_i = 1/d_i
                    end else begin
                        rf_wa = RF_RSLO;
                        rf_wd = {1'b0, div_q_rnd[38:0]};           // rs low 39 bits
                    end
                end
            end
            ST_UC_RSHI: begin
                rf_we = 1'b1;
                rf_wa = RF_RSHI;
                rf_wd = {14'd0, div_q_rnd[64:39]};                 // rs high 26 bits
            end
            default: ;
        endcase
    end

    always @(posedge clk) begin
        if (rf_we) begin
            rf[rf_wa] <= rf_wd;
        end
    end

    // ------------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state           <= ST_IDLE;
            pc              <= 8'd0;
            cnt             <= 5'd0;
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
            msb_p           <= 6'd0;
            tn_sum          <= 40'd0;
            delta           <= 40'd0;
            acc             <= 88'sd0;
            for (k = 0; k < 16; k = k + 1) begin
                R[k] <= {ACC_W{1'b0}};
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
                ST_IDLE: begin
                    if (solve_start) begin
                        cycle_cnt <= 32'd0;
                        S_rd_addr <= 4'd0;
                        cnt       <= 5'd0;
                        state     <= ST_COV_RD;
                    end
                end

                // ------------------------------------------------------------
                // Covariance update, one entry at a time, S read directly:
                //   block 0 : R = S
                //   later   : R = R + round((S - R) * C1 / 2^24)
                // ------------------------------------------------------------
                ST_COV_RD: begin
                    if (!have_cov) begin
                        R[cnt[3:0]] <= s_rd;
                        if (cnt == 5'd15) begin
                            have_cov <= 1'b1;
                            state    <= ST_TRACE;
                        end else begin
                            cnt       <= cnt + 5'd1;
                            S_rd_addr <= cnt[3:0] + 4'd1;
                        end
                    end else begin
                        mul_a <= s_ext - r_ext;            // exact in 40 bits
                        mul_b <= LAMBDA_ONE_MINUS_COEF;
                        state <= ST_COV_W1;
                    end
                end

                ST_COV_W1: state <= ST_COV_W2;
                ST_COV_W2: state <= ST_COV_UPD;

                ST_COV_UPD: begin
                    R[cnt[3:0]] <= r_sel + ((prod_p + 80'sd8388608) >>> 24);
                    if (cnt == 5'd15) begin
                        state <= ST_TRACE;
                    end else begin
                        cnt       <= cnt + 5'd1;
                        S_rd_addr <= cnt[3:0] + 4'd1;
                        state     <= ST_COV_RD;
                    end
                end

                // ------------------------------------------------------------
                // Trace and MSB position (same logic as before)
                // ------------------------------------------------------------
                ST_TRACE: begin
                    begin : b_trace
                        reg [ACC_W+1:0] trace_int;
                        reg [5:0]       msb_tmp;
                        trace_int = R[0] + R[1] + R[2] + R[3];
                        if (trace_int == 0) begin
                            info_zero_trace <= 1'b1;
                            cnt             <= 5'd0;
                            state           <= ST_FALLBACK_W;
                        end else begin
                            msb_tmp = 6'd0;
                            for (b = 0; b < ACC_W + 2; b = b + 1) begin
                                if (trace_int[b]) msb_tmp = b[5:0];
                            end
                            msb_p  <= msb_tmp;
                            tn_sum <= 40'd0;
                            cnt    <= 5'd0;
                            state  <= ST_NORM;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Serial normalisation into rf (writes via the write port):
                //   cnt 0..15  : A[cnt] = norm(R[cnt])
                //   cnt 16..23 : a_re / a_im in Q16.24
                //   cnt 24     : ONE
                // ------------------------------------------------------------
                ST_NORM: begin
                    if (cnt < 5'd4) begin
                        tn_sum <= tn_sum + norm_val;
                    end
                    if (cnt == 5'd24) begin
                        state <= ST_DL_MUL;
                    end else begin
                        cnt <= cnt + 5'd1;
                    end
                end

                // ------------------------------------------------------------
                // Diagonal loading: delta = (Tn[31:0] * LOAD_COEF + 2^31) >> 32
                // computed on the shared multiplier
                // ------------------------------------------------------------
                ST_DL_MUL: begin
                    mul_a <= {8'd0, tn_sum[31:0]};
                    mul_b <= LOAD_COEF;
                    state <= ST_DL_W1;
                end

                ST_DL_W1: state <= ST_DL_W2;
                ST_DL_W2: state <= ST_DL_DELTA;

                ST_DL_DELTA: begin
                    begin : b_delta
                        reg [63:0] tn_prod;
                        tn_prod = prod_p[63:0] + 64'h0000_0000_8000_0000;
                        delta <= {8'd0, tn_prod[63:32]};
                    end
                    cnt   <= 5'd0;
                    state <= ST_DL_ADD;
                end

                ST_DL_ADD: begin   // A_diag[cnt] += delta (rf write port)
                    if (cnt == 5'd3) begin
                        pc    <= 8'd0;
                        state <= ST_UC_ISSUE;
                    end else begin
                        cnt <= cnt + 5'd1;
                    end
                end

                // ------------------------------------------------------------
                // Micro-op execution: ISSUE -> W1 -> W2 -> ACC (4 cycles/op)
                // ------------------------------------------------------------
                ST_UC_ISSUE: begin
                    if (uc_kind == K_END) begin
                        state <= ST_DONE;
                    end else begin
                        mul_a <= rd_a;
                        mul_b <= rd_b;
                        state <= ST_UC_W1;
                    end
                end

                ST_UC_W1: state <= ST_UC_W2;
                ST_UC_W2: state <= ST_UC_ACC;

                ST_UC_ACC: begin
                    case (uc_kind)
                        K_MAC: begin
                            if (uc_last && acc_sat) sat_solver <= 1'b1;
                            acc   <= acc_new;
                            pc    <= pc + 8'd1;
                            state <= ST_UC_ISSUE;
                        end

                        K_PIV: begin
                            if (acc_sat) sat_solver <= 1'b1;
                            acc <= acc_new;
                            if (acc_rnd < D_MIN) begin
                                err_pivot <= 1'b1;
                                div_d     <= D_MIN;
                            end else begin
                                div_d     <= acc_rnd;
                            end
                            div_n     <= 65'h0001_0000_0000_0000; // 2^48
                            div_start <= 1'b1;
                            state     <= ST_UC_DIV;
                        end

                        K_S: begin
                            if (acc_sat) sat_solver <= 1'b1;
                            acc <= acc_new;
                            if (acc_rnd <= 40'sd1024) begin
                                err_solver <= 1'b1;
                                cnt        <= 5'd0;
                                state      <= ST_FALLBACK_W;
                            end else begin
                                div_n     <= {1'b1, 64'd0}; // 2^64
                                div_d     <= acc_rnd;
                                div_start <= 1'b1;
                                state     <= ST_UC_DIV;
                            end
                        end

                        K_WLO: begin
                            acc   <= wlo_val;
                            pc    <= pc + 8'd1;
                            state <= ST_UC_ISSUE;
                        end

                        K_WHI: begin
                            // Saturate to 25-bit signed Q5.20 (same limits as before)
                            if (whi_sh > 88'sd16777215) begin
                                w_wr_data <= 25'sh0FF_FFFF;
                                sat_w     <= 1'b1;
                            end else if (whi_sh < -88'sd16777216) begin
                                w_wr_data <= -25'sh100_0000;
                                sat_w     <= 1'b1;
                            end else begin
                                w_wr_data <= whi_sh[24:0];
                            end
                            w_wr_en  <= 1'b1;
                            w_wr_idx <= uc_dst[2:0];
                            pc       <= pc + 8'd1;
                            state    <= ST_UC_ISSUE;
                        end

                        default: state <= ST_DONE;
                    endcase
                end

                // Wait for r_i = 2^48/d_i (K_PIV) or rs = 2^64/s (K_S)
                ST_UC_DIV: begin
                    if (div_done) begin
                        if (uc_kind == K_PIV) begin
                            pc    <= pc + 8'd1;
                            state <= ST_UC_ISSUE;
                        end else begin
                            state <= ST_UC_RSHI;
                        end
                    end
                end

                ST_UC_RSHI: begin   // rf[62] <= rs[64:39] (write port)
                    pc    <= pc + 8'd1;
                    state <= ST_UC_ISSUE;
                end

                // ------------------------------------------------------------
                // Fallback: w = a / 4
                // ------------------------------------------------------------
                ST_FALLBACK_W: begin
                    w_wr_en   <= 1'b1;
                    w_wr_idx  <= cnt[2:0];
                    w_wr_data <= {{5{a_slice[15]}}, a_slice, 4'd0};
                    if (cnt == 5'd7) begin
                        state <= ST_DONE;
                    end else begin
                        cnt <= cnt + 5'd1;
                    end
                end

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