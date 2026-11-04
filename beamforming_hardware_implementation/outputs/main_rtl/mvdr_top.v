// ============================================================================
// mvdr_top.v -- Top-Level Causal MVDR Beamformer
// Strict Verilog-2001. Synthesizable for Spartan-7 XC7S50.
// Instantiates:
//   - u_mem  : input_mem
//   - u_ctrl : block_controller
//   - u_eng  : sample_engine
//   - u_sol  : block_solver
//   - u_wr   : weight_regs
//   - u_out  : output_interface
// ============================================================================

`timescale 1ns/1ps

module mvdr_top #(
    parameter BLOCK_LEN             = 76,
    parameter NUM_SAMPLES           = 1536,
    parameter MEM_DEPTH             = 2048,
    parameter MEM_FILE              = "x_mem.mem",
    parameter READ_LATENCY          = 1,
    parameter PACED                 = 0,
    parameter LAMBDA_ONE_MINUS_COEF = 3355443, // round(0.2 * 2^24)
    parameter LOAD_COEF             = 1073742, // round(2.5e-4 * 2^32)
    parameter D_MIN                 = 1024     // 2^-14 in Q16.24
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 start,
    input  wire                 sample_en,
    input  wire                 m_ready,

    output wire                 m_valid,
    output wire [47:0]          m_data,      // {im[23:0], re[23:0]} Q2.22
    output wire                 m_last,
    output wire [15:0]          m_idx,
    output wire [7:0]           m_blk,
    output wire                 m_is_tail,
    output wire                 busy,
    output wire                 done,
    output wire [7:0]           status,

    // Debug ports
    output wire [199:0]         dbg_w_active,
    output wire                 dbg_w_swap,
    output wire                 dbg_blk_done,
    output wire [31:0]          dbg_solve_cycles,
    output wire [4:0]           dbg_state
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

    localparam ADDR_W = clog2(MEM_DEPTH);
    localparam ACC_W  = 32 + clog2(BLOCK_LEN);

    // ------------------------------------------------------------------------
    // Internal Wires and Interconnect
    // ------------------------------------------------------------------------
    wire [ADDR_W-1:0]  mem_sample_idx;
    wire [127:0]       x_snap;
    wire [127:0]       a_vec;
    wire               a_valid;
    wire               err_cfg_mem;

    wire               sample_start;
    wire               accum_en;
    wire               clear_cov;
    wire               sample_done;
    wire               y_valid;
    wire [47:0]        y_out;
    wire               sat_y;

    wire [3:0]         S_rd_addr;
    wire [ACC_W-1:0]   S_rd_data;

    wire               solve_start;
    wire               solve_done;
    wire [31:0]        solve_cycles_int;
    wire               err_pivot_int;
    wire               err_solver_int;
    wire               sat_solver_int;
    wire               sat_w_int;
    wire               info_zero_trace_int;

    wire               load_quiescent;
    wire               w_swap;
    wire               w_wr_en;
    wire [2:0]         w_wr_idx;
    wire signed [24:0] w_wr_data;
    wire [199:0]       w_active;
    wire [199:0]       w_next;

    wire               push;
    wire [47:0]        in_data;
    wire [15:0]        in_idx;
    wire [7:0]         in_blk;
    wire               in_is_tail;
    wire               in_last;
    wire               out_busy;

    // ------------------------------------------------------------------------
    // Instance u_mem: Input Memory
    // ------------------------------------------------------------------------
    input_mem #(
        .MEM_DEPTH(MEM_DEPTH),
        .NUM_SAMPLES(NUM_SAMPLES),
        .MEM_FILE(MEM_FILE),
        .READ_LATENCY(READ_LATENCY)
    ) u_mem (
        .clk(clk),
        .rst(rst),
        .sample_idx(mem_sample_idx),
        .x_snap(x_snap),
        .a_vec(a_vec),
        .a_valid(a_valid),
        .err_cfg(err_cfg_mem)
    );

    // ------------------------------------------------------------------------
    // Instance u_ctrl: Main Block Controller
    // ------------------------------------------------------------------------
    block_controller #(
        .BLOCK_LEN(BLOCK_LEN),
        .NUM_SAMPLES(NUM_SAMPLES),
        .MEM_DEPTH(MEM_DEPTH),
        .READ_LATENCY(READ_LATENCY),
        .PACED(PACED)
    ) u_ctrl (
        .clk(clk),
        .rst(rst),
        .start(start),
        .sample_en(sample_en),
        .a_valid(a_valid),
        .out_busy(out_busy),
        .mem_sample_idx(mem_sample_idx),
        .err_cfg_in(err_cfg_mem),
        .sample_start(sample_start),
        .accum_en(accum_en),
        .clear_cov(clear_cov),
        .sample_done(sample_done),
        .y_valid(y_valid),
        .y_out(y_out),
        .sat_y(sat_y),
        .solve_start(solve_start),
        .solve_done(solve_done),
        .solve_cycles_in(solve_cycles_int),
        .err_pivot_in(err_pivot_int),
        .err_solver_in(err_solver_int),
        .sat_solver_in(sat_solver_int),
        .sat_w_in(sat_w_int),
        .info_zero_trace_in(info_zero_trace_int),
        .load_quiescent(load_quiescent),
        .w_swap(w_swap),
        .push(push),
        .in_data(in_data),
        .in_idx(in_idx),
        .in_blk(in_blk),
        .in_is_tail(in_is_tail),
        .in_last(in_last),
        .busy(busy),
        .done(done),
        .status(status),
        .dbg_blk_done(dbg_blk_done),
        .dbg_solve_cycles(dbg_solve_cycles),
        .dbg_state(dbg_state)
    );

    // ------------------------------------------------------------------------
    // Instance u_eng: Sample Engine (Beamformer + Covariance Accumulator)
    // ------------------------------------------------------------------------
    sample_engine #(
        .BLOCK_LEN(BLOCK_LEN),
        .ACC_W(ACC_W)
    ) u_eng (
        .clk(clk),
        .rst(rst),
        .sample_start(sample_start),
        .accum_en(accum_en),
        .clear_cov(clear_cov),
        .x_snap(x_snap),
        .w_active(w_active),
        .y_out(y_out),
        .y_valid(y_valid),
        .sample_done(sample_done),
        .sat_y(sat_y),
        .S_rd_addr(S_rd_addr),
        .S_rd_data(S_rd_data)
    );

    // ------------------------------------------------------------------------
    // Instance u_sol: Block Solver
    // ------------------------------------------------------------------------
    block_solver #(
        .BLOCK_LEN(BLOCK_LEN),
        .ACC_W(ACC_W),
        .LAMBDA_ONE_MINUS_COEF(LAMBDA_ONE_MINUS_COEF),
        .LOAD_COEF(LOAD_COEF),
        .D_MIN(D_MIN)
    ) u_sol (
        .clk(clk),
        .rst(rst),
        .solve_start(solve_start),
        .a_vec(a_vec),
        .S_rd_addr(S_rd_addr),
        .S_rd_data(S_rd_data),
        .w_wr_en(w_wr_en),
        .w_wr_idx(w_wr_idx),
        .w_wr_data(w_wr_data),
        .solve_done(solve_done),
        .solve_cycles(solve_cycles_int),
        .err_pivot(err_pivot_int),
        .err_solver(err_solver_int),
        .sat_solver(sat_solver_int),
        .sat_w(sat_w_int),
        .info_zero_trace(info_zero_trace_int)
    );

    // ------------------------------------------------------------------------
    // Instance u_wr: Weight Registers
    // ------------------------------------------------------------------------
    weight_regs u_wr (
        .clk(clk),
        .rst(rst),
        .load_quiescent(load_quiescent),
        .w_swap(w_swap),
        .a_vec(a_vec),
        .w_wr_en(w_wr_en),
        .w_wr_idx(w_wr_idx),
        .w_wr_data(w_wr_data),
        .w_active(w_active),
        .w_next(w_next),
        .dbg_w_active(dbg_w_active),
        .dbg_w_swap(dbg_w_swap)
    );

    // ------------------------------------------------------------------------
    // Instance u_out: Output Interface
    // ------------------------------------------------------------------------
    output_interface u_out (
        .clk(clk),
        .rst(rst),
        .push(push),
        .in_data(in_data),
        .in_idx(in_idx),
        .in_blk(in_blk),
        .in_is_tail(in_is_tail),
        .in_last(in_last),
        .out_busy(out_busy),
        .m_valid(m_valid),
        .m_data(m_data),
        .m_idx(m_idx),
        .m_blk(m_blk),
        .m_is_tail(m_is_tail),
        .m_last(m_last),
        .m_ready(m_ready)
    );

endmodule
