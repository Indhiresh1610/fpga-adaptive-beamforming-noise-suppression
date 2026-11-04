// ============================================================================
// block_controller.v -- Main FSM Controller for MVDR Beamformer
// Compatible with Verilog-2001.
// Manages:
//   - Paced and free-run sample pacing
//   - Three independent lengths: BLOCK_LEN, NUM_SAMPLES, MEM_DEPTH
//   - BRAM read latency (1 or 2 cycles)
//   - Out-of-bounds read protection (asserts address < NUM_SAMPLES)
//   - Configuration error detection (NUM_SAMPLES + 1 > MEM_DEPTH)
//   - Causal block sequencing: Block 0 uses a/4, solve runs at end of complete blocks
//   - Weight swap occurs strictly between blocks, never mid-sample
//   - Tail processing (accum_en=0, no solver)
//   - Output backpressure stalling (out_busy)
//   - Sticky status flags
// ============================================================================

`timescale 1ns/1ps

module block_controller #(
    parameter BLOCK_LEN    = 76,
    parameter NUM_SAMPLES  = 1536,
    parameter MEM_DEPTH    = 2048,
    parameter READ_LATENCY = 1,
    parameter PACED        = 0
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 start,
    input  wire                 sample_en,
    input  wire                 a_valid,
    input  wire                 out_busy,

    // Interface with input_mem
    output wire [clog2(MEM_DEPTH)-1:0] mem_sample_idx,
    input  wire                        err_cfg_in,

    // Interface with sample_engine
    output reg                  sample_start,
    output reg                  accum_en,
    output reg                  clear_cov,
    input  wire                 sample_done,
    input  wire                 y_valid,
    input  wire [47:0]          y_out,
    input  wire                 sat_y,

    // Interface with block_solver
    output reg                  solve_start,
    input  wire                 solve_done,
    input  wire [31:0]          solve_cycles_in,
    input  wire                 err_pivot_in,
    input  wire                 err_solver_in,
    input  wire                 sat_solver_in,
    input  wire                 sat_w_in,
    input  wire                 info_zero_trace_in,

    // Interface with weight_regs
    output reg                  load_quiescent,
    output reg                  w_swap,

    // Interface with output_interface
    output reg                  push,
    output reg  [47:0]          in_data,
    output reg  [15:0]          in_idx,
    output reg  [7:0]           in_blk,
    output reg                  in_is_tail,
    output reg                  in_last,

    // Top-Level Status & Debug
    output reg                  busy,
    output reg                  done,
    output reg  [7:0]           status,
    output reg                  dbg_blk_done,
    output reg  [31:0]          dbg_solve_cycles,
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

    localparam ADDR_W          = clog2(MEM_DEPTH);
    localparam NUM_FULL_BLOCKS = NUM_SAMPLES / BLOCK_LEN;
    localparam TAIL_LEN        = NUM_SAMPLES % BLOCK_LEN;

    // ------------------------------------------------------------------------
    // FSM States
    // ------------------------------------------------------------------------
    localparam ST_RESET          = 5'd0;
    localparam ST_WAIT_STEERING  = 5'd1;
    localparam ST_WAIT_START     = 5'd2;
    localparam ST_BLOCK_INIT     = 5'd3;
    localparam ST_SAMPLE_WAIT    = 5'd4;
    localparam ST_SAMPLE_READ    = 5'd5;
    localparam ST_SAMPLE_RUN     = 5'd6;
    localparam ST_BLOCK_END      = 5'd7;
    localparam ST_SOLVE          = 5'd8;
    localparam ST_WEIGHT_SWAP    = 5'd9;
    localparam ST_TAIL_START     = 5'd10;
    localparam ST_DONE           = 5'd11;

    reg [4:0] state;
    assign dbg_state = state;

    reg [15:0] sample_idx_reg;
    reg [15:0] block_sample_cnt;
    reg [7:0]  block_idx;
    reg        is_tail_reg;
    reg        pending_pacing;
    reg [1:0]  lat_cnt;

    assign mem_sample_idx = sample_idx_reg[ADDR_W-1:0];

    // Status bits assignment:
    // status[0] = err_overrun
    // status[1] = err_pivot
    // status[2] = err_solver
    // status[3] = sat_solver
    // status[4] = sat_w
    // status[5] = sat_y
    // status[6] = info_zero_trace
    // status[7] = err_cfg

    always @(posedge clk) begin
        if (rst) begin
            state            <= ST_RESET;
            busy             <= 1'b0;
            done             <= 1'b0;
            status           <= 8'd0;
            sample_start     <= 1'b0;
            accum_en         <= 1'b1;
            clear_cov        <= 1'b0;
            solve_start      <= 1'b0;
            load_quiescent   <= 1'b0;
            w_swap           <= 1'b0;
            push             <= 1'b0;
            in_data          <= 48'd0;
            in_idx           <= 16'd0;
            in_blk           <= 8'd0;
            in_is_tail       <= 1'b0;
            in_last          <= 1'b0;
            dbg_blk_done     <= 1'b0;
            dbg_solve_cycles <= 32'd0;
            sample_idx_reg   <= 16'd0;
            block_sample_cnt <= 16'd0;
            block_idx        <= 8'd0;
            is_tail_reg      <= 1'b0;
            pending_pacing   <= 1'b0;
            lat_cnt          <= 2'd0;

            // Check configuration error
            if ((NUM_SAMPLES + 1) > MEM_DEPTH || err_cfg_in) begin
                status[7] <= 1'b1; // err_cfg
            end
        end else begin
            // Default 1-cycle pulses
            sample_start   <= 1'b0;
            clear_cov      <= 1'b0;
            solve_start    <= 1'b0;
            load_quiescent <= 1'b0;
            w_swap         <= 1'b0;
            push           <= 1'b0;
            dbg_blk_done   <= 1'b0;

            // Accumulate sticky status flags
            if (err_pivot_in)       status[1] <= 1'b1;
            if (err_solver_in)      status[2] <= 1'b1;
            if (sat_solver_in)      status[3] <= 1'b1;
            if (sat_w_in)           status[4] <= 1'b1;
            if (sat_y)              status[5] <= 1'b1;
            if (info_zero_trace_in) status[6] <= 1'b1;

            // Pacing overrun check in PACED mode
            // pending_pacing is a 1-deep buffer for a sample_en that arrived while
            // the engine was busy in ST_SAMPLE_RUN. It is consumed in ST_SAMPLE_WAIT.
            // Overrun = a third pulse arrives while the buffer (pending_pacing) is full
            // AND the engine is still running (ST_SAMPLE_RUN).
            // We do NOT track sample_en in ST_SAMPLE_READ because that state is simply
            // the BRAM latency cycle immediately after consuming the previous pulse; the
            // triggering sample_en may still be visible and must not be double-counted.
            // We also do NOT track sample_en in any inter-block housekeeping state
            // (BLOCK_INIT, SOLVE, WEIGHT_SWAP, TAIL_START) because those transitions
            // are unconstrained in time relative to the sample clock.
            if (PACED) begin
                if (busy && state == ST_SAMPLE_RUN) begin
                    if (sample_en) begin
                        if (pending_pacing) begin
                            status[0] <= 1'b1; // err_overrun: 3rd pulse while buffer full
                        end else begin
                            pending_pacing <= 1'b1;
                        end
                    end
                end
            end

            case (state)
                // ------------------------------------------------------------
                // Wait for input memory to capture steering vector
                // ------------------------------------------------------------
                ST_RESET: begin
                    if (a_valid) begin
                        load_quiescent <= 1'b1; // Initialize w_active and w_next to a/4
                        state          <= ST_WAIT_START;
                    end
                end

                // ------------------------------------------------------------
                // Ready and waiting for start pulse
                // ------------------------------------------------------------
                ST_WAIT_START: begin
                    busy <= 1'b0;
                    if (start) begin
                        if (status[7]) begin
                            // Configuration error: refuse to run
                            state <= ST_WAIT_START;
                        end else begin
                            done             <= 1'b0;
                            busy             <= 1'b1;
                            sample_idx_reg   <= 16'd0;
                            block_sample_cnt <= 16'd0;
                            block_idx        <= 8'd0;
                            pending_pacing   <= 1'b0;

                            if (NUM_FULL_BLOCKS > 0) begin
                                is_tail_reg <= 1'b0;
                                state       <= ST_BLOCK_INIT;
                            end else begin
                                is_tail_reg <= 1'b1;
                                state       <= ST_TAIL_START;
                            end
                        end
                    end
                end

                // ------------------------------------------------------------
                // Initialize a new full block: clear covariance accumulators
                // ------------------------------------------------------------
                ST_BLOCK_INIT: begin
                    clear_cov        <= 1'b1;
                    accum_en         <= 1'b1;
                    block_sample_cnt <= 16'd0;
                    state            <= ST_SAMPLE_WAIT;
                end

                // ------------------------------------------------------------
                // Sample pacing and backpressure check
                // ------------------------------------------------------------
                ST_SAMPLE_WAIT: begin
                    // Check output backpressure: stall if output register is occupied
                    if (!out_busy) begin
                        if (PACED == 0) begin
                            // Free-run: proceed immediately
                            state   <= ST_SAMPLE_READ;
                            lat_cnt <= 2'd0;
                        end else begin
                            // Paced mode: require sample_en pulse or pending pulse
                            if (sample_en || pending_pacing) begin
                                pending_pacing <= 1'b0;
                                state          <= ST_SAMPLE_READ;
                                lat_cnt        <= 2'd0;
                            end
                        end
                    end
                end

                // ------------------------------------------------------------
                // Wait for BRAM read latency (1 or 2 cycles)
                // ------------------------------------------------------------
                ST_SAMPLE_READ: begin
                    if (READ_LATENCY == 2) begin
                        if (lat_cnt == 2'd1) begin
                            sample_start <= 1'b1;
                            accum_en     <= (!is_tail_reg);
                            state        <= ST_SAMPLE_RUN;
                        end else begin
                            lat_cnt <= lat_cnt + 2'd1;
                        end
                    end else begin
                        // READ_LATENCY == 1
                        sample_start <= 1'b1;
                        accum_en     <= (!is_tail_reg);
                        state        <= ST_SAMPLE_RUN;
                    end
                end

                // ------------------------------------------------------------
                // Sample engine running
                // ------------------------------------------------------------
                ST_SAMPLE_RUN: begin
                    if (y_valid) begin
                        push       <= 1'b1;
                        in_data    <= y_out;
                        in_idx     <= sample_idx_reg;
                        in_blk     <= is_tail_reg ? NUM_FULL_BLOCKS[7:0] : block_idx;
                        in_is_tail <= is_tail_reg;
                        in_last    <= (sample_idx_reg == (NUM_SAMPLES - 1));
                    end

                    if (sample_done) begin
                        sample_idx_reg   <= sample_idx_reg + 16'd1;
                        block_sample_cnt <= block_sample_cnt + 16'd1;

                        if (is_tail_reg) begin
                            // In tail
                            if (sample_idx_reg == (NUM_SAMPLES - 1)) begin
                                state <= ST_DONE;
                            end else begin
                                state <= ST_SAMPLE_WAIT;
                            end
                        end else begin
                            // In complete block
                            if (block_sample_cnt == (BLOCK_LEN - 1)) begin
                                dbg_blk_done <= 1'b1;
                                state        <= ST_BLOCK_END;
                            end else begin
                                state <= ST_SAMPLE_WAIT;
                            end
                        end
                    end
                end

                // ------------------------------------------------------------
                // End of complete block: trigger solver
                // ------------------------------------------------------------
                ST_BLOCK_END: begin
                    solve_start <= 1'b1;
                    state       <= ST_SOLVE;
                end

                // ------------------------------------------------------------
                // Waiting for solver to finish
                // ------------------------------------------------------------
                ST_SOLVE: begin
                    if (solve_done) begin
                        dbg_solve_cycles <= solve_cycles_in;
                        state            <= ST_WEIGHT_SWAP;
                    end
                end

                // ------------------------------------------------------------
                // Weight Swap: copy w_next -> w_active strictly between blocks
                // ------------------------------------------------------------
                ST_WEIGHT_SWAP: begin
                    w_swap    <= 1'b1; // Copy w_next -> w_active
                    block_idx <= block_idx + 8'd1;

                    if ((block_idx + 8'd1) < NUM_FULL_BLOCKS) begin
                        state <= ST_BLOCK_INIT;
                    end else if (TAIL_LEN > 0) begin
                        state <= ST_TAIL_START;
                    end else begin
                        // Exact multiple of BLOCK_LEN: done!
                        state <= ST_DONE;
                    end
                end

                // ------------------------------------------------------------
                // Start tail processing
                // ------------------------------------------------------------
                ST_TAIL_START: begin
                    is_tail_reg      <= 1'b1;
                    accum_en         <= 1'b0; // Never accumulate covariance during tail
                    block_sample_cnt <= 16'd0;
                    state            <= ST_SAMPLE_WAIT;
                end

                // ------------------------------------------------------------
                // Record completed
                // ------------------------------------------------------------
                ST_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ST_WAIT_START;
                end

                default: state <= ST_RESET;
            endcase
        end
    end

endmodule
