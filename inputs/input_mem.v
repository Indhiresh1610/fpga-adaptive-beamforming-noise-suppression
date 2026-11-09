// ============================================================================
// input_mem.v -- Single-file signal + steering-vector memory.
// Compatible with Verilog-2001.
//
// mem[0]       = steering vector a   (Q2.14) -> captured into a_vec after reset
// mem[1..N]    = snapshots x[0..N-1] (Q1.15) -> read with sample_idx = 0..N-1
// Word layout: {s3, s2, s1, s0}; sensor m = word[32*m +: 32];
// re = [15:0], im = [31:16] (signed).
// ============================================================================

`timescale 1ns/1ps

module input_mem #(
    parameter MEM_DEPTH    = 2048,
    parameter NUM_SAMPLES  = 1536,
    parameter MEM_FILE     = "x_mem.mem",
    parameter READ_LATENCY = 1
)(
    input  wire                               clk,
    input  wire                               rst,
    input  wire [clog2(MEM_DEPTH)-1:0]        sample_idx, // 0 .. NUM_SAMPLES-1
    output wire [127:0]                       x_snap,     // snapshot data
    output reg  [127:0]                       a_vec,      // steering vector
    output reg                                a_valid,    // asserted once a_vec is captured
    output wire                               err_cfg     // asserted if NUM_SAMPLES + 1 > MEM_DEPTH
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

    assign err_cfg = ((NUM_SAMPLES + 1) > MEM_DEPTH) ? 1'b1 : 1'b0;

    (* ram_style = "block" *) reg [127:0] mem [0:MEM_DEPTH-1];

    initial begin
        if (MEM_FILE != "") begin
            $readmemh(MEM_FILE, mem);
        end
    end

    // Address is 0 until a_valid is asserted, then sample_idx + 1
    wire [ADDR_W-1:0] addr;
    assign addr = (a_valid == 1'b1) ? (sample_idx + {{(ADDR_W-1){1'b0}}, 1'b1}) : {ADDR_W{1'b0}};

    reg [127:0] dout_stage1;
    reg [127:0] dout_stage2;

    always @(posedge clk) begin
        dout_stage1 <= mem[addr];
    end

    generate
        if (READ_LATENCY == 2) begin : gen_lat2
            always @(posedge clk) begin
                dout_stage2 <= dout_stage1;
            end
            assign x_snap = dout_stage2;
        end else begin : gen_lat1
            assign x_snap = dout_stage1;
        end
    endgenerate

    // Steering vector capture logic
    reg [2:0] init_cnt;

    always @(posedge clk) begin
        if (rst) begin
            a_valid  <= 1'b0;
            a_vec    <= 128'd0;
            init_cnt <= 3'd0;
        end else if (!a_valid) begin
            init_cnt <= init_cnt + 3'd1;
            if (READ_LATENCY == 2) begin
                if (init_cnt == 3'd2) begin
                    a_vec   <= dout_stage2;
                    a_valid <= 1'b1;
                end
            end else begin
                // READ_LATENCY == 1
                if (init_cnt == 3'd1) begin
                    a_vec   <= dout_stage1;
                    a_valid <= 1'b1;
                end
            end
        end
    end

endmodule
