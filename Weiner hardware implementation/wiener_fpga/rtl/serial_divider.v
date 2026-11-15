// ============================================================================
// serial_divider.v -- Sequential (restoring) unsigned divider.
// Compatible with Verilog-2001.
// Computes quotient = floor(dividend / divisor), remainder = dividend % divisor.
// Also provides quotient_rnd = round_half_up(dividend / divisor).
//
// Latency: exactly WIDTH_N clock cycles from start.
// ============================================================================

`timescale 1ns/1ps

module serial_divider #(
    parameter WIDTH_N = 65,
    parameter WIDTH_D = 40
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 start,
    input  wire [WIDTH_N-1:0]   dividend,
    input  wire [WIDTH_D-1:0]   divisor,
    output reg  [WIDTH_N-1:0]   quotient,
    output reg  [WIDTH_N-1:0]   quotient_rnd,
    output reg  [WIDTH_D-1:0]   remainder,
    output reg                  busy,
    output reg                  done,
    output reg                  div_by_zero
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

    localparam CNT_W = clog2(WIDTH_N + 1);

    localparam S_IDLE = 2'd0;
    localparam S_DIV  = 2'd1;
    localparam S_DONE = 2'd2;

    reg [1:0] state;
    reg [CNT_W-1:0] count;
    reg [WIDTH_N-1:0] rem_d;     // shift register for dividend
    reg [WIDTH_D:0]   acc;       // accumulator holding current remainder (1 extra bit)
    reg [WIDTH_D-1:0] div_reg;   // registered divisor
    reg [WIDTH_N-1:0] q_reg;     // quotient shift register

    wire [WIDTH_D:0] acc_shifted;
    wire [WIDTH_D:0] sub_res;
    wire bit_in;

    assign bit_in = rem_d[WIDTH_N-1];
    assign acc_shifted = {acc[WIDTH_D-1:0], bit_in};
    assign sub_res = acc_shifted - {1'b0, div_reg};

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            div_by_zero  <= 1'b0;
            quotient     <= {WIDTH_N{1'b0}};
            quotient_rnd <= {WIDTH_N{1'b0}};
            remainder    <= {WIDTH_D{1'b0}};
            count        <= {CNT_W{1'b0}};
            rem_d        <= {WIDTH_N{1'b0}};
            acc          <= {(WIDTH_D+1){1'b0}};
            div_reg      <= {WIDTH_D{1'b0}};
            q_reg        <= {WIDTH_N{1'b0}};
        end else begin
            done <= 1'b0; // default 1-cycle pulse
            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        if (divisor == {WIDTH_D{1'b0}}) begin
                            div_by_zero  <= 1'b1;
                            quotient     <= {WIDTH_N{1'b0}};
                            quotient_rnd <= {WIDTH_N{1'b0}};
                            remainder    <= {WIDTH_D{1'b0}};
                            busy         <= 1'b1;
                            state        <= S_DONE;
                        end else begin
                            busy        <= 1'b1;
                            div_by_zero <= 1'b0;
                            div_reg     <= divisor;
                            rem_d       <= dividend;
                            acc         <= {(WIDTH_D+1){1'b0}};
                            q_reg       <= {WIDTH_N{1'b0}};
                            count       <= WIDTH_N[CNT_W-1:0];
                            state       <= S_DIV;
                        end
                    end
                end

                S_DIV: begin
                    // Restoring division step
                    if (sub_res[WIDTH_D] == 1'b0) begin
                        // acc_shifted >= div_reg
                        acc   <= sub_res;
                        q_reg <= {q_reg[WIDTH_N-2:0], 1'b1};
                    end else begin
                        // acc_shifted < div_reg
                        acc   <= acc_shifted;
                        q_reg <= {q_reg[WIDTH_N-2:0], 1'b0};
                    end
                    rem_d <= {rem_d[WIDTH_N-2:0], 1'b0};

                    if (count == {{(CNT_W-1){1'b0}}, 1'b1}) begin
                        state <= S_DONE;
                    end else begin
                        count <= count - {{(CNT_W-1){1'b0}}, 1'b1};
                    end
                end

                S_DONE: begin
                    if (!div_by_zero) begin
                        quotient  <= q_reg;
                        remainder <= acc[WIDTH_D-1:0];
                        if ({acc[WIDTH_D-1:0], 1'b0} >= div_reg) begin
                            quotient_rnd <= q_reg + {{(WIDTH_N-1){1'b0}}, 1'b1};
                        end else begin
                            quotient_rnd <= q_reg;
                        end
                    end
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
