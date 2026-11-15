// ============================================================================
// tb_wiener_only.v -- Wiener filter alone: reads wf_in.mem (beamformer output made by wiener_check.py gen),
// writes wf_out.csv (header idx,re_raw,im_raw ; Q2.22 ints), to be compared with wf_expected.csv.
//   +NIN=<n>   number of input samples in wf_in.mem (default 1971 = case_2 without block 0)
//   +GAPS      random idle cycles on the input and random backpressure on the output; the result must NOT change
// Needs: wiener_cplx_top.v wiener_cplx_core.v serial_divider.v (team's) and xfft_0_behav.v (or the real xfft_0 IP).
//   iverilog -g2005 -o tb_wo tb_wiener_only.v wiener_cplx_top.v wiener_cplx_core.v xfft_0_behav.v serial_divider.v
//   vvp tb_wo +NIN=1971 [+GAPS]  ;  python3 wiener_check.py compare wf_expected.csv wf_out.csv
// ============================================================================
`timescale 1ns/1ps
module tb_wiener_only;
    localparam LAT = 48, NFLUSH = 80;
    reg clk = 1'b0, rst = 1'b1;
    always #10 clk = ~clk;

    reg                s_valid = 1'b0;
    reg  signed [23:0] s_re = 24'sd0, s_im = 24'sd0;
    wire               s_ready, m_valid;
    reg                m_ready = 1'b1;
    wire signed [23:0] m_re, m_im;
    wire [7:0]         status;

    wiener_cplx_top dut (.clk(clk), .rst(rst), .restart(1'b0),
        .s_valid(s_valid), .s_ready(s_ready), .s_re(s_re), .s_im(s_im),
        .m_valid(m_valid), .m_ready(m_ready), .m_re(m_re), .m_im(m_im), .status(status));

    reg [47:0] mem [0:4095];
    integer nin, gaps, i, j, f, n_skip, n_out;

    always @(negedge clk) if (gaps != 0) m_ready <= (($random & 3) != 0);

    task send(input signed [23:0] re, input signed [23:0] im);
        begin
            if (gaps != 0) begin
                j = $random & 3;
                while (j > 0) begin @(posedge clk); j = j - 1; end
            end
            s_re <= re; s_im <= im; s_valid <= 1'b1;
            @(posedge clk);
            while (!s_ready) @(posedge clk);
            s_valid <= 1'b0;
        end
    endtask

    // output side: skip the LAT warm-up outputs, then record nin samples
    always @(posedge clk) if (m_valid && m_ready) begin
        if (n_skip < LAT) n_skip = n_skip + 1;
        else if (n_out < nin) begin
            $fwrite(f, "%0d,%0d,%0d\n", n_out, m_re, m_im);
            n_out = n_out + 1;
        end
    end

    initial begin #500000000; $display("FATAL: watchdog, %0d of %0d outputs", n_out, nin); $finish; end

    initial begin
        nin = 1971; gaps = 0; n_skip = 0; n_out = 0;
        if ($value$plusargs("NIN=%d", nin)) ;
        if ($test$plusargs("GAPS")) gaps = 1;
        $readmemh("wf_in.mem", mem);
        f = $fopen("wf_out.csv", "w");
        $fwrite(f, "idx,re_raw,im_raw\n");
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);
        for (i = 0; i < nin; i = i + 1) send($signed(mem[i][23:0]), $signed(mem[i][47:24]));
        for (i = 0; i < NFLUSH; i = i + 1) send(24'sd0, 24'sd0);          // push the last samples through the latency
        while (n_out < nin) @(posedge clk);
        repeat (20) @(posedge clk);
        $display("done: %0d outputs written to wf_out.csv, wiener status %h (bit0 noise learned, bit1 gamma sat, bit3 IFFT-input sat)", n_out, status);
        $fclose(f);
        $finish;
    end
endmodule
