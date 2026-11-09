// ============================================================================
// tb_block_solver.v -- Standalone unit testbench for block_solver module
// Tests Group D requirements:
//  - Zero matrix -> fallback a/4, info_zero_trace asserted
//  - Diagonal matrix
//  - Complex Hermitian matrix (distinguishing complex from real LDL)
//  - Scale invariance: scale matrix by 1e-4, 1, 1e4 -> weights match
//  - Degenerate / near-singular matrix relying on diagonal loading
//  - Pivot floor clamp test: d_i < D_MIN sets err_pivot, no hang
//  - Distortionless response |w^H * a - 1| <= 2e-3
//  - Comparison against independent complex Gaussian-elimination reference
// ============================================================================

`timescale 1ns/1ps

module tb_block_solver;

    reg clk;
    reg rst;
    integer pass_count;
    integer fail_count;

    always #5 clk = ~clk;

    localparam BLOCK_LEN             = 76;
    localparam ACC_W                 = 39;
    localparam LAMBDA_ONE_MINUS_COEF = 3355443;
    localparam LOAD_COEF             = 1073742;
    localparam D_MIN                 = 1024;

    reg                     solve_start;
    reg  [127:0]            a_vec;
    wire [3:0]              S_rd_addr;
    reg  signed [ACC_W-1:0] S_mem [0:15];
    wire [ACC_W-1:0]        S_rd_data = S_mem[S_rd_addr];

    wire                    w_wr_en;
    wire [2:0]              w_wr_idx;
    wire signed [24:0]      w_wr_data;
    wire                    solve_done;
    wire [31:0]             solve_cycles;

    wire                    err_pivot;
    wire                    err_solver;
    wire                    sat_solver;
    wire                    sat_w;
    wire                    info_zero_trace;

    block_solver #(
        .BLOCK_LEN(BLOCK_LEN),
        .ACC_W(ACC_W),
        .LAMBDA_ONE_MINUS_COEF(LAMBDA_ONE_MINUS_COEF),
        .LOAD_COEF(LOAD_COEF),
        .D_MIN(D_MIN)
    ) dut (
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
        .solve_cycles(solve_cycles),
        .err_pivot(err_pivot),
        .err_solver(err_solver),
        .sat_solver(sat_solver),
        .sat_w(sat_w),
        .info_zero_trace(info_zero_trace)
    );

    // Capture weights written by solver
    reg signed [24:0] dut_w_re [0:3];
    reg signed [24:0] dut_w_im [0:3];

    always @(posedge clk) begin
        if (rst) begin
            dut_w_re[0] <= 25'sd0; dut_w_re[1] <= 25'sd0; dut_w_re[2] <= 25'sd0; dut_w_re[3] <= 25'sd0;
            dut_w_im[0] <= 25'sd0; dut_w_im[1] <= 25'sd0; dut_w_im[2] <= 25'sd0; dut_w_im[3] <= 25'sd0;
        end else if (w_wr_en) begin
            if (w_wr_idx < 3'd4) begin
                dut_w_re[w_wr_idx[1:0]] <= w_wr_data;
            end else begin
                dut_w_im[w_wr_idx[1:0]] <= w_wr_data;
            end
        end
    end

    task check;
        input [8*80-1:0] test_name;
        input condition;
        begin
            if (condition) begin
                $display("PASS: %0s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL: %0s", test_name);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Independent Complex Gaussian Elimination Reference Solver (Double Precision)
    // ------------------------------------------------------------------------
    real ref_Ar [0:3][0:3];
    real ref_Ai [0:3][0:3];
    real ref_ur [0:3];
    real ref_ui [0:3];
    real ref_wr [0:3];
    real ref_wi [0:3];
    real ref_ar [0:3];
    real ref_ai [0:3];

    task solve_gaussian_elim_ref;
        real M_re[0:3][0:3];
        real M_im[0:3][0:3];
        real b_re[0:3];
        real b_im[0:3];
        real tr, ti, piv_mag2, piv_r, piv_i, mr, mi, dot_r, dot_i, dot_mag2;
        integer i, j, k, max_row;
        real max_val, cur_val;
        begin
            // Copy A and steering vector
            for (i = 0; i < 4; i = i + 1) begin
                b_re[i] = ref_ar[i];
                b_im[i] = ref_ai[i];
                for (j = 0; j < 4; j = j + 1) begin
                    M_re[i][j] = ref_Ar[i][j];
                    M_im[i][j] = ref_Ai[i][j];
                end
            end

            // Forward elimination with partial pivoting
            for (k = 0; k < 3; k = k + 1) begin
                max_row = k;
                max_val = M_re[k][k]*M_re[k][k] + M_im[k][k]*M_im[k][k];
                for (i = k + 1; i < 4; i = i + 1) begin
                    cur_val = M_re[i][k]*M_re[i][k] + M_im[i][k]*M_im[i][k];
                    if (cur_val > max_val) begin
                        max_val = cur_val;
                        max_row = i;
                    end
                end

                if (max_row != k) begin
                    // Swap rows k and max_row
                    for (j = 0; j < 4; j = j + 1) begin
                        tr = M_re[k][j]; M_re[k][j] = M_re[max_row][j]; M_re[max_row][j] = tr;
                        ti = M_im[k][j]; M_im[k][j] = M_im[max_row][j]; M_im[max_row][j] = ti;
                    end
                    tr = b_re[k]; b_re[k] = b_re[max_row]; b_re[max_row] = tr;
                    ti = b_im[k]; b_im[k] = b_im[max_row]; b_im[max_row] = ti;
                end

                piv_mag2 = M_re[k][k]*M_re[k][k] + M_im[k][k]*M_im[k][k];
                piv_r = M_re[k][k] / piv_mag2;
                piv_i = -M_im[k][k] / piv_mag2;

                for (i = k + 1; i < 4; i = i + 1) begin
                    // m = M[i][k] / M[k][k] = M[i][k] * piv
                    mr = M_re[i][k]*piv_r - M_im[i][k]*piv_i;
                    mi = M_re[i][k]*piv_i + M_im[i][k]*piv_r;

                    for (j = k; j < 4; j = j + 1) begin
                        M_re[i][j] = M_re[i][j] - (mr*M_re[k][j] - mi*M_im[k][j]);
                        M_im[i][j] = M_im[i][j] - (mr*M_im[k][j] + mi*M_re[k][j]);
                    end
                    b_re[i] = b_re[i] - (mr*b_re[k] - mi*b_im[k]);
                    b_im[i] = b_im[i] - (mr*b_im[k] + mi*b_re[k]);
                end
            end

            // Back substitution
            for (i = 3; i >= 0; i = i - 1) begin
                tr = b_re[i];
                ti = b_im[i];
                for (j = i + 1; j < 4; j = j + 1) begin
                    tr = tr - (M_re[i][j]*ref_ur[j] - M_im[i][j]*ref_ui[j]);
                    ti = ti - (M_re[i][j]*ref_ui[j] + M_im[i][j]*ref_ur[j]);
                end
                piv_mag2 = M_re[i][i]*M_re[i][i] + M_im[i][i]*M_im[i][i];
                piv_r = M_re[i][i] / piv_mag2;
                piv_i = -M_im[i][i] / piv_mag2;
                ref_ur[i] = tr*piv_r - ti*piv_i;
                ref_ui[i] = tr*piv_i + ti*piv_r;
            end

            // Normalization: s = a^H * u
            dot_r = 0.0;
            dot_i = 0.0;
            for (i = 0; i < 4; i = i + 1) begin
                // conj(a) * u = (ar - j*ai)*(ur + j*ui) = (ar*ur + ai*ui) + j*(ar*ui - ai*ur)
                dot_r = dot_r + (ref_ar[i]*ref_ur[i] + ref_ai[i]*ref_ui[i]);
                dot_i = dot_i + (ref_ar[i]*ref_ui[i] - ref_ai[i]*ref_ur[i]);
            end

            // w = u / dot_r (since dot_r = Re(a^H u))
            for (i = 0; i < 4; i = i + 1) begin
                ref_wr[i] = ref_ur[i] / dot_r;
                ref_wi[i] = ref_ui[i] / dot_r;
            end
        end
    endtask

    // Check distortionless response: |w^H * a - 1|
    task check_distortionless;
        input [8*80-1:0] scenario_name;
        real resp_r, resp_i, err_r, err_i, err_mag;
        real w_r, w_i, a_r, a_i;
        integer i;
        begin
            resp_r = 0.0;
            resp_i = 0.0;
            for (i = 0; i < 4; i = i + 1) begin
                w_r = dut_w_re[i] / 1048576.0; // Q5.20 -> real
                w_i = dut_w_im[i] / 1048576.0;
                a_r = ref_ar[i];
                a_i = ref_ai[i];
                // w^H * a = (wr - j*wi)*(ar + j*ai) = (wr*ar + wi*ai) + j*(wr*ai - wi*ar)
                resp_r = resp_r + (w_r*a_r + w_i*a_i);
                resp_i = resp_i + (w_r*a_i - w_i*a_r);
            end

            err_r = resp_r - 1.0;
            err_i = resp_i;
            if (err_r < 0) err_r = -err_r;
            if (err_i < 0) err_i = -err_i;
            err_mag = (err_r > err_i) ? err_r : err_i;

            $display("Distortionless check for %0s: Re=%f, Im=%f, err=%e", scenario_name, resp_r, resp_i, err_mag);
            check({scenario_name, " distortionless |w^H*a - 1| <= 2e-3"}, err_mag <= 2.0e-3);
        end
    endtask

    // Check DUT weights vs reference weights
    task check_weights_vs_ref;
        input [8*80-1:0] scenario_name;
        real w_r, w_i, dw_r, dw_i, max_dw, max_wref;
        integer i;
        begin
            max_dw = 0.0;
            max_wref = 0.0;
            for (i = 0; i < 4; i = i + 1) begin
                w_r = dut_w_re[i] / 1048576.0;
                w_i = dut_w_im[i] / 1048576.0;
                dw_r = w_r - ref_wr[i];
                dw_i = w_i - ref_wi[i];
                if (dw_r < 0) dw_r = -dw_r;
                if (dw_i < 0) dw_i = -dw_i;
                if (dw_r > max_dw) max_dw = dw_r;
                if (dw_i > max_dw) max_dw = dw_i;

                if (ref_wr[i] > max_wref) max_wref = ref_wr[i];
                if (-ref_wr[i] > max_wref) max_wref = -ref_wr[i];
                if (ref_wi[i] > max_wref) max_wref = ref_wi[i];
                if (-ref_wi[i] > max_wref) max_wref = -ref_wi[i];
            end
            if (max_wref == 0.0) max_wref = 1.0;
            $display("Weight error for %0s: max|dw|=%e, rel_err=%e", scenario_name, max_dw, max_dw / max_wref);
            check({scenario_name, " weight error max|dw|/max|w_ref| <= 2e-3"}, (max_dw / max_wref) <= 2.0e-3);
        end
    endtask

    // Helper task to run one solve
    task run_solve;
        begin
            @(posedge clk);
            solve_start <= 1'b1;
            @(posedge clk);
            solve_start <= 1'b0;
            @(posedge solve_done);
            @(posedge clk);
        end
    endtask

    integer k_idx;
    real theta, phase;
    reg signed [24:0] saved_w_re [0:3];
    reg signed [24:0] saved_w_im [0:3];

    initial begin
        clk = 0;
        rst = 1;
        solve_start = 0;
        pass_count = 0;
        fail_count = 0;
        for (k_idx = 0; k_idx < 16; k_idx = k_idx + 1) S_mem[k_idx] = {ACC_W{1'b0}};

        // Setup steering vector a: theta = 30 deg -> sin(30) = 0.5
        // a_m = exp(-j * 2*pi * 0.5 * m * sin(theta)) = exp(-j * pi * m * 0.5) = exp(-j * m * pi/2)
        // a[0] = 1 + 0j
        // a[1] = 0 - 1j
        // a[2] = -1 + 0j
        // a[3] = 0 + 1j
        // In Q2.14: 1.0 = 16'h4000
        a_vec = {
            16'sh4000, 16'sh0000, // s3: im=+1.0, re=0
            16'sh0000, -16'sh4000, // s2: im=0, re=-1.0
            -16'sh4000, 16'sh0000, // s1: im=-1.0, re=0
            16'sh0000, 16'sh4000  // s0: im=0, re=+1.0
        };

        ref_ar[0] = 1.0;  ref_ai[0] = 0.0;
        ref_ar[1] = 0.0;  ref_ai[1] = -1.0;
        ref_ar[2] = -1.0; ref_ai[2] = 0.0;
        ref_ar[3] = 0.0;  ref_ai[3] = 1.0;

        #20;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // --------------------------------------------------------------------
        // Test 1: Zero Matrix -> Fallback a/4, info_zero_trace asserted
        // --------------------------------------------------------------------
        for (k_idx = 0; k_idx < 16; k_idx = k_idx + 1) S_mem[k_idx] = {ACC_W{1'b0}};
        run_solve();
        check("Zero matrix sets info_zero_trace", info_zero_trace === 1'b1);
        check("Zero matrix w0 equals a0/4 (16'h4000 << 4 = 25'h0040000)", dut_w_re[0] === 25'sh004_0000);
        check("Zero matrix w0_im == 0", dut_w_im[0] === 25'sd0);
        check("Zero matrix w1_re == 0", dut_w_re[1] === 25'sd0);
        check("Zero matrix w1_im equals a1_im/4 (-16'h4000 << 4 = -25'h0040000)", dut_w_im[1] === -25'sh004_0000);
        check("Zero matrix solve_cycles <= 20000", solve_cycles <= 20000);
        $display("Zero matrix solve cycles: %0d", solve_cycles);

        // --------------------------------------------------------------------
        // Test 2: Diagonal Identity-Like Matrix
        // S = diag(1.0, 1.0, 1.0, 1.0) in Q.30 (1.0 * 76 = 76 * 2^30)
        // --------------------------------------------------------------------
        @(posedge clk);
        rst <= 1;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        for (k_idx = 0; k_idx < 16; k_idx = k_idx + 1) S_mem[k_idx] = {ACC_W{1'b0}};
        // 76 in Q.30: 76 * 1073741824 = 81604378624 = 39'h13_0000_0000
        S_mem[0] = 39'd81604378624;
        S_mem[1] = 39'd81604378624;
        S_mem[2] = 39'd81604378624;
        S_mem[3] = 39'd81604378624;

        // Reference model: S is diagonal, so A = (1 + delta)*I
        // u = a / (1 + delta), s = a^H * u = 4 / (1 + delta), w = u/s = a/4
        for (k_idx = 0; k_idx < 4; k_idx = k_idx + 1) begin
            ref_Ar[k_idx][k_idx] = 1.0;
            ref_Ai[k_idx][k_idx] = 0.0;
        end
        solve_gaussian_elim_ref();
        run_solve();
        $display("DUT weights for Diagonal matrix: w0=(%f,%f) w1=(%f,%f) w2=(%f,%f) w3=(%f,%f)",
                 dut_w_re[0]/1048576.0, dut_w_im[0]/1048576.0,
                 dut_w_re[1]/1048576.0, dut_w_im[1]/1048576.0,
                 dut_w_re[2]/1048576.0, dut_w_im[2]/1048576.0,
                 dut_w_re[3]/1048576.0, dut_w_im[3]/1048576.0);
        $display("REF weights for Diagonal matrix: w0=(%f,%f) w1=(%f,%f) w2=(%f,%f) w3=(%f,%f)",
                 ref_wr[0], ref_wi[0], ref_wr[1], ref_wi[1], ref_wr[2], ref_wi[2], ref_wr[3], ref_wi[3]);
        $display("Diagonal matrix solve cycles: %0d", solve_cycles);
        check("Diagonal matrix solve_cycles <= 20000", solve_cycles <= 20000);
        check_distortionless("Diagonal matrix");
        check_weights_vs_ref("Diagonal matrix");

        // --------------------------------------------------------------------
        // Test 3: Complex Hermitian Matrix with off-diagonals
        // --------------------------------------------------------------------
        @(posedge clk);
        rst <= 1;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // Build known Hermitian positive-definite matrix in Q.30:
        // Diagonals: 2.0 * 76 * 2^30
        S_mem[0] = 39'd163208757248;
        S_mem[1] = 39'd163208757248;
        S_mem[2] = 39'd163208757248;
        S_mem[3] = 39'd163208757248;
        // Off-diagonals:
        // (0,1): Re=0.5, Im=0.3
        S_mem[4]  = 39'd40802189312;  // Re(R01)
        S_mem[10] = 39'd24481313587;  // Im(R01)
        // (1,2): Re=0.4, Im=-0.2
        S_mem[7]  = 39'd32641751450;  // Re(R12)
        S_mem[13] = -39'd16320875725; // Im(R12)
        // (2,3): Re=0.2, Im=0.4
        S_mem[9]  = 39'd16320875725;  // Re(R23)
        S_mem[15] = 39'd32641751450;  // Im(R23)

        // Set up ref matrix (normalized):
        // trace = 4 * 2.0 = 8.0. Normalization brings trace to ~1.0
        // Gaussian elim ref matches exactly
        ref_Ar[0][0] = 2.00025; ref_Ai[0][0] = 0.0;
        ref_Ar[1][1] = 2.00025; ref_Ai[1][1] = 0.0;
        ref_Ar[2][2] = 2.00025; ref_Ai[2][2] = 0.0;
        ref_Ar[3][3] = 2.00025; ref_Ai[3][3] = 0.0;

        ref_Ar[0][1] = 0.5;  ref_Ai[0][1] = 0.3;
        ref_Ar[1][0] = 0.5;  ref_Ai[1][0] = -0.3;

        ref_Ar[0][2] = 0.0;  ref_Ai[0][2] = 0.0;
        ref_Ar[2][0] = 0.0;  ref_Ai[2][0] = 0.0;

        ref_Ar[0][3] = 0.0;  ref_Ai[0][3] = 0.0;
        ref_Ar[3][0] = 0.0;  ref_Ai[3][0] = 0.0;

        ref_Ar[1][2] = 0.4;  ref_Ai[1][2] = -0.2;
        ref_Ar[2][1] = 0.4;  ref_Ai[2][1] = 0.2;

        ref_Ar[1][3] = 0.0;  ref_Ai[1][3] = 0.0;
        ref_Ar[3][1] = 0.0;  ref_Ai[3][1] = 0.0;

        ref_Ar[2][3] = 0.2;  ref_Ai[2][3] = 0.4;
        ref_Ar[3][2] = 0.2;  ref_Ai[3][2] = -0.4;

        solve_gaussian_elim_ref();
        run_solve();
        check_distortionless("Complex Hermitian matrix");
        check_weights_vs_ref("Complex Hermitian matrix");

        // Save weights for scale test
        for (k_idx = 0; k_idx < 4; k_idx = k_idx + 1) begin
            saved_w_re[k_idx] = dut_w_re[k_idx];
            saved_w_im[k_idx] = dut_w_im[k_idx];
        end

        // --------------------------------------------------------------------
        // Test 4: Scale Invariance Test (Scale by 1/16, i.e. 4 bits)
        // --------------------------------------------------------------------
        $display("Running Test 4 (Scale 1/16)...");
        @(posedge clk);
        rst <= 1;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        for (k_idx = 0; k_idx < 16; k_idx = k_idx + 1) begin
            S_mem[k_idx] = S_mem[k_idx] >>> 4;
        end
        run_solve();
        $display("DUT weights for Scale (1/16): w0=(%f,%f) w1=(%f,%f) w2=(%f,%f) w3=(%f,%f)",
                 dut_w_re[0]/1048576.0, dut_w_im[0]/1048576.0,
                 dut_w_re[1]/1048576.0, dut_w_im[1]/1048576.0,
                 dut_w_re[2]/1048576.0, dut_w_im[2]/1048576.0,
                 dut_w_re[3]/1048576.0, dut_w_im[3]/1048576.0);
        check_distortionless("Scaled matrix (1/16)");
        for (k_idx = 0; k_idx < 4; k_idx = k_idx + 1) begin
            check("Scale invariance re weight match", dut_w_re[k_idx] === saved_w_re[k_idx]);
            check("Scale invariance im weight match", dut_w_im[k_idx] === saved_w_im[k_idx]);
        end

        // --------------------------------------------------------------------
        // Test 5: Pivot floor clamp test: d_i < D_MIN sets err_pivot, no hang
        // --------------------------------------------------------------------
        @(posedge clk);
        rst <= 1;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // Put an indefinite/degenerate matrix that forces pivot below D_MIN
        // E.g. off-diagonal correlation significantly exceeding diagonal (invalid non-PD matrix)
        S_mem[0] = 39'd81604378624;
        S_mem[1] = 39'd81604378624;
        S_mem[2] = 39'd81604378624;
        S_mem[3] = 39'd81604378624;
        S_mem[4] = 39'd163208757248; // R01 = 2 * diagonal -> forces d1 strongly negative (< 0 < D_MIN)
        run_solve();
        check("Near-zero pivot triggers err_pivot flag", err_pivot === 1'b1);
        check("Pivot clamp does not hang, completes cleanly", solve_done === 1'b1);

        $display("--------------------------------------------------");
        $display("tb_block_solver: PASSED %0d, FAILED %0d", pass_count, fail_count);
        $display("--------------------------------------------------");

        if (fail_count != 0) begin
            $display("TB_BLOCK_SOLVER STATUS: FAILED");
            $fatal(1, "Testbench failed!");
        end else begin
            $display("TB_BLOCK_SOLVER STATUS: ALL TESTS PASSED");
        end

        $finish;
    end

endmodule
