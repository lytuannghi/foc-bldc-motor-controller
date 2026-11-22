// =============================================================================
// File: clarke_transform.v
// Module: clarke_transform
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Forward Clarke Transform converting 3-phase currents (Ia, Ib)
//              into 2-phase stationary orthogonal currents (I_alpha, I_beta)
//              conforming to:
//              I_alpha = Ia
//              I_beta  = (Ia + 2*Ib) / sqrt(3)
//              Uses Q15 fixed-point arithmetic (1/sqrt(3) ~= 18919 / 32768).
// =============================================================================

`timescale 1ns / 1ps

module clarke_transform (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire signed [15:0] i_a,      // Phase A current (Q15 signed)
    input  wire signed [15:0] i_b,      // Phase B current (Q15 signed)

    output reg         out_valid,
    output reg  signed [15:0] i_alpha,  // Alpha current (Q15 signed)
    output reg  signed [15:0] i_beta    // Beta current (Q15 signed)
);

    // Constant: 1/sqrt(3) in Q15 = round(1/sqrt(3) * 32768) = 18919
    localparam signed [16:0] INV_SQRT3 = 17'sd18919;

    reg signed [16:0] sum_term;
    reg signed [33:0] prod_beta;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            i_alpha   <= 16'sd0;
            i_beta    <= 16'sd0;
            sum_term  <= 17'sd0;
            prod_beta <= 34'sd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                // I_alpha = Ia
                i_alpha <= i_a;

                // sum_term = Ia + 2*Ib
                sum_term = i_a + (i_b <<< 1);

                // prod_beta = sum_term * (1/sqrt(3))
                prod_beta = sum_term * INV_SQRT3;

                // Scale down by 2^15
                i_beta <= prod_beta[30:15];
            end
        end
    end

endmodule
