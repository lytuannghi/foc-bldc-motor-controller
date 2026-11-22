// =============================================================================
// File: park_transform.v
// Module: park_transform
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Forward Park Transform converting stationary 2-phase currents
//              (I_alpha, I_beta) into rotating reference frame currents (I_d, I_q)
//              using rotor flux angle trigonometric components (cos_theta, sin_theta):
//              I_d =  I_alpha * cos_theta + I_beta * sin_theta
//              I_q = -I_alpha * sin_theta + I_beta * cos_theta
// =============================================================================

`timescale 1ns / 1ps

module park_transform (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire signed [15:0] i_alpha,
    input  wire signed [15:0] i_beta,
    input  wire signed [15:0] cos_theta,
    input  wire signed [15:0] sin_theta,

    output reg         out_valid,
    output reg  signed [15:0] i_d,       // Direct axis current (flux component)
    output reg  signed [15:0] i_q        // Quadrature axis current (torque component)
);

    reg signed [31:0] p_alpha_cos;
    reg signed [31:0] p_beta_sin;
    reg signed [31:0] p_alpha_sin;
    reg signed [31:0] p_beta_cos;

    reg signed [31:0] sum_d;
    reg signed [31:0] sum_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid   <= 1'b0;
            i_d         <= 16'sd0;
            i_q         <= 16'sd0;
            p_alpha_cos <= 32'sd0;
            p_beta_sin  <= 32'sd0;
            p_alpha_sin <= 32'sd0;
            p_beta_cos  <= 32'sd0;
            sum_d       <= 32'sd0;
            sum_q       <= 32'sd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                p_alpha_cos = i_alpha * cos_theta;
                p_beta_sin  = i_beta * sin_theta;
                p_alpha_sin = i_alpha * sin_theta;
                p_beta_cos  = i_beta * cos_theta;

                sum_d = p_alpha_cos + p_beta_sin;
                sum_q = p_beta_cos - p_alpha_sin;

                // Scale down by Q15 (>>> 15)
                i_d <= sum_d[30:15];
                i_q <= sum_q[30:15];
            end
        end
    end

endmodule
