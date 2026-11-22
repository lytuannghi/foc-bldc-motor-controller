// =============================================================================
// File: inv_park_transform.v
// Module: inv_park_transform
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Inverse Park Transform converting rotating voltage references
//              (V_d, V_q) from PI controllers into stationary reference frame
//              voltages (V_alpha, V_beta) for Space Vector Modulation:
//              V_alpha = V_d * cos_theta - V_q * sin_theta
//              V_beta  = V_d * sin_theta + V_q * cos_theta
// =============================================================================

`timescale 1ns / 1ps

module inv_park_transform (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire signed [15:0] v_d,
    input  wire signed [15:0] v_q,
    input  wire signed [15:0] cos_theta,
    input  wire signed [15:0] sin_theta,

    output reg         out_valid,
    output reg  signed [15:0] v_alpha,
    output reg  signed [15:0] v_beta
);

    reg signed [31:0] p_vd_cos;
    reg signed [31:0] p_vq_sin;
    reg signed [31:0] p_vd_sin;
    reg signed [31:0] p_vq_cos;

    reg signed [31:0] sum_alpha;
    reg signed [31:0] sum_beta;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            v_alpha   <= 16'sd0;
            v_beta    <= 16'sd0;
            p_vd_cos  <= 32'sd0;
            p_vq_sin  <= 32'sd0;
            p_vd_sin  <= 32'sd0;
            p_vq_cos  <= 32'sd0;
            sum_alpha <= 32'sd0;
            sum_beta  <= 32'sd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                p_vd_cos = v_d * cos_theta;
                p_vq_sin = v_q * sin_theta;
                p_vd_sin = v_d * sin_theta;
                p_vq_cos = v_q * cos_theta;

                sum_alpha = p_vd_cos - p_vq_sin;
                sum_beta  = p_vd_sin + p_vq_cos;

                v_alpha <= sum_alpha[30:15];
                v_beta  <= sum_beta[30:15];
            end
        end
    end

endmodule
