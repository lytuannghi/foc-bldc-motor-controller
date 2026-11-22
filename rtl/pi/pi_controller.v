// =============================================================================
// File: pi_controller.v
// Module: pi_controller
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Proportional-Integral (PI) Regulator with Anti-Windup Clamping.
//              Operates in Q15 fixed-point. Used for d-axis (flux=0) and
//              q-axis (torque/speed demand) current regulation loops.
//              Output = Clamp(Kp * e + Integral, -OUT_MAX, +OUT_MAX)
// =============================================================================

`timescale 1ns / 1ps

module pi_controller (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        enable,
    input  wire signed [15:0] ref_val,    // Target reference in Q15
    input  wire signed [15:0] fdbk_val,   // Actual feedback in Q15

    input  wire signed [15:0] kp,         // Proportional gain in Q12
    input  wire signed [15:0] ki,         // Integral gain in Q15

    output reg  signed [15:0] out_val     // Regulated voltage output in Q15
);

    localparam signed [31:0] OUT_MAX = 32'sd30000;
    localparam signed [31:0] OUT_MIN = -32'sd30000;

    reg signed [16:0] err;
    reg signed [31:0] p_term;
    reg signed [31:0] i_accum;
    reg signed [31:0] i_term;
    reg signed [31:0] total_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_val   <= 16'sd0;
            err       <= 17'sd0;
            p_term    <= 32'sd0;
            i_accum   <= 32'sd0;
            i_term    <= 32'sd0;
            total_out <= 32'sd0;
        end else begin
            if (!enable) begin
                out_val <= 16'sd0;
                i_accum <= 32'sd0;
            end else begin
                // Compute Error e = Ref - Feedback
                err = ref_val - fdbk_val;

                // Proportional Term: (Kp * e) >>> 12
                p_term = (kp * err) >>> 12;

                // Integral Term Accumulation: i_accum = i_accum + ((Ki * e) >>> 15)
                i_term = (ki * err) >>> 15;
                i_accum = i_accum + i_term;

                // Anti-windup on Integral accumulator
                if (i_accum > OUT_MAX)
                    i_accum = OUT_MAX;
                else if (i_accum < OUT_MIN)
                    i_accum = OUT_MIN;

                // Total Output = P + I
                total_out = p_term + i_accum;

                // Clamping final output
                if (total_out > OUT_MAX)
                    out_val <= OUT_MAX[15:0];
                else if (total_out < OUT_MIN)
                    out_val <= OUT_MIN[15:0];
                else
                    out_val <= total_out[15:0];
            end
        end
    end

endmodule
