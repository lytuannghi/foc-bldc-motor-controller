// =============================================================================
// File: smo_observer.v
// Module: smo_observer
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Sliding Mode Observer (SMO) for Sensorless Rotor Position and
//              Velocity Estimation.
//              - Estimates back-EMF (E_alpha, E_beta) using sliding control surface
//              - Computes rotor electrical angle theta_hat and angular speed omega_hat
// =============================================================================

`timescale 1ns / 1ps

module smo_observer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        enable,
    input  wire signed [15:0] i_alpha,     // Measured current I_alpha (Q15)
    input  wire signed [15:0] i_beta,      // Measured current I_beta (Q15)
    input  wire signed [15:0] v_alpha,     // Phase voltage V_alpha (Q15)
    input  wire signed [15:0] v_beta,      // Phase voltage V_beta (Q15)

    input  wire signed [15:0] k_smo,       // Sliding mode gain (Q15)

    output reg  signed [15:0] theta_hat,   // Estimated rotor angle (-pi to +pi)
    output reg  signed [15:0] omega_hat    // Estimated rotor angular velocity
);

    reg signed [15:0] i_alpha_est;
    reg signed [15:0] i_beta_est;
    reg signed [15:0] e_alpha;
    reg signed [15:0] e_beta;
    reg signed [15:0] theta_prev;

    // Intermediate sliding variables
    reg signed [15:0] err_alpha, err_beta;
    reg signed [15:0] z_alpha, z_beta;

    // Filtered back-EMF
    reg signed [31:0] emf_alpha_accum;
    reg signed [31:0] emf_beta_accum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_alpha_est     <= 16'sd0;
            i_beta_est      <= 16'sd0;
            e_alpha         <= 16'sd0;
            e_beta          <= 16'sd0;
            theta_hat       <= 16'sd0;
            theta_prev      <= 16'sd0;
            omega_hat       <= 16'sd0;
            emf_alpha_accum <= 32'sd0;
            emf_beta_accum  <= 32'sd0;
            err_alpha       <= 16'sd0;
            err_beta        <= 16'sd0;
            z_alpha         <= 16'sd0;
            z_beta          <= 16'sd0;
        end else begin
            if (!enable) begin
                theta_hat <= 16'sd0;
                omega_hat <= 16'sd0;
            end else begin
                theta_prev <= theta_hat;

                // Current estimation error
                err_alpha = i_alpha_est - i_alpha;
                err_beta  = i_beta_est  - i_beta;

                // Signum function for sliding control
                z_alpha = (err_alpha > 0) ? k_smo : -k_smo;
                z_beta  = (err_beta  > 0) ? k_smo : -k_smo;

                // Low-pass filter for Back-EMF reconstruction
                e_alpha <= e_alpha + ((z_alpha - e_alpha) >>> 4);
                e_beta  <= e_beta  + ((z_beta  - e_beta)  >>> 4);

                // Update estimated current
                i_alpha_est <= i_alpha_est + ((v_alpha - e_alpha) >>> 5);
                i_beta_est  <= i_beta_est  + ((v_beta  - e_beta)  >>> 5);

                // Angle estimation: theta_hat = -atan2(e_alpha, -e_beta)
                // Linearized quadrant approximation
                if (e_beta >= 0)
                    theta_hat <= (e_alpha >>> 1) + 16'sd16384;
                else
                    theta_hat <= -(e_alpha >>> 1) - 16'sd16384;

                // Velocity = delta_theta
                omega_hat <= (theta_hat - theta_prev) <<< 2;
            end
        end
    end

endmodule
