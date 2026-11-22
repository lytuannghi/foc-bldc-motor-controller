// =============================================================================
// File: foc_controller_top.v
// Module: foc_controller_top
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Full-Flow Synthesizable Top-Level ASIC Motor Controller Core.
//              Integrates:
//              - Forward Clarke & Park Transforms
//              - 16-Stage Pipelined CORDIC Engine for trigonometric generation
//              - Dual Proportional-Integral (PI) Current Controllers with Anti-Windup
//              - Inverse Park Transform
//              - Space Vector Pulse Width Modulation (SVPWM) with Dead-Time
//              - Sliding Mode Observer (SMO) for Sensorless Operation
// =============================================================================

`timescale 1ns / 1ps

module foc_controller_top (
    input  wire        clk,          // Core Clock (100 MHz - 200 MHz)
    input  wire        rst_n,

    // Command & Mode Interface
    input  wire        foc_enable,
    input  wire        sensorless_mode, // 1: SMO estimated angle, 0: external encoder angle
    input  wire signed [15:0] ext_theta,       // External rotor angle (when sensorless=0)
    input  wire signed [15:0] id_ref,          // Target d-axis current (typically 0 for MTPA)
    input  wire signed [15:0] iq_ref,          // Target q-axis current (torque/speed demand)

    // Current Sensor ADC Sampling Inputs (Phase A & Phase B)
    input  wire        sample_trigger,
    input  wire signed [15:0] adc_ia,          // Measured Phase A current (Q15)
    input  wire signed [15:0] adc_ib,          // Measured Phase B current (Q15)

    // PI Gain Tuning Parameters
    input  wire signed [15:0] kp_d, ki_d,
    input  wire signed [15:0] kp_q, ki_q,

    // PWM Inverter Parameters
    input  wire [11:0] pwm_period,     // Carrier frequency divisor
    input  wire [7:0]  dead_time,      // Dead-time duration (clock cycles)

    // 3-Phase Complementary PWM Inverter Outputs
    output wire        pwm_a, pwm_an,  // Phase A High/Low side
    output wire        pwm_b, pwm_bn,  // Phase B High/Low side
    output wire        pwm_c, pwm_cn,  // Phase C High/Low side

    // Telemetry & Diagnostic Outputs
    output wire [2:0]  svpwm_sector,
    output wire signed [15:0] estimated_theta,
    output wire signed [15:0] estimated_omega,
    output wire signed [15:0] telemetry_id,
    output wire signed [15:0] telemetry_iq
);

    // Clarke Transform
    wire clarke_valid;
    wire signed [15:0] i_alpha;
    wire signed [15:0] i_beta;

    clarke_transform u_clarke (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (sample_trigger),
        .i_a       (adc_ia),
        .i_b       (adc_ib),
        .out_valid (clarke_valid),
        .i_alpha   (i_alpha),
        .i_beta    (i_beta)
    );

    // Sliding Mode Observer (Sensorless Estimator)
    wire signed [15:0] smo_theta;
    wire signed [15:0] smo_omega;
    wire signed [15:0] v_alpha;
    wire signed [15:0] v_beta;

    smo_observer u_smo (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (foc_enable && sensorless_mode),
        .i_alpha   (i_alpha),
        .i_beta    (i_beta),
        .v_alpha   (v_alpha),
        .v_beta    (v_beta),
        .k_smo     (16'sd4096),
        .theta_hat (smo_theta),
        .omega_hat (smo_omega)
    );

    assign estimated_theta = smo_theta;
    assign estimated_omega = smo_omega;

    // Angle MUX: Select between External Encoder and SMO
    wire signed [15:0] active_theta = sensorless_mode ? smo_theta : ext_theta;

    // CORDIC Rotator for Sine / Cosine Generation
    wire cordic_valid;
    wire signed [15:0] cos_theta;
    wire signed [15:0] sin_theta;

    cordic_rotator u_cordic (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (clarke_valid),
        .theta_in  (active_theta),
        .out_valid (cordic_valid),
        .cos_out   (cos_theta),
        .sin_out   (sin_theta)
    );

    // Forward Park Transform
    wire park_valid;
    wire signed [15:0] i_d;
    wire signed [15:0] i_q;

    park_transform u_park (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (cordic_valid),
        .i_alpha   (i_alpha),
        .i_beta    (i_beta),
        .cos_theta (cos_theta),
        .sin_theta (sin_theta),
        .out_valid (park_valid),
        .i_d       (i_d),
        .i_q       (i_q)
    );

    assign telemetry_id = i_d;
    assign telemetry_iq = i_q;

    // Dual PI Controllers
    wire signed [15:0] v_d;
    wire signed [15:0] v_q;

    pi_controller u_pi_d (
        .clk      (clk),
        .rst_n    (rst_n),
        .enable   (foc_enable),
        .ref_val  (id_ref),
        .fdbk_val (i_d),
        .kp       (kp_d),
        .ki       (ki_d),
        .out_val  (v_d)
    );

    pi_controller u_pi_q (
        .clk      (clk),
        .rst_n    (rst_n),
        .enable   (foc_enable),
        .ref_val  (iq_ref),
        .fdbk_val (i_q),
        .kp       (kp_q),
        .ki       (ki_q),
        .out_val  (v_q)
    );

    // Inverse Park Transform
    wire inv_park_valid;

    inv_park_transform u_inv_park (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (park_valid),
        .v_d       (v_d),
        .v_q       (v_q),
        .cos_theta (cos_theta),
        .sin_theta (sin_theta),
        .out_valid (inv_park_valid),
        .v_alpha   (v_alpha),
        .v_beta    (v_beta)
    );

    // Space Vector Pulse Width Modulation (SVPWM) Generator
    svpwm_generator u_svpwm (
        .clk        (clk),
        .rst_n      (rst_n),
        .enable     (foc_enable),
        .v_alpha    (v_alpha),
        .v_beta     (v_beta),
        .pwm_period (pwm_period),
        .dead_time  (dead_time),
        .sector     (svpwm_sector),
        .pwm_a      (pwm_a),
        .pwm_an     (pwm_an),
        .pwm_b      (pwm_b),
        .pwm_bn     (pwm_bn),
        .pwm_c      (pwm_c),
        .pwm_cn     (pwm_cn)
    );

endmodule
