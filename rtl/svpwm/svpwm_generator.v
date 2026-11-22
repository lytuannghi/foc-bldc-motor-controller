// =============================================================================
// File: svpwm_generator.v
// Module: svpwm_generator
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: Space Vector Pulse Width Modulation (SVPWM) Generator.
//              - Determines active voltage vector Sector (1 to 6)
//              - Calculates dwelling times T1, T2, and zero-vector time T0
//              - Center-aligned symmetrical triangular carrier modulation
//              - 3-Phase complementary PWM outputs (A, B, C and AN, BN, CN)
//              - Configurable hardware dead-time insertion
// =============================================================================

`timescale 1ns / 1ps

module svpwm_generator (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        enable,
    input  wire signed [15:0] v_alpha,    // Stationary voltage V_alpha (Q15)
    input  wire signed [15:0] v_beta,     // Stationary voltage V_beta (Q15)

    input  wire [11:0] pwm_period,        // Half-carrier period (e.g. 1000 cycles)
    input  wire [7:0]  dead_time,         // Dead-time duration in clock cycles

    output reg  [2:0]  sector,            // Active sector (1 to 6)
    output reg         pwm_a, pwm_an,     // Phase A High-side & Low-side
    output reg         pwm_b, pwm_bn,     // Phase B High-side & Low-side
    output reg         pwm_c, pwm_cn      // Phase C High-side & Low-side
);

    // sqrt(3)/2 in Q15 = round(sqrt(3)/2 * 32768) = 28378
    localparam signed [16:0] SQRT3_DIV2 = 17'sd28378;

    // Reference voltages for sector determination
    reg signed [31:0] p_alpha_sqrt3;
    reg signed [16:0] v_ref1;
    reg signed [16:0] v_ref2;
    reg signed [16:0] v_ref3;
    reg [2:0]         sector_code;

    // Intermediate times
    reg signed [31:0] x_time, y_time, z_time;
    reg [11:0] t_a, t_b, t_c;

    // Symmetrical triangle counter
    reg [11:0] carrier_cnt;
    reg        carrier_dir; // 0: counting up, 1: counting down

    always @(*) begin
        // V_ref1 = V_beta
        v_ref1 = v_beta;

        // V_ref2 = (sqrt(3)/2)*V_alpha - 0.5*V_beta
        p_alpha_sqrt3 = v_alpha * SQRT3_DIV2;
        v_ref2 = (p_alpha_sqrt3 >>> 15) - (v_beta >>> 1);

        // V_ref3 = -(sqrt(3)/2)*V_alpha - 0.5*V_beta
        v_ref3 = -(p_alpha_sqrt3 >>> 15) - (v_beta >>> 1);

        // Sector identification code N = A + 2B + 4C
        sector_code = {(v_ref3 > 0), (v_ref2 > 0), (v_ref1 > 0)};

        case (sector_code)
            3'b001: sector = 3'd2;
            3'b010: sector = 3'd6;
            3'b011: sector = 3'd1;
            3'b100: sector = 3'd4;
            3'b101: sector = 3'd3;
            3'b110: sector = 3'd5;
            default: sector = 3'd1;
        endcase

        // Normalized time projections
        x_time = (v_beta * pwm_period) >>> 15;
        y_time = ((p_alpha_sqrt3 >>> 15) * pwm_period + (v_beta >>> 1) * pwm_period) >>> 15;
        z_time = (-(p_alpha_sqrt3 >>> 15) * pwm_period + (v_beta >>> 1) * pwm_period) >>> 15;

        // Assign compare times based on sector
        case (sector)
            3'd1: begin
                t_a = (pwm_period - x_time - y_time) >> 1;
                t_b = t_a + x_time;
                t_c = t_b + y_time;
            end
            3'd2: begin
                t_b = (pwm_period - y_time + z_time) >> 1;
                t_a = t_b + y_time;
                t_c = t_a - z_time;
            end
            3'd3: begin
                t_b = (pwm_period - z_time - x_time) >> 1;
                t_c = t_b + z_time;
                t_a = t_c + x_time;
            end
            3'd4: begin
                t_c = (pwm_period + x_time + y_time) >> 1;
                t_b = t_c - x_time;
                t_a = t_b - y_time;
            end
            3'd5: begin
                t_c = (pwm_period + y_time - z_time) >> 1;
                t_a = t_c - y_time;
                t_b = t_a + z_time;
            end
            default: begin // Sector 6
                t_a = (pwm_period + z_time + x_time) >> 1;
                t_c = t_a - z_time;
                t_b = t_c - x_time;
            end
        endcase
    end

    // Triangle carrier generator & dead-time logic
    reg raw_pwm_a, raw_pwm_b, raw_pwm_c;
    reg [7:0] dt_cnt_a, dt_cnt_b, dt_cnt_c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            carrier_cnt <= 12'd0;
            carrier_dir <= 1'b0;
            raw_pwm_a   <= 1'b0;
            raw_pwm_b   <= 1'b0;
            raw_pwm_c   <= 1'b0;
            pwm_a       <= 1'b0; pwm_an <= 1'b0;
            pwm_b       <= 1'b0; pwm_bn <= 1'b0;
            pwm_c       <= 1'b0; pwm_cn <= 1'b0;
            dt_cnt_a    <= 8'd0;
            dt_cnt_b    <= 8'd0;
            dt_cnt_c    <= 8'd0;
        end else begin
            if (!enable) begin
                carrier_cnt <= 12'd0;
                pwm_a       <= 1'b0; pwm_an <= 1'b0;
                pwm_b       <= 1'b0; pwm_bn <= 1'b0;
                pwm_c       <= 1'b0; pwm_cn <= 1'b0;
            end else begin
                // Symmetrical Triangle Carrier
                if (!carrier_dir) begin
                    if (carrier_cnt >= pwm_period)
                        carrier_dir <= 1'b1;
                    else
                        carrier_cnt <= carrier_cnt + 12'd1;
                end else begin
                    if (carrier_cnt <= 12'd1)
                        carrier_dir <= 1'b0;
                    else
                        carrier_cnt <= carrier_cnt - 12'd1;
                end

                // Compare threshold to produce raw PWM
                raw_pwm_a <= (carrier_cnt > t_a);
                raw_pwm_b <= (carrier_cnt > t_b);
                raw_pwm_c <= (carrier_cnt > t_c);

                // Dead-Time Insertion for Phase A
                if (raw_pwm_a) begin
                    pwm_an <= 1'b0;
                    if (dt_cnt_a >= dead_time)
                        pwm_a <= 1'b1;
                    else
                        dt_cnt_a <= dt_cnt_a + 8'd1;
                end else begin
                    pwm_a <= 1'b0;
                    dt_cnt_a <= 8'd0;
                    pwm_an <= 1'b1;
                end

                // Dead-Time Insertion for Phase B
                if (raw_pwm_b) begin
                    pwm_bn <= 1'b0;
                    if (dt_cnt_b >= dead_time)
                        pwm_b <= 1'b1;
                    else
                        dt_cnt_b <= dt_cnt_b + 8'd1;
                end else begin
                    pwm_b <= 1'b0;
                    dt_cnt_b <= 8'd0;
                    pwm_bn <= 1'b1;
                end

                // Dead-Time Insertion for Phase C
                if (raw_pwm_c) begin
                    pwm_cn <= 1'b0;
                    if (dt_cnt_c >= dead_time)
                        pwm_c <= 1'b1;
                    else
                        dt_cnt_c <= dt_cnt_c + 8'd1;
                end else begin
                    pwm_c <= 1'b0;
                    dt_cnt_c <= 8'd0;
                    pwm_cn <= 1'b1;
                end
            end
        end
    end

endmodule
