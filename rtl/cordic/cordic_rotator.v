// =============================================================================
// File: cordic_rotator.v
// Module: cordic_rotator
// Project: Sensorless Field-Oriented Control (FOC) Motor Controller ASIC
// Description: 16-Stage Pipelined CORDIC Engine in Rotation Mode.
//              Generates high-precision sine and cosine values in signed Q15
//              format from rotor electrical angle theta in [-pi, +pi].
//              Theta range: -32768 (-pi) to +32767 (+pi).
// =============================================================================

`timescale 1ns / 1ps

module cordic_rotator (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire signed [15:0] theta_in, // Angle in Q15 format (-pi to +pi)

    output reg         out_valid,
    output reg  signed [15:0] cos_out,  // cos(theta) in Q15 (-32768 to +32767)
    output reg  signed [15:0] sin_out   // sin(theta) in Q15
);

    // Initial scale factor K = prod(1/sqrt(1 + 2^(-2i))) ~= 0.607252935
    // In Q15: round(0.607252935 * 32767) = 19896
    localparam signed [17:0] CORDIC_K = 18'sd19896;

    // Pre-computed arctan(2^(-i)) lookup table in Q15 (pi = 32768)
    wire signed [17:0] atan_table [0:15];
    assign atan_table[0]  = 18'sd8192;  // pi/4
    assign atan_table[1]  = 18'sd4836;
    assign atan_table[2]  = 18'sd2555;
    assign atan_table[3]  = 18'sd1297;
    assign atan_table[4]  = 18'sd651;
    assign atan_table[5]  = 18'sd326;
    assign atan_table[6]  = 18'sd163;
    assign atan_table[7]  = 18'sd81;
    assign atan_table[8]  = 18'sd41;
    assign atan_table[9]  = 18'sd20;
    assign atan_table[10] = 18'sd10;
    assign atan_table[11] = 18'sd5;
    assign atan_table[12] = 18'sd3;
    assign atan_table[13] = 18'sd1;
    assign atan_table[14] = 18'sd1;
    assign atan_table[15] = 18'sd0;

    // Pipeline registers with guard bits: 16 stages
    reg signed [17:0] x_pipe [0:16];
    reg signed [17:0] y_pipe [0:16];
    reg signed [17:0] z_pipe [0:16];
    reg [16:0]        v_pipe;

    integer s;

    // Quadrant mapping and initial rotation
    reg signed [17:0] x_init, y_init, z_init;
    always @(*) begin
        // Map theta from [-pi, pi] to [-pi/2, pi/2]
        if (theta_in > 16'sd16384) begin // Quadrant II (> pi/2)
            x_init = -CORDIC_K;
            y_init = 18'sd0;
            z_init = {{2{theta_in[15]}}, theta_in} - 18'sd32768; // theta - pi
        end else if (theta_in < -16'sd16384) begin // Quadrant III (< -pi/2)
            x_init = -CORDIC_K;
            y_init = 18'sd0;
            z_init = {{2{theta_in[15]}}, theta_in} + 18'sd32768; // theta + pi
        end else begin // Quadrant I & IV
            x_init = CORDIC_K;
            y_init = 18'sd0;
            z_init = {{2{theta_in[15]}}, theta_in};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_pipe    <= 17'd0;
            out_valid <= 1'b0;
            cos_out   <= 16'sd0;
            sin_out   <= 16'sd0;
            for (s = 0; s <= 16; s = s + 1) begin
                x_pipe[s] <= 18'sd0;
                y_pipe[s] <= 18'sd0;
                z_pipe[s] <= 18'sd0;
            end
        end else begin
            v_pipe[0]   <= in_valid;
            x_pipe[0]   <= x_init;
            y_pipe[0]   <= y_init;
            z_pipe[0]   <= z_init;

            // 16 Pipeline stages
            for (s = 0; s < 16; s = s + 1) begin
                v_pipe[s+1] <= v_pipe[s];
                if (z_pipe[s] >= 0) begin
                    x_pipe[s+1] <= x_pipe[s] - (y_pipe[s] >>> s);
                    y_pipe[s+1] <= y_pipe[s] + (x_pipe[s] >>> s);
                    z_pipe[s+1] <= z_pipe[s] - atan_table[s];
                end else begin
                    x_pipe[s+1] <= x_pipe[s] + (y_pipe[s] >>> s);
                    y_pipe[s+1] <= y_pipe[s] - (x_pipe[s] >>> s);
                    z_pipe[s+1] <= z_pipe[s] + atan_table[s];
                end
            end

            out_valid <= v_pipe[16];

            // Saturation clamping to signed Q15
            if (x_pipe[16] > 18'sd32767)
                cos_out <= 16'sd32767;
            else if (x_pipe[16] < -18'sd32767)
                cos_out <= -16'sd32767;
            else
                cos_out <= x_pipe[16][15:0];

            if (y_pipe[16] > 18'sd32767)
                sin_out <= 16'sd32767;
            else if (y_pipe[16] < -18'sd32767)
                sin_out <= -16'sd32767;
            else
                sin_out <= y_pipe[16][15:0];
        end
    end

endmodule
