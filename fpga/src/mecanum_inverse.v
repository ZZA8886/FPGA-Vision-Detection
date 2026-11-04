`timescale 1ns/1ps
// Body frame: +X forward, +Y left, +yaw counterclockwise, top-view X rollers.
// Wheel order: front-left, front-right, rear-left, rear-right.
// Output is signed wheel tangential speed, in integer mm/s.
module mecanum_inverse (
    input wire signed [15:0] vx_mm_s,
    input wire signed [15:0] vy_mm_s,
    input wire signed [15:0] yaw_mrad_s,
    output wire signed [31:0] wheel1_mm_s,
    output wire signed [31:0] wheel2_mm_s,
    output wire signed [31:0] wheel3_mm_s,
    output wire signed [31:0] wheel4_mm_s
);
    // (205 + 157)/2 = 181 mm. Verilog signed division truncates toward zero.
    wire signed [31:0] rotation_product = yaw_mrad_s * 32'sd181;
    wire signed [31:0] rotation_mm_s = rotation_product / 32'sd1000;
    wire signed [31:0] vx = {{16{vx_mm_s[15]}}, vx_mm_s};
    wire signed [31:0] vy = {{16{vy_mm_s[15]}}, vy_mm_s};
    assign wheel1_mm_s = vx - vy - rotation_mm_s;
    assign wheel2_mm_s = vx + vy + rotation_mm_s;
    assign wheel3_mm_s = vx + vy - rotation_mm_s;
    assign wheel4_mm_s = vx - vy + rotation_mm_s;
endmodule
