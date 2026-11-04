`timescale 1ns/1ps
// Menu-independent controller; the board/menu top supplies operator settings.
module navbot_motion_core #(
    parameter integer CLK_HZ=50000000,
    parameter integer CONTROL_HZ=200,
    // Confirmed 13 periods/rev * 30 reduction * 4 edges = 1560 wheel CPR.
    parameter integer DISTANCE_PER_COUNT_Q16=10162,
    parameter integer COUNTS_Q16_PER_MM_S=2113,
    parameter integer SHARED_PI=0
) (
    input wire clk,reset_n,arm,clear_odometry,stop_brake,
    input wire signed [15:0] vx_mm_s,vy_mm_s,yaw_mrad_s,
    input wire [15:0] wheel_limit_mm_s,kp_q8,ki_q8,
    input wire [3:0] motor_invert,encoder_invert,encoder_a,encoder_b,
    output wire [3:0] motor_in1,motor_in2,motor_pwm,
    output wire [63:0] encoder_counts,encoder_deltas,encoder_errors,
    output wire signed [63:0] x_mm_q16,y_mm_q16,
    output wire signed [31:0] heading_mrad_q16,
    output wire signed [31:0] measured_vx_mm_s,measured_vy_mm_s,measured_yaw_mrad_s,
    output wire odometry_valid,
    input wire manual_mode,
    input wire [1:0] manual_wheel,
    input wire signed [11:0] manual_effort,
    output wire [127:0] wheel_targets_q16,
    output wire [47:0] wheel_efforts,
    output wire wheel_sample_valid
);
    wire tick;
    control_tick #(.CLK_HZ(CLK_HZ),.CONTROL_HZ(CONTROL_HZ)) timer(clk,reset_n,tick);
    wire [3:0] sampled;
    wire signed [15:0] delta[0:3];
    reg signed [15:0] captured_delta[0:3];
    wire signed [31:0] mixed[0:3];
    wire signed [31:0] target[0:3];
    wire targets_valid,mixer_valid;
    localparam integer MONITOR_BITS=SHARED_PI?6:4;
    reg [MONITOR_BITS-1:0] monitor_delay;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) monitor_delay<=0;
        else if(targets_valid) monitor_delay<=SHARED_PI?44:12;
        else if(monitor_delay!=0) monitor_delay<=monitor_delay-1'b1;
    end
    // Observe all four efforts after either parallel or shared PI completion.
    assign wheel_sample_valid=monitor_delay==1;
    wire enabled=arm && wheel_limit_mm_s!=0;
    wire [47:0] shared_efforts;
    wire [63:0] captured_deltas;
    wire [127:0] pi_targets;
    generate if(SHARED_PI) begin: shared_control
        wheel_pi_bank pi(clk,reset_n,enabled && !manual_mode,targets_valid,
            pi_targets,captured_deltas,kp_q8,ki_q8,shared_efforts);
    end else begin: unused_shared_control
        assign shared_efforts=48'b0;
    end endgenerate
    mecanum_mixer inverse(clk,reset_n,sampled[0],vx_mm_s,vy_mm_s,yaw_mrad_s,
        mixer_valid,mixed[0],mixed[1],mixed[2],mixed[3]);
    wheel_targets #(.COUNTS_Q16_PER_MM_S(COUNTS_Q16_PER_MM_S)) targets(
        clk,reset_n,mixer_valid,mixed[0],mixed[1],mixed[2],mixed[3],wheel_limit_mm_s,
        targets_valid,target[0],target[1],target[2],target[3]);
    genvar k;
    generate for(k=0;k<4;k=k+1) begin:wheel
        wire [15:0] raw_count,raw_errors;
        wire signed [15:0] raw_delta;
        wire signed [11:0] effort;
        quadrature_encoder enc(clk,reset_n,encoder_a[k],encoder_b[k],tick,
            raw_count,raw_delta,sampled[k],raw_errors);
        assign delta[k]=encoder_invert[k] ? -raw_delta : raw_delta;
        assign encoder_counts[k*16 +: 16]=encoder_invert[k] ? -raw_count : raw_count;
        assign encoder_deltas[k*16 +: 16]=delta[k];
        assign encoder_errors[k*16 +: 16]=raw_errors;
        always @(posedge clk or negedge reset_n) begin
            if(!reset_n) captured_delta[k]<=0;
            else if(sampled[k]) captured_delta[k]<=delta[k];
        end
        wire signed [11:0] drive_effort=manual_mode ? (manual_wheel==k?manual_effort:12'sd0) : effort;
        assign wheel_targets_q16[k*32 +: 32]=(enabled && !manual_mode)?target[k]:32'sd0;
        assign wheel_efforts[k*12 +: 12]=enabled?drive_effort:12'sd0;
        assign pi_targets[k*32 +: 32]=target[k];
        assign captured_deltas[k*16 +: 16]=captured_delta[k];
        if(SHARED_PI) begin: shared
            assign effort=shared_efforts[k*12 +: 12];
        end else begin: parallel
            wheel_pi pi(clk,reset_n,enabled && !manual_mode,targets_valid,target[k],captured_delta[k],kp_q8,ki_q8,effort);
        end
        tb6612_motor #(.CLK_HZ(CLK_HZ)) driver(
            clk,reset_n,enabled,stop_brake,motor_invert[k],drive_effort,motor_in1[k],motor_in2[k],motor_pwm[k]);
    end endgenerate
    mecanum_odometry #(.DISTANCE_PER_COUNT_Q16(DISTANCE_PER_COUNT_Q16),.CONTROL_HZ(CONTROL_HZ),.SERIAL_TRIG(SHARED_PI)) odometry(
        clk,reset_n,clear_odometry,sampled[0],delta[0],delta[1],delta[2],delta[3],
        x_mm_q16,y_mm_q16,heading_mrad_q16,measured_vx_mm_s,measured_vy_mm_s,measured_yaw_mrad_s,
        odometry_valid);
endmodule
