`timescale 1ns/1ps
// Position command unit: 0.01 degree. Pulse width updates only between frames.
module servo_pwm #(
    parameter integer CLK_HZ = 50000000
) (
    input wire clk,
    input wire reset_n,
    input wire [15:0] angle_cdeg,
    output wire pwm
);
    localparam integer FRAME_TICKS = CLK_HZ / 50;
    localparam integer CENTER_TICKS = CLK_HZ * 3 / 2000;
    function integer gcd;
        input integer a,b;
        integer r;
        begin while(b!=0) begin r=a%b;a=b;b=r;end gcd=a;end
    endfunction
    localparam integer COMMON=gcd(CLK_HZ,9000000);
    localparam integer FACTOR=CLK_HZ/COMMON;
    localparam integer DIVISOR=9000000/COMMON;
    localparam integer PHASE_BITS=$clog2(FRAME_TICKS)>0?$clog2(FRAME_TICKS):1;
    reg [PHASE_BITS-1:0] phase;
    reg [31:0] pulse_ticks;
    wire [15:0] clamped_angle = angle_cdeg > 16'd18000 ? 16'd18000 : angle_cdeg;
    // At 50 MHz this reduces exactly to round(angle*50/9), plus 25000.
    // Compute off the frame path, using a 32-cycle divider rather than a wide
    // combinational divider. Changes within 0.7 us of a boundary may wait a frame.
    reg [15:0] seen_angle;
    reg [31:0] numerator,computed_ticks;
    wire [31:0] scaled_angle /* synthesis syn_dspstyle="logic" */;
    assign scaled_angle=clamped_angle*FACTOR;
    reg divide_start;
    wire divide_busy,divide_done;
    wire [31:0] quotient;
    unsigned_divider32 divider(clk,reset_n,divide_start,numerator,DIVISOR,
        divide_busy,divide_done,quotient);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            seen_angle<=9000;numerator<=0;computed_ticks<=CENTER_TICKS;divide_start<=0;
        end else begin
            divide_start<=0;
            if(!divide_busy && !divide_start && clamped_angle!=seen_angle) begin
                numerator<=scaled_angle+DIVISOR/2;
                seen_angle<=clamped_angle;divide_start<=1;
            end
            if(divide_done) computed_ticks<=CLK_HZ/2000+quotient;
        end
    end
    assign pwm = reset_n && phase < pulse_ticks;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            phase <= 0;
            pulse_ticks <= CENTER_TICKS;
        end else if (phase == FRAME_TICKS - 1) begin
            phase <= 0;
            pulse_ticks <= computed_ticks;
        end else begin
            phase <= phase + 1'b1;
        end
    end
endmodule
