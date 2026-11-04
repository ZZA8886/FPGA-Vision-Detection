`timescale 1ns/1ps
// 20 kHz by default, signed effort -1000..1000. Direction reversal inserts
// one full PWM frame in coast before energizing the opposite direction.
module tb6612_motor #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_HZ = 20000
) (
    input wire clk,
    input wire reset_n,
    input wire enable,
    input wire stop_brake,
    input wire invert_direction,
    input wire signed [11:0] effort,
    output wire in1,
    output wire in2,
    output wire pwm
);
    localparam integer PERIOD = CLK_HZ/PWM_HZ;
    function integer gcd;
        input integer a,b;
        integer r;
        begin while(b!=0) begin r=a%b;a=b;b=r;end gcd=a;end
    endfunction
    localparam integer COMMON=gcd(1000,PERIOD);
    localparam integer PHASE_FACTOR=1000/COMMON,DUTY_FACTOR=PERIOD/COMMON;
    localparam integer PHASE_BITS=$clog2(PERIOD)>0?$clog2(PERIOD):1;
    reg [PHASE_BITS-1:0] phase;
    reg [10:0] active_magnitude;
    reg active_reverse;
    reg reversing;
    wire requested_reverse=effort[11] ^ invert_direction;
    wire [11:0] magnitude=effort[11] ? -effort : effort;
    wire [10:0] bounded=magnitude>1000 ? 11'd1000 : magnitude[10:0];
    wire stopped=!reset_n || !enable || active_magnitude==0;
    wire brake=stopped && stop_brake && !reversing;
    assign in1=stopped ? brake : !active_reverse;
    assign in2=stopped ? brake : active_reverse;
    // IN1=IN2=0,PWM=1: coast. IN1=IN2=1,PWM=1: short brake.
    // Cancel a common positive factor exactly. At 50 MHz/20 kHz the
    // comparison becomes phase*2 < magnitude*5, requiring no DSP blocks.
    wire [31:0] phase_scaled /* synthesis syn_dspstyle="logic" */;
    wire [31:0] duty_scaled /* synthesis syn_dspstyle="logic" */;
    assign phase_scaled=phase*PHASE_FACTOR;
    assign duty_scaled=active_magnitude*DUTY_FACTOR;
    assign pwm=stopped ? 1'b1 : phase_scaled<duty_scaled;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            phase<=0;active_magnitude<=0;active_reverse<=0;reversing<=0;
        end else begin
            if(phase==PERIOD-1) phase<=0;
            else phase<=phase+1'b1;
            if(!enable) begin active_magnitude<=0;reversing<=0;end
            else if(phase==PERIOD-1) begin
                if(active_magnitude!=0 && requested_reverse!=active_reverse && bounded!=0) begin
                    active_magnitude<=0;active_reverse<=requested_reverse;reversing<=1;
                end else begin
                    active_magnitude<=bounded;active_reverse<=requested_reverse;reversing<=0;
                end
            end
        end
    end
endmodule
