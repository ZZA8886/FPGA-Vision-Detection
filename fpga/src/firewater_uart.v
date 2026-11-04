`timescale 1ns/1ps
// VOFA+ FireWater: target_rpm,actual_rpm,signed_pwm\n, 50 frames/s.
// RX is a byte-only interface reserved for future commands; never echoed.
module firewater_uart #(
    parameter integer CLK_HZ=50000000
) (
    input wire clk,reset_n,rx,
    output wire tx,
    input wire [1:0] selected_wheel,
    input wire [127:0] target_rpm_centi,actual_rpm_centi,
    input wire [47:0] pwm_commands,
    output wire [7:0] rx_data,
    output wire rx_valid,framing_error
);
    localparam integer FRAME_TICKS=CLK_HZ/50;
    localparam integer TIMER_BITS=$clog2(FRAME_TICKS)>0?$clog2(FRAME_TICKS):1;
    reg [TIMER_BITS-1:0] timer;
    reg [2:0] state;
    reg [1:0] field;
    reg signed [31:0] saved_target,saved_actual,saved_pwm;
    reg format_start;
    wire signed [31:0] number=field==0?saved_target:field==1?saved_actual:saved_pwm;
    wire [103:0] formatted;
    wire [3:0] length;
    wire format_done,ready;
    reg [3:0] byte_index;
    signed_decimal format(clk,reset_n,format_start,number,field!=2,formatted,length,format_done);
    wire [7:0] tx_data=state==3 ? formatted[byte_index*8 +: 8] : (field==2?8'd10:8'd44);
    wire tx_valid=state==3 || state==4;
    uart_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(115200)) uart(
        clk,reset_n,rx,tx,tx_data,tx_valid,ready,rx_data,rx_valid,framing_error);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            timer<=0;state<=0;field<=0;format_start<=0;byte_index<=0;
            saved_target<=0;saved_actual<=0;saved_pwm<=0;
        end else begin
            format_start<=0;
            if(timer==FRAME_TICKS-1) timer<=0;else timer<=timer+1'b1;
            case(state)
                0:if(timer==FRAME_TICKS-1) begin
                    // Snapshot the selected wheel and all fields once per line.
                    saved_target<=target_rpm_centi[selected_wheel*32 +: 32];
                    saved_actual<=actual_rpm_centi[selected_wheel*32 +: 32];
                    saved_pwm<={{20{pwm_commands[selected_wheel*12+11]}},pwm_commands[selected_wheel*12 +: 12]};
                    field<=0;state<=1;
                end
                1:begin format_start<=1;state<=2;end
                2:if(format_done) begin byte_index<=0;state<=3;end
                3:if(ready) begin
                    if(byte_index==length-1) state<=4;
                    else byte_index<=byte_index+1'b1;
                end
                4:if(ready) begin
                    if(field==2) state<=0;
                    else begin field<=field+1'b1;state<=1;end
                end
                default:state<=0;
            endcase
        end
    end
endmodule
