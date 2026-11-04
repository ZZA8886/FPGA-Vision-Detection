`timescale 1ns/1ps
// A common scale preserves the requested chassis motion when a wheel saturates.
// Conversion coefficient is an explicit parameter derived from wheel CPR.
module wheel_targets #(
    parameter integer COUNTS_Q16_PER_MM_S = 2113
) (
    input wire clk,reset_n,start,
    input wire signed [31:0] wheel1,wheel2,wheel3,wheel4,
    input wire [15:0] wheel_limit_mm_s,
    output reg valid,
    output reg signed [31:0] target1,target2,target3,target4
);
    reg [17:0] a1,a2,a3,a4,peak;
    reg [15:0] limit;
    wire [17:0] m12=a1>a2?a1:a2,m34=a3>a4?a3:a4;
    reg signed [17:0] w1,w2,w3,w4;
    reg [2:0] state;
    reg [1:0] wheel_index;
    reg divide_start;
    reg [31:0] numerator,denominator;
    reg [16:0] scale;
    wire divider_busy,divider_valid;
    wire [31:0] quotient;
    unsigned_divider32 divider(clk,reset_n,divide_start,numerator,denominator,
        divider_busy,divider_valid,quotient);
    wire signed [17:0] selected_wheel=wheel_index==0?w1:wheel_index==1?w2:wheel_index==2?w3:w4;
    reg signed [35:0] scaled_wheel;
    reg signed [63:0] scaled_counts;
    wire signed [63:0] converted=scaled_counts/64'sd65536;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            w1<=0;w2<=0;w3<=0;w4<=0;state<=0;divide_start<=0;
            numerator<=0;denominator<=1;scale<=0;valid<=0;wheel_index<=0;scaled_wheel<=0;scaled_counts<=0;
            target1<=0;target2<=0;target3<=0;target4<=0;
            a1<=0;a2<=0;a3<=0;a4<=0;peak<=0;limit<=0;
        end else begin
            valid<=0;divide_start<=0;
            case(state)
                0: if(start) begin
                    wheel_index<=0;
                    w1<=wheel1[17:0];w2<=wheel2[17:0];w3<=wheel3[17:0];w4<=wheel4[17:0];
                    a1<=wheel1[17]?-wheel1[17:0]:wheel1[17:0];
                    a2<=wheel2[17]?-wheel2[17:0]:wheel2[17:0];
                    a3<=wheel3[17]?-wheel3[17:0]:wheel3[17:0];
                    a4<=wheel4[17]?-wheel4[17:0]:wheel4[17:0];
                    limit<=wheel_limit_mm_s;state<=5;
                end
                5:begin peak<=m12>m34?m12:m34;state<=6;end
                6:begin
                    if(limit==0) begin scale<=0;state<=2;end
                    else if(peak<=limit) begin scale<=65536;state<=2;end
                    else begin
                        numerator<={limit,16'b0};denominator<={14'b0,peak};
                        divide_start<=1;state<=1;
                    end
                end
                1: if(divider_valid) begin scale<=quotient[16:0];state<=2;end
                2:begin scaled_wheel<=selected_wheel*$signed({1'b0,scale});state<=3;end
                3:begin scaled_counts<=scaled_wheel*COUNTS_Q16_PER_MM_S;state<=4;end
                4: begin
                    case(wheel_index)
                        0:target1<=converted[31:0];1:target2<=converted[31:0];
                        2:target3<=converted[31:0];3:target4<=converted[31:0];
                    endcase
                    if(wheel_index==3) begin valid<=1;state<=0;end
                    else begin wheel_index<=wheel_index+1'b1;state<=2;end
                end
                default:state<=0;
            endcase
        end
    end
endmodule
