`timescale 1ns/1ps
// Registered inverse kinematics with an iterative /1000 conversion.
module mecanum_mixer (
    input wire clk,reset_n,start,
    input wire signed [15:0] vx,vy,yaw,
    output reg valid,
    output reg signed [31:0] wheel1,wheel2,wheel3,wheel4
);
    reg signed [31:0] sum,difference,rotation;
    reg negative,divide_start;
    reg [31:0] numerator;
    reg [1:0] state;
    wire [15:0] abs_yaw=yaw[15]?-yaw:yaw;
    wire busy,done;
    wire [31:0] quotient;
    unsigned_divider32 divider(clk,reset_n,divide_start,numerator,32'd1000,busy,done,quotient);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            sum<=0;difference<=0;rotation<=0;negative<=0;divide_start<=0;numerator<=0;
            state<=0;valid<=0;wheel1<=0;wheel2<=0;wheel3<=0;wheel4<=0;
        end else begin
            valid<=0;divide_start<=0;
            case(state)
                0:if(start) begin
                    sum<=$signed(vx)+$signed({{16{vy[15]}},vy});
                    difference<=$signed(vx)-$signed({{16{vy[15]}},vy});
                    numerator<=abs_yaw*32'd181;negative<=yaw[15];state<=1;
                end
                1:begin divide_start<=1;state<=2;end
                2:if(done) begin rotation<=negative?-$signed(quotient):$signed(quotient);state<=3;end
                3:begin
                    wheel1<=difference-rotation;wheel2<=sum+rotation;
                    wheel3<=sum-rotation;wheel4<=difference+rotation;valid<=1;state<=0;
                end
            endcase
        end
    end
endmodule
