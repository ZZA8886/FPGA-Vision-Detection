`timescale 1ns/1ps
// Sequential binary-to-ASCII, no multipliers/dividers. Byte zero is bits 7:0.
// decimals=1 prints a centi-unit value with exactly two decimal places.
module signed_decimal (
    input wire clk,reset_n,start,
    input wire signed [31:0] value,
    input wire decimals,
    output reg [103:0] ascii,
    output reg [3:0] length,
    output reg done
);
    reg [2:0] state;
    reg [31:0] binary;
    reg [39:0] bcd,adjusted;
    reg [5:0] count;
    reg [3:0] digit_index;
    reg fractional,negative,leading;
    integer j;
    wire [39:0] shifted=bcd>>(digit_index*4);
    wire [3:0] digit=shifted[3:0];
    always @* begin
        adjusted=bcd;
        for(j=0;j<10;j=j+1)
            if(bcd[j*4 +: 4]>=5) adjusted[j*4 +: 4]=bcd[j*4 +: 4]+4'd3;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;binary<=0;bcd<=0;count<=0;digit_index<=0;
            fractional<=0;negative<=0;leading<=0;ascii<=0;length<=0;done<=0;
        end else begin
            done<=0;
            case(state)
                0:if(start) begin
                    binary<=value[31]?-value:value;negative<=value[31];fractional<=decimals;
                    bcd<=0;count<=0;ascii<=0;length<=0;leading<=1;state<=1;
                end
                1:begin
                    bcd<={adjusted[38:0],binary[31]};binary<=binary<<1;
                    if(count==31) state<=2;else count<=count+1'b1;
                end
                2:begin
                    if(negative) begin ascii[7:0]<=8'd45;length<=1;end
                    digit_index<=9;state<=3;
                end
                3:begin
                    if(!(leading && digit==0 && digit_index>(fractional?2:0))) begin
                        ascii[length*8 +: 8]<=8'd48+{4'b0,digit};length<=length+1'b1;leading<=0;
                    end
                    if(digit_index==0) begin done<=1;state<=0;end
                    else begin
                        digit_index<=digit_index-1'b1;
                        if(fractional && digit_index==2) state<=4;
                    end
                end
                4:begin ascii[length*8 +: 8]<=8'd46;length<=length+1'b1;state<=3;end
                default:state<=0;
            endcase
        end
    end
endmodule
