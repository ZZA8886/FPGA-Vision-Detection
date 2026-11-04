`timescale 1ns/1ps
// Signed 32x32 product. Iterative arithmetic uses no DSP blocks.
module serial_mul32(
    input wire clk,reset_n,start,
    input wire signed [31:0] a,b,
    output reg busy,valid,
    output reg signed [63:0] result
);
    reg [63:0] multiplicand,accumulator;
    reg [31:0] multiplier;
    reg [4:0] bit_index;
    reg negative;
    wire [63:0] next_sum=accumulator+(multiplier[0]?multiplicand:64'd0);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin busy<=0;valid<=0;result<=0;multiplicand<=0;
            accumulator<=0;multiplier<=0;bit_index<=0;negative<=0;end
        else begin
            valid<=0;
            if(start && !busy) begin
                multiplicand<={32'b0,(a[31]?-$unsigned(a):$unsigned(a))};
                multiplier<=b[31]?-$unsigned(b):$unsigned(b);
                negative<=a[31]^b[31];accumulator<=0;bit_index<=0;busy<=1;
            end else if(busy) begin
                accumulator<=next_sum;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
                if(bit_index==31) begin busy<=0;valid<=1;result<=negative?-$signed(next_sum):$signed(next_sum);end
                else bit_index<=bit_index+1'b1;
            end
        end
    end
endmodule
