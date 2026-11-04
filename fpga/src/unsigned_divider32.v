`timescale 1ns/1ps
// Restoring divider, one quotient bit per cycle. Caller must supply divisor>0.
module unsigned_divider32 (
    input wire clk,reset_n,start,
    input wire [31:0] dividend,divisor,
    output reg busy,valid,
    output reg [31:0] quotient
);
    reg [31:0] numerator,denominator,result;
    reg [32:0] remainder;
    reg [5:0] bit_index;
    wire [32:0] shifted={remainder[31:0],numerator[31]};
    wire subtract=shifted>={1'b0,denominator};
    wire [31:0] next_result={result[30:0],subtract};
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            busy<=0;valid<=0;quotient<=0;numerator<=0;
            denominator<=1;result<=0;remainder<=0;bit_index<=0;
        end else begin
            valid<=0;
            if(start && !busy) begin
                numerator<=dividend;denominator<=divisor;result<=0;
                remainder<=0;bit_index<=0;busy<=1;
            end else if(busy) begin
                remainder<=subtract ? shifted-{1'b0,denominator} : shifted;
                numerator<=numerator<<1;result<=next_result;
                if(bit_index==31) begin
                    quotient<=next_result;busy<=0;valid<=1;
                end else bit_index<=bit_index+1'b1;
            end
        end
    end
endmodule
