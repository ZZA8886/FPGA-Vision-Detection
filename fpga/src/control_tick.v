`timescale 1ns/1ps
module control_tick #(
    parameter integer CLK_HZ = 50000000,
    parameter integer CONTROL_HZ = 200
) (
    input wire clk,
    input wire reset_n,
    output reg tick
);
    localparam integer TICKS = CLK_HZ / CONTROL_HZ;
    localparam integer COUNTER_BITS=$clog2(TICKS)>0?$clog2(TICKS):1;
    reg [COUNTER_BITS-1:0] counter;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin counter <= 0; tick <= 0; end
        else begin
            tick <= counter == TICKS - 1;
            if (counter == TICKS - 1) counter <= 0;
            else counter <= counter + 1'b1;
        end
    end
endmodule
