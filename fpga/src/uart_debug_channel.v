`timescale 1ns/1ps
// Optional menu-controlled byte echo for wiring checks; no packet parsing.
module uart_debug_channel #(
    parameter integer CLK_HZ=50000000
) (
    input wire clk,reset_n,rx,echo_enable,
    output wire tx,
    output reg [15:0] receive_count,
    output wire [7:0] receive_data,
    output wire receive_valid,
    output wire framing_error,
    output reg overflow
);
    reg pending;
    reg [7:0] pending_data;
    wire ready;
    uart_8n1 #(.CLK_HZ(CLK_HZ)) uart(clk,reset_n,rx,tx,pending_data,
        pending && echo_enable,ready,receive_data,receive_valid,framing_error);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin pending<=0;pending_data<=0;receive_count<=0;overflow<=0;end
        else begin
            if(!echo_enable || (pending && ready)) pending<=0;
            if(receive_valid) begin
                receive_count<=receive_count+1'b1;
                if(echo_enable) begin
                    if(pending && !ready) overflow<=1;
                    else begin pending_data<=receive_data;pending<=1;end
                end
            end
        end
    end
endmodule
