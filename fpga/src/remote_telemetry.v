`timescale 1ns/1ps
// 10 Hz ASCII hex telemetry. T + 80 words MSB nibble first + *XX + LF.
// Word i is read atomically when serialized; the frame spans ~56 ms.
module remote_telemetry #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,
    output reg [6:0] word_index,input wire [31:0] word_data,
    output wire [7:0] tx_data,output wire tx_valid,input wire tx_ready
);
    localparam integer TIMER_BITS=$clog2(CLK_HZ/10)>0?$clog2(CLK_HZ/10):1;
    reg [TIMER_BITS-1:0] timer;
    reg [31:0] saved_word;
    reg [2:0] state,nibble_index;
    reg [7:0] checksum;
    wire [3:0] nibble=saved_word[31:28];
    function [7:0] hexchar;
        input [3:0] n;
        begin hexchar=n<10 ? 8'd48+{4'b0,n} : 8'd55+{4'b0,n};end
    endfunction
    assign tx_data=state==1 ? 8'd84 : state==3 ? hexchar(nibble) :
        state==4 ? 8'd42 : state==5 ? hexchar(checksum[7:4]) :
        state==6 ? hexchar(checksum[3:0]) : 8'd10;
    assign tx_valid=state==1 || state>=3;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin timer<=0;state<=0;word_index<=0;saved_word<=0;checksum<=0;nibble_index<=0;end
        else begin
            if(timer==CLK_HZ/10-1) timer<=0;else timer<=timer+1'b1;
            case(state)
                0: if(timer==CLK_HZ/10-1) begin state<=1;word_index<=0;checksum<=0;end
                1: if(tx_ready) begin checksum<=tx_data;state<=2;end
                2: begin saved_word<=word_data;nibble_index<=0;state<=3;end
                3: if(tx_ready) begin
                    checksum<=checksum^tx_data;saved_word<=saved_word<<4;
                    if(nibble_index==7) begin
                        if(word_index==79) state<=4;
                        else begin word_index<=word_index+1'b1;state<=2;end
                    end else nibble_index<=nibble_index+1'b1;
                end
                4: if(tx_ready) state<=5;
                5: if(tx_ready) state<=6;
                6: if(tx_ready) state<=7;
                7: if(tx_ready) state<=0;
            endcase
        end
    end
endmodule
