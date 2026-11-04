`timescale 1ns/1ps
// SSD1306 128x64, page mode, external I2C pullups. Default 7-bit address 0x3c.
module oled_ssd1306 #(
    parameter integer CLK_HZ=50000000,
    parameter [6:0] ADDRESS=7'h3c
) (
    input wire clk,reset_n,
    inout wire scl,sda,
    output reg screen_start,
    input wire screen_done,
    output reg [2:0] page,
    output reg [6:0] column,
    input wire [7:0] pixel_byte,
    output reg ready,
    output wire bus_error
);
    reg start;
    reg [7:0] control,payload;
    wire busy,done,error;
    i2c_write3 #(.CLK_HZ(CLK_HZ)) bus(clk,reset_n,start,{ADDRESS,1'b0,control,payload},scl,sda,busy,done,error);
    assign bus_error=error;
    reg [3:0] state;
    reg [5:0] init_index;
    reg [1:0] page_command;
    reg [31:0] delay_counter;
    function [7:0] init_command;
        input [5:0] index;
        begin case(index)
            0:init_command=8'hae;1:init_command=8'hd5;2:init_command=8'h80;
            3:init_command=8'ha8;4:init_command=8'h3f;5:init_command=8'hd3;
            6:init_command=8'h00;7:init_command=8'h40;8:init_command=8'h8d;
            9:init_command=8'h14;10:init_command=8'h20;11:init_command=8'h02;
            12:init_command=8'ha1;13:init_command=8'hc8;14:init_command=8'hda;
            15:init_command=8'h12;16:init_command=8'h81;17:init_command=8'h7f;
            18:init_command=8'hd9;19:init_command=8'hf1;20:init_command=8'hdb;
            21:init_command=8'h40;22:init_command=8'ha4;23:init_command=8'ha6;
            default:init_command=8'haf;
        endcase end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            start<=0;control<=0;payload<=0;state<=0;init_index<=0;page_command<=0;
            delay_counter<=0;screen_start<=0;page<=0;column<=0;ready<=0;
        end else begin
            start<=0;screen_start<=0;
            case(state)
                0:if(delay_counter>=CLK_HZ/10) begin delay_counter<=0;init_index<=0;state<=1;end
                  else delay_counter<=delay_counter+1'b1;
                1:begin control<=0;payload<=init_command(init_index);start<=1;state<=2;end
                2:if(done) begin
                    if(error) begin ready<=0;delay_counter<=0;state<=0;end
                    else if(init_index==24) state<=3;
                    else begin init_index<=init_index+1'b1;state<=1;end
                end
                3:begin screen_start<=1;state<=4;end
                4:if(screen_done) begin page<=0;column<=0;page_command<=0;state<=5;end
                5:begin
                    control<=0;
                    case(page_command)
                        0:payload<=8'hb0+{5'b0,page};1:payload<=0;default:payload<=8'h10;
                    endcase
                    start<=1;state<=6;
                end
                6:if(done) begin
                    if(error) begin ready<=0;delay_counter<=0;state<=0;end
                    else if(page_command==2) state<=7;
                    else begin page_command<=page_command+1'b1;state<=5;end
                end
                9:state<=7; // synchronous character RAM read latency
                7:begin control<=8'h40;payload<=pixel_byte;start<=1;state<=8;end
                8:if(done) begin
                    if(error) begin ready<=0;delay_counter<=0;state<=0;end
                    else if(column==127) begin
                        column<=0;
                        if(page==7) begin ready<=1;state<=3;end
                        else begin page<=page+1'b1;page_command<=0;state<=5;end
                    end else begin column<=column+1'b1;state<=9;end
                end
                default:state<=0;
            endcase
        end
    end
endmodule
