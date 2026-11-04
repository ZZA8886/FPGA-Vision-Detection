`timescale 1ns/1ps
// Open-drain I2C: address/control/data transaction, ACK and clock stretching.
module i2c_write3 #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=400000
) (
    input wire clk,reset_n,start,
    input wire [23:0] bytes,
    inout wire scl,sda,
    output reg busy,done,error
);
    localparam integer QUARTER=(CLK_HZ+4*I2C_HZ-1)/(4*I2C_HZ);
    reg scl_low,sda_low;
    (* syn_preserve=1 *) reg [1:0] scl_sync,sda_sync;
    assign scl=scl_low?1'b0:1'bz;
    assign sda=sda_low?1'b0:1'bz;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin scl_sync<=3;sda_sync<=3;end
        else begin scl_sync<={scl_sync[0],scl};sda_sync<={sda_sync[0],sda};end
    end
    reg [31:0] timer,timeout_count;
    reg [4:0] state;
    reg [23:0] shift;
    reg [2:0] bit_index;
    reg [1:0] byte_index;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            scl_low<=0;sda_low<=0;busy<=0;done<=0;error<=0;
            timer<=0;timeout_count<=0;state<=0;shift<=0;bit_index<=7;byte_index<=0;
        end else begin
            done<=0;
            if(!busy) begin
                timer<=0;timeout_count<=0;scl_low<=0;sda_low<=0;
                if(start) begin busy<=1;error<=0;shift<=bytes;state<=1;bit_index<=7;byte_index<=0;end
            end else if(timeout_count>=CLK_HZ/100) begin
                // 10 ms bus timeout releases both lines and reports failure.
                busy<=0;done<=1;error<=1;scl_low<=0;sda_low<=0;state<=0;
            end else begin
                timeout_count<=timeout_count+1'b1;
                if(timer==QUARTER-1) begin
                    timer<=0;
                    case(state)
                        1:if(scl_sync[1] && sda_sync[1]) state<=2;
                        2:begin sda_low<=1;state<=3;end
                        3:begin scl_low<=1;state<=4;end
                        4:begin sda_low<=!shift[23];state<=5;end
                        5:begin scl_low<=0;state<=6;end
                        6:if(scl_sync[1]) state<=7;
                        7:begin
                            scl_low<=1;
                            if(bit_index==0) state<=8;
                            else begin bit_index<=bit_index-1'b1;shift<=shift<<1;state<=4;end
                        end
                        8:begin sda_low<=0;state<=9;end
                        9:begin scl_low<=0;state<=10;end
                        10:if(scl_sync[1]) begin if(sda_sync[1]) error<=1;state<=11;end
                        11:begin
                            scl_low<=1;
                            if(error || byte_index==2) state<=12;
                            else begin byte_index<=byte_index+1'b1;bit_index<=7;shift<=shift<<1;state<=4;end
                        end
                        12:begin sda_low<=1;state<=13;end
                        13:begin scl_low<=0;state<=14;end
                        14:if(scl_sync[1]) begin sda_low<=0;state<=15;end
                        15:begin busy<=0;done<=1;state<=0;end
                        default:state<=12;
                    endcase
                end else timer<=timer+1'b1;
            end
        end
    end
endmodule
