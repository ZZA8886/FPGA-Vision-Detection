`timescale 1ns/1ps

module imu660ra_i2c_master #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=400000
) (
    input wire clk,reset_n,
    inout wire sda,
    output wire scl,
    input wire start,write_mode,config_mode,
    input wire [7:0] reg_addr,write_data,
    input wire [13:0] write_count,
    input wire [3:0] read_count,
    output wire [12:0] config_addr,
    input wire [7:0] config_data,
    output reg busy,done,ack_error,
    output reg [47:0] read_data
);
    localparam integer HALF_TICKS=(CLK_HZ/(I2C_HZ*2))>1?(CLK_HZ/(I2C_HZ*2)):1;
    localparam integer TIMER_BITS=$clog2(HALF_TICKS)>0?$clog2(HALF_TICKS):1;
    localparam [4:0] IDLE=0,START_H=1,START_L=2,SEND_L=3,SEND_H=4,
        ACK_L=5,ACK_H=6,READ_L=7,READ_H=8,READ_ACK_L=9,READ_ACK_H=10,
        STOP_L=11,STOP_H=12,STOP_RELEASE=13;

    reg [4:0] state;
    reg [TIMER_BITS-1:0] timer;
    reg scl_low,sda_low;
    reg [7:0] tx_byte,rx_shift,rx_byte;
    reg [2:0] bit_index;
    reg [1:0] tx_phase;
    reg [13:0] data_index;
    reg [13:0] latched_write_count;
    reg [3:0] read_index,latched_read_count;
    reg latched_write,latched_config;
    reg [7:0] latched_reg,latched_data;

    assign sda=sda_low?1'b0:1'bz;
    assign scl=scl_low?1'b0:1'bz;
    wire sda_in=sda;
    assign config_addr=config_mode && latched_config ?
        ((state==ACK_H && tx_phase==2)?data_index[12:0]+1'b1:data_index[12:0]):13'd0;

    always @* begin
        scl_low=1'b0;
        sda_low=1'b0;
        case(state)
            START_H: sda_low=1'b1;
            START_L: begin scl_low=1'b1;sda_low=1'b1;end
            SEND_L: begin
                scl_low=1'b1;
                sda_low=~tx_byte[7-bit_index];
            end
            SEND_H: sda_low=~tx_byte[7-bit_index];
            ACK_L: scl_low=1'b1;
            READ_L: scl_low=1'b1;
            READ_ACK_L: begin
                scl_low=1'b1;
                sda_low=read_index+1'b1<latched_read_count;
            end
            READ_ACK_H: sda_low=read_index+1'b1<latched_read_count;
            STOP_L: begin scl_low=1'b1;sda_low=1'b1;end
            STOP_H: sda_low=1'b1;
            default:;
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;timer<=0;busy<=0;done<=0;ack_error<=0;read_data<=0;
            tx_byte<=0;rx_shift<=0;rx_byte<=0;bit_index<=0;tx_phase<=0;
            data_index<=0;latched_write_count<=0;read_index<=0;
            latched_read_count<=0;latched_write<=0;latched_config<=0;
            latched_reg<=0;latched_data<=0;
        end else begin
            done<=0;
            if(state==IDLE) begin
                if(start && !busy) begin
                    busy<=1;ack_error<=0;read_data<=0;
                    latched_write<=write_mode;latched_config<=config_mode;
                    latched_write_count<=write_count;latched_read_count<=read_count;
                    latched_reg<=reg_addr;latched_data<=write_data;
                    tx_phase<=0;data_index<=0;read_index<=0;bit_index<=0;
                    tx_byte<=write_mode?8'hD2:8'hD2;
                    state<=START_H;
                end
            end else if(timer!=0) begin
                timer<=timer-1'b1;
            end else begin
                timer<=HALF_TICKS-1;
                case(state)
                    START_H: state<=START_L;
                    START_L: begin bit_index<=0;state<=SEND_L;end
                    SEND_L: state<=SEND_H;
                    SEND_H: begin
                        if(bit_index==7) state<=ACK_L;
                        else begin bit_index<=bit_index+1'b1;state<=SEND_L;end
                    end
                    ACK_L: state<=ACK_H;
                    ACK_H: begin
                        if(sda_in) begin
                            ack_error<=1;state<=STOP_L;
                        end else if(latched_write) begin
                            if(tx_phase==0) begin
                                tx_phase<=1;tx_byte<=latched_reg;bit_index<=0;state<=SEND_L;
                            end else if(tx_phase==1) begin
                                tx_phase<=2;data_index<=0;bit_index<=0;
                                tx_byte<=latched_config?config_data:latched_data;state<=SEND_L;
                            end else if(data_index+1'b1<latched_write_count) begin
                                data_index<=data_index+1'b1;bit_index<=0;
                                tx_byte<=latched_config?config_data:latched_data;state<=SEND_L;
                            end else state<=STOP_L;
                        end else if(tx_phase==0) begin
                            tx_phase<=1;tx_byte<=latched_reg;bit_index<=0;state<=SEND_L;
                        end else if(tx_phase==1) begin
                            tx_phase<=2;tx_byte<=8'hD3;bit_index<=0;state<=START_H;
                        end else begin
                            bit_index<=0;rx_shift<=0;read_index<=0;state<=READ_L;
                        end
                    end
                    READ_L: state<=READ_H;
                    READ_H: begin
                        if(bit_index==7) begin
                            rx_byte<={rx_shift[6:0],sda_in};
                            state<=READ_ACK_L;
                        end else begin
                            rx_shift<={rx_shift[6:0],sda_in};
                            bit_index<=bit_index+1'b1;state<=READ_L;
                        end
                    end
                    READ_ACK_L: state<=READ_ACK_H;
                    READ_ACK_H: begin
                        read_data<={read_data[39:0],rx_byte};
                        if(read_index+1'b1>=latched_read_count) state<=STOP_L;
                        else begin
                            read_index<=read_index+1'b1;bit_index<=0;state<=READ_L;
                        end
                    end
                    STOP_L: state<=STOP_H;
                    STOP_H: state<=STOP_RELEASE;
                    STOP_RELEASE: begin
                        state<=IDLE;busy<=0;done<=1;
                    end
                    default: state<=IDLE;
                endcase
            end
        end
    end
endmodule
