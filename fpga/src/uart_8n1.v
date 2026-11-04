`timescale 1ns/1ps
// One-byte ready/valid transmitter and receiver; no application protocol.
module uart_8n1 #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire reset_n,
    input wire rx,
    output wire tx,
    input wire [7:0] tx_data,
    input wire tx_valid,
    output wire tx_ready,
    output reg [7:0] rx_data,
    output reg rx_valid,
    output reg framing_error
);
    localparam integer BIT_TICKS = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer TIMER_BITS=$clog2(BIT_TICKS)>0?$clog2(BIT_TICKS):1;
    reg [9:0] tx_shift;
    reg [3:0] tx_remaining;
    reg [TIMER_BITS-1:0] tx_timer;
    assign tx_ready = tx_remaining == 0;
    assign tx = tx_remaining == 0 ? 1'b1 : tx_shift[0];
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            tx_shift <= 10'h3ff;
            tx_remaining <= 0;
            tx_timer <= 0;
        end else if (tx_ready) begin
            if (tx_valid) begin
                tx_shift <= {1'b1, tx_data, 1'b0};
                tx_remaining <= 10;
                tx_timer <= BIT_TICKS - 1;
            end
        end else if (tx_timer == 0) begin
            tx_shift <= {1'b1, tx_shift[9:1]};
            tx_remaining <= tx_remaining - 1'b1;
            tx_timer <= BIT_TICKS - 1;
        end else begin
            tx_timer <= tx_timer - 1'b1;
        end
    end

    (* syn_preserve = 1 *) reg [1:0] rx_sync;
    reg [1:0] rx_state;
    reg [TIMER_BITS-1:0] rx_timer;
    reg [2:0] rx_bit;
    reg [7:0] rx_shift;
    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) rx_sync <= 2'b11;
        else rx_sync <= {rx_sync[0], rx};
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_state <= IDLE;
            rx_timer <= 0;
            rx_bit <= 0;
            rx_shift <= 0;
            rx_data <= 0;
            rx_valid <= 0;
            framing_error <= 0;
        end else begin
            rx_valid <= 0;
            framing_error <= 0;
            case (rx_state)
                IDLE: if (!rx_sync[1]) begin
                    rx_timer <= BIT_TICKS / 2 - 1;
                    rx_state <= START;
                end
                START: if (rx_timer != 0) rx_timer <= rx_timer - 1'b1;
                else if (rx_sync[1]) rx_state <= IDLE;
                else begin
                    rx_timer <= BIT_TICKS - 1;
                    rx_bit <= 0;
                    rx_state <= DATA;
                end
                DATA: if (rx_timer != 0) rx_timer <= rx_timer - 1'b1;
                else begin
                    rx_shift[rx_bit] <= rx_sync[1];
                    rx_timer <= BIT_TICKS - 1;
                    if (rx_bit == 7) rx_state <= STOP;
                    else rx_bit <= rx_bit + 1'b1;
                end
                STOP: if (rx_timer != 0) rx_timer <= rx_timer - 1'b1;
                else begin
                    if (rx_sync[1]) begin
                        rx_data <= rx_shift;
                        rx_valid <= 1;
                    end else framing_error <= 1;
                    rx_state <= IDLE;
                end
                default: rx_state <= IDLE;
            endcase
        end
    end
endmodule
