`timescale 1ns/1ps
// Navigation-module command receiver on P6 3 (RX), 115200/8N1.
// Valid frames are AA 01..06 55. P6 5 (TX) is intentionally idle-high.
module nav_sequence_uart #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,rx,
    output wire tx,
    output reg [15:0] receive_count,
    output reg frame_valid,
    output reg [2:0] sequence_id
);
    wire [7:0] rx_data;
    wire uart_tx;
    wire rx_valid,framing_error;
    reg [1:0] state;
    reg [2:0] pending_id;
    // This interface is receive-only for the navigation module.  Keep the
    // physical TX pin at the UART idle level and do not drive the UART TX
    // output onto the same net (which would create a multiple-driver net).
    assign tx=1'b1;
    uart_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(115200)) uart(
        .clk(clk),.reset_n(reset_n),.rx(rx),.tx(uart_tx),
        .tx_data(8'd0),.tx_valid(1'b0),.tx_ready(),
        .rx_data(rx_data),.rx_valid(rx_valid),.framing_error(framing_error));
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            receive_count<=0; frame_valid<=0; sequence_id<=0;
            state<=0; pending_id<=0;
        end else begin
            frame_valid<=0;
            if(framing_error) state<=0;
            else if(rx_valid) begin
                receive_count<=receive_count+1'b1;
                case(state)
                    0: if(rx_data==8'hAA) state<=1;
                    1: begin
                        if(rx_data>=8'd1 && rx_data<=8'd6) begin
                            pending_id<=rx_data[2:0]; state<=2;
                        end else if(rx_data==8'hAA) state<=1;
                        else state<=0;
                    end
                    2: begin
                        if(rx_data==8'h55) begin
                            sequence_id<=pending_id; frame_valid<=1; state<=0;
                        end else if(rx_data==8'hAA) state<=1;
                        else state<=0;
                    end
                    default: state<=0;
                endcase
            end
        end
    end
endmodule
