`timescale 1ns/1ps
// Mode commands have priority over optional idle wiring-test echo.
// New requests coalesce to the latest mode, without interleaving command bytes.
module vision_uart #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,rx,echo_enable,
    input wire request,qr_mode,qr_data_valid,
    output wire tx,busy,
    output reg [15:0] receive_count,
    output wire [7:0] receive_data,
    output wire receive_valid,framing_error,
    output reg overflow
);
    reg pending,pending_mode,active_mode;
    reg [2:0] state;
    reg echo_pending;
    reg [7:0] echo_data;
    reg [31:0] retry_count;
    reg qr_seen;
    wire tx_ready;
    wire retry_request = qr_mode && !qr_seen && state==0 && !pending &&
        (retry_count >= (CLK_HZ*2-1));
    wire issue_request = request || retry_request;
    wire command_byte=state>=1 && state<=3;
    wire [7:0] tx_data=state==1?8'hac:state==2?(active_mode?8'h01:8'h02):state==3?8'h55:echo_data;
    wire tx_valid=command_byte || (state==0 && !pending && !request && echo_enable && echo_pending);
    assign busy=state!=0 || pending || issue_request;
    uart_8n1 #(.CLK_HZ(CLK_HZ)) uart(clk,reset_n,rx,tx,tx_data,tx_valid,tx_ready,
        receive_data,receive_valid,framing_error);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            pending<=0;pending_mode<=0;active_mode<=0;state<=0;
            echo_pending<=0;echo_data<=0;receive_count<=0;overflow<=0;
            retry_count<=0;qr_seen<=0;
        end else begin
            if(state==0 && pending && tx_ready) begin
                pending<=0;active_mode<=pending_mode;state<=1;
            end else if(command_byte && tx_ready) state<=state+1'b1;
            else if(state==4 && tx_ready) state<=0;
            if(issue_request) begin pending<=1;pending_mode<=qr_mode;retry_count<=0;qr_seen<=0;end
            else if(!qr_mode) begin retry_count<=0;qr_seen<=0;end
            else if(qr_data_valid) begin retry_count<=0;qr_seen<=1;end
            else if(retry_count < CLK_HZ*2) retry_count<=retry_count+1'b1;
            if(!echo_enable || busy || (echo_pending && tx_ready)) echo_pending<=0;
            if(receive_valid) begin
                receive_count<=receive_count+1'b1;
                if(echo_enable && !busy) begin
                    if(echo_pending && !tx_ready) overflow<=1;
                    else begin echo_data<=receive_data;echo_pending<=1;end
                end
            end
        end
    end
endmodule
