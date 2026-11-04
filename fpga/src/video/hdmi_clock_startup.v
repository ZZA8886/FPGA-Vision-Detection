`timescale 1ns/1ps
// Clock-stopped reset release for the common pixel/serializer reset in the
// supplied DVI IP. UG306 DHCE contains four falling-edge enable stages.
// Each guard below is 160 ns at 50 MHz. SDC bounds reset/CEN delivery to
// 40 ns, leaving >100 ns before clocks can run (750 MHz DHCE latency <6 ns).
module hdmi_clock_startup(
    input wire clk,
    input wire ready,
    output reg clock_disable=1'b1,
    output reg ip_reset_n=1'b0
);
    reg [3:0] guard_count=0;
    reg released=0;
    always @(posedge clk or negedge ready) begin
        if(!ready) begin
            clock_disable<=1'b1;
            ip_reset_n<=1'b0;
            guard_count<=0;
            released<=0;
        end else if(!released) begin
            if(guard_count==8) begin
                ip_reset_n<=1'b1;
                guard_count<=0;
                released<=1;
            end else guard_count<=guard_count+1'b1;
        end else if(clock_disable) begin
            if(guard_count==7) clock_disable<=1'b0;
            else guard_count<=guard_count+1'b1;
        end
    end
endmodule
