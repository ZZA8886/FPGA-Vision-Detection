`timescale 1ns/1ps
// Iterative 24-step CORDIC. SERIAL_SHIFT trades latency for smaller routing:
// 324 clocks instead of 24, far below a 5 ms control period at 50 MHz.
// Input radians Q3.28, normalized to [-pi, pi].
// Output sine/cosine Q2.30. start is accepted only when !busy.
module cordic_sincos #(parameter integer SERIAL_SHIFT=0)(
    input wire clk,
    input wire reset_n,
    input wire start,
    input wire signed [31:0] angle_q28,
    output reg busy,
    output reg valid,
    output reg signed [31:0] cosine_q30,
    output reg signed [31:0] sine_q30
);
    reg signed [31:0] x,y,z;
    reg [4:0] iteration;
    reg negate_result;
    reg signed [31:0] shift_x,shift_y;
    reg [4:0] shift_remaining;
    reg [1:0] shift_phase;
    function signed [31:0] atan_q28;
        input [4:0] index;
        begin
            case(index)
                0: atan_q28=210828714; 1: atan_q28=124459457;
                2: atan_q28=65760959; 3: atan_q28=33381290;
                4: atan_q28=16755422; 5: atan_q28=8385879;
                6: atan_q28=4193963; 7: atan_q28=2097109;
                8: atan_q28=1048571; 9: atan_q28=524287;
                10: atan_q28=262144; 11: atan_q28=131072;
                12: atan_q28=65536; 13: atan_q28=32768;
                14: atan_q28=16384; 15: atan_q28=8192;
                16: atan_q28=4096; 17: atan_q28=2048;
                18: atan_q28=1024; 19: atan_q28=512;
                20: atan_q28=256; 21: atan_q28=128;
                22: atan_q28=64; default: atan_q28=32;
            endcase
        end
    endfunction
    wire signed [31:0] x_operand=SERIAL_SHIFT?shift_x:(x >>> iteration);
    wire signed [31:0] y_operand=SERIAL_SHIFT?shift_y:(y >>> iteration);
    wire signed [31:0] next_x = z >= 0 ? x-y_operand : x+y_operand;
    wire signed [31:0] next_y = z >= 0 ? y+x_operand : y-x_operand;
    wire signed [31:0] next_z = z >= 0 ? z-atan_q28(iteration) : z+atan_q28(iteration);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            busy<=0;valid<=0;x<=0;y<=0;z<=0;iteration<=0;
            negate_result<=0;cosine_q30<=1073741824;sine_q30<=0;
            shift_x<=0;shift_y<=0;shift_remaining<=0;shift_phase<=0;
        end else begin
            valid<=0;
            if(start && !busy) begin
                x<=652032874;y<=0;iteration<=0;busy<=1;
                shift_phase<=0;
                if(angle_q28>32'sd421657428) begin
                    z<=angle_q28-32'sd843314857;negate_result<=1;
                end else if(angle_q28 < -32'sd421657428) begin
                    z<=angle_q28+32'sd843314857;negate_result<=1;
                end else begin z<=angle_q28;negate_result<=0;end
            end else if(busy && SERIAL_SHIFT && shift_phase==0) begin
                shift_x<=x;shift_y<=y;shift_remaining<=iteration;
                shift_phase<=iteration==0?2'd2:2'd1;
            end else if(busy && SERIAL_SHIFT && shift_phase==1) begin
                shift_x<=shift_x >>> 1;shift_y<=shift_y >>> 1;
                shift_remaining<=shift_remaining-1'b1;
                if(shift_remaining==1) shift_phase<=2;
            end else if(busy) begin
                x<=next_x;y<=next_y;z<=next_z;
                shift_phase<=0;
                if(iteration==23) begin
                    busy<=0;valid<=1;
                    cosine_q30<=negate_result ? -next_x : next_x;
                    sine_q30<=negate_result ? -next_y : next_y;
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule
