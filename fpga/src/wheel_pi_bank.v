`timescale 1ns/1ps
// Four independent PI histories, one arithmetic datapath. All targets, encoder
// deltas and gains are captured together. Outputs commit together within 38
// clocks (<1 us at 50 MHz), well within the 5 ms control period.
module wheel_pi_bank (
    input wire clk,reset_n,enable,sample_valid,
    input wire [127:0] targets_q16,
    input wire [63:0] measured_deltas,
    input wire [15:0] kp_q8,ki_q8,
    output reg [47:0] efforts
);
    localparam signed [50:0] LIMIT=51'sd16777216000;
    reg [3:0] state;
    reg [1:0] wheel;
    reg [127:0] targets;
    reg [63:0] deltas;
    reg [47:0] pending;
    reg [15:0] kp,ki;
    reg signed [35:0] integral[0:3];
    reg signed [32:0] error;
    reg signed [35:0] accepted,bounded;
    reg signed [50:0] p,increment,trial_integral,trial_output,output_q24;
    wire signed [31:0] target=targets[wheel*32 +: 32];
    wire signed [15:0] delta=deltas[wheel*16 +: 16];
    wire signed [49:0] product=error*$signed({1'b0,(state==2?kp:ki)});
    wire signed [50:0] integer_output=output_q24/51'sd16777216;
    integer i;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;wheel<=0;targets<=0;deltas<=0;pending<=0;kp<=0;ki<=0;
            error<=0;accepted<=0;bounded<=0;p<=0;increment<=0;
            trial_integral<=0;trial_output<=0;output_q24<=0;efforts<=0;
            for(i=0;i<4;i=i+1) integral[i]<=0;
        end else if(!enable) begin
            state<=0;efforts<=0;
            for(i=0;i<4;i=i+1) integral[i]<=0;
        end else case(state)
            0:if(sample_valid) begin
                targets<=targets_q16;deltas<=measured_deltas;
                kp<=kp_q8;ki<=ki_q8;wheel<=0;state<=1;
            end
            1:begin
                error<=$signed({target[31],target})-$signed({delta[15],delta,16'b0});
                state<=2;
            end
            2:begin p<=product;state<=3;end
            3:begin increment<=product;state<=4;end
            4:begin trial_integral<=$signed(integral[wheel])+increment;state<=5;end
            5:begin
                if(trial_integral>LIMIT) bounded<=36'sd16777216000;
                else if(trial_integral < -LIMIT) bounded<=-36'sd16777216000;
                else bounded<=trial_integral[35:0];
                state<=6;
            end
            6:begin trial_output<=p+$signed(bounded);state<=7;end
            7:begin
                if((trial_output>LIMIT && error>0)||(trial_output < -LIMIT && error<0))
                    accepted<=integral[wheel];
                else accepted<=bounded;
                state<=8;
            end
            8:begin integral[wheel]<=accepted;output_q24<=p+$signed(accepted);state<=9;end
            9:begin
                if(output_q24>=LIMIT) pending[wheel*12 +: 12]<=12'sd1000;
                else if(output_q24 <= -LIMIT) pending[wheel*12 +: 12]<=-12'sd1000;
                else pending[wheel*12 +: 12]<=integer_output[11:0];
                if(wheel==3) state<=10;
                else begin wheel<=wheel+1'b1;state<=1;end
            end
            10:begin efforts<=pending;state<=0;end
            default:state<=0;
        endcase
    end
endmodule
