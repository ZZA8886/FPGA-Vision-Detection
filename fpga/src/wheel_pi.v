`timescale 1ns/1ps
// Q16.16 counts/sample error; unsigned Q8.8 gains; PWM -1000..1000.
// Shared P/I multiplier; bounded integral and conditional anti-windup.
module wheel_pi (
    input wire clk,reset_n,enable,sample_valid,
    input wire signed [31:0] target_q16,
    input wire signed [15:0] measured_delta,
    input wire [15:0] kp_q8,ki_q8,
    output reg signed [11:0] effort
);
    localparam signed [50:0] LIMIT=51'sd16777216000;
    reg [3:0] state;
    reg signed [32:0] error;
    reg [15:0] kp,ki;
    reg signed [35:0] integral,accepted,bounded;
    reg signed [50:0] p,increment,trial_integral,trial_output,output_q24;
    wire signed [49:0] product=error*$signed({1'b0,(state==1?kp:ki)});
    wire signed [50:0] integer_output=output_q24/51'sd16777216;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;error<=0;kp<=0;ki<=0;integral<=0;accepted<=0;bounded<=0;
            p<=0;increment<=0;trial_integral<=0;trial_output<=0;output_q24<=0;effort<=0;
        end else if(!enable) begin state<=0;integral<=0;effort<=0;end
        else case(state)
            0:if(sample_valid) begin
                error<=$signed({target_q16[31],target_q16})-$signed({measured_delta[15],measured_delta,16'b0});
                kp<=kp_q8;ki<=ki_q8;state<=1;
            end
            1:begin p<=product;state<=2;end
            2:begin increment<=product;state<=3;end
            3:begin trial_integral<=$signed(integral)+increment;state<=4;end
            4:begin
                if(trial_integral>LIMIT) bounded<=36'sd16777216000;
                else if(trial_integral < -LIMIT) bounded<=-36'sd16777216000;
                else bounded<=trial_integral[35:0];
                state<=5;
            end
            5:begin trial_output<=p+$signed(bounded);state<=6;end
            6:begin
                if((trial_output>LIMIT && error>0)||(trial_output < -LIMIT && error<0)) accepted<=integral;
                else accepted<=bounded;
                state<=7;
            end
            7:begin integral<=accepted;output_q24<=p+$signed(accepted);state<=8;end
            8:begin
                if(output_q24>=LIMIT) effort<=1000;
                else if(output_q24 <= -LIMIT) effort<=-1000;
                else effort<=integer_output[11:0];
                state<=0;
            end
            default:state<=0;
        endcase
    end
endmodule
