`timescale 1ns/1ps
// 10 Hz outer heading loop. cdeg inputs, gains in hundredths:
// Kp [mrad/s / degree], Ki [mrad/s / (degree*s)], Kd [mrad/s / (degree/s)].
// Three bit-serial multiplies share add/shift logic, no DSP consumption.
// Derivative on measurement avoids setpoint kick. Error uses shortest arc.
module heading_pid (
    input wire clk,reset_n,enable,sample_valid,
    input wire signed [15:0] measured_cdeg,target_cdeg,
    input wire [16:0] kp_centi,
    input wire [15:0] ki_centi,kd_centi,
    input wire [15:0] limit_mrad_s,
    output reg signed [15:0] command_mrad_s,
    output wire signed [15:0] error_cdeg
);
    function signed [15:0] wrap;
        input signed [16:0] value;
        reg signed [16:0] wrapped;
        begin
            wrapped=value>=18000 ? value-17'sd36000 : value< -18000 ? value+17'sd36000 : value;
            wrap=wrapped[15:0];
        end
        endfunction
    assign error_cdeg=wrap($signed({target_cdeg[15],target_cdeg})-$signed({measured_cdeg[15],measured_cdeg}));
    reg [3:0] state;
    reg [1:0] term;
    reg [4:0] bit_count;
    reg [31:0] multiplicand,product;
    reg [16:0] multiplier;
    reg negative,history;
    reg signed [15:0] previous,delta,error_saved;
    reg [16:0] kp;
    reg [15:0] ki,kd,limit;
    reg signed [31:0] p,i,integral,trial,d,total;
    wire signed [15:0] operand=term==2?delta:error_saved;
    wire [16:0] gain=term==0?kp:{1'b0,(term==1?ki:kd)};
    wire [31:0] magnitude=operand[15]?-{{16{operand[15]}},operand}:{{16{operand[15]}},operand};
    wire [31:0] next_product=product+(multiplier[0]?multiplicand:32'd0);
    wire [31:0] wide_limit={16'b0,limit};
    wire signed [31:0] integral_bound=$signed((wide_limit<<16)+(wide_limit<<15)+
        (wide_limit<<10)+(wide_limit<<9)+(wide_limit<<7)+(wide_limit<<5));
    reg divide_start;
    reg [31:0] numerator,denominator;
    wire divide_busy,divide_done;
    wire [31:0] quotient;
    wire signed [31:0] result=negative?-$signed(quotient):$signed(quotient);
    unsigned_divider32 divider(clk,reset_n && enable,divide_start,numerator,denominator,divide_busy,divide_done,quotient);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;term<=0;bit_count<=0;multiplicand<=0;product<=0;multiplier<=0;negative<=0;
            history<=0;previous<=0;delta<=0;error_saved<=0;kp<=0;ki<=0;kd<=0;limit<=0;
            p<=0;i<=0;integral<=0;trial<=0;d<=0;total<=0;command_mrad_s<=0;
            divide_start<=0;numerator<=0;denominator<=1;
        end else if(!enable) begin
            state<=0;integral<=0;history<=0;command_mrad_s<=0;divide_start<=0;
        end else begin
          divide_start<=0;
          case(state)
            0:if(sample_valid) begin
                error_saved<=error_cdeg;
                delta<=history?wrap($signed({measured_cdeg[15],measured_cdeg})-$signed({previous[15],previous})):16'sd0;
                previous<=measured_cdeg;history<=1;
                kp<=kp_centi;ki<=ki_centi;kd<=kd_centi;limit<=limit_mrad_s;
                term<=0;state<=1;
            end
            1:begin
                multiplicand<=magnitude;multiplier<=gain;negative<=operand[15];
                product<=0;bit_count<=0;state<=2;
            end
            2:begin
                product<=next_product;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
                if(bit_count==16) state<=3;else bit_count<=bit_count+1'b1;
            end
            3:begin
                numerator<=product;denominator<=term==0?32'd10000:term==1?32'd1:32'd1000;
                divide_start<=1;state<=7;
            end
            7:if(divide_done) begin
                if(term==0) p<=result;
                else if(term==1) i<=result;
                else d<=-result;
                if(term==2) state<=4;else begin term<=term+1'b1;state<=1;end
            end
            4:begin
                if(ki==0) trial<=0;
                else if(integral+i>integral_bound) trial<=integral_bound;
                else if(integral+i < -integral_bound) trial<=-integral_bound;
                else trial<=integral+i;
                state<=5;
            end
            5:begin
                numerator<=trial[31]?-trial:trial;negative<=trial[31];denominator<=100000;
                divide_start<=1;state<=8;
            end
            8:if(divide_done) begin total<=p+result+d;state<=6;end
            6:begin
                if(!((total>$signed({16'b0,limit}) && error_saved>0) ||
                     (total< -$signed({16'b0,limit}) && error_saved<0))) integral<=trial;
                if(total>$signed({16'b0,limit})) command_mrad_s<=limit;
                else if(total< -$signed({16'b0,limit})) command_mrad_s<=-$signed(limit);
                else command_mrad_s<=total[15:0];
                state<=0;
            end
            default:state<=0;
          endcase
        end
    end
endmodule
