`timescale 1ns/1ps
// FPGA-owned finite motions. Angle convention: CCW from the launch body +X.
// Wheel sums remove yaw. Gyro-referenced projection measures distance along
// the requested fixed direction; velocity is rotated back to the current body.
module auto_motion #(parameter integer SERIAL_TRIG=0)(
    input wire clk,reset_n,
    input wire config_valid,input wire [5:0] config_id,
    input wire signed [31:0] config_value,
    input wire run,abort_motion,imu_fresh,imu_update,ready,
    input wire [3:0] block_reason,
    input wire signed [15:0] imu_heading,
    input wire sample_valid,input wire [63:0] deltas,
    output wire active,output reg finish,
    output reg signed [15:0] vx,vy,target_heading,
    output reg [2:0] status,output reg [3:0] reason,
    output reg [15:0] sequence_id,
    output wire [255:0] settings,
    output wire signed [31:0] progress_mm,remaining_mm,
    output reg signed [15:0] locked_heading,body_angle
);
    reg [15:0] rotate_angle,move_angle,distance_mm,speed_mm_s;
    reg [15:0] angle_tolerance,settle_ms,distance_tolerance,decel_mm;
    assign settings={16'b0,decel_mm,16'b0,distance_tolerance,16'b0,settle_ms,
        16'b0,angle_tolerance,16'b0,speed_mm_s,16'b0,distance_mm,
        16'b0,move_angle,16'b0,rotate_angle};
    assign active=status==1 || status==2;
    reg skip_sample;
    reg [15:0] stable_ms,current_speed;
    reg signed [39:0] position_q16;
    wire signed [39:0] remaining_q16=$signed({8'b0,distance_mm,16'b0})-position_q16;
    assign progress_mm=position_q16 >>> 16;
    assign remaining_mm=remaining_q16 >>> 16;
    wire [15:0] tolerance=distance_tolerance<(distance_mm>>1)?distance_tolerance:(distance_mm>>1);
    wire signed [39:0] tolerance_q16=$signed({8'b0,tolerance,16'b0});
    function signed [15:0] wrap;
        input signed [17:0] a;
        reg signed [17:0] wrapped;
        begin wrapped=a>=18000?a-18'sd36000:a< -18000?a+18'sd36000:a;wrap=wrapped[15:0];end
    endfunction
    wire signed [15:0] heading_error=wrap($signed(target_heading)-$signed({imu_heading[15],imu_heading}));
    wire [15:0] abs_heading_error=heading_error[15]?-heading_error:heading_error;
    wire signed [17:0] normalized_angle=move_angle>=18000?$signed({2'b0,move_angle})-18'sd36000:$signed({2'b0,move_angle});
    wire signed [15:0] launch_difference=wrap($signed(locked_heading)-$signed({imu_heading[15],imu_heading}));
    reg [4:0] state;
    reg signed [31:0] sum_x,sum_y;
    reg signed [15:0] sampled_difference;
    reg signed [63:0] projection_x,projection_sum;
    reg mul_start,trig_start,divide_start;
    reg signed [31:0] mul_a,mul_b,angle_q28;
    wire mul_busy,mul_valid,trig_busy,trig_valid,divide_busy,divide_valid;
    wire signed [63:0] product;
    wire signed [31:0] cosine,sine;
    reg [31:0] numerator;
    wire [31:0] quotient;
    serial_mul32 arithmetic(clk,reset_n && active,mul_start,mul_a,mul_b,mul_busy,mul_valid,product);
    cordic_sincos #(.SERIAL_SHIFT(SERIAL_TRIG)) trig(clk,reset_n && active,trig_start,angle_q28,trig_busy,trig_valid,cosine,sine);
    unsigned_divider32 divider(clk,reset_n && active,divide_start,numerator,{16'b0,decel_mm},divide_busy,divide_valid,quotient);
    wire signed [17:0] d1=$signed(deltas[15:0]),d2=$signed(deltas[31:16]),
        d3=$signed(deltas[47:32]),d4=$signed(deltas[63:48]);
    wire signed [16:0] normalized_target=rotate_angle>=18000?$signed({1'b0,rotate_angle})-17'sd36000:$signed({1'b0,rotate_angle});
    wire signed [63:0] rounded_velocity=product+64'sd536870912;
    wire signed [63:0] rounded_projection=projection_sum+64'sd131072;
    wire signed [63:0] distance_increment=product>>>14;
    wire [15:0] crawl=speed_mm_s<20?speed_mm_s:16'd20;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            rotate_angle<=0;move_angle<=0;distance_mm<=100;speed_mm_s<=100;
            angle_tolerance<=100;settle_ms<=500;distance_tolerance<=5;decel_mm<=100;
            vx<=0;vy<=0;target_heading<=0;locked_heading<=0;body_angle<=0;
            status<=0;reason<=0;sequence_id<=0;finish<=0;stable_ms<=0;current_speed<=0;
            position_q16<=0;state<=0;skip_sample<=0;sum_x<=0;sum_y<=0;sampled_difference<=0;
            projection_x<=0;projection_sum<=0;mul_start<=0;trig_start<=0;divide_start<=0;
            mul_a<=0;mul_b<=0;angle_q28<=0;numerator<=0;
        end else begin
            finish<=0;mul_start<=0;trig_start<=0;divide_start<=0;
            if(config_valid && config_id>=32 && config_id<=39) begin
                case(config_id)
                    32:rotate_angle<=config_value[15:0];33:move_angle<=config_value[15:0];
                    34:distance_mm<=config_value[15:0];35:speed_mm_s<=config_value[15:0];
                    36:angle_tolerance<=config_value[15:0];37:settle_ms<=config_value[15:0];
                    38:distance_tolerance<=config_value[15:0];39:decel_mm<=config_value[15:0];
                endcase
            end
            if(abort_motion || (active && (!run || !imu_fresh))) begin
                if(active) begin status<=4;reason<=!imu_fresh?4'd1:4'd0;finish<=1;end
                vx<=0;vy<=0;state<=0;
            end else if(config_valid && config_id==40) begin
                sequence_id<=sequence_id+1'b1;vx<=0;vy<=0;state<=0;
                position_q16<=0;stable_ms<=0;skip_sample<=1;body_angle<=0;
                if(!ready) begin status<=5;reason<=block_reason;finish<=1;end
                else begin
                    status<=config_value==1?3'd1:3'd2;reason<=0;
                    locked_heading<=imu_heading;
                    target_heading<=config_value==1?normalized_target[15:0]:imu_heading;
                end
            end else if(active) begin
                if(status==1) begin
                    vx<=0;vy<=0;
                    if(abs_heading_error>angle_tolerance) stable_ms<=0;
                    else if(sample_valid) begin
                        if(stable_ms+16'd5>=settle_ms) begin status<=3;finish<=1;end
                        else stable_ms<=stable_ms+16'd5;
                    end
                end else case(state)
                    0:if(sample_valid) begin
                        if(skip_sample) skip_sample<=0;
                        else if(remaining_q16<=tolerance_q16) begin status<=3;finish<=1;vx<=0;vy<=0;end
                        else begin
                            sum_x<=d1+d2+d3+d4;sum_y<=-d1+d2+d3-d4;
                            sampled_difference<=launch_difference;state<=17;
                        end
                    end
                    17:begin body_angle<=wrap(normalized_angle+$signed(sampled_difference));state<=1;end
                    1:begin mul_a<=body_angle;mul_b<=32'sd46851;mul_start<=1;state<=2;end
                    2:if(mul_valid) begin angle_q28<=product[31:0];trig_start<=1;state<=3;end
                    3:if(trig_valid) begin
                        if(remaining_mm>=$signed({16'b0,decel_mm})) begin current_speed<=speed_mm_s;state<=7;end
                        else state<=4;
                    end
                    4:begin mul_a<=remaining_mm>0?remaining_mm:32'sd1;mul_b<={16'b0,speed_mm_s};mul_start<=1;state<=5;end
                    5:if(mul_valid) begin numerator<=product[31:0];divide_start<=1;state<=6;end
                    6:if(divide_valid) begin current_speed<=quotient<crawl?crawl:quotient>speed_mm_s?speed_mm_s:quotient[15:0];state<=7;end
                    7:begin mul_a<={16'b0,current_speed};mul_b<=cosine;mul_start<=1;state<=8;end
                    8:if(mul_valid) begin vx<=rounded_velocity[45:30];state<=9;end
                    9:begin mul_b<=sine;mul_start<=1;state<=10;end
                    10:if(mul_valid) begin vy<=rounded_velocity[45:30];state<=11;end
                    11:begin mul_a<=sum_x;mul_b<=cosine;mul_start<=1;state<=12;end
                    12:if(mul_valid) begin projection_x<=product;state<=13;end
                    13:begin mul_a<=sum_y;mul_b<=sine;mul_start<=1;state<=14;end
                    14:if(mul_valid) begin projection_sum<=projection_x+product;state<=15;end
                    15:begin mul_a<=rounded_projection[49:18];mul_b<=32'sd10162;mul_start<=1;state<=16;end
                    16:if(mul_valid) begin position_q16<=position_q16+distance_increment[39:0];state<=0;end
                    default:state<=0;
                endcase
            end else begin vx<=0;vy<=0;state<=0;end
        end
    end
endmodule
