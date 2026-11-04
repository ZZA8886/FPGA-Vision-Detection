`timescale 1ns/1ps
// Encoder-only midpoint integration. One shared multiplier, staged arithmetic.
module mecanum_odometry #(
    parameter integer DISTANCE_PER_COUNT_Q16=10162,
    parameter integer CONTROL_HZ=200,
    parameter integer SERIAL_TRIG=0
) (
    input wire clk,reset_n,clear,sample_valid,
    input wire signed [15:0] delta1,delta2,delta3,delta4,
    output reg signed [63:0] x_mm_q16,y_mm_q16,
    output reg signed [31:0] heading_mrad_q16,
    output reg signed [31:0] vx_mm_s,vy_mm_s,yaw_mrad_s,
    output reg valid
);
    localparam signed [32:0] PI=33'sd205887416,TWO_PI=33'sd411774832;
    localparam integer H_FACTOR=(DISTANCE_PER_COUNT_Q16*1000+362)/724;
    wire signed [17:0] s1=delta1,s2=delta2,s3=delta3,s4=delta4;
    reg signed [17:0] sum_x,sum_y,sum_h;
    reg signed [31:0] dx_product,dy_product,dx,dy,dh,mid_heading;
    reg signed [32:0] heading_work,mid_work;
    reg signed [63:0] angle_product,p1,p2,p3,p4,rot_x,rot_y;
    reg signed [31:0] angle_q28;
    reg [3:0] state;
    reg trig_start;
    wire trig_busy,trig_valid;
    wire signed [31:0] cs,sn;
    cordic_sincos #(.SERIAL_SHIFT(SERIAL_TRIG)) trig(clk,reset_n && !clear,trig_start,angle_q28,trig_busy,trig_valid,cs,sn);
    reg signed [31:0] mul_a,mul_b;
    wire signed [63:0] product=mul_a*mul_b;
    always @* begin
        mul_a=0;mul_b=0;
        case(state)
            5: begin mul_a=mid_heading;mul_b=32'sd1099511628;end
            8: begin mul_a=dx;mul_b=cs;end
            9: begin mul_a=dy;mul_b=sn;end
            10:begin mul_a=dx;mul_b=sn;end
            11:begin mul_a=dy;mul_b=cs;end
            default:begin mul_a=0;mul_b=0;end
        endcase
    end
    reg signed [63:0] vx_product,vy_product,yaw_product;
    wire signed [63:0] vx_scaled=vx_product/64'sd65536,vy_scaled=vy_product/64'sd65536;
    wire signed [63:0] yaw_scaled=yaw_product/64'sd65536;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;trig_start<=0;valid<=0;
            x_mm_q16<=0;y_mm_q16<=0;heading_mrad_q16<=0;
            vx_mm_s<=0;vy_mm_s<=0;yaw_mrad_s<=0;
            sum_x<=0;sum_y<=0;sum_h<=0;dx_product<=0;dy_product<=0;
            dx<=0;dy<=0;dh<=0;mid_heading<=0;heading_work<=0;mid_work<=0;
            angle_product<=0;angle_q28<=0;p1<=0;p2<=0;p3<=0;p4<=0;rot_x<=0;rot_y<=0;
            vx_product<=0;vy_product<=0;yaw_product<=0;
        end else if(clear) begin
            state<=0;trig_start<=0;valid<=0;
            x_mm_q16<=0;y_mm_q16<=0;heading_mrad_q16<=0;
            vx_mm_s<=0;vy_mm_s<=0;yaw_mrad_s<=0;
        end else begin
            valid<=0;trig_start<=0;
            case(state)
                0:if(sample_valid) begin
                    sum_x<=s1+s2+s3+s4;sum_y<=-s1+s2+s3-s4;
                    sum_h<=-s1+s2-s3+s4;state<=1;
                end
                1:begin
                    dx_product<=sum_x*DISTANCE_PER_COUNT_Q16;
                    dy_product<=sum_y*DISTANCE_PER_COUNT_Q16;
                    dh<=sum_h*H_FACTOR;state<=2;
                end
                2:begin dx<=dx_product/32'sd4;dy<=dy_product/32'sd4;state<=3;end
                3:begin
                    heading_work<=$signed(heading_mrad_q16)+$signed({dh[31],dh});
                    mid_work<=$signed(heading_mrad_q16)+$signed({dh[31],dh})/33'sd2;
                    vx_product<=dx*CONTROL_HZ;vy_product<=dy*CONTROL_HZ;yaw_product<=dh*CONTROL_HZ;
                    state<=4;
                end
                4:begin
                    vx_mm_s<=vx_scaled[31:0];vy_mm_s<=vy_scaled[31:0];yaw_mrad_s<=yaw_scaled[31:0];
                    if(heading_work>PI) heading_work<=heading_work-TWO_PI;
                    else if(heading_work < -PI) heading_work<=heading_work+TWO_PI;
                    if(mid_work>PI) mid_work<=mid_work-TWO_PI;
                    else if(mid_work < -PI) mid_work<=mid_work+TWO_PI;
                    if(heading_work<=PI && heading_work>=-PI && mid_work<=PI && mid_work>=-PI) begin
                        heading_mrad_q16<=heading_work[31:0];mid_heading<=mid_work[31:0];state<=5;
                    end
                end
                5:begin angle_product<=product;state<=6;end
                6:begin angle_q28<=angle_product[59:28];trig_start<=1;state<=7;end
                7:if(trig_valid) state<=8;
                8:begin p1<=product;state<=9;end
                9:begin p2<=product;state<=10;end
                10:begin p3<=product;state<=11;end
                11:begin p4<=product;state<=12;end
                12:begin rot_x<=(p1-p2) >>> 30;rot_y<=(p3+p4) >>> 30;state<=13;end
                13:begin x_mm_q16<=x_mm_q16+rot_x;y_mm_q16<=y_mm_q16+rot_y;valid<=1;state<=0;end
                default:state<=0;
            endcase
        end
    end
endmodule
