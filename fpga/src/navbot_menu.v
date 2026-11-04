`timescale 1ns/1ps
module navbot_menu (
    input wire clk,reset_n,previous,next,confirm,stop,
    input wire remote_valid,
    input wire [5:0] remote_id,
    input wire signed [31:0] remote_value,
    input wire remote_move_valid,
    input wire signed [15:0] remote_vx,remote_vy,remote_wz,
    output reg [4:0] item,
    output reg editing,arm,manual_mode,echo_enable,clear_odometry,
    output reg [1:0] test_wheel,
    output reg signed [11:0] test_pwm,
    output reg [3:0] motor_invert,encoder_invert,
    output reg [15:0] kp_q8,ki_q8,wheel_limit_mm_s,servo1_cdeg,servo2_cdeg,
    output reg signed [15:0] vx_mm_s,vy_mm_s,yaw_mrad_s,
    output reg signed [31:0] selected_value,
    output reg need_gain,
    output reg heading_enable,heading_invert,heading_zero,
    output reg signed [15:0] heading_target_cdeg,
    output reg [16:0] heading_kp,
    output reg [15:0] heading_ki,heading_kd,
    output reg [15:0] heading_limit,
    output reg remote_owned,
    output reg [1023:0] config_words
);
    reg [3:0] direction;
    reg [15:0] speed,turn,servo_step;
    reg signed [15:0] custom_vx,custom_vy,custom_yaw;
    wire up=next;
    function [15:0] bump;
        input [15:0] value,step,maximum;
        input increase;
        reg [16:0] candidate;
        begin
            if(increase) begin candidate={1'b0,value}+step;bump=candidate>maximum?maximum:candidate[15:0];end
            else bump=value<step?16'd0:value-step;
        end
    endfunction
    wire signed [15:0] adjusted_test_pwm=signed_bump({{4{test_pwm[11]}},test_pwm},16'd10,16'd1000,up);
    function signed [15:0] signed_bump;
        input signed [15:0] value;
        input [15:0] step,maximum;
        input increase;
        reg signed [17:0] candidate;
        begin
            candidate=$signed(value)+(increase?$signed({1'b0,step}):-$signed({1'b0,step}));
            if(candidate>$signed({1'b0,maximum})) signed_bump=maximum;
            else if(candidate < -$signed({1'b0,maximum})) signed_bump=-$signed({1'b0,maximum});
            else signed_bump=candidate[15:0];
        end
    endfunction
    function [16:0] heading_bump;
        input [16:0] value;
        input increase;
        reg [17:0] candidate;
        begin
            if(increase) begin
                candidate={1'b0,value}+18'd10;
                heading_bump = candidate > 18'd100000 ? 17'd100000 : candidate[16:0];
            end else heading_bump = value < 17'd10 ? 17'd0 : value - 17'd10;
        end
    endfunction
    always @* begin
        config_words=0;
        config_words[0*32 +: 32]={31'b0,arm};
        config_words[1*32 +: 32]={28'b0,direction};
        config_words[2*32 +: 32]={16'b0,speed};
        config_words[3*32 +: 32]={16'b0,turn};
        config_words[4*32 +: 32]={{16{vx_mm_s[15]}},vx_mm_s};
        config_words[5*32 +: 32]={{16{vy_mm_s[15]}},vy_mm_s};
        config_words[6*32 +: 32]={{16{yaw_mrad_s[15]}},yaw_mrad_s};
        config_words[7*32 +: 32]={16'b0,servo1_cdeg};
        config_words[8*32 +: 32]={16'b0,servo2_cdeg};
        config_words[9*32 +: 32]={16'b0,servo_step};
        config_words[10*32 +: 32]={31'b0,manual_mode};
        config_words[11*32 +: 32]={30'b0,test_wheel}+32'd1;
        config_words[12*32 +: 32]={{20{test_pwm[11]}},test_pwm};
        config_words[13*32 +: 32]={28'b0,motor_invert};
        config_words[14*32 +: 32]={28'b0,encoder_invert};
        config_words[15*32 +: 32]={16'b0,kp_q8};
        config_words[16*32 +: 32]={16'b0,ki_q8};
        config_words[17*32 +: 32]={16'b0,wheel_limit_mm_s};
        config_words[20*32 +: 32]={31'b0,echo_enable};
        config_words[23*32 +: 32]={31'b0,heading_enable};
        config_words[24*32 +: 32]={{16{heading_target_cdeg[15]}},heading_target_cdeg};
        config_words[25*32 +: 32]={15'b0,heading_kp};
        config_words[26*32 +: 32]={16'b0,heading_ki};
        config_words[27*32 +: 32]={16'b0,heading_kd};
        config_words[28*32 +: 32]={16'b0,heading_limit};
        config_words[29*32 +: 32]={31'b0,heading_invert};
    end
    always @* begin
        vx_mm_s=0;vy_mm_s=0;yaw_mrad_s=0;
        case(direction)
            0:begin vx_mm_s=custom_vx;vy_mm_s=custom_vy;yaw_mrad_s=custom_yaw;end
            1:vx_mm_s=speed;2:vx_mm_s=-$signed(speed);
            3:vy_mm_s=speed;4:vy_mm_s=-$signed(speed);
            5:begin vx_mm_s=speed;vy_mm_s=speed;end
            6:begin vx_mm_s=speed;vy_mm_s=-$signed(speed);end
            7:begin vx_mm_s=-$signed(speed);vy_mm_s=speed;end
            8:begin vx_mm_s=-$signed(speed);vy_mm_s=-$signed(speed);end
            9:yaw_mrad_s=turn;10:yaw_mrad_s=-$signed(turn);
            default:begin vx_mm_s=0;vy_mm_s=0;yaw_mrad_s=0;end
        endcase
        selected_value=0;
        case(item)
            0:selected_value={31'b0,arm};1:selected_value={28'b0,direction};
            2:selected_value={16'b0,speed};3:selected_value={16'b0,turn};
            4:selected_value=vx_mm_s;5:selected_value=vy_mm_s;6:selected_value=yaw_mrad_s;
            7:selected_value={16'b0,servo1_cdeg};8:selected_value={16'b0,servo2_cdeg};
            9:selected_value={16'b0,servo_step};10:selected_value={31'b0,manual_mode};
            11:selected_value={30'b0,test_wheel}+1;12:selected_value=test_pwm;
            13:selected_value={28'b0,motor_invert};14:selected_value={28'b0,encoder_invert};
            15:selected_value={24'b0,kp_q8[15:8]};16:selected_value={24'b0,ki_q8[15:8]};
            17:selected_value={16'b0,wheel_limit_mm_s};20:selected_value={31'b0,echo_enable};
            23:selected_value={31'b0,heading_enable};24:selected_value=heading_target_cdeg;
            25:selected_value={15'b0,heading_kp};26:selected_value={16'b0,heading_ki};
            27:selected_value={16'b0,heading_kd};28:selected_value={16'b0,heading_limit};
            29:selected_value={31'b0,heading_invert};
            default:selected_value=0;
        endcase
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            remote_owned<=0;item<=0;editing<=0;arm<=0;manual_mode<=0;echo_enable<=0;clear_odometry<=0;
            // User-calibrated forward direction after encoder connector remap:
            // motors 1/3, motor-indexed encoder feedback 1/2/3 inverted.
            test_wheel<=0;test_pwm<=0;motor_invert<=4'd5;encoder_invert<=4'd7;
            kp_q8<=16'd11008;ki_q8<=16'd256;wheel_limit_mm_s<=1201;servo1_cdeg<=9000;servo2_cdeg<=9000;
            direction<=0;speed<=300;turn<=800;servo_step<=1;
            custom_vx<=0;custom_vy<=0;custom_yaw<=0;need_gain<=0;
            heading_enable<=1;heading_invert<=0;heading_zero<=0;heading_target_cdeg<=0;
            heading_kp<=17'd10000;heading_ki<=0;heading_kd<=16'd2000;heading_limit<=800;
        end else begin
            clear_odometry<=0;heading_zero<=0;
            if(stop) begin remote_owned<=0;arm<=0;editing<=0;item<=0;need_gain<=0;end
            else if(remote_valid || remote_move_valid) begin
                // Remote commands use the same numbering as the OLED menu.
                // Parameter changes disarm first; RUN is explicit and safe.
                editing<=0;need_gain<=0;
                if(remote_valid && remote_id!=0 && remote_id!=7 && remote_id!=8 && remote_id!=9) arm<=0;
                if(remote_move_valid) begin
                    arm<=0; // MOVE updates velocity; an explicit RUN is always required.
                    direction<=0;custom_vx<=remote_vx;custom_vy<=remote_vy;custom_yaw<=remote_wz;
                end else case(remote_id)
                    0: begin
                        if(remote_value==0) begin arm<=0;remote_owned<=0;end
                        else if(manual_mode || kp_q8!=0) begin arm<=1;remote_owned<=1;end
                        else begin arm<=0;need_gain<=1;end
                    end
                    // Internal-only finite-motion launch; UART does not allow SET 31.
                    31: begin arm<=1;remote_owned<=1;direction<=0;
                        custom_vx<=0;custom_vy<=0;custom_yaw<=0;end
                    1: direction<=remote_value[3:0]>11?4'd0:remote_value[3:0];
                    2: speed<=remote_value<0?16'd0:remote_value>1000?16'd1000:remote_value[15:0];
                    3: turn<=remote_value<0?16'd0:remote_value>3000?16'd3000:remote_value[15:0];
                    4: begin direction<=0;custom_vx<=remote_value[15:0];custom_vy<=vy_mm_s;custom_yaw<=yaw_mrad_s;end
                    5: begin direction<=0;custom_vy<=remote_value[15:0];custom_vx<=vx_mm_s;custom_yaw<=yaw_mrad_s;end
                    6: begin direction<=0;custom_yaw<=remote_value[15:0];custom_vx<=vx_mm_s;custom_vy<=vy_mm_s;end
                    7: servo1_cdeg<=remote_value<0?0:remote_value>18000?18000:remote_value[15:0];
                    8: servo2_cdeg<=remote_value<0?0:remote_value>18000?18000:remote_value[15:0];
                    9: servo_step<=remote_value<1?1:remote_value>100?100:remote_value[15:0];
                    10: manual_mode<=remote_value!=0;
                    11: test_wheel<=remote_value<1?0:remote_value>4?2'd3:remote_value[1:0]-2'd1;
                    12: test_pwm<=remote_value[11:0];
                    13: begin motor_invert<=remote_value[3:0];clear_odometry<=1;end
                    14: begin encoder_invert<=remote_value[3:0];clear_odometry<=1;end
                    15: kp_q8<=remote_value<0?0:remote_value>65280?65280:remote_value[15:0];
                    16: ki_q8<=remote_value<0?0:remote_value>65280?65280:remote_value[15:0];
                    17: wheel_limit_mm_s<=remote_value<0?0:remote_value>2000?2000:remote_value[15:0];
                    18: begin arm<=0;clear_odometry<=1;end
                    20: echo_enable<=remote_value!=0;
                    23: heading_enable<=remote_value!=0;
                    24: heading_target_cdeg<=remote_value<-18000?-16'sd18000:remote_value>18000?16'sd18000:remote_value[15:0];
                    25: heading_kp<=remote_value<0?0:remote_value>100000?100000:remote_value[16:0];
                    26: heading_ki<=remote_value<0?0:remote_value>10000?10000:remote_value[15:0];
                    27: heading_kd<=remote_value<0?0:remote_value>10000?10000:remote_value[15:0];
                    28: heading_limit<=remote_value<0?0:remote_value>3000?3000:remote_value[15:0];
                    29: heading_invert<=remote_value!=0;
                    30: begin arm<=0;heading_zero<=1;heading_target_cdeg<=0;end
                    default: begin end
                endcase
            end
            else if(confirm) begin
                if(item==0) remote_owned<=0;
                if(item==0) begin
                    if(arm) arm<=0;
                    else if(manual_mode || kp_q8!=0) begin arm<=1;need_gain<=0;end
                    else need_gain<=1;
                end else if(item==18) begin arm<=0;clear_odometry<=1;end
                else if(item==30) begin arm<=0;heading_zero<=1;heading_target_cdeg<=0;end
                else if(item!=19 && item!=21 && item!=22) begin
                    editing<=!editing;
                    if(item!=7 && item!=8 && item!=9) arm<=0;
                end
            end else if(previous || next) begin
                need_gain<=0;
                if(!editing) begin
                    if(next) item<=item==30?5'd0:item+5'd1;
                    else item<=item==0?5'd30:item-5'd1;
                end else case(item)
                    1:if(up) direction<=direction==11?4'd0:direction+4'd1;
                      else direction<=direction==0?4'd11:direction-4'd1;
                    2:speed<=bump(speed,10,1000,up);
                    3:turn<=bump(turn,50,3000,up);
                    4,5,6:begin
                        direction<=0;custom_vx<=vx_mm_s;custom_vy<=vy_mm_s;custom_yaw<=yaw_mrad_s;
                        if(item==4) custom_vx<=signed_bump(vx_mm_s,10,1000,up);
                        if(item==5) custom_vy<=signed_bump(vy_mm_s,10,1000,up);
                        if(item==6) custom_yaw<=signed_bump(yaw_mrad_s,50,3000,up);
                    end
                    7:servo1_cdeg<=bump(servo1_cdeg,servo_step,18000,up);
                    8:servo2_cdeg<=bump(servo2_cdeg,servo_step,18000,up);
                    9:if(up) servo_step<=servo_step==1?10:servo_step==10?100:1;
                      else servo_step<=servo_step==1?100:servo_step==100?10:1;
                    10:manual_mode<=!manual_mode;
                    11:if(up) test_wheel<=test_wheel+1'b1;else test_wheel<=test_wheel-1'b1;
                    12:test_pwm<=adjusted_test_pwm[11:0];
                    13:begin
                        motor_invert<=up?motor_invert+1'b1:motor_invert-1'b1;clear_odometry<=1;
                    end
                    14:begin
                        encoder_invert<=up?encoder_invert+1'b1:encoder_invert-1'b1;clear_odometry<=1;
                    end
                    15:kp_q8<=bump(kp_q8,256,65280,up);
                    16:ki_q8<=bump(ki_q8,256,65280,up);
                    17:wheel_limit_mm_s<=bump(wheel_limit_mm_s,10,2000,up);
                    20:echo_enable<=!echo_enable;
                    23:heading_enable<=!heading_enable;
                    24:heading_target_cdeg<=signed_bump(heading_target_cdeg,100,18000,up);
                    25:heading_kp<=heading_bump(heading_kp,up);
                    26:heading_ki<=bump(heading_ki,16'd10,16'd10000,up);
                    27:heading_kd<=bump(heading_kd,16'd10,16'd10000,up);
                    28:heading_limit<=bump(heading_limit,50,3000,up);
                    29:heading_invert<=!heading_invert;
                    default:editing<=0;
                endcase
            end
        end
    end
endmodule
