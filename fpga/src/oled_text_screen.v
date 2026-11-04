`timescale 1ns/1ps
module oled_text_screen (
    input wire clk,reset_n,start,
    input wire [4:0] item,
    input wire editing,arm,manual_mode,need_gain,
    input wire signed [31:0] selected_value,x_mm,y_mm,heading_mrad,
    input wire [63:0] encoder_counts,
    input wire [47:0] uart_counts,
    input wire [2:0] page,
    input wire [6:0] column,
    output wire [7:0] pixels,
    output reg done,
    input wire [127:0] target_rpm_centi,actual_rpm_centi,
    input wire [47:0] pwm_commands,
    input wire [1:0] selected_wheel,
    input wire signed [15:0] imu_heading_cdeg,heading_target_cdeg,heading_error_cdeg,heading_command,
    input wire imu_fresh,heading_enable,heading_block
);
    (* syn_ramstyle="block_ram" *) reg [7:0] text[0:255];
    reg [4:0] saved_item;
    reg saved_editing,saved_arm,saved_manual,saved_need_gain;
    reg signed [31:0] saved_value,saved_x,saved_y,saved_heading;
    reg [63:0] saved_enc;
    reg [47:0] saved_uart;
    reg signed [15:0] saved_imu_heading,saved_heading_target,saved_heading_error,saved_heading_command;
    reg saved_imu_fresh,saved_heading_enable,saved_heading_block;
    reg [127:0] saved_target_rpm,saved_actual_rpm;
    reg [47:0] saved_pwm;
    reg [1:0] saved_selected_wheel,monitor_wheel,monitor_kind;
    reg monitor_start;
    reg [2:0] monitor_char;
    wire signed [31:0] monitor_number=monitor_kind==0 ? saved_target_rpm[monitor_wheel*32 +: 32] :
        monitor_kind==1 ? saved_actual_rpm[monitor_wheel*32 +: 32] :
        {{20{saved_pwm[monitor_wheel*12+11]}},saved_pwm[monitor_wheel*12 +: 12]};
    wire [103:0] monitor_ascii;
    wire [3:0] monitor_length;
    wire monitor_done;
    signed_decimal monitor_format(clk,reset_n,monitor_start,monitor_number,monitor_kind!=2,
        monitor_ascii,monitor_length,monitor_done);
    wire [3:0] monitor_width=monitor_kind==2?4'd6:4'd7;
    wire [3:0] monitor_padding=monitor_length<=monitor_width?monitor_width-monitor_length:4'd0;
    wire [3:0] monitor_ascii_index={1'b0,monitor_char}-monitor_padding;
    wire [7:0] monitor_byte=monitor_length>monitor_width ? 8'd35 :
        ({1'b0,monitor_char}<monitor_padding ? 8'd32 : monitor_ascii[monitor_ascii_index*8 +: 8]);
    wire [7:0] monitor_address=(8'd3+{6'b0,monitor_wheel})*8'd21+8'd1+
        {6'b0,monitor_kind}*8'd7+{5'b0,monitor_char};
    reg [3:0] state;
    reg [7:0] cursor;
    reg [1:0] field;
    reg [31:0] binary;
    reg [39:0] bcd,bcd_adjusted;
    reg [5:0] bit_count;
    reg [3:0] digit_index,first_digit;
    reg [2:0] number_row;
    reg negative,fractional,leading;
    reg signed [31:0] number;
    integer j;
    function [167:0] label;
        input [4:0] index;
        begin case(index)
                0:label="RUN 0:STOP 1:GO      ";
                1:label="DIR 0:CUSTOM 1:FRONT ";
                2:label="SPEED MM/S           ";
                3:label="TURN MRAD/S          ";
                4:label="VX MM/S              ";
                5:label="VY MM/S              ";
                6:label="YAW MRAD/S           ";
                7:label="SERVO1 DEG           ";
                8:label="SERVO2 DEG           ";
                9:label="SERVO STEP 0.01 DEG  ";
                10:label="MODE 0:BODY 1:TEST   ";
                11:label="TEST WHEEL 1..4      ";
                12:label="TEST PWM -1000..1000 ";
                13:label="MOTOR POL BITS 0..3  ";
                14:label="ENC POL BITS 0..3    ";
                15:label="KP PWM/COUNT         ";
                16:label="KI PWM/COUNT/TICK    ";
                17:label="WHEEL LIMIT MM/S     ";
                18:label="CLEAR ODOM: PRESS OK ";
                19:label="ODOMETRY STATUS      ";
                20:label="UART ECHO 0:OFF 1:ON ";
                22:label="IMU660RA YAW STATUS ";
                23:label="YAW LOOP 0:OFF 1:ON ";
                24:label="TARGET HEADING DEG  ";
                25:label="YAW KP              ";
                26:label="YAW KI              ";
                27:label="YAW KD              ";
                28:label="YAW LIMIT MRAD/S    ";
                29:label="IMU INVERT 0:NO 1:YES";
                30:label="ZERO YAW: PRESS OK  ";
            default:label="                     ";
        endcase end
    endfunction
    function [7:0] hex;
        input [3:0] nibble;
        begin hex=nibble<10 ? 8'd48+{4'b0,nibble} : 8'd55+{4'b0,nibble};end
    endfunction
    reg [167:0] row_text;
    reg [167:0] shifted_text;
    reg [7:0] template_character;
    reg [2:0] row;
    reg [4:0] col;
    reg [2:0] template_row;
    reg [4:0] template_col;
    always @* begin
        row=template_row;col=template_col;
        case(row)
            0:if(saved_heading_block) row_text="CHECK IMU AND YAW KP ";
              else if(saved_need_gain) row_text="SET KP BEFORE RUN    ";
              else if(saved_arm) row_text="NAVBOT RUN           ";
              else row_text="NAVBOT STOP          ";
            1:row_text=saved_editing?"ITEM 00/30 EDIT      ":"ITEM 00/30 SELECT    ";
            2:row_text=saved_item==19?"X MM                 ":label(saved_item);
            3:row_text=saved_item==19?"Y MM                 ":"VALUE                ";
            4:if(saved_item==19) row_text="YAW MRAD             ";
              else row_text=saved_manual?"MODE: SINGLE MOTOR   ":"MODE: CHASSIS        ";
            5:row_text="E1 0000 E2 0000      ";
            6:row_text="E3 0000 E4 0000      ";
            default:row_text=saved_item==19?"U 0000 0000 0000     ":"OK:EDIT HOLD:STOP    ";
        endcase
        if(saved_item==21) begin
            case(row)
                2:row_text="W T:RPM  A:RPM   PWM ";
                3:row_text="1                    ";
                4:row_text="2                    ";
                5:row_text="3                    ";
                6:row_text="4                    ";
                7:row_text="UART W1 HOLD:STOP    ";
                default:begin end
            endcase
        end
        if(saved_item==22) begin
            case(row)
                2:row_text="ACT                  ";
                3:row_text="TGT                  ";
                4:row_text="ERR                  ";
                5:row_text="WZ                   ";
                6:row_text=saved_imu_fresh?"IMU OK 10HZ          ":"IMU WAIT / TIMEOUT   ";
                7:row_text=saved_heading_enable?"YAW ON HOLD:STOP     ":"YAW OFF HOLD:STOP    ";
                default:begin end
            endcase
        end
        shifted_text=row_text >> ((20-col)*8);
        template_character=shifted_text[7:0];
        if(row==1 && col==5) template_character=8'd48+saved_item/10;
        if(row==1 && col==6) template_character=8'd48+saved_item%10;
        if(saved_item!=21 && saved_item!=22) begin
            if(row==5 && col>=3 && col<=6) template_character=hex(saved_enc >> ((6-col)*4));
            if(row==5 && col>=11 && col<=14) template_character=hex(saved_enc >> (16+(14-col)*4));
            if(row==6 && col>=3 && col<=6) template_character=hex(saved_enc >> (32+(6-col)*4));
            if(row==6 && col>=11 && col<=14) template_character=hex(saved_enc >> (48+(14-col)*4));
        end
        if(saved_item==21 && row==7 && col==6) template_character=8'd49+{6'b0,saved_selected_wheel};
        if(row==7 && saved_item==19) begin
            if(col>=2 && col<=5) template_character=hex(saved_uart >> ((5-col)*4));
            if(col>=7 && col<=10) template_character=hex(saved_uart >> (16+(10-col)*4));
            if(col>=12 && col<=15) template_character=hex(saved_uart >> (32+(15-col)*4));
        end
        if(saved_item==19) begin
            case(field)
                0:number=saved_x;1:number=saved_y;default:number=saved_heading;
            endcase
        end else if(saved_item==22) begin
            case(field)
                0:number=saved_imu_heading;1:number=saved_heading_target;
                2:number=saved_heading_error;default:number=saved_heading_command;
            endcase
        end else number=saved_value;
        bcd_adjusted=bcd;
        for(j=0;j<10;j=j+1)
            if(bcd[j*4 +: 4]>=5) bcd_adjusted[j*4 +: 4]=bcd[j*4 +: 4]+4'd3;
    end
    // A single write port and synchronous read infer block RAM rather than
    // thousands of flip-flops/muxes. SSD1306 waits one clock after address changes.
    reg text_write;
    reg [7:0] text_address,text_data,character;
    wire [8:0] sign_address={6'b0,number_row}*9'd21+9'd6;
    wire [8:0] decimal_address=sign_address+9'd4;
    always @* begin
        text_write=0;text_address=0;text_data=0;
        case(state)
            1:begin text_write=1;text_address=cursor;text_data=template_character;end
            4:begin text_write=1;text_address=sign_address[7:0];text_data=negative?8'd45:8'd43;end
            9:begin text_write=1;text_address=decimal_address[7:0];text_data=8'd46;end
            5:begin text_write=1;text_address=number_address;
                text_data=leading && digit==0 && digit_index>(fractional?2:0)?8'd32:8'd48+{4'b0,digit};end
            8:begin text_write=1;text_address=monitor_address;text_data=monitor_byte;end
        endcase
    end
    always @(posedge clk) begin
        if(text_write) text[text_address]<=text_data;
        character<=column<126 ? text[page*21+column/6] : 8'd32;
    end
    wire [2:0] glyph_column=column%6;
    font5x7 font(character,glyph_column,pixels);
    wire [39:0] shifted_bcd=bcd >> (digit_index*4);
    wire [3:0] digit=shifted_bcd[3:0];
    wire [15:0] number_address_full=number_row*16'd21+16'd7+{12'b0,first_digit}-{12'b0,digit_index}
        +((fractional && digit_index<2)?16'd1:16'd0);
    wire [7:0] number_address=number_address_full[7:0];
    wire decimal_field=saved_item==7 || saved_item==8 || (saved_item>=24 && saved_item<=27) ||
        (saved_item==22 && field<3);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;done<=0;cursor<=0;field<=0;binary<=0;bcd<=0;bit_count<=0;
            template_row<=0;template_col<=0;
            digit_index<=0;first_digit<=0;number_row<=0;negative<=0;fractional<=0;leading<=0;
            saved_item<=0;saved_editing<=0;saved_arm<=0;saved_manual<=0;saved_need_gain<=0;
            saved_value<=0;saved_x<=0;saved_y<=0;saved_heading<=0;saved_enc<=0;saved_uart<=0;
            saved_imu_heading<=0;saved_heading_target<=0;saved_heading_error<=0;saved_heading_command<=0;
            saved_imu_fresh<=0;saved_heading_enable<=0;saved_heading_block<=0;
            saved_target_rpm<=0;saved_actual_rpm<=0;saved_pwm<=0;saved_selected_wheel<=0;
            monitor_wheel<=0;monitor_kind<=0;monitor_start<=0;monitor_char<=0;
        end else begin
            done<=0;monitor_start<=0;
            case(state)
                0:if(start) begin
                    saved_item<=item;saved_editing<=editing;saved_arm<=arm;saved_manual<=manual_mode;
                    saved_need_gain<=need_gain;saved_value<=selected_value;
                    saved_x<=x_mm;saved_y<=y_mm;saved_heading<=heading_mrad;
                    saved_enc<=encoder_counts;saved_uart<=uart_counts;
                    saved_imu_heading<=imu_heading_cdeg;saved_heading_target<=heading_target_cdeg;
                    saved_heading_error<=heading_error_cdeg;saved_heading_command<=heading_command;
                    saved_imu_fresh<=imu_fresh;saved_heading_enable<=heading_enable;saved_heading_block<=heading_block;
                    saved_target_rpm<=target_rpm_centi;saved_actual_rpm<=actual_rpm_centi;
                    saved_pwm<=pwm_commands;saved_selected_wheel<=selected_wheel;
                    monitor_wheel<=0;monitor_kind<=0;
                    cursor<=0;template_row<=0;template_col<=0;field<=0;state<=1;
                end
                1:begin
                    if(template_col==20) begin template_col<=0;template_row<=template_row+1'b1;end
                    else template_col<=template_col+1'b1;
                    if(cursor==167) state<=saved_item==21?4'd6:4'd2;else cursor<=cursor+1'b1;
                end
                2:begin
                    binary<=number[31]?-number:number;negative<=number[31];
                    bcd<=0;bit_count<=0;leading<=1;
                    fractional<=decimal_field;
                    first_digit<=decimal_field?4:9;
                    digit_index<=decimal_field?4:9;
                    number_row<=(saved_item==19 || saved_item==22) ? 3'd2+field : 3'd3;
                    state<=3;
                end
                3:begin
                    bcd<={bcd_adjusted[38:0],binary[31]};binary<=binary<<1;
                    if(bit_count==31) state<=4;else bit_count<=bit_count+1'b1;
                end
                4:begin
                    state<=fractional?4'd9:4'd5;
                end
                9:state<=5;
                5:begin
                    if(!(leading && digit==0 && digit_index>(fractional?2:0))) leading<=0;
                    if(digit_index==0) begin
                        if((saved_item==19 && field<2) || (saved_item==22 && field<3)) begin field<=field+1'b1;state<=2;end
                        else begin done<=1;state<=0;end
                    end else digit_index<=digit_index-1'b1;
                end
                6:begin monitor_start<=1;state<=7;end
                7:if(monitor_done) begin monitor_char<=0;state<=8;end
                8:begin
                    if({1'b0,monitor_char}==monitor_width-1'b1) begin
                        if(monitor_kind==2) begin
                            monitor_kind<=0;
                            if(monitor_wheel==3) begin done<=1;state<=0;end
                            else begin monitor_wheel<=monitor_wheel+1'b1;state<=6;end
                        end else begin monitor_kind<=monitor_kind+1'b1;state<=6;end
                    end else monitor_char<=monitor_char+1'b1;
                end
                default:state<=0;
            endcase
        end
    end
endmodule
