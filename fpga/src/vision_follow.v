`timescale 1ns/1ps
// Raspberry Pi 640x480 centres, 14 binary bytes, big endian:
// AA x0[15:8] x0[7:0] y0[15:8] y0[7:0] 00 00
//    x1[15:8] x1[7:0] y1[15:8] y1[7:0] 00 01 55
// A sliding window tolerates inserted/dropped bytes and AA/55 in coordinates.
// No checksum is present in the specified protocol. Only complete, in-range
// frames refresh age. (0,0) means absent; points on either single axis are valid.
module vision_follow #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,
    input wire [7:0] rx_data,input wire rx_valid,framing_error,
    input wire config_valid,input wire [5:0] config_id,
    input wire signed [31:0] config_value,
    input wire cancel,ready,
    input wire signed [15:0] imu_heading,
    output reg enabled,
    output wire fresh,tracking,centered,selected_class,
    output wire present0,present1,
    output reg [9:0] x0,x1,output reg [8:0] y0,y1,
    output reg signed [15:0] vx,vy,locked_heading,
    output wire [31:0] tuning_word,mode_word,
    output reg frame_valid,
    input wire mode_busy,
    input wire auto_enable,
    input wire task_active,
    input wire rearm_qr,
    output reg qr_done,
    output reg [2:0] task_id,
    output reg qr_complete
);
    localparam integer MS_TICKS=CLK_HZ/1000;
    localparam integer MS_BITS=$clog2(MS_TICKS)>0?$clog2(MS_TICKS):1;
    reg [MS_BITS-1:0] ms_count;
    wire ms_tick=ms_count==MS_TICKS-1;
    reg [15:0] age_ms,timeout_ms;
    reg [15:0] center_ms;
    reg [3:0] byte_age;
    reg received;
    reg qr_mode,qr_present,await_idle,lock_pending;
    reg [103:0] window;
    reg [6:0] tolerance;
    reg [7:0] gain_q4;
    reg [9:0] speed_limit;
    assign tuning_word={7'd0,speed_limit,gain_q4,tolerance};
    // bit23 advertises QR support; bit17 selects QR (0 = objects).
    assign mode_word={8'h56,1'b1,4'd0,auto_enable,qr_mode,enabled,timeout_ms};
    assign fresh=received && age_ms<timeout_ms;
    assign present0=qr_mode?qr_present:(x0!=0 || y0!=0);
    assign present1=!qr_mode && (x1!=0 || y1!=0);
    assign selected_class=!present0;
    wire [9:0] target_x=present0?x0:x1;
    wire [8:0] target_y=present0?y0:y1;
    wire signed [10:0] error_x=11'sd320-$signed({1'b0,target_x});
    wire signed [10:0] error_y=11'sd240-$signed({2'b0,target_y});
    wire [9:0] abs_x=error_x[10]?-error_x:error_x;
    wire [9:0] abs_y=error_y[10]?-error_y:error_y;
    wire qr_outside_center=abs_x>{3'd0,tolerance} || abs_y>{3'd0,tolerance};
    assign tracking=enabled && fresh && (present0 || present1) && !(qr_mode && qr_done);
    assign centered=tracking && abs_x<={3'd0,tolerance} && abs_y<={3'd0,tolerance};
    wire object_ok=window[103:96]==8'haa && rx_data==8'h55 &&
        window[63:48]==16'd0 && window[15:0]==16'd1 &&
        window[95:80]<16'd640 && window[79:64]<16'd480 &&
        window[47:32]<16'd640 && window[31:16]<16'd480;
    wire qr_ok=window[47:40]==8'haa && rx_data==8'h55 &&
        window[39:24]<16'd640 && window[23:8]<16'd480 &&
        (window[7:0]>=1 && window[7:0]<=4 || window[39:0]==0);
    wire packet_ok=qr_mode?qr_ok:object_ok;
    wire accept_packet=rx_valid && !framing_error && !mode_busy && !await_idle && packet_ok && byte_age<10;
    // One small serial multiplier is shared by the two proportional axes.
    // Gains have four fractional bits: 16 = 1 mm/s per pixel.
    reg [1:0] state;
    reg axis,negative;
    reg [2:0] bit_count;
    reg [7:0] multiplier;
    reg [17:0] multiplicand,product;
    reg signed [10:0] saved_x,saved_y;
    reg [7:0] saved_gain;
    reg [9:0] saved_limit;
    reg signed [15:0] pending_vx;
    wire signed [10:0] operand=axis?saved_x:saved_y;
    wire [17:0] sum=product+(multiplier[0]?multiplicand:18'd0);
    wire [13:0] magnitude=product[17:4];
    wire [15:0] clamped=magnitude>{4'd0,saved_limit}?{6'd0,saved_limit}:{2'd0,magnitude};
    wire signed [15:0] result=negative?-$signed(clamped):$signed(clamped);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            ms_count<=0;age_ms<=16'hffff;center_ms<=0;byte_age<=15;received<=0;window<=0;
            timeout_ms<=300;tolerance<=5;gain_q4<=16;speed_limit<=150;
            enabled<=0;x0<=0;y0<=0;x1<=0;y1<=0;locked_heading<=0;
            qr_mode<=0;qr_present<=0;qr_done<=0;task_id<=0;qr_complete<=0;
            await_idle<=0;lock_pending<=0;
            vx<=0;vy<=0;frame_valid<=0;state<=0;axis<=0;negative<=0;
            bit_count<=0;multiplier<=0;multiplicand<=0;product<=0;
            saved_x<=0;saved_y<=0;saved_gain<=0;saved_limit<=0;pending_vx<=0;
        end else begin
            frame_valid<=0;
            qr_complete<=0;
            if(qr_mode && rearm_qr && (!fresh || !qr_present || qr_outside_center)) qr_done<=0;
            if(ms_tick) begin
                ms_count<=0;
                if(age_ms!=16'hffff) age_ms<=age_ms+1'b1;
                if(qr_mode && !task_active && tracking && centered && center_ms<16'd400)
                    center_ms<=center_ms+1'b1;
                else if(!qr_mode || task_active || !tracking || !centered)
                    center_ms<=0;
                if(byte_age!=15) byte_age<=byte_age+1'b1;
                if(byte_age>=9) window<=0;
            end else ms_count<=ms_count+1'b1;
            // Discard in-flight data while issuing the mode command, then
            // require a 2 ms RX idle boundary before accepting the new stream.
            if(mode_busy) begin
                window<=0;received<=0;await_idle<=1;
                if(rx_valid) byte_age<=0;
            end
            else if(await_idle) begin
                window<=0;
                if(rx_valid) byte_age<=0;
                else if(byte_age>=2) await_idle<=0;
            end
            else if(framing_error) begin window<=0;end
            else if(rx_valid) begin
                byte_age<=0;window<={window[95:0],rx_data};
                if(packet_ok && byte_age<10) begin
                    if(qr_mode) begin
                        // A running route owns the motion controller.  Ignore
                        // all incoming QR identities and coordinates until it
                        // returns to WAIT_NEXT.
                        if(!task_active) begin
                            x0<=window[33:24];y0<=window[16:8];x1<=0;y1<=0;
                            qr_present<=window[7:0]!=0;
                            if(window[7:0]!=0) begin
                                if(task_id!=window[2:0]) qr_done<=0;
                                task_id<=window[2:0];
                            end
                        end
                    end else begin
                        x0<=window[89:80];y0<=window[72:64];
                        x1<=window[41:32];y1<=window[24:16];
                    end
                    received<=1;age_ms<=0;frame_valid<=1;
                    if(lock_pending && (!qr_mode || window[7:0]!=0)) begin
                        locked_heading<=imu_heading;lock_pending<=0;
                    end
                end
            end
            if(config_valid) case(config_id)
                42:tolerance<=config_value[6:0];
                43:gain_q4<=config_value[7:0];
                44:speed_limit<=config_value[9:0];
                45:timeout_ms<=config_value[15:0];
                46:begin
                    qr_mode<=config_value[0];qr_done<=0;task_id<=0;qr_present<=0;
                    center_ms<=0;
                    received<=0;window<=0;x0<=0;y0<=0;x1<=0;y1<=0;
                end
            endcase
            if(qr_mode && !task_active && centered && center_ms>=16'd399 &&
               !accept_packet && !cancel && !config_valid) begin qr_done<=1;qr_complete<=1;center_ms<=16'd400;end
            if(cancel) begin enabled<=0;lock_pending<=0;end
            else if(config_valid && config_id==41) begin
                enabled<=config_value==1 && ready;
                received<=0;window<=0;qr_done<=0;center_ms<=0;
                lock_pending<=config_value==1 && ready;
                if(config_value==1 && ready) begin locked_heading<=imu_heading;task_id<=0;end
            end
            if(!tracking || cancel || mode_busy || (qr_mode && centered) || accept_packet ||
               (config_valid && config_id>=41 && config_id<=46)) begin
                vx<=0;vy<=0;state<=0;
            end else case(state)
                0: begin
                    saved_x<=abs_x<={3'd0,tolerance}?11'sd0:error_x;
                    saved_y<=abs_y<={3'd0,tolerance}?11'sd0:error_y;
                    saved_gain<=gain_q4;saved_limit<=speed_limit;axis<=0;state<=1;
                end
                1:begin
                    negative<=operand[10];
                    multiplicand<=operand[10]?-$signed(operand):operand;
                    multiplier<=saved_gain;product<=0;bit_count<=0;state<=2;
                end
                2:begin
                    product<=sum;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
                    if(bit_count==7) state<=3;else bit_count<=bit_count+1'b1;
                end
                3:if(!axis) begin pending_vx<=result;axis<=1;state<=1;end
                    else begin vx<=pending_vx;vy<=result;state<=0;end
            endcase
        end
    end
endmodule
