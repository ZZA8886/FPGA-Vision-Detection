`timescale 1ns/1ps
module navbot_controller #(
    parameter integer CLK_HZ=50000000,
    parameter integer OLED_ENABLE=1,
    parameter integer SHARED_PI=0,
    parameter [6:0] OLED_ADDRESS=7'h3c
) (
    input wire clk,
    input wire [2:0] key_n,
    input wire [3:0] encoder_a,encoder_b,
    output wire [3:0] motor_in1,motor_in2,motor_pwm,
    output wire [1:0] servo_pwm,
    input wire vision_rx,tof_rx,
    inout wire imu_sda,
    output wire vision_tx,tof_tx,imu_scl,
    inout wire oled_scl,oled_sda,
    input wire debug_rx,
    output wire debug_tx,
    input wire nav_rx,
    output wire nav_tx,
    output wire signed [15:0] video_yaw_cdeg,
    output wire video_yaw_fresh,video_reset_n
);
    // Gowin configuration INIT supplies a clocked power-on reset. Holding
    // KEY0+KEY1 for 1.5 s restarts this sequence without consuming another pin.
    reg [15:0] por_count=0;
    wire reset_request;
    wire reset_n=&por_count;
    always @(posedge clk) begin
        if(reset_request) por_count<=0;
        else if(!reset_n) por_count<=por_count+1'b1;
    end
    wire previous,next,confirm,stop;
    wire oled_ready,i2c_error;
    menu_keys #(.CLK_HZ(CLK_HZ)) keys(clk,reset_n,key_n,previous,next,confirm,stop,reset_request);
    wire [4:0] item;
    wire editing,ui_arm,manual_mode,echo_enable,clear_odometry,need_gain;
    wire [1:0] test_wheel;
    wire signed [11:0] test_pwm;
    wire [3:0] motor_invert,encoder_invert;
    wire [15:0] kp,ki,wheel_limit,angle1,angle2;
    wire signed [15:0] vx,vy,wz;
    wire signed [31:0] selected_value;
    wire remote_valid,remote_move_valid,remote_owned,remote_heartbeat;
    // Declare command fields before the power-on parameter process.  Keeping
    // these declarations ahead of their first use avoids implicit-net
    // creation in older Gowin Verilog elaborators.
    wire [5:0] remote_id;
    wire signed [31:0] remote_value;
    wire [15:0] remote_accepted,remote_rejected;
    wire [1023:0] remote_config;
    reg qr_auto_enable;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) qr_auto_enable<=1'b1;
        else if(remote_valid && remote_id==47) qr_auto_enable<=remote_value!=0;
        // A wireless timeout must not cancel an already selected QR mission;
        // the FPGA owns the route after centre lock.  Explicit STOP, local
        // stop, manual takeover, or SET 41 0 still clears it.
        else if(stop || remote_move_valid || (remote_valid && remote_id==41 && remote_value==0)) qr_auto_enable<=1'b0;
    end
    localparam integer REMOTE_AGE_BITS=$clog2(CLK_HZ/2+1)>0?$clog2(CLK_HZ/2+1):1;
    reg [REMOTE_AGE_BITS-1:0] remote_age;
    wire remote_timeout=remote_age>=CLK_HZ/2;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) remote_age<=CLK_HZ/2;
        else if(auto_launch || vision_launch || remote_heartbeat || (remote_valid && remote_id==0 && remote_value==1 && !remote_owned)) remote_age<=0;
        else if(!remote_timeout) remote_age<=remote_age+1'b1;
    end
    wire remote_stop=remote_owned && remote_timeout;
    wire signed [15:0] remote_vx,remote_vy,remote_wz;
    wire heading_enable,heading_invert,heading_zero,imu_fresh,imu_update;
    wire signed [15:0] imu_heading,heading_target,heading_error,heading_command;
    wire [16:0] heading_kp;
    wire [15:0] heading_ki,heading_kd;
    wire [15:0] heading_limit,imu_frames;
    wire auto_active,auto_finish;
    wire [2:0] auto_status;
    wire [3:0] auto_reason;
    wire [15:0] auto_sequence;
    wire task_config_valid,task_rearm_qr,task_active;
    wire [5:0] task_config_id;
    wire signed [31:0] task_config_value;
    wire task_vision_config_valid;
    wire [5:0] task_vision_config_id;
    wire signed [31:0] task_vision_config_value;
    wire [3:0] task_id_active;
    wire [4:0] task_step_active;
    wire nav_frame_valid;
    wire [2:0] nav_sequence_id;
    wire [255:0] auto_settings;
    wire signed [31:0] auto_progress,auto_remaining;
    wire signed [15:0] auto_vx,auto_vy,auto_target,auto_locked,auto_body_angle;
    wire auto_ready=imu_fresh && kp!=0 && heading_kp!=0 && wheel_limit!=0 && heading_limit!=0 && !manual_mode && (!ui_arm || (qr_auto_enable && vision_enabled)) && !stop;
    wire [3:0] auto_block_reason=!imu_fresh?4'd1:(kp==0 || heading_kp==0)?4'd2:
        wheel_limit==0?4'd3:heading_limit==0?4'd4:manual_mode?4'd5:4'd6;
    wire auto_launch=remote_valid && remote_id==40 && auto_ready;
    wire vision_enabled,vision_fresh,vision_tracking,vision_centered,vision_class;
    wire vision_present0,vision_present1,vision_frame;
    wire vision_qr_done,vision_qr_complete,vision_mode_busy;
    wire vision_qr_rearm;
    wire [2:0] vision_task_id;
    // Mode changes are armed by reset or an explicit STOP.  This prevents a
    // late SET 46 0 from another controller instance from changing an active
    // or completed QR session to object mode.  Switching modes through the UI
    // remains supported because it sends STOP before SET 46.
    reg vision_mode_stop_armed;
    wire vision_mode_change_allowed=vision_mode_stop_armed ||
        (remote_value[0]==vision_mode[17]);
    wire stale_vision_mode=remote_valid && remote_id==46 && !vision_mode_change_allowed;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) vision_mode_stop_armed<=1'b1;
        else if(stop || (remote_valid && remote_id==0 && remote_value==0))
            vision_mode_stop_armed<=1'b1;
        else if(remote_valid && remote_id==46 && vision_mode_change_allowed)
            vision_mode_stop_armed<=1'b0;
    end
    wire [9:0] vision_x0,vision_x1;
    wire [8:0] vision_y0,vision_y1;
    wire signed [15:0] vision_vx,vision_vy,vision_heading;
    wire [31:0] vision_tuning,vision_mode;
    wire vision_ready=auto_ready && !auto_active;
    wire vision_launch=remote_valid && remote_id==41 && remote_value==1 && vision_ready;
    // QR automatic ownership starts when QR mode + automatic task mode are
    // selected, before the first valid QR frame arrives.  This is the
    // intentional disconnected-UI window: vision_follow still outputs zero
    // velocity until a fresh target is received, while the wireless timeout
    // cannot cancel the pending mission.
    wire qr_autonomous=qr_auto_enable && vision_mode[17];
    // Once QR automatic mode is selected, the FPGA owns the whole mission,
    // including the initial wait/centering phase.  A wireless heartbeat loss
    // must therefore not cancel visual tracking before the first route step.
    // Local stop, manual takeover, configuration changes and IMU interlocks
    // remain active through the normal conditions below.
    wire vision_cancel=stop || (remote_stop && !qr_autonomous) || previous || next || confirm ||
        (vision_enabled && (!ui_arm || heading_block)) || remote_move_valid ||
        (remote_valid && remote_id!=7 && remote_id!=8 && remote_id!=9 && remote_id!=41 && remote_id!=46 && remote_id!=47);
    wire auto_cancel=auto_active && (remote_move_valid || previous || next || confirm ||
        (remote_valid && remote_id!=7 && remote_id!=8 && remote_id!=9 && remote_id!=40));
    wire auto_loop=heading_enable || auto_active || task_active || vision_enabled;
    wire heading_block=auto_loop && !manual_mode && (!imu_fresh || heading_kp==0);
    wire menu_remote_valid=remote_valid && !stale_vision_mode;
    wire [5:0] menu_remote_id=remote_id<32?remote_id:((auto_launch || vision_launch)?6'd31:6'd0);
    wire signed [31:0] menu_remote_value=remote_id<32?remote_value:32'sd0;
    // A route step completion must not disarm the menu arm; the task runner
    // immediately launches the next step.  Preserve the original one-shot
    // auto-motion behaviour when no QR route is active.
    navbot_menu menu(clk,reset_n,previous && OLED_ENABLE,next && OLED_ENABLE,confirm && OLED_ENABLE,
        stop || (auto_finish && !task_active && !qr_autonomous) || (((remote_stop && !task_active && !qr_autonomous) || (!remote_owned && i2c_error) || heading_block) && ui_arm),
        menu_remote_valid,menu_remote_id,menu_remote_value,remote_move_valid,remote_vx,remote_vy,remote_wz,
        item,editing,ui_arm,manual_mode,
        echo_enable,clear_odometry,test_wheel,test_pwm,motor_invert,encoder_invert,
        kp,ki,wheel_limit,angle1,angle2,vx,vy,wz,selected_value,need_gain,
        heading_enable,heading_invert,heading_zero,heading_target,heading_kp,heading_ki,heading_kd,heading_limit,remote_owned,remote_config);
    wire [63:0] counts,deltas,encoder_errors;
    wire signed [63:0] x,y;
    wire signed [31:0] heading,measured_vx,measured_vy,measured_wz;
    wire pose_valid;
    // QR automatic mode owns the drive permission as soon as it is enabled,
    // not only after centre lock.  This keeps the tracking phase alive when
    // the wireless UI disconnects; vision_follow still commands zero speed
    // while no valid QR frame is available.
    wire drive_arm=ui_arm && (((remote_owned && !remote_timeout) || (!remote_owned && oled_ready && !i2c_error)) || qr_autonomous) && !heading_block && !auto_finish && !auto_cancel &&
        (auto_active || task_active || !vision_enabled || (vision_tracking && !vision_cancel));
    // Heading mode replaces open-loop yaw rate; VX/VY remain body-frame commands.
    wire signed [15:0] effective_target=auto_active?auto_target:vision_enabled?vision_heading:heading_target;
    wire signed [15:0] effective_vx=auto_active?auto_vx:vision_enabled?vision_vx:vx;
    wire signed [15:0] effective_vy=auto_active?auto_vy:vision_enabled?vision_vy:vy;
    wire signed [15:0] effective_wz=auto_loop && !manual_mode?heading_command:wz;
    heading_pid yaw_loop(clk,reset_n,drive_arm && auto_loop && !manual_mode,
        imu_update,imu_heading,effective_target,heading_kp,heading_ki,heading_kd,heading_limit,
        heading_command,heading_error);
    wire [127:0] wheel_targets_q16,target_rpm,actual_rpm;
    wire [47:0] wheel_efforts,monitor_pwm;
    wire wheel_sample_valid,monitor_valid;
    navbot_motion_core #(.CLK_HZ(CLK_HZ),.SHARED_PI(SHARED_PI)) motion(
        clk,reset_n,drive_arm,clear_odometry,1'b0,effective_vx,effective_vy,effective_wz,wheel_limit,kp,ki,
        motor_invert,encoder_invert,encoder_a,encoder_b,motor_in1,motor_in2,motor_pwm,
        counts,deltas,encoder_errors,x,y,heading,measured_vx,measured_vy,measured_wz,pose_valid,
        manual_mode,test_wheel,test_pwm,wheel_targets_q16,wheel_efforts,wheel_sample_valid);
    qr_task_runner task_runner(
        .clk(clk),.reset_n(reset_n),.automatic_enable(qr_auto_enable && vision_enabled),
        .qr_complete(vision_qr_complete),.qr_task_id(vision_task_id),
        .auto_active(auto_active),.auto_finish(auto_finish),.imu_heading(imu_heading),.vision_centered(vision_centered),
        .config_valid(task_config_valid),.config_id(task_config_id),.config_value(task_config_value),
        .vision_config_valid(task_vision_config_valid),.vision_config_id(task_vision_config_id),.vision_config_value(task_vision_config_value),
        .rearm_qr(task_rearm_qr),.task_active(task_active),.task_id(task_id_active),.task_step(task_step_active),
        .nav_frame_valid(nav_frame_valid),.nav_sequence_id(nav_sequence_id));
    wire motion_config_valid=remote_valid || task_config_valid;
    wire [5:0] motion_config_id=task_config_valid?task_config_id:remote_id;
    wire signed [31:0] motion_config_value=task_config_valid?task_config_value:remote_value;
    auto_motion #(.SERIAL_TRIG(SHARED_PI)) automatic_motion(clk,reset_n,motion_config_valid,motion_config_id,motion_config_value,
        ui_arm || (qr_autonomous && task_active) || auto_active,
        stop || (remote_stop && !qr_autonomous) || auto_cancel,
        imu_fresh,imu_update,auto_ready,auto_block_reason,
        imu_heading,wheel_sample_valid,deltas,auto_active,auto_finish,auto_vx,auto_vy,auto_target,
        auto_status,auto_reason,auto_sequence,auto_settings,auto_progress,auto_remaining,auto_locked,auto_body_angle);
    wheel_telemetry monitor(clk,reset_n,wheel_sample_valid,wheel_targets_q16,deltas,wheel_efforts,
        target_rpm,actual_rpm,monitor_pwm,monitor_valid);
    wire [7:0] debug_rx_data;
    wire debug_rx_valid,debug_framing;
    firewater_uart #(.CLK_HZ(CLK_HZ)) debug_stream(clk,reset_n,debug_rx,debug_tx,test_wheel,
        target_rpm,actual_rpm,monitor_pwm,debug_rx_data,debug_rx_valid,debug_framing);
    wire [15:0] effective_servo1=(task_active && task_id_active==4'd3 && task_step_active==5'd2)?16'd6000:angle1;
    servo_pwm #(.CLK_HZ(CLK_HZ)) servo1(clk,reset_n,effective_servo1,servo_pwm[0]);
    servo_pwm #(.CLK_HZ(CLK_HZ)) servo2(clk,reset_n,angle2,servo_pwm[1]);
    wire [15:0] uart_count[0:2];
    wire [15:0] nav_receive_count;
    nav_sequence_uart #(.CLK_HZ(CLK_HZ)) navigation(
        .clk(clk),.reset_n(reset_n),.rx(nav_rx),.tx(nav_tx),
        .receive_count(nav_receive_count),.frame_valid(nav_frame_valid),.sequence_id(nav_sequence_id));
    // Raspberry Pi detections use the dedicated vision byte interface.
    wire [15:0] uart_rx_data;
    wire [1:0] uart_rx_valid,uart_framing,uart_overflow;
    wire vision_config_valid=task_vision_config_valid ||
        (remote_valid && (remote_id!=46 || vision_mode_change_allowed));
    wire vision_mode_request=remote_valid &&
        ((remote_id==46 && vision_mode_change_allowed) || vision_launch) ||
        (task_vision_config_valid && (task_vision_config_id==6'd46 || task_vision_config_id==6'd41));
    vision_uart #(.CLK_HZ(CLK_HZ)) vision(
        .clk(clk),.reset_n(reset_n),.rx(vision_rx),.echo_enable(echo_enable && !vision_enabled),
        .request(vision_mode_request),.qr_mode(task_vision_config_valid && task_vision_config_id==6'd46 ?
            task_vision_config_value[0] : (remote_id==46?remote_value[0]:vision_mode[17])),
        .qr_data_valid(vision_frame),
        .tx(vision_tx),.busy(vision_mode_busy),.receive_count(uart_count[0]),
        .receive_data(uart_rx_data[7:0]),.receive_valid(uart_rx_valid[0]),
        .framing_error(uart_framing[0]),.overflow(uart_overflow[0]));
    vision_follow #(.CLK_HZ(CLK_HZ)) visual_follow(
        .clk(clk),.reset_n(reset_n),.rx_data(uart_rx_data[7:0]),.rx_valid(uart_rx_valid[0]),
        .framing_error(uart_framing[0]),.config_valid(vision_config_valid),
        .config_id(task_vision_config_valid?task_vision_config_id:remote_id),
        .config_value(task_vision_config_valid?task_vision_config_value:remote_value),
        .cancel(vision_cancel),.ready(vision_ready),.imu_heading(imu_heading),
        .enabled(vision_enabled),.fresh(vision_fresh),.tracking(vision_tracking),
        .centered(vision_centered),.selected_class(vision_class),
        .present0(vision_present0),.present1(vision_present1),.x0(vision_x0),.y0(vision_y0),
        .x1(vision_x1),.y1(vision_y1),.vx(vision_vx),.vy(vision_vy),.locked_heading(vision_heading),
        .tuning_word(vision_tuning),.mode_word(vision_mode),.frame_valid(vision_frame),
        .mode_busy(vision_mode_busy),.auto_enable(qr_auto_enable),.task_active(task_active),.rearm_qr(task_rearm_qr),
        .qr_done(vision_qr_done),.task_id(vision_task_id),.qr_complete(vision_qr_complete));
    wire [6:0] remote_word_index;
    wire [7:0] remote_tx_data;
    wire remote_tx_valid,remote_tx_ready;
    reg [31:0] remote_word;
    remote_command #(.CLK_HZ(CLK_HZ)) command_uart(
        .clk(clk),.reset_n(reset_n),.rx(tof_rx),.tx(tof_tx),
        .receive_count(uart_count[1]),.receive_data(uart_rx_data[15:8]),
        .receive_valid(uart_rx_valid[1]),.framing_error(uart_framing[1]),
        .command_valid(remote_valid),.command_id(remote_id),.command_value(remote_value),
        .move_valid(remote_move_valid),.move_vx(remote_vx),.move_vy(remote_vy),.move_wz(remote_wz),
        .tx_data(remote_tx_data),.tx_valid(remote_tx_valid),.tx_ready(remote_tx_ready),
        .heartbeat(remote_heartbeat),.accepted(remote_accepted),.rejected(remote_rejected));
    remote_telemetry #(.CLK_HZ(CLK_HZ)) host_stream(clk,reset_n,remote_word_index,
        remote_word,remote_tx_data,remote_tx_valid,remote_tx_ready);
    always @* begin
        remote_word=0;
        if(remote_word_index<32) begin
            remote_word=remote_config[remote_word_index*32 +: 32];
            // Reserved v2 words carry the optional vision extension; frame
            // length and 10 Hz bandwidth stay unchanged. Mode marker is 0x56.
            case(remote_word_index)
                19:remote_word={vision_heading,4'd0,vision_qr_done,vision_task_id,vision_ready,vision_tracking,vision_centered,vision_class,
                    vision_present1,vision_present0,vision_fresh,vision_enabled};
                21:remote_word={7'd0,vision_y0,6'd0,vision_x0};
                22:remote_word={7'd0,vision_y1,6'd0,vision_x1};
                30:remote_word=vision_tuning;
                31:remote_word={vision_mode[31:19],qr_auto_enable,vision_mode[17:0]};
            endcase
        end
        else case(remote_word_index)
            32: remote_word=32'h4e420002;
            33: remote_word={22'b0,auto_active,need_gain,remote_timeout,i2c_error,oled_ready,heading_block,imu_fresh,remote_owned,ui_arm,drive_arm};
            34: remote_word={16'b0,remote_accepted};
            35: remote_word={16'b0,remote_rejected};
            36: remote_word=target_rpm[31:0];
            37: remote_word=target_rpm[63:32];
            38: remote_word=target_rpm[95:64];
            39: remote_word=target_rpm[127:96];
            40: remote_word=actual_rpm[31:0];
            41: remote_word=actual_rpm[63:32];
            42: remote_word=actual_rpm[95:64];
            43: remote_word=actual_rpm[127:96];
            44: remote_word={{20{monitor_pwm[11]}},monitor_pwm[11:0]};
            45: remote_word={{20{monitor_pwm[23]}},monitor_pwm[23:12]};
            46: remote_word={{20{monitor_pwm[35]}},monitor_pwm[35:24]};
            47: remote_word={{20{monitor_pwm[47]}},monitor_pwm[47:36]};
            48: remote_word={{16{imu_heading[15]}},imu_heading};
            49: remote_word={{16{heading_error[15]}},heading_error};
            50: remote_word={{16{heading_command[15]}},heading_command};
            51: remote_word=x_integer;
            52: remote_word=y_integer;
            53: remote_word=heading_integer;
            54: remote_word={16'b0,counts[15:0]};
            55: remote_word={16'b0,counts[31:16]};
            56: remote_word={16'b0,counts[47:32]};
            57: remote_word={16'b0,counts[63:48]};
            58: remote_word={16'b0,uart_count[0]};
            59: remote_word={16'b0,uart_count[1]};
            60: remote_word={16'b0,uart_count[2]};
            61: remote_word=measured_vx;
            62: remote_word=measured_vy;
            63: remote_word=measured_wz;
            64: remote_word={29'b0,auto_status};
            65,66,67,68,69,70,71,72: remote_word=auto_settings[(remote_word_index-65)*32 +: 32];
            73: remote_word=auto_progress;
            74: remote_word=auto_remaining;
            75: remote_word={{16{auto_locked[15]}},auto_locked};
            76: remote_word={16'b0,auto_sequence};
            77: remote_word={28'b0,auto_reason};
            78: remote_word={{16{auto_target[15]}},auto_target};
            79: remote_word={{16{auto_body_angle[15]}},auto_body_angle};
        endcase
    end
    imu660ra_i2c #(.CLK_HZ(CLK_HZ)) imu(clk,reset_n,imu_sda,imu_scl,
        heading_invert,heading_zero,imu_heading,imu_update,imu_fresh,imu_frames);
    assign uart_count[2]=imu_frames;
    wire screen_start,screen_done;
    wire [2:0] page;
    wire [6:0] column;
    wire [7:0] pixels;
    wire signed [31:0] x_integer=x[47:16],y_integer=y[47:16],heading_integer=heading >>> 16;
    assign video_yaw_cdeg=imu_heading;
    assign video_yaw_fresh=imu_fresh;
    assign video_reset_n=reset_n;
    generate if(OLED_ENABLE) begin: display_enabled
    oled_text_screen screen(clk,reset_n,screen_start,item,editing,drive_arm,manual_mode,need_gain,
        selected_value,x_integer,y_integer,heading_integer,counts,
        {uart_count[2],uart_count[1],uart_count[0]},page,column,pixels,screen_done,
        target_rpm,actual_rpm,monitor_pwm,test_wheel,
        imu_heading,effective_target,heading_error,heading_command,imu_fresh,auto_loop,heading_block);
    oled_ssd1306 #(.CLK_HZ(CLK_HZ),.ADDRESS(OLED_ADDRESS)) oled(
        clk,reset_n,oled_scl,oled_sda,screen_start,screen_done,page,column,pixels,oled_ready,i2c_error);
    end else begin: display_disabled
        assign oled_scl=1'bz;
        assign oled_sda=1'bz;
        assign oled_ready=1'b0;
        assign i2c_error=1'b0;
    end endgenerate
endmodule

// Original board interface. Video-only signals remain internal, so the regular
// chassis project does not acquire any additional or unconstrained package pins.
module navbot_top #(
    parameter integer CLK_HZ=50000000,
    parameter [6:0] OLED_ADDRESS=7'h3c
)(
    input wire clk,
    input wire [2:0] key_n,
    input wire [3:0] encoder_a,encoder_b,
    output wire [3:0] motor_in1,motor_in2,motor_pwm,
    output wire [1:0] servo_pwm,
    input wire vision_rx,tof_rx,
    inout wire imu_sda,
    output wire vision_tx,tof_tx,imu_scl,
    inout wire oled_scl,oled_sda,
    input wire debug_rx,
    output wire debug_tx,
    input wire nav_rx,
    output wire nav_tx
);
    navbot_controller #(.CLK_HZ(CLK_HZ),.OLED_ADDRESS(OLED_ADDRESS)) controller(
        .clk(clk),.key_n(key_n),.encoder_a(encoder_a),.encoder_b(encoder_b),
        .motor_in1(motor_in1),.motor_in2(motor_in2),.motor_pwm(motor_pwm),.servo_pwm(servo_pwm),
        .vision_rx(vision_rx),.tof_rx(tof_rx),.imu_sda(imu_sda),
        .vision_tx(vision_tx),.tof_tx(tof_tx),.imu_scl(imu_scl),
        .oled_scl(oled_scl),.oled_sda(oled_sda),.debug_rx(debug_rx),.debug_tx(debug_tx),
        .nav_rx(nav_rx),.nav_tx(nav_tx));
endmodule
