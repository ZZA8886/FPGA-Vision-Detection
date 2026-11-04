`timescale 1ns/1ps
// Executes the four user-supplied QR routes after a QR centre-lock pulse.
// A route is one-shot: after its last finite motion, wait for a fresh QR
// completion pulse.  The waiting level also rearms the QR latch only after
// the previous visual frame has gone stale, so the same continuously visible
// code cannot retrigger itself.
module qr_task_runner(
    input wire clk,reset_n,
    input wire automatic_enable,
    input wire qr_complete,
    input wire [2:0] qr_task_id,
    input wire auto_active,auto_finish,
    input wire signed [15:0] imu_heading,
    input wire vision_centered,
    output reg config_valid,
    output reg [5:0] config_id,
    output reg signed [31:0] config_value,
    output reg rearm_qr,
    output wire task_active,
    output reg [3:0] task_id,
    output reg [4:0] task_step,
    output reg vision_config_valid,
    output reg [5:0] vision_config_id,
    output reg signed [31:0] vision_config_value,
    input wire nav_frame_valid,
    input wire [2:0] nav_sequence_id
);
    localparam IDLE=0,LOAD=1,RUN=2,WAIT_NEXT=3,WAIT_NAV=4;
    reg [2:0] state;
    reg rotate_repeat;
    reg [2:0] phase;
    // Navigation sequence 6 maps to route 10, so the route index needs four bits.
    reg [3:0] route;
    reg [4:0] step;
    // Expose only the route execution window.  WAIT_NEXT intentionally
    // releases vision_follow so a new (or re-centred) QR can arm the next
    // route after the previous code has left the centre.
    assign task_active=(state==LOAD || state==RUN);

    function [4:0] route_len;
        input [3:0] r;
        begin case(r)
            1:route_len=9; 2:route_len=7; 3:route_len=8; 4:route_len=4;
            5:route_len=5; 6:route_len=5; 7:route_len=4; 8:route_len=4;
            9:route_len=5; 10:route_len=4;
            default:route_len=0;
        endcase end
    endfunction
    function is_rotate;
        input [3:0] r; input [4:0] s;
        begin is_rotate=0;
            if(r==1 && (s==1 || s==5 || s==6)) is_rotate=1;
            if(r==3 && (s==0 || s==4 || s==6)) is_rotate=1;
        end
    endfunction
    function is_visual_wait;
        input [3:0] r; input [4:0] s;
        begin is_visual_wait=(r==3 && (s==3 || s==5)); end
    endfunction
    function is_servo;
        input [3:0] r; input [4:0] s;
        begin is_servo=(r==3 && s==2); end
    endfunction
    function signed [31:0] move_angle;
        input [3:0] r; input [4:0] s;
        begin move_angle=0;
            case(r)
                1:case(s) 0:move_angle=9000;2:move_angle=0;3:move_angle=18000;
                    4:move_angle=0;7:move_angle=31500;8:move_angle=27000;endcase
                2:case(s) 0:move_angle=9000;1:move_angle=18000;2:move_angle=9000;
                    3:move_angle=6000;4:move_angle=33500;5:move_angle=31500;6:move_angle=27000;endcase
                3:if(s==1 || s==7) move_angle=0;
                4:case(s) 0:move_angle=9000;1:move_angle=18000;2:move_angle=9000;3:move_angle=33000;endcase
                5:case(s) 0:move_angle=0;1:move_angle=9000;2:move_angle=22500;3:move_angle=13000;4:move_angle=32500;endcase
                6:case(s) 0:move_angle=0;1:move_angle=9000;2:move_angle=18000;3:move_angle=31000;4:move_angle=33000;endcase
                7:case(s) 0:move_angle=0;1:move_angle=5000;2:move_angle=18000;3:move_angle=32500;endcase
                8:case(s) 0:move_angle=0;1:move_angle=13000;2:move_angle=0;3:move_angle=30500;endcase
                9:case(s) 0:move_angle=0;1:move_angle=9000;2:move_angle=0;3:move_angle=22500;4:move_angle=33000;endcase
                10:case(s) 0:move_angle=9000;1:move_angle=9000;2:move_angle=31000;3:move_angle=5500;endcase
            endcase
        end
    endfunction
    function signed [31:0] move_distance;
        input [3:0] r; input [4:0] s;
        begin move_distance=0;
            case(r)
                1:case(s) 0:move_distance=1550;2:move_distance=600;3:move_distance=600;
                    4:move_distance=700;7:move_distance=900;8:move_distance=950;endcase
                2:case(s) 0:move_distance=300;1:move_distance=2400;2:move_distance=1050;
                    3:move_distance=750;4:move_distance=850;5:move_distance=1800;6:move_distance=250;endcase
                3:if(s==1) move_distance=1700; else if(s==7) move_distance=1800;
                4:case(s) 0:move_distance=300;1:move_distance=2400;2:move_distance=510;3:move_distance=550;endcase
                5:case(s) 0:move_distance=1350;1:move_distance=300;2:move_distance=400;3:move_distance=400;4:move_distance=1330;endcase
                6:case(s) 0:move_distance=1350;1:move_distance=300;2:move_distance=500;3:move_distance=350;4:move_distance=950;endcase
                7:case(s) 0:move_distance=1100;1:move_distance=400;2:move_distance=500;3:move_distance=1330;endcase
                8:case(s) 0:move_distance=1100;1:move_distance=400;2:move_distance=500;3:move_distance=100;endcase
                9:case(s) 0:move_distance=850;1:move_distance=300;2:move_distance=500;3:move_distance=400;4:move_distance=950;endcase
                10:case(s) 0:move_distance=300;1:move_distance=300;2:move_distance=400;3:move_distance=400;endcase
            endcase
        end
    endfunction
    function signed [31:0] rotate_delta;
        input [3:0] r; input [4:0] s;
        begin rotate_delta=0;
            if(r==1 && (s==1 || s==5 || s==6)) rotate_delta=18000;
            if(r==3 && s==0) rotate_delta=18000;
            if(r==3 && (s==4 || s==6)) rotate_delta=9000;
        end
    endfunction
    function signed [31:0] absolute_target;
        input signed [15:0] now; input signed [31:0] delta;
        reg signed [32:0] v;
        begin
            v=$signed(now)+delta;
            if(v<0) v=v+36000;
            if(v<0) v=v+36000;
            if(v>=36000) v=v-36000;
            if(v>=36000) v=v-36000;
            absolute_target=v[31:0];
        end
    endfunction

    always @* begin
        config_valid = state==LOAD;
        config_id = 0;
        config_value = 0;
        vision_config_valid = 1'b0;
        vision_config_id = 0;
        vision_config_value = 0;
        if(state==LOAD) begin
            if(is_visual_wait(route,step)) begin
                // Step 3 switches the Pi to object mode and starts tracking;
                // step 5 only waits for the next object to be centred.
                if(step==3 && phase==0) begin vision_config_valid=1'b1;vision_config_id=46;vision_config_value=0;end
                if(step==3 && phase==1) begin vision_config_valid=1'b1;vision_config_id=41;vision_config_value=1;end
                config_valid=1'b0;
            end else if(is_servo(route,step)) begin
                config_valid=1'b0;
            end else if(is_rotate(route,step)) begin
                case(phase)
                    0:begin config_id=32;config_value=absolute_target(imu_heading,rotate_delta(route,step));end
                    1:begin config_id=28;config_value=800;end
                    2:begin config_id=36;config_value=100;end
                    3:begin config_id=37;config_value=500;end
                    default:begin config_id=40;config_value=1;end
                endcase
            end else begin
                case(phase)
                    0:begin config_id=33;config_value=move_angle(route,step);end
                    1:begin config_id=34;config_value=move_distance(route,step);end
                    2:begin config_id=35;config_value=300;end
                    3:begin config_id=38;config_value=5;end
                    4:begin config_id=39;config_value=100;end
                    default:begin config_id=40;config_value=2;end
                endcase
            end
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;phase<=0;route<=0;step<=0;task_id<=0;task_step<=0;rotate_repeat<=0;
        end else begin
            rearm_qr<=state==WAIT_NEXT;
            task_id<=route;task_step<=step;
            if(!automatic_enable) begin state<=IDLE;phase<=0;route<=0;step<=0;end
            else case(state)
                IDLE: if(qr_complete && !auto_active && qr_task_id>=1 && qr_task_id<=4) begin
                    route<=qr_task_id;step<=0;phase<=0;rotate_repeat<=0;state<=LOAD;
                end
                LOAD: begin
                    if(is_visual_wait(route,step)) begin
                        if((step==3 && phase>=1) || step==5) begin phase<=0;state<=RUN;end
                        else phase<=phase+1'b1;
                    end else if(is_servo(route,step)) begin
                        phase<=0;state<=RUN;
                    end else if((is_rotate(route,step) && phase>=3) || (!is_rotate(route,step) && phase>=5)) begin
                        phase<=0;state<=RUN;
                    end else phase<=phase+1'b1;
                end
                RUN: if(is_visual_wait(route,step)) begin
                    if(vision_centered) begin
                        if(step+1<route_len(route)) begin step<=step+1'b1;phase<=0;rotate_repeat<=0;state<=LOAD;end
                        else state<=WAIT_NEXT;
                    end
                end else if(is_servo(route,step)) begin
                    if(step+1<route_len(route)) begin step<=step+1'b1;phase<=0;rotate_repeat<=0;state<=LOAD;end
                    else state<=WAIT_NEXT;
                end else if(auto_finish) begin
                    if(is_rotate(route,step) && route==1 && step==5 && !rotate_repeat) begin rotate_repeat<=1;phase<=0;state<=LOAD;end
                    else if(step+1<route_len(route)) begin step<=step+1'b1;phase<=0;rotate_repeat<=0;state<=LOAD;end
                    else if(route==4) begin state<=WAIT_NAV;phase<=0;end
                    else begin state<=WAIT_NEXT;phase<=0;end
                end
                WAIT_NAV: if(nav_frame_valid && nav_sequence_id>=1 && nav_sequence_id<=6) begin
                    // Route 4 is reserved for QR4's public leg.  Navigation
                    // commands therefore occupy routes 5..10 (AA 01..06).
                    route<={1'b0,nav_sequence_id}+4'd4;step<=0;phase<=0;rotate_repeat<=0;state<=LOAD;
                end
                WAIT_NEXT: if(qr_complete && !auto_active) begin
                    route<=qr_task_id;step<=0;phase<=0;rotate_repeat<=0;state<=LOAD;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
