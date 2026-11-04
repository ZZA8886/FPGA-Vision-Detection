`timescale 1ns/1ps

module imu660ra_i2c #(
    parameter integer CLK_HZ=50000000,
    parameter integer SAMPLE_HZ=200,
    parameter integer TIMEOUT_MS=500
) (
    input wire clk,reset_n,
    inout wire sda,
    output wire scl,
    input wire invert,zero,
    output reg signed [15:0] heading_cdeg,
    output reg update,
    output wire fresh,
    output reg [15:0] frame_count
);
    localparam integer BOOT_TICKS=CLK_HZ/50;
    localparam integer SAMPLE_TICKS=CLK_HZ/SAMPLE_HZ;
    localparam integer TIMEOUT_TICKS=(CLK_HZ/1000)*TIMEOUT_MS;
    localparam integer AGE_BITS=$clog2(TIMEOUT_TICKS+1)>0?$clog2(TIMEOUT_TICKS+1):1;
    localparam integer TIMER_BITS=$clog2(BOOT_TICKS+1)>0?$clog2(BOOT_TICKS+1):1;
    localparam [3:0] BOOT=0,PWR_CONF=1,INIT_CTRL0=2,CONFIG=3,INIT_CTRL1=4,
        INIT_WAIT=5,INT_STATUS=6,PWR_CTRL=7,ACC_CONF=8,GYR_CONF=9,
        ACC_RANGE=10,GYR_RANGE=11,READY=12,READ_GYRO=13,FAULT=14;

    reg [3:0] phase;
    reg [TIMER_BITS-1:0] timer;
    reg [31:0] sample_timer;
    reg [AGE_BITS-1:0] age;
    reg request_issued;
    reg signed [31:0] gyro_sum;
    reg signed [15:0] gyro_bias;
    reg [7:0] calibration_samples;
    reg start_master,master_write,master_config;
    reg [7:0] master_reg,master_write_data;
    reg [13:0] master_write_count;
    reg [3:0] master_read_count;
    wire master_busy,master_done,master_ack_error;
    wire [47:0] master_read_data;
    wire [12:0] config_addr;
    wire [7:0] config_data;
    wire signed [15:0] gyro_z_sample=$signed(master_read_data[15:0]);
    wire signed [31:0] gyro_delta=(($signed(gyro_z_sample)-$signed(gyro_bias))*32'sd5)/32'sd164;
    wire signed [31:0] directed_delta=invert?-gyro_delta:gyro_delta;
    wire [31:0] age_limit=TIMEOUT_TICKS;

    imu660ra_config_rom config_rom(.addr(config_addr),.data(config_data));
    imu660ra_i2c_master #(.CLK_HZ(CLK_HZ)) master(
        .clk(clk),.reset_n(reset_n),.sda(sda),.scl(scl),.start(start_master),
        .write_mode(master_write),.config_mode(master_config),.reg_addr(master_reg),
        .write_data(master_write_data),.write_count(master_write_count),
        .read_count(master_read_count),.config_addr(config_addr),.config_data(config_data),
        .busy(master_busy),.done(master_done),.ack_error(master_ack_error),.read_data(master_read_data));

    assign fresh=phase==READY && age<age_limit && calibration_samples==8'd128;

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            phase<=BOOT;timer<=0;sample_timer<=0;age<=TIMEOUT_TICKS;
            request_issued<=0;gyro_sum<=0;gyro_bias<=0;calibration_samples<=0;
            start_master<=0;master_write<=0;master_config<=0;master_reg<=0;
            master_write_data<=0;master_write_count<=0;master_read_count<=0;
            heading_cdeg<=0;update<=0;frame_count<=0;
        end else begin
            start_master<=0;update<=0;
            if(age<age_limit) age<=age+1'b1;
            if(zero && fresh) begin heading_cdeg<=0;age<=0;end
            case(phase)
                BOOT: begin
                    if(timer<BOOT_TICKS) timer<=timer+1'b1;
                    else begin phase<=PWR_CONF;request_issued<=0;end
                end
                PWR_CONF,INIT_CTRL0,CONFIG,INIT_CTRL1,PWR_CTRL,ACC_CONF,GYR_CONF,ACC_RANGE,GYR_RANGE: begin
                    if(!request_issued && !master_busy) begin
                        start_master<=1;request_issued<=1;master_write<=1;master_config<=phase==CONFIG;
                        master_read_count<=0;
                        case(phase)
                            PWR_CONF: begin master_reg<=8'h7C;master_write_data<=8'h00;master_write_count<=1;end
                            INIT_CTRL0: begin master_reg<=8'h59;master_write_data<=8'h00;master_write_count<=1;end
                            CONFIG: begin master_reg<=8'h5E;master_write_data<=0;master_write_count<=14'd8192;end
                            INIT_CTRL1: begin master_reg<=8'h59;master_write_data<=8'h01;master_write_count<=1;end
                            PWR_CTRL: begin master_reg<=8'h7D;master_write_data<=8'h0E;master_write_count<=1;end
                            ACC_CONF: begin master_reg<=8'h40;master_write_data<=8'hA7;master_write_count<=1;end
                            GYR_CONF: begin master_reg<=8'h42;master_write_data<=8'hA9;master_write_count<=1;end
                            ACC_RANGE: begin master_reg<=8'h41;master_write_data<=8'h02;master_write_count<=1;end
                            default: begin master_reg<=8'h43;master_write_data<=8'h00;master_write_count<=1;end
                        endcase
                    end
                    if(master_done) begin
                        request_issued<=0;
                        if(master_ack_error) phase<=FAULT;
                        else if(phase==PWR_CONF) phase<=INIT_CTRL0;
                        else if(phase==INIT_CTRL0) phase<=CONFIG;
                        else if(phase==CONFIG) begin phase<=INIT_CTRL1;timer<=0;end
                        else if(phase==INIT_CTRL1) phase<=INIT_WAIT;
                        else if(phase==PWR_CTRL) phase<=ACC_CONF;
                        else if(phase==ACC_CONF) phase<=GYR_CONF;
                        else if(phase==GYR_CONF) phase<=ACC_RANGE;
                        else if(phase==ACC_RANGE) phase<=GYR_RANGE;
                        else begin phase<=READY;sample_timer<=0;end
                    end
                end
                INIT_WAIT: begin
                    if(timer<BOOT_TICKS) timer<=timer+1'b1;
                    else begin phase<=INT_STATUS;request_issued<=0;end
                end
                INT_STATUS: begin
                    if(!request_issued && !master_busy) begin
                        start_master<=1;request_issued<=1;master_write<=0;master_config<=0;
                        master_reg<=8'h21;master_read_count<=1;master_write_count<=0;
                    end
                    if(master_done) begin
                        request_issued<=0;
                        if(master_ack_error || master_read_data[7:0]!=8'h01) phase<=FAULT;
                        else phase<=PWR_CTRL;
                    end
                end
                READY: begin
                    if(sample_timer<SAMPLE_TICKS) sample_timer<=sample_timer+1'b1;
                    else if(!master_busy) begin
                        start_master<=1;request_issued<=1;master_write<=0;master_config<=0;
                        master_reg<=8'h12;master_read_count<=6;master_write_count<=0;
                        phase<=READ_GYRO;sample_timer<=0;
                    end
                end
                READ_GYRO: begin
                    if(master_done) begin
                        request_issued<=0;
                        if(master_ack_error) phase<=FAULT;
                        else begin
                            frame_count<=frame_count+1'b1;
                            if(calibration_samples<8'd128) begin
                                gyro_sum<=gyro_sum+gyro_z_sample;
                                calibration_samples<=calibration_samples+1'b1;
                                if(calibration_samples==8'd127) begin
                                    gyro_bias<=(gyro_sum+gyro_z_sample)>>>7;
                                    age<=0;
                                end
                            end else begin
                                heading_cdeg<=heading_cdeg+directed_delta;
                                if(heading_cdeg+directed_delta>=18000) heading_cdeg<=heading_cdeg+directed_delta-36000;
                                else if(heading_cdeg+directed_delta< -18000) heading_cdeg<=heading_cdeg+directed_delta+36000;
                                age<=0;update<=1;
                            end
                            phase<=READY;
                        end
                    end
                end
                default: begin
                    request_issued<=0;
                    if(!master_busy) phase<=BOOT;
                end
            endcase
        end
    end
endmodule

