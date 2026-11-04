`timescale 1ns/1ps
// ASCII commands with XOR checksum: SET id value*HH\n, MOVE vx vy wz*HH\n,
// STOP*HH\n and PING*HH\n. HH is XOR of all bytes preceding '*'.
module remote_command # (parameter integer CLK_HZ=50000000) (
    input wire clk,reset_n,rx,output wire tx,
    output reg [15:0] receive_count,
    output wire [7:0] receive_data,output wire receive_valid,framing_error,
    output reg command_valid,output reg [5:0] command_id,
    output reg signed [31:0] command_value,output reg move_valid,
    output reg signed [15:0] move_vx,move_vy,move_wz,
    input wire [7:0] tx_data,
    input wire tx_valid,output wire tx_ready,
    output reg heartbeat,output reg [15:0] accepted,rejected
);
    uart_8n1 #(.CLK_HZ(CLK_HZ)) uart(clk,reset_n,rx,tx,tx_data,tx_valid,tx_ready,
        receive_data,receive_valid,framing_error);
    localparam CMD=0,GAP=1,NUM=2,CS1=3,CS2=4,ENDLINE=5,DROP=6;
    reg [2:0] state,op;
    reg [31:0] keyword;
    reg [2:0] letters,digits;
    reg [1:0] count;
    reg [31:0] magnitude;
    reg negative;
    reg signed [31:0] a,b,c;
    reg [7:0] checksum,received_checksum;
    localparam integer AGE_BITS=$clog2(CLK_HZ/5)>0?$clog2(CLK_HZ/5):1;
    reg [AGE_BITS-1:0] age;
    wire space=receive_data==32;
    wire digit=receive_data>=48 && receive_data<=57;
    wire hexok=digit || (receive_data>=65 && receive_data<=70);
    wire [3:0] hexval=digit ? receive_data[3:0] : receive_data[3:0]+4'd9;
    wire signed [31:0] number=negative ? -$signed(magnitude) : $signed(magnitude);
    function valid_set;
        input signed [31:0] id,v;
        begin
            valid_set=0;
            case(id)
                0,10,20,23,29: valid_set=v==0 || v==1;
                1: valid_set=v>=0 && v<=11;
                2: valid_set=v>=0 && v<=1000;
                3,28: valid_set=v>=0 && v<=3000;
                4,5: valid_set=v>=-1000 && v<=1000;
                6: valid_set=v>=-3000 && v<=3000;
                7,8: valid_set=v>=0 && v<=18000;
                9: valid_set=v==1 || v==10 || v==100;
                11: valid_set=v>=1 && v<=4;
                12: valid_set=v>=-1000 && v<=1000;
                13,14: valid_set=v>=0 && v<=15;
                15,16: valid_set=v>=0 && v<=65280;
                17: valid_set=v>=0 && v<=2000;
                18,30: valid_set=v==1;
                24: valid_set=v>=-18000 && v<=18000;
                25: valid_set=v>=0 && v<=100000;
                26,27: valid_set=v>=0 && v<=10000;
                32,33: valid_set=v>=0 && v<=36000;
                34: valid_set=v>=1 && v<=10000;
                35: valid_set=v>=1 && v<=1000;
                36: valid_set=v>=1 && v<=1000;
                37: valid_set=v>=100 && v<=5000;
                38: valid_set=v>=1 && v<=100;
                39: valid_set=v>=1 && v<=2000;
                40: valid_set=v==1 || v==2;
                41,46,47: valid_set=v==0 || v==1;
                42: valid_set=v>=0 && v<=100;
                43: valid_set=v>=0 && v<=255;
                44: valid_set=v>=1 && v<=1000;
                45: valid_set=v>=100 && v<=2000;
            endcase
        end
    endfunction
    task store_number;
        begin
            if(count==0) a<=number;else if(count==1) b<=number;else c<=number;
            count<=count+1'b1;
        end
    endtask
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=CMD;op<=0;keyword<=0;letters<=0;digits<=0;count<=0;
            magnitude<=0;negative<=0;a<=0;b<=0;c<=0;checksum<=0;received_checksum<=0;
            receive_count<=0;command_valid<=0;move_valid<=0;command_id<=0;command_value<=0;
            move_vx<=0;move_vy<=0;move_wz<=0;heartbeat<=0;accepted<=0;rejected<=0;age<=0;
        end else begin
            command_valid<=0;move_valid<=0;heartbeat<=0;
            // Abandon a partial line after 200 ms (allows wireless packet gaps).
            if(receive_valid) age<=0;
            else if(state!=CMD || letters!=0) begin
                if(age==CLK_HZ/5-1) begin state<=DROP;end
                else age<=age+1'b1;
            end
            if(framing_error) state<=DROP;
            else if(receive_valid) begin
                receive_count<=receive_count+1'b1;
                if(receive_data==10) begin
                    if(state==ENDLINE && checksum==received_checksum) begin
                        if(op==1 && count==2 && valid_set(a,b)) begin
                            command_valid<=1;command_id<=a[5:0];command_value<=b;accepted<=accepted+1'b1;
                        end else if(op==2 && count==3 && a>=-1000 && a<=1000 && b>=-1000 && b<=1000 && c>=-3000 && c<=3000) begin
                            move_valid<=1;move_vx<=a[15:0];move_vy<=b[15:0];move_wz<=c[15:0];accepted<=accepted+1'b1;
                        end else if(op==3 && count==0) begin
                            command_valid<=1;command_id<=0;command_value<=0;accepted<=accepted+1'b1;
                        end else if(op==4 && count==0) begin heartbeat<=1;end
                        else rejected<=rejected+1'b1;
                    end else rejected<=rejected+1'b1;
                    state<=CMD;keyword<=0;letters<=0;count<=0;checksum<=0;digits<=0;age<=0;
                end else if(receive_data==13 && state==ENDLINE) begin end
                else begin
                    if(state<CS1 && receive_data!=42) checksum<=checksum^receive_data;
                    case(state)
                        CMD: begin
                            if(space || receive_data==42) begin
                                if(keyword==32'h00534554 && space) begin op<=1;state<=GAP;end
                                else if(keyword==32'h4d4f5645 && space) begin op<=2;state<=GAP;end
                                else if(keyword==32'h53544f50 && receive_data==42) begin op<=3;state<=CS1;end
                                else if(keyword==32'h50494e47 && receive_data==42) begin op<=4;state<=CS1;end
                                else state<=DROP;
                            end else if(letters<4 && receive_data>=65 && receive_data<=90) begin
                                keyword<={keyword[23:0],receive_data};letters<=letters+1'b1;
                            end else state<=DROP;
                        end
                        GAP: begin
                            if(space) begin end
                            else if(receive_data==42 && count!=0) state<=CS1;
                            else if(count<3 && (digit || receive_data==45 || receive_data==43)) begin
                                magnitude<=digit ? receive_data-48 : 0;digits<=digit ? 1 : 0;
                                negative<=receive_data==45;state<=NUM;
                            end else state<=DROP;
                        end
                        NUM: begin
                            if(digit && digits<6) begin magnitude<=(magnitude<<3)+(magnitude<<1)+(receive_data-48);digits<=digits+1'b1;end
                            else if((space || receive_data==42) && digits!=0) begin
                                store_number;state<=receive_data==42 ? CS1 : GAP;
                            end else state<=DROP;
                        end
                        CS1: if(hexok) begin received_checksum<={hexval,4'b0};state<=CS2;end else state<=DROP;
                        CS2: if(hexok) begin received_checksum[3:0]<=hexval;state<=ENDLINE;end else state<=DROP;
                        ENDLINE: state<=DROP;
                        default: state<=DROP;
                    endcase
                end
            end
        end
    end
endmodule
