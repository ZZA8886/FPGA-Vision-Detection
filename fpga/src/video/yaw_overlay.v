`timescale 1ns/1ps
// IMU heading centidegrees -> fixed-point text in the 50 MHz domain. A request/ack
// mailbox holds the entire text stable across the clock boundary. The pixel
// domain only switches the displayed string during vertical blanking.
module yaw_overlay #(
    parameter integer CLK_HZ=50000000
)(
    input wire clk,reset_n,
    input wire signed [15:0] yaw_cdeg,
    input wire yaw_fresh,
    input wire pixel_clk,pixel_reset_n,
    input wire [23:0] rgb_in,
    input wire hs_in,vs_in,de_in,
    output reg [23:0] rgb_out,
    output reg hs_out,vs_out,de_out
);
    // Bytes are stored in reading order, least significant byte first.
    localparam [95:0] INVALID_TEXT={8'd32,8'hb0,8'd45,8'd45,8'd46,
        8'd45,8'd45,8'd45,8'd58,8'd119,8'd97,8'd89};
    reg [95:0] mailbox;
    reg request;
    reg acknowledge;
    (* syn_preserve=1 *) reg ack_meta,ack_sync;
    (* syn_preserve=1 *) reg req_meta,req_sync;
    reg [1:0] state;
    reg signed [15:0] last_yaw;
    reg last_fresh,started,negative;
    reg [15:0] binary;
    reg [19:0] bcd,adjusted;
    reg [4:0] bits_left;
    reg [31:0] refresh_count;
    reg [95:0] formatted;
    integer i;
    always @* begin
        adjusted=bcd;
        for(i=0;i<5;i=i+1)
            if(bcd[i*4 +: 4]>=5) adjusted[i*4 +: 4]=bcd[i*4 +: 4]+4'd3;
        formatted=INVALID_TEXT;
        if(last_fresh) begin
            formatted[4*8 +: 8]=negative?8'd45:8'd43;
            if(bcd[19:16]!=0) begin
                formatted[5*8 +: 8]=8'd48+{4'd0,bcd[19:16]};
                formatted[6*8 +: 8]=8'd48+{4'd0,bcd[15:12]};
                formatted[7*8 +: 8]=8'd48+{4'd0,bcd[11:8]};
                formatted[8*8 +: 8]=8'd46;
                formatted[9*8 +: 8]=8'd48+{4'd0,bcd[7:4]};
                formatted[10*8 +: 8]=8'd48+{4'd0,bcd[3:0]};
                formatted[11*8 +: 8]=8'hb0;
            end else begin
                formatted[5*8 +: 8]=8'd48+{4'd0,bcd[15:12]};
                formatted[6*8 +: 8]=8'd48+{4'd0,bcd[11:8]};
                formatted[7*8 +: 8]=8'd46;
                formatted[8*8 +: 8]=8'd48+{4'd0,bcd[7:4]};
                formatted[9*8 +: 8]=8'd48+{4'd0,bcd[3:0]};
                formatted[10*8 +: 8]=8'hb0;
                formatted[11*8 +: 8]=8'd32;
            end
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            ack_meta<=0;ack_sync<=0;request<=0;mailbox<=INVALID_TEXT;
            state<=0;last_yaw<=0;last_fresh<=0;started<=0;negative<=0;
            binary<=0;bcd<=0;bits_left<=0;
            refresh_count<=0;
        end else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;
            if(refresh_count!=0) refresh_count<=refresh_count-1'b1;
            case(state)
                // Periodic refresh also recovers after an independent HDMI PLL
                // reset, including when request and reset acknowledge are both 0.
                0:if(request==ack_sync && (!started || refresh_count==0 || yaw_cdeg!=last_yaw || yaw_fresh!=last_fresh)) begin
                    started<=1;last_yaw<=yaw_cdeg;last_fresh<=yaw_fresh;
                    negative<=yaw_cdeg[15];binary<=yaw_cdeg[15]?-yaw_cdeg:yaw_cdeg;
                    bcd<=0;bits_left<=15;state<=1;refresh_count<=CLK_HZ/20;
                end
                1:begin
                    bcd<={adjusted[18:0],binary[15]};binary<=binary<<1;
                    if(bits_left==0) state<=2;else bits_left<=bits_left-1'b1;
                end
                2:begin mailbox<=formatted;request<=~request;state<=0;end
                default:state<=0;
            endcase
        end
    end
    reg [95:0] pixel_text,frame_text;
    reg vs_previous,de_previous;
    reg [11:0] x;
    reg [10:0] y;
    // Original font: 4x4 pixels per dot, 32x32 character cells.
    // New font: 12x12 pixels per dot, 96x96 cells (exactly 3x each axis).
    // Counters avoid pixel-rate division by 12/96 and additional DSP usage.
    reg [3:0] dot_x,dot_y,character_index;
    reg [2:0] font_column,font_row;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n) begin
            req_meta<=0;req_sync<=0;acknowledge<=0;
            pixel_text<=INVALID_TEXT;frame_text<=INVALID_TEXT;
            vs_previous<=0;de_previous<=0;x<=0;y<=0;
            dot_x<=0;dot_y<=0;character_index<=0;font_column<=0;font_row<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;
            if(req_sync!=acknowledge) begin
                pixel_text<=mailbox;acknowledge<=req_sync;
            end
            vs_previous<=vs_in;de_previous<=de_in;
            if(vs_in && !vs_previous) frame_text<=pixel_text;
            if(!de_in) x<=0;else x<=x+1'b1;
            if(vs_in) y<=0;else if(de_previous && !de_in) y<=y+1'b1;
            if(!de_in) begin
                dot_x<=0;font_column<=0;character_index<=0;
            end else if(x>=16 && x<1168) begin
                if(dot_x==11) begin
                    dot_x<=0;font_column<=font_column+1'b1;
                    if(font_column==7) character_index<=character_index+1'b1;
                end else dot_x<=dot_x+1'b1;
            end
            if(vs_in) begin dot_y<=0;font_row<=0;end
            else if(de_previous && !de_in && y>=16 && y<112) begin
                if(dot_y==11) begin dot_y<=0;font_row<=font_row+1'b1;end
                else dot_y<=dot_y+1'b1;
            end
        end
    end
    function [39:0] glyph;
        input [7:0] ch;
        begin case(ch)
            48:glyph=40'h3e4549513e;49:glyph=40'h00407f4200;
            50:glyph=40'h4649516142;51:glyph=40'h314b454121;
            52:glyph=40'h107f121418;53:glyph=40'h3945454527;
            54:glyph=40'h3049494a3c;55:glyph=40'h0305097101;
            56:glyph=40'h3649494936;57:glyph=40'h1e29494906;
            89:glyph=40'h0708700807;97:glyph=40'h7854545420;
            119:glyph=40'h3c4030403c;58:glyph=40'h0000363600;
            43:glyph=40'h08083e0808;45:glyph=40'h0808080808;
            46:glyph=40'h0000606000;176:glyph=40'h0006090906;
            default:glyph=0;
        endcase end
    endfunction
    // Continuous shadow/midtone lift with highlight compression:
    // y = x + floor(min(x,255-x)/2), black=0, white=255, no clipping.
    // Three 8-bit adders, no multipliers, extra RAM, or extra video latency.
    function [7:0] brighten;
        input [7:0] value;
        reg [7:0] headroom;
        begin
            headroom=value[7]?~value:value;
            brighten=value+{1'b0,headroom[7:1]};
        end
    endfunction
    reg [7:0] character;
    reg [2:0] column,row0,row1;
    reg box0,box1,hs0,vs0,de0,hs1,vs1,de1;
    reg [23:0] rgb0,rgb1;
    reg [7:0] glyph_column;
    wire [39:0] glyph_bits=glyph(character);
    wire [39:0] shifted=glyph_bits>>(column*8);
    // Three equal pipeline stages for colour and all video timing signals.
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n) begin
            character<=32;column<=0;row0<=0;row1<=0;box0<=0;box1<=0;
            rgb0<=0;rgb1<=0;glyph_column<=0;rgb_out<=0;
            hs0<=0;vs0<=0;de0<=0;hs1<=0;vs1<=0;de1<=0;
            hs_out<=0;vs_out<=0;de_out<=0;
        end else begin
            character<=frame_text[character_index*8 +: 8];column<=font_column;row0<=font_row;
            box0<=de_in && x>=16 && x<1168 && y>=16 && y<112;
            rgb0<={brighten(rgb_in[23:16]),brighten(rgb_in[15:8]),brighten(rgb_in[7:0])};
            hs0<=hs_in;vs0<=vs_in;de0<=de_in;
            glyph_column<=column<5?shifted[7:0]:8'd0;
            row1<=row0;box1<=box0;rgb1<=rgb0;hs1<=hs0;vs1<=vs0;de1<=de0;
            rgb_out<=box1?(glyph_column[row1]?24'hffff00:{1'b0,rgb1[23:17],1'b0,rgb1[15:9],1'b0,rgb1[7:1]}):rgb1;
            hs_out<=hs1;vs_out<=vs1;de_out<=de1;
        end
    end
endmodule
