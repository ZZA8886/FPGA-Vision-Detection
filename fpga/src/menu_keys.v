`timescale 1ns/1ps
module menu_keys #(
    parameter integer CLK_HZ=50000000,
    parameter integer DEBOUNCE_MS=10,
    parameter integer LONG_MS=800,
    parameter integer RESET_MS=1500
) (
    input wire clk,reset_n,
    input wire [2:0] key_n,
    output reg previous,next,confirm,stop,reset_request
);
    (* syn_preserve=1 *) reg [2:0] sync1,sync2;
    localparam integer MS_BITS=$clog2(CLK_HZ/1000)>0?$clog2(CLK_HZ/1000):1;
    localparam integer HOLD_LIMIT=LONG_MS>500?LONG_MS:500;
    localparam integer HOLD_BITS=$clog2(HOLD_LIMIT+1)>0?$clog2(HOLD_LIMIT+1):1;
    localparam integer DEBOUNCE_BITS=$clog2(DEBOUNCE_MS)>0?$clog2(DEBOUNCE_MS):1;
    localparam integer CHORD_BITS=$clog2(RESET_MS)>0?$clog2(RESET_MS):1;
    reg [MS_BITS-1:0] ms_counter;
    reg [2:0] pressed,long_sent;
    reg [DEBOUNCE_BITS-1:0] debounce[0:2];
    reg [HOLD_BITS-1:0] held[0:2];
    reg [6:0] repeat_count[0:1];
    reg [CHORD_BITS-1:0] chord_ms;
    integer k;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            sync1<=7;sync2<=7;ms_counter<=0;pressed<=0;long_sent<=0;
            previous<=0;next<=0;confirm<=0;stop<=0;reset_request<=0;chord_ms<=0;
            for(k=0;k<3;k=k+1) begin debounce[k]<=0;held[k]<=0;end
            repeat_count[0]<=0;repeat_count[1]<=0;
        end else begin
            sync1<=key_n;sync2<=sync1;
            previous<=0;next<=0;confirm<=0;stop<=0;reset_request<=0;
            if(ms_counter==CLK_HZ/1000-1) begin
                ms_counter<=0;
                for(k=0;k<3;k=k+1) begin
                    if((!sync2[k])==pressed[k]) debounce[k]<=0;
                    else if(debounce[k]==DEBOUNCE_MS-1) begin
                        debounce[k]<=0;pressed[k]<=!sync2[k];
                        if(!sync2[k]) begin
                            held[k]<=0;long_sent[k]<=0;
                            if(k==0 && !pressed[1]) previous<=1;
                            if(k==1 && !pressed[0]) next<=1;
                        end else if(k==2 && !long_sent[k]) confirm<=1;
                    end else debounce[k]<=debounce[k]+1'b1;
                    if(pressed[k]) begin
                        if(held[k]<HOLD_LIMIT) held[k]<=held[k]+1'b1;
                        if(k==2 && held[k]>=LONG_MS-1 && !long_sent[k]) begin stop<=1;long_sent[k]<=1;end
                        if(k<2) begin
                            if(held[k]>=500) begin
                                if(repeat_count[k]==79) begin
                                    repeat_count[k]<=0;
                                    if(!pressed[1-k]) begin
                                        if(k==0) previous<=1;else next<=1;
                                    end
                                end else repeat_count[k]<=repeat_count[k]+1'b1;
                            end
                        end
                    end else begin
                        held[k]<=0;
                        if(k<2) repeat_count[k]<=0;
                    end
                end
                if(pressed[0] && pressed[1]) begin
                    if(chord_ms==RESET_MS-1) begin reset_request<=1;chord_ms<=0;end
                    else chord_ms<=chord_ms+1'b1;
                end else chord_ms<=0;
            end else ms_counter<=ms_counter+1'b1;
        end
    end
endmodule
