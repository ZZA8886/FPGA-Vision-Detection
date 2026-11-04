`timescale 1ns/1ps
// Assert even when the pixel clock has stopped; release after three clean
// pixel edges. Keep the clock divider on the upstream reset to avoid deadlock.
module video_reset_release(
    input wire pixel_clk,
    input wire async_reset_n,
    output wire pixel_reset_n
);
    reg [2:0] release_pipe = 3'b000;
    always @(posedge pixel_clk or negedge async_reset_n)
        if (!async_reset_n) release_pipe <= 3'b000;
        else release_pipe <= {release_pipe[1:0],1'b1};
    assign pixel_reset_n = release_pipe[2];
endmodule
