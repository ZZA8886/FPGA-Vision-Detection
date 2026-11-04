`timescale 1ns/1ps
// Four-edge decoder, 16-bit modulo count; positive when A leads B.
module quadrature_encoder #(
    parameter integer FILTER_TICKS = 8,
    parameter integer INVERT = 0
) (
    input wire clk,
    input wire reset_n,
    input wire phase_a,
    input wire phase_b,
    input wire sample_tick,
    output reg [15:0] count,
    output reg signed [15:0] delta,
    output reg sample_valid,
    output reg [15:0] invalid_transitions
);
    (* syn_preserve = 1 *) reg [1:0] a_sync, b_sync;
    reg [1:0] stable_ab, previous_ab, candidate_ab;
    reg [15:0] filter_count;
    reg [15:0] previous_count;
    reg [15:0] startup_count;
    reg initialized;
    reg signed [1:0] step;
    wire [1:0] incoming_ab = {a_sync[1], b_sync[1]};
    always @* begin
        case ({previous_ab, stable_ab})
            4'b0010, 4'b1011, 4'b1101, 4'b0100: step = INVERT ? -2'sd1 : 2'sd1;
            4'b0001, 4'b0111, 4'b1110, 4'b1000: step = INVERT ? 2'sd1 : -2'sd1;
            default: step = 0;
        endcase
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            a_sync <= 0; b_sync <= 0;
            stable_ab <= 0; previous_ab <= 0; candidate_ab <= 0;
            filter_count <= 0;
            count <= 0; previous_count <= 0;
            delta <= 0; sample_valid <= 0;
            invalid_transitions <= 0;
            startup_count <= 0; initialized <= 0;
        end else begin
            a_sync <= {a_sync[0], phase_a};
            b_sync <= {b_sync[0], phase_b};
            // Only the second synchronizer stage feeds functional logic.
            if (incoming_ab == stable_ab) filter_count <= 0;
            else if (incoming_ab != candidate_ab) begin
                candidate_ab <= incoming_ab;
                if(FILTER_TICKS==1) begin stable_ab<=incoming_ab;filter_count<=0;end
                else filter_count <= 1;
            end
            else if (filter_count == FILTER_TICKS - 1) begin
                stable_ab <= incoming_ab;
                filter_count <= 0;
            end else filter_count <= filter_count + 1'b1;
            previous_ab <= stable_ab;
            // Settle the actual power-on AB state before decoding motion.
            if(!initialized) begin
                if(startup_count==FILTER_TICKS+5) initialized<=1;
                else startup_count<=startup_count+1'b1;
            end
            if (initialized && step == 1) count <= count + 1'b1;
            else if (initialized && step == -1) count <= count - 1'b1;
            if (initialized && (previous_ab ^ stable_ab) == 2'b11)
                invalid_transitions <= invalid_transitions + 1'b1;
            sample_valid <= sample_tick;
            if (sample_tick) begin
                // Assignment truncates to 16 bits before signed interpretation:
                // valid for |net motion per sample| < 32768 counts.
                delta <= count - previous_count;
                previous_count <= count;
            end
        end
    end
endmodule
