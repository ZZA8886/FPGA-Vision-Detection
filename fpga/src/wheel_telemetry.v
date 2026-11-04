`timescale 1ns/1ps
// Monitor only: no changes to PI feedback. One divider serves all eight
// conversions; rpm is signed centi-rpm, using 1560 counts/rev and 200 Hz.
module wheel_telemetry (
    input wire clk,reset_n,sample_valid,
    input wire [127:0] targets_q16,
    input wire [63:0] deltas,
    input wire [47:0] efforts,
    output reg [127:0] target_rpm_centi,actual_rpm_centi,
    output reg [47:0] pwm_commands,
    output reg valid
);
    reg [127:0] saved_targets,work_targets,work_actual;
    reg [63:0] saved_deltas;
    reg [47:0] saved_efforts;
    reg [3:0] field;
    reg [2:0] state;
    reg negative,start;
    reg [31:0] numerator,denominator;
    wire busy,done;
    wire [31:0] quotient;
    unsigned_divider32 divide(clk,reset_n,start,numerator,denominator,busy,done,quotient);
    wire [1:0] wheel=field[2:1];
    wire signed [31:0] value=field[0] ?
        {{16{saved_deltas[wheel*16+15]}},saved_deltas[wheel*16 +: 16]} :
        saved_targets[wheel*32 +: 32];
    wire [31:0] magnitude=value[31]?-value:value;
    // target: Q16 counts/sample * 10000 / (13*65536) = *625/53248.
    // actual: integer counts/sample * 10000/13. Explicit shifts avoid DSPs.
    wire [31:0] scaled_target=(magnitude<<9)+(magnitude<<7)-(magnitude<<4)+magnitude;
    wire [31:0] scaled_actual=(magnitude<<13)+(magnitude<<10)+(magnitude<<9)+(magnitude<<8)+(magnitude<<4);
    wire [31:0] signed_result=negative ? -quotient : quotient;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            saved_targets<=0;saved_deltas<=0;saved_efforts<=0;work_targets<=0;work_actual<=0;
            target_rpm_centi<=0;actual_rpm_centi<=0;pwm_commands<=0;
            field<=0;state<=0;negative<=0;start<=0;numerator<=0;denominator<=1;valid<=0;
        end else begin
            start<=0;valid<=0;
            case(state)
                0:if(sample_valid) begin
                    saved_targets<=targets_q16;saved_deltas<=deltas;saved_efforts<=efforts;
                    field<=0;state<=1;
                end
                1:begin
                    numerator<=field[0]?scaled_actual:scaled_target;
                    denominator<=field[0]?32'd13:32'd53248;
                    negative<=value[31];start<=1;state<=2;
                end
                2:if(done) begin
                    if(field[0]) work_actual[wheel*32 +: 32]<=signed_result;
                    else work_targets[wheel*32 +: 32]<=signed_result;
                    if(field==7) state<=3;
                    else begin field<=field+1'b1;state<=1;end
                end
                3:begin
                    target_rpm_centi<=work_targets;actual_rpm_centi<=work_actual;
                    pwm_commands<=saved_efforts;valid<=1;state<=0;
                end
                default:state<=0;
            endcase
        end
    end
endmodule
