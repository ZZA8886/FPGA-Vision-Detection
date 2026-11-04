`timescale 1ns/1ps

module imu660ra_config_rom(
    input wire [12:0] addr,
    output wire [7:0] data
);
    reg [7:0] rom [0:8191];
    initial $readmemh("src/imu660ra_config.hex",rom);
    assign data=rom[addr];
endmodule
