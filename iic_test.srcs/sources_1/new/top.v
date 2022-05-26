`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/05/23 19:40:27
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top(
    input clk,
    input rst_n,

    output scl,
    inout sda,

    output dri_clk,

    output i2c_done,
    output [7:0] data
    );

i2c_dri inst_i2c_dri
    (
        .clk                (clk),
        .rst_n              (rst_n),
        .slave_address      (7'b100_1000),
        .iic_inner_reg_addr (8'b0),
        .i2c_w_data         (8'b0),
        .i2c_r_data         (data),
        .type16_type8       (0),
        .read1_write0       (1),
        .i2c_exec           (1),
        .i2c_done           (i2c_done),
        .scl                (scl),
        .sda                (sda),
        .dri_clk            (dri_clk)
    );

endmodule
