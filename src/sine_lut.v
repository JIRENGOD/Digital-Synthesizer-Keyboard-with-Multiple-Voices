`timescale 1ns/1ps
// =============================================================================
// sine_lut.v —— 正弦查找表（只读，同步读，1 拍延迟）
// =============================================================================
module sine_lut #(
    parameter integer ADDR_W   = 12,              // 表深度 = 2^ADDR_W（>= 10，即 >= 1024 点）
    parameter integer DATA_W   = 16,              // 样本位宽（有符号）
    parameter         LUT_FILE = "sine_lut.hex"   // $readmemh 数据文件
) (
    input  wire                     clk,
    input  wire                     rd_en,        // 读使能：为 1 的下一拍 data 更新
    input  wire [ADDR_W-1:0]        addr,
    output reg  signed [DATA_W-1:0] data
);

    reg [DATA_W-1:0] rom [0:(1<<ADDR_W)-1];

    initial begin
        $readmemh(LUT_FILE, rom);
    end

    always @(posedge clk) begin
        if (rd_en)
            data <= rom[addr];
    end

endmodule
