`timescale 1ns/1ps
// =============================================================================
// sine_lut.v —— 正弦查找表（只读，同步读，1 拍延迟）
// -----------------------------------------------------------------------------
// 内容：rom[i] = round(sin(2*pi*i/N) * 32767)，N = 2^ADDR_W，16 位二进制补码。
//       由 scripts/gen_sine_lut.py 生成 sine_lut.hex，这里用 $readmemh 加载。
//
// 实现要点：
//   * 用 reg 数组 + 同步读（always @(posedge clk)）描述，GowinSynthesis 会把它
//     推断成 BSRAM（pROM），4096x16 = 64 Kbit 约占 4 块 BSRAM（GW5AST-138 共 340 块）。
//   * initial + $readmemh 只用于给 ROM 赋初值（这是唯一允许的 initial 用法），
//     不会初始化任何其他可综合寄存器。
//   * 输出寄存器不加复位，便于映射进 BSRAM 的输出寄存器；
//     上电/复位期间的静音由上层 nco_sine 负责（输出门控为 0）。
//
// LUT_FILE 路径说明：
//   GowinSynthesis 以调用 $readmemh 的这个 .v 文件所在目录（src/）为起点解析相对路径，
//   所以默认的 "sine_lut.hex" 就是 src/sine_lut.hex。找不到文件时只报 EX3988 警告、不报错，
//   但 ROM 全为 0（钢琴只剩琴槌噪声）；遇到这种情况把 LUT_FILE 改成绝对路径，例如
//   "C:/Users/xxx/music_piano/src/sine_lut.hex"（用正斜杠）。
//   仿真时由 testbench 通过参数传入相对于仿真目录的路径。
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
