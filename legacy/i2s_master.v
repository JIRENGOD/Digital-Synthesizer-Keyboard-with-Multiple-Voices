`timescale 1ns/1ps
// =============================================================================
// i2s_master.v —— I2S 主模式发送器（产生 MCLK / BCK / LRCK / SDATA）
// -----------------------------------------------------------------------------
// 时钟关系（全部由 tick = 512*Fs 的时钟使能派生，彼此严格成整数比）：
//     MCLK = 256*Fs   每 1 个 tick 翻转    -> tcnt[0]
//     BCK  =  64*Fs   每 4 个 tick 翻转    -> tcnt[2]
//     LRCK =     Fs   每 256 个 tick 翻转  -> tcnt[8]
//   tcnt[8:0] 是帧内位置：tcnt[8] = 声道，tcnt[7:3] = 槽内 bit 序号，tcnt[2:0] = BCK 内相位。
//
// 帧格式：BCK = 64Fs，每声道 32 个 BCK；data_l/data_r 为 24 位有符号数
//        （顶层送入的是 {16 位样本, 8'h00}，即左对齐 16 位有效数据 + 低 8 位补零），MSB 先发。
//
// FORMAT = 0：标准 I2S（Philips），LRCK=0 为左声道，MSB 比 LRCK 边沿晚 1 个 BCK：
//
//          |<-------------- 左声道 32 BCK -------------->|<------ 右声道 32 BCK ------
//   LRCK  ‾‾\_____________________________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
//   BCK   _/‾\_/‾\_/‾\_/‾\_/‾\_ ... _/‾\_/‾\_ ... _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_ ...
//   SDATA ==X 0 X L23X L22X L21X ... X L0 X 0 ... 0 X 0 X R23X R22X R21X ...
//              ^ 空 1 位   ^ MSB              ^ 24 位后补 0
//
//   SDATA、LRCK 在 BCK 下降沿更新，在 BCK 上升沿处稳定，DAC 在上升沿采样
//   （建立/保持余量均为半个 BCK 周期，>= 160 ns）。
//
// FORMAT = 1：左对齐（MSB-justified），LRCK=1 为左声道，MSB 紧跟 LRCK 边沿。
// FORMAT = 2：右对齐 16 位（LSB-justified / “日本格式”，如 PT8211），LRCK=1 为左声道，
//             16 位数据的 LSB 落在槽的最后一个 BCK。
//   如果板载 DAC 只认左/右对齐格式，改这个参数即可，其他模块不用动。
//
// 采样交接：每帧开始（tcnt 511 -> 0 的 BCK 下降沿）同时锁存 data_l 和 data_r，
//          下一拍发出 sample_req，请求上游（NCO）计算下一帧的样本。
//          复位后第一帧发送全 0（静音）。
// =============================================================================
module i2s_master #(
    parameter integer DATA_W = 24,       // data_l / data_r 位宽，16..24
    parameter integer FORMAT = 0         // 0 = 标准 I2S，1 = 左对齐，2 = 右对齐 16 位
) (
    input  wire                     clk,          // 系统时钟（50 MHz）
    input  wire                     rst_n,        // 同步复位，低有效
    input  wire                     tick,         // 时钟使能，平均 512*Fs
    input  wire signed [DATA_W-1:0] data_l,       // 左声道样本（在帧开始时被锁存）
    input  wire signed [DATA_W-1:0] data_r,       // 右声道样本
    output reg                      sample_req,   // 每帧 1 拍脉冲（频率 = Fs）
    output reg                      o_mclk,       // 256*Fs
    output reg                      o_bck,        // 64*Fs
    output reg                      o_lrck,       // Fs
    output reg                      o_sdata
);

    // 左声道对应的 LRCK 电平：标准 I2S 为 0，左/右对齐格式为 1
    localparam LRCK_LEFT = (FORMAT == 0) ? 1'b0 : 1'b1;

    // 32 位声道槽内容（bit31 最先发出）
    function [31:0] slot_bits;
        input [31:0] w;                  // 已左对齐到 32 位的样本
        begin
            case (FORMAT)
                1:       slot_bits = w;                             // 左对齐
                2:       slot_bits = {{16{w[31]}}, w[31:16]};       // 右对齐 16 位
                default: slot_bits = {1'b0, w[31:1]};               // 标准 I2S：延迟 1 位
            endcase
        end
    endfunction

    wire [31:0] slot_l = slot_bits({data_l, {(32-DATA_W){1'b0}}});
    wire [31:0] slot_r = slot_bits({data_r, {(32-DATA_W){1'b0}}});

    reg  [8:0]  tcnt;       // 帧内位置（单位：tick）
    reg  [62:0] shreg;      // 本帧还没发出的 63 位
    wire [8:0]  tcnt_nx   = tcnt + 9'd1;
    wire        bit_end   = (tcnt[2:0] == 3'b111);           // 再来一个 tick 就是 BCK 下降沿
    wire        frame_end = bit_end && (tcnt[8:3] == 6'd63); // 再来一个 tick 就是新帧

    always @(posedge clk) begin
        if (!rst_n) begin
            tcnt       <= 9'd0;
            shreg      <= 63'd0;
            o_mclk     <= 1'b0;
            o_bck      <= 1'b0;
            o_lrck     <= LRCK_LEFT;
            o_sdata    <= 1'b0;
            sample_req <= 1'b0;
        end else begin
            sample_req <= tick && frame_end;
            if (tick) begin
                tcnt   <= tcnt_nx;
                o_mclk <= tcnt_nx[0];
                o_bck  <= tcnt_nx[2];
                o_lrck <= tcnt_nx[8] ^ LRCK_LEFT;
                if (bit_end) begin                    // BCK 下降沿：送出下一位
                    if (frame_end)
                        {o_sdata, shreg} <= {slot_l, slot_r};   // 新帧：同时锁存左右声道
                    else
                        {o_sdata, shreg} <= {shreg, 1'b0};
                end
            end
        end
    end

endmodule
