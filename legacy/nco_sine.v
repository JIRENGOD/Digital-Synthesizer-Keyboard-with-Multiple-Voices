`timescale 1ns/1ps
// =============================================================================
// nco_sine.v —— 数控振荡器（NCO）：32 位相位累加器 + 正弦查找表
// -----------------------------------------------------------------------------
// 频率公式（采样率 Fs = sample_en 的脉冲频率）：
//     F_out     = phase_inc * Fs / 2^32
//     phase_inc = round(F_out * 2^32 / Fs)
//   例：Fs = 48 kHz，440 Hz -> 39370534（实际 440.000005 Hz）
//                    1 kHz  -> 89478485（实际 999.999996 Hz）
//
// 关键：相位累加器虽然写在 50 MHz 的 always 块里，但只在 sample_en=1 的那一拍
//       （每个采样周期 1 次）才前进一步，因此频率严格由 Fs 决定，
//       而不是由 50 MHz 决定（否则频率会高出约 1000 倍）。
//
// 流水线（clk = 50 MHz，三拍延迟，远小于一个采样周期的 ~1042 拍）：
//   T0  sample_en=1：lut_addr <= phase_acc 高 ADDR_W 位；phase_acc += phase_inc
//   T1  sine_lut 同步读 rom[lut_addr]
//   T2  sample <= LUT 输出（enable=0 时强制为 0），sample_valid 拉高 1 拍
//
// 启动行为：enable=0 时相位保持为 0、输出 0；enable 变 1 后第一个样本是
//          sin(0)=0，波形从零点连续起振，不会产生直流台阶（爆音）。
// =============================================================================
module nco_sine #(
    parameter integer PHASE_W  = 32,              // 相位累加器位宽
    parameter integer ADDR_W   = 12,              // 查表地址位宽（取相位高 ADDR_W 位）
    parameter integer OUT_W    = 16,              // 输出样本位宽（有符号）
    parameter         LUT_FILE = "sine_lut.hex"
) (
    input  wire                    clk,           // 系统时钟（50 MHz）
    input  wire                    rst_n,         // 同步复位，低有效
    input  wire                    sample_en,     // 采样使能：每个采样周期 1 拍（Fs = 48 kHz）
    input  wire                    enable,        // 1 = 出声；0 = 相位清零、输出 0
    input  wire [PHASE_W-1:0]      phase_inc,     // 相位增量（决定频率）
    output reg  signed [OUT_W-1:0] sample,        // 16 位有符号正弦样本
    output reg                     sample_valid   // 新样本有效，1 拍脉冲
);

    reg  [PHASE_W-1:0] phase_acc;   // 相位累加器，2^32 对应一个完整周期
    reg  [ADDR_W-1:0]  lut_addr;
    reg                rd_en_s1;    // T1：读 LUT
    reg                on_s1;       // 与流水线对齐的 enable
    reg                vld_s2;      // T2：LUT 数据有效
    reg                on_s2;
    wire signed [OUT_W-1:0] lut_q;

    // ---- T0：相位累加（只在 sample_en 时前进） -----------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            phase_acc <= {PHASE_W{1'b0}};
            lut_addr  <= {ADDR_W{1'b0}};
            rd_en_s1  <= 1'b0;
            on_s1     <= 1'b0;
        end else begin
            rd_en_s1 <= sample_en;
            if (sample_en) begin
                on_s1 <= enable;
                if (enable) begin
                    // 先用当前相位查表，再前进：使能后的第一个样本对应相位 0
                    lut_addr  <= phase_acc[PHASE_W-1 -: ADDR_W];
                    phase_acc <= phase_acc + phase_inc;
                end else begin
                    lut_addr  <= {ADDR_W{1'b0}};
                    phase_acc <= {PHASE_W{1'b0}};
                end
            end
        end
    end

    // ---- T1：查表 ---------------------------------------------------------
    sine_lut #(
        .ADDR_W   (ADDR_W),
        .DATA_W   (OUT_W),
        .LUT_FILE (LUT_FILE)
    ) u_sine_lut (
        .clk   (clk),
        .rd_en (rd_en_s1),
        .addr  (lut_addr),
        .data  (lut_q)
    );

    // ---- T2：输出寄存 -----------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            vld_s2       <= 1'b0;
            on_s2        <= 1'b0;
            sample       <= {OUT_W{1'b0}};
            sample_valid <= 1'b0;
        end else begin
            vld_s2       <= rd_en_s1;
            on_s2        <= on_s1;
            sample_valid <= vld_s2;
            if (vld_s2)
                sample <= on_s2 ? lut_q : {OUT_W{1'b0}};
        end
    end

endmodule
