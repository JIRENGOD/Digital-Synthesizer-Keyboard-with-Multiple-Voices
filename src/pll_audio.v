`timescale 1ns/1ps
// =============================================================================
// pll_audio.v —— 音频时钟方案
// -----------------------------------------------------------------------------
// =============================================================================
module pll_audio #(
    parameter integer TICK_NUM = 1536,   // o_tick 频率 / 50 MHz 化简后的分子（默认 512*Fs）
    parameter integer TICK_DEN = 3125,   // 分母，要求 TICK_NUM < TICK_DEN < 2^ACC_W
    parameter integer ACC_W    = 16      // 累加器位宽
) (
    input  wire i_clk_50m,
    input  wire i_rst_n,                 // 同步复位，低有效
    output wire o_clk_audio,             // 音频逻辑使用的时钟（本版本 = 50 MHz 直通）
    output reg  o_tick                   // 时钟使能脉冲：平均 50 MHz × TICK_NUM/TICK_DEN，宽 1 个时钟周期
);

    localparam [ACC_W:0] NUM = TICK_NUM;
    localparam [ACC_W:0] DEN = TICK_DEN;

    assign o_clk_audio = i_clk_50m;

    reg  [ACC_W-1:0] acc;                       // 取值范围 [0, TICK_DEN)
    wire [ACC_W:0]   acc_sum  = {1'b0, acc} + NUM;
    wire [ACC_W:0]   acc_wrap = acc_sum - DEN;

    always @(posedge i_clk_50m) begin
        if (!i_rst_n) begin
            acc    <= {ACC_W{1'b0}};
            o_tick <= 1'b0;
        end else if (acc_sum >= DEN) begin
            acc    <= acc_wrap[ACC_W-1:0];
            o_tick <= 1'b1;
        end else begin
            acc    <= acc_sum[ACC_W-1:0];
            o_tick <= 1'b0;
        end
    end

endmodule
