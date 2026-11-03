`timescale 1ns/1ps
// =============================================================================
// pll_audio.v —— 音频时钟方案（里程碑 1：不例化 PLL 原语）
// -----------------------------------------------------------------------------
// 为什么模块叫 pll_audio 却没有 PLL？
//   保留这个模块位置是为了以后换真正的 PLL 时只改这里，顶层不动。本版本不用 PLL：
//   1) GW5A 系列（Arora V，包括 GW5AT-60、GW5AST-138）没有 rPLL 原语（rPLL 只属于 GW1N/GW2A），
//      对应的是 PLLA 原语，参数和端口完全不同；
//   2) 云源 IP 生成器给 GW5A 系列生成 PLL 时，还会附带一段 PLL_INIT 校准状态机
//      （安装目录 IDE/ipcore/PLL_ADV/data/PLLA/pll_init.v），手写裸原语难以保证可靠锁定；
//   3) 12.288 MHz / 50 MHz = 768/3125，整数分频 PLL 本来也得不到精确值。
//
// 采用的方案：分数分频“时钟使能”（单一 50 MHz 时钟域，无跨时钟域问题）
//   累加器每拍加 TICK_NUM，满 TICK_DEN 回绕并输出 1 拍 o_tick。
//   o_tick 的长期平均频率 = 50 MHz × TICK_NUM / TICK_DEN，精确无累积误差；
//   相邻 tick 间隔为 floor/ceil(TICK_DEN/TICK_NUM) 拍，即边沿抖动 ≤ 1 个系统时钟（20 ns）。
//
//   o_tick 的目标频率为 512 × Fs（一帧 = 512 个 tick），i2s_master 由它派生
//   MCLK = 256Fs、BCK = 64Fs、LRCK = Fs：
//
//   CLK_MODE=0（默认，精确 48 kHz）：
//       512 × 48 kHz = 24.576 MHz；24.576 / 50 = 1536 / 3125
//       -> Fs = 48 000 Hz（平均），MCLK 12.288 MHz，BCK 3.072 MHz，边沿抖动 ≤ 20 ns
//       适合不需要 MCLK 的 DAC（如 PCM5102A 的 SCK 接地模式、PT8211、MAX98357A）。
//   CLK_MODE=1（整数分频，零附加抖动）：
//       TICK_NUM/TICK_DEN = 1/2 -> tick = 25 MHz
//       -> Fs = 48 828.125 Hz，MCLK 12.5 MHz（=256Fs 且与 LRCK 严格同步），BCK 3.125 MHz
//       DAC 必须使用 MCLK（如 CS4344、WM8978、ES8388 等）时请用这个模式。
//
// 以后若改用真正的 PLL：o_clk_audio 改为 PLL 输出（如 24.576 MHz），
// TICK_NUM/TICK_DEN 改为对应的整数比，并把 PLL 的 lock 信号并入顶层复位。
//
// 钢琴工程 top_piano_do 复用本模块：TICK_NUM/TICK_DEN = 204/3125，
//   o_tick 平均 3.264 MHz = 68 × 48 kHz（PT8211 每声道 17 个 BCK，每个 BCK 两个 tick）。
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
