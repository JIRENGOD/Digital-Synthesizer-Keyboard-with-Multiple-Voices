`timescale 1ns/1ps
// =============================================================================
// i2s_dac_tx.v —— PT8211 串行音频发送器（照官方例程 audio_drive 的格式）
// -----------------------------------------------------------------------------
// 官方例程 audio_sinewave / audio_music 的 audio_driver.v 的做法：
//   * 每帧 32 个 BCK：左、右声道各 16 个，HP_WS = 0 为左声道、1 为右声道；
//   * 16 位数据 MSB 先发，WS 跳变的同一时刻送出 MSB（不像标准 I2S 那样晚 1 位），
//     所以 16 位正好填满 WS 的半个周期；
//   * PA_EN 常为 1（功放常开，在顶层处理）。
// 本模块照搬这个格式，只有两点不同：
//   1) 官方把分频得到的信号直接当时钟用；这里全部逻辑都在 50 MHz 下，用 tick 时钟使能；
//   2) 官方 BCK 实际约 0.74 MHz；这里 BCK = 50 MHz / 32 = 1.5625 MHz（整数分频，无抖动），
//      采样率 Fs = BCK / 32 = 48 828.125 Hz，钢琴参数表按这个采样率生成。
//   另外，WS、DIN 在 BCK 上升沿之后 1/4 个 BCK 周期（160 ns）才变化，
//   不管 DAC 在 BCK 上升沿还是下降沿采样，都有 ≥ 160 ns 的余量，采到的位序列与官方完全一样。
//
// 时序：tick = 4 × BCK = 6.25 MHz（每 8 个 50 MHz 时钟 1 拍），一帧 128 个 tick，
//   q[6:2] = 位序号 n（0..31），q[1:0] = 位内的 1/4 周期：
//     q[1:0] = 0：BCK ↑     q[1:0] = 1：送出第 n 位（WS、DIN 同时更新）     q[1:0] = 2：BCK ↓
//     n = 0..15 ：WS = 0，DIN = 左声道 D15..D0
//     n = 16..31：WS = 1，DIN = 右声道 D15..D0
//   n = 0 送 MSB 的同时锁存 data_l、data_r，并发出 1 拍 sample_req 请求上游计算下一帧样本
//   （上游须在一帧 1024 个时钟内给出；钢琴引擎约需 400 个时钟）。
// =============================================================================
module i2s_dac_tx (
    input  wire               clk,          // 50 MHz
    input  wire               rst_n,        // 同步复位，低有效
    input  wire               tick,         // 时钟使能，4 × BCK = 6.25 MHz
    input  wire signed [15:0] data_l,
    input  wire signed [15:0] data_r,
    output reg                sample_req,   // 每帧 1 拍（48.828 kHz）
    output reg                o_bck,
    output reg                o_lrck,       // 0 = 左声道
    output reg                o_sdata
);

    reg [6:0]  q;             // 帧内位置（单位：tick）
    reg [15:0] shreg;         // 当前声道还没送出的位
    reg [15:0] hold_r;        // 本帧右声道数据

    always @(posedge clk) begin
        if (!rst_n) begin
            q          <= 7'd0;
            shreg      <= 16'd0;
            hold_r     <= 16'd0;
            o_bck      <= 1'b0;
            o_lrck     <= 1'b0;
            o_sdata    <= 1'b0;
            sample_req <= 1'b0;
        end else begin
            sample_req <= 1'b0;
            if (tick) begin
                q <= q + 7'd1;
                case (q[1:0])
                2'd0: o_bck <= 1'b1;                          // BCK ↑
                2'd1: begin                                   // 送出第 q[6:2] 位
                    if (q[6:2] == 5'd0) begin                 // 新的一帧：左声道 MSB
                        o_lrck     <= 1'b0;
                        o_sdata    <= data_l[15];
                        shreg      <= {data_l[14:0], 1'b0};
                        hold_r     <= data_r;
                        sample_req <= 1'b1;
                    end else if (q[6:2] == 5'd16) begin       // 右声道 MSB
                        o_lrck  <= 1'b1;
                        o_sdata <= hold_r[15];
                        shreg   <= {hold_r[14:0], 1'b0};
                    end else begin
                        o_sdata <= shreg[15];
                        shreg   <= {shreg[14:0], 1'b0};
                    end
                end
                2'd2: o_bck <= 1'b0;                          // BCK ↓
                default: ;
                endcase
            end
        end
    end

endmodule
