`timescale 1ns/1ps
// =============================================================================
// gtr_amp.v —— 电吉他音箱：过载失真 + 音箱频响模拟（只处理吉他声部之和，钢琴不经过这里）
// -----------------------------------------------------------------------------
//   u = 输入 × DRIVE，限制在 ±1（1.0 = 32768）
//   y = 1.5·u - 0.5·u³                       三次软削波：小信号近似放大，大信号圆滑地压到 ±1，
//                                             类似电子管过载，削波越深谐波越多、余音越长（压缩感）
//   两级一阶低通（各约 4 kHz）                 模拟吉他音箱喇叭，去掉失真产生的刺耳高频
//   输出 = 结果 × OUT_GAIN
// 每个采样 in_valid 来一次，8 个时钟后 out_valid（dout 同拍有效并保持到下一次）。
// =============================================================================
module gtr_amp #(
    parameter integer DRIVE    = 4,        // 失真前增益（倍），越大越“脏”
    parameter integer CAB_K    = 26214,    // 音箱低通系数 × 65536（0.4，约 4 kHz）
    parameter integer OUT_GAIN = 19661     // 输出增益 × 65536（0.3）
) (
    input  wire               clk,
    input  wire               rst_n,        // 同步复位，低有效
    input  wire               in_valid,
    input  wire signed [17:0] din,          // 吉他声部之和（1.0 = 32768）
    output reg                out_valid,
    output reg  signed [17:0] dout
);

    localparam signed [17:0] K_CAB = CAB_K;
    localparam signed [17:0] K_OUT = OUT_GAIN;

    localparam [2:0] S_IDLE = 3'd0,
                     S_SQ   = 3'd1,   // u²
                     S_CUBE = 3'd2,   // u³
                     S_CLIP = 3'd3,   // 1.5u - 0.5u³
                     S_LP1  = 3'd4,
                     S_LP2  = 3'd5,
                     S_MUL  = 3'd6,
                     S_OUT  = 3'd7;

    reg  [2:0]         state;
    reg  signed [15:0] u;              // 限幅后的输入，±32767
    reg  signed [15:0] u2;             // u² / 32768，0 ~ 32766
    reg  signed [15:0] u3;             // u³ / 32768²
    reg  signed [17:0] y;              // 削波输出，约 ±32768
    reg  signed [17:0] s1, s2;         // 两级低通状态
    reg  signed [35:0] prod;

    // 注意：乘积都先放进足够宽的 wire，避免 Verilog 按上下文位宽截断
    wire signed [31:0] xd    = din * DRIVE;
    wire signed [15:0] u_lim = (xd > 32'sd32767)  ?  16'sd32767 :
                               (xd < -32'sd32767) ? -16'sd32767 : xd[15:0];
    wire signed [31:0] sq    = u * u;
    wire signed [31:0] cube  = u2 * u;
    wire signed [17:0] clip  = u + (u >>> 1) - (u3 >>> 1);
    wire signed [18:0] dif1  = y - s1;
    wire signed [18:0] dif2  = s1 - s2;
    wire signed [36:0] pk1   = dif1 * K_CAB;
    wire signed [36:0] pk2   = dif2 * K_CAB;
    wire signed [36:0] s1_nx = s1 + (pk1 >>> 16);
    wire signed [36:0] s2_nx = s2 + (pk2 >>> 16);

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            u         <= 16'sd0;
            u2        <= 16'sd0;
            u3        <= 16'sd0;
            y         <= 18'sd0;
            s1        <= 18'sd0;
            s2        <= 18'sd0;
            prod      <= 36'sd0;
            dout      <= 18'sd0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            case (state)
            S_IDLE: if (in_valid) begin
                u     <= u_lim;
                state <= S_SQ;
            end
            S_SQ: begin
                u2    <= sq[30:15];
                state <= S_CUBE;
            end
            S_CUBE: begin
                u3    <= cube[30:15];
                state <= S_CLIP;
            end
            S_CLIP: begin
                y     <= clip;
                state <= S_LP1;
            end
            S_LP1: begin                              // s1 += (y - s1) × K
                s1    <= s1_nx[17:0];
                state <= S_LP2;
            end
            S_LP2: begin                              // s2 += (s1 - s2) × K
                s2    <= s2_nx[17:0];
                state <= S_MUL;
            end
            S_MUL: begin
                prod  <= s2 * K_OUT;
                state <= S_OUT;
            end
            default: begin                            // S_OUT
                dout      <= prod[33:16];
                out_valid <= 1'b1;
                state     <= S_IDLE;
            end
            endcase
        end
    end

endmodule
