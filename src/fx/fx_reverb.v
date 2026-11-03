`timescale 1ns/1ps
// =============================================================================
// fx_reverb.v —— 混响效果器（Freeverb 结构，单声道送入、立体声输出）
// -----------------------------------------------------------------------------
//   8 个并联梳状滤波器（延迟 25 ~ 37 ms 互不相同，反馈支路带低通阻尼：高频衰减得更快，像真实房间）
//   → 求和 → 左声道 4 个串联全通 / 右声道另 4 个延迟略长的串联全通（左右不相关，声场宽）
//   余响约 1.7 s（衰减 60 dB），输出 = 原声 + 湿声 × 2^WET_SHIFT。
// 16 条延迟线共用一块 16K × 18 存储器，每个采样依次处理（约 90 个时钟，每个采样有 1024 个时钟）。
// 关掉时只停止送入新声音（256 个采样渐变），已有的余响自然消失。
// 延迟长度 = Freeverb 原始值（44.1 kHz）× 48828 / 44100，右声道全通 +23 个采样（原算法的立体声展宽）。
// =============================================================================
module fx_reverb #(
    parameter integer FBK       = 57672,   // 梳状滤波器反馈 0.88（× 65536），越大余响越长
    parameter integer DAMP      = 16384,   // 阻尼 0.25（× 65536），越大余响越暗
    parameter integer WET_SHIFT = 1        // 湿声 = 全通输出 × 2^WET_SHIFT
) (
    input  wire               clk,
    input  wire               rst_n,       // 同步复位，低有效
    input  wire               en,          // 1 = 打开
    input  wire               in_valid,
    input  wire signed [17:0] din_l,
    input  wire signed [17:0] din_r,
    output reg                out_valid,
    output reg  signed [17:0] dout_l,
    output reg  signed [17:0] dout_r
);

    localparam signed [17:0] K_FB   = FBK;
    localparam signed [17:0] K_DAMP = DAMP;

    // ---------------- 延迟线表：0 ~ 7 梳状，8 ~ 11 左全通，12 ~ 15 右全通 ----------------
    function [13:0] lbase;
        input [3:0] k;
        begin
            case (k)
            4'd0:  lbase = 14'd0;      4'd1:  lbase = 14'd1236;   4'd2:  lbase = 14'd2551;
            4'd3:  lbase = 14'd3965;   4'd4:  lbase = 14'd5466;   4'd5:  lbase = 14'd7040;
            4'd6:  lbase = 14'd8691;   4'd7:  lbase = 14'd10415;  4'd8:  lbase = 14'd12205;
            4'd9:  lbase = 14'd12821;  4'd10: lbase = 14'd13309;  4'd11: lbase = 14'd13687;
            4'd12: lbase = 14'd13936;  4'd13: lbase = 14'd14577;  4'd14: lbase = 14'd15091;
            default: lbase = 14'd15494;
            endcase
        end
    endfunction

    function [10:0] llast;                       // 长度 - 1
        input [3:0] k;
        begin
            case (k)
            4'd0:  llast = 11'd1235;   4'd1:  llast = 11'd1314;   4'd2:  llast = 11'd1413;
            4'd3:  llast = 11'd1500;   4'd4:  llast = 11'd1573;   4'd5:  llast = 11'd1650;
            4'd6:  llast = 11'd1723;   4'd7:  llast = 11'd1789;   4'd8:  llast = 11'd615;
            4'd9:  llast = 11'd487;    4'd10: llast = 11'd377;    4'd11: llast = 11'd248;
            4'd12: llast = 11'd640;    4'd13: llast = 11'd513;    4'd14: llast = 11'd402;
            default: llast = 11'd274;
            endcase
        end
    endfunction

    function signed [17:0] sat18;
        input signed [36:0] x;
        begin
            if (x > 37'sd131071)
                sat18 = 18'sd131071;
            else if (x < -37'sd131072)
                sat18 = -18'sd131072;
            else
                sat18 = x[17:0];
        end
    endfunction

    // ---------------- 存储器 16K × 18（简单双口：读地址 ra、写地址 wa 分开，写入晚 1 拍） ----------------
    reg  signed [17:0] mem [0:16383];
    reg  [13:0]        ra, wa;
    reg                we;
    reg  signed [17:0] wd, rd;

    always @(posedge clk) begin
        if (we)
            mem[wa] <= wd;
        rd <= mem[ra];
    end

    // ---------------- 状态 ----------------
    reg  [10:0]        ptr [0:15];               // 每条延迟线的当前位置
    reg  signed [17:0] fs  [0:7];                // 梳状滤波器阻尼低通的状态
    reg  [3:0]         k;
    reg  signed [17:0] in_l, in_r, inp, rdv, xs, xap, xl, xr;
    reg  signed [20:0] acc;                      // 梳状输出之和
    reg  signed [36:0] prod1, prod2;
    reg  [8:0]         m;                        // 送入量 0 ~ 256

    wire [10:0]        p_k    = ptr[k];
    wire signed [17:0] fs_k   = fs[k[2:0]];
    wire signed [18:0] mono   = in_l + in_r;
    wire signed [28:0] feed_p = mono * $signed({1'b0, m});
    wire signed [18:0] dmp_d  = fs_k - rd;
    wire signed [36:0] dmp_p  = dmp_d * K_DAMP;
    wire signed [36:0] fs_nx  = rdv + (prod1 >>> 16);
    wire signed [36:0] fb_p   = fs_k * K_FB;
    wire signed [36:0] c_w    = inp + (prod2 >>> 16);
    wire signed [21:0] acc_n  = acc + rdv;
    wire signed [18:0] a_w    = xap + (rdv >>> 1);
    wire signed [18:0] a_y    = rdv - xap;
    wire signed [20:0] o_l    = in_l + (xl <<< WET_SHIFT);
    wire signed [20:0] o_r    = in_r + (xr <<< WET_SHIFT);

    localparam [3:0] S_IDLE = 4'd0, S_FEED = 4'd1, S_ADDR = 4'd2, S_WAIT = 4'd3, S_DATA = 4'd4,
                     S_C1   = 4'd5, S_C2   = 4'd6, S_C3   = 4'd7, S_A1   = 4'd8, S_OUT  = 4'd9;
    reg  [3:0] state;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            k         <= 4'd0;
            ra        <= 14'd0;
            wa        <= 14'd0;
            we        <= 1'b0;
            wd        <= 18'sd0;
            in_l      <= 18'sd0;
            in_r      <= 18'sd0;
            inp       <= 18'sd0;
            rdv       <= 18'sd0;
            xs        <= 18'sd0;
            xap       <= 18'sd0;
            xl        <= 18'sd0;
            xr        <= 18'sd0;
            acc       <= 21'sd0;
            prod1     <= 37'sd0;
            prod2     <= 37'sd0;
            m         <= 9'd0;
            dout_l    <= 18'sd0;
            dout_r    <= 18'sd0;
            out_valid <= 1'b0;
            for (i = 0; i < 16; i = i + 1)
                ptr[i] <= 11'd0;
            for (i = 0; i < 8; i = i + 1)
                fs[i] <= 18'sd0;
        end else begin
            we        <= 1'b0;
            out_valid <= 1'b0;
            case (state)
            S_IDLE: if (in_valid) begin
                in_l  <= din_l;
                in_r  <= din_r;
                state <= S_FEED;
            end
            S_FEED: begin
                inp   <= feed_p >>> 14;                         // (L+R)/2 × m/256 / 32
                acc   <= 21'sd0;
                k     <= 4'd0;
                state <= S_ADDR;
            end
            S_ADDR: begin
                ra    <= lbase(k) + p_k;
                state <= S_WAIT;
            end
            S_WAIT: state <= S_DATA;                           // RAM 读出
            S_DATA: begin
                rdv   <= rd;
                prod1 <= dmp_p;                                 // (fs - out) × 阻尼
                state <= k[3] ? S_A1 : S_C1;
            end
            S_C1: begin                                         // fs = out + (fs - out) × 阻尼
                fs[k[2:0]] <= fs_nx[17:0];
                state      <= S_C2;
            end
            S_C2: begin
                prod2 <= fb_p;                                  // fs × 反馈
                state <= S_C3;
            end
            S_C3: begin                                         // 写回：输入 + fs × 反馈
                we     <= 1'b1;
                wa     <= ra;
                wd     <= sat18(c_w);
                acc    <= acc_n[20:0];
                ptr[k] <= (p_k == llast(k)) ? 11'd0 : p_k + 11'd1;
                if (k == 4'd7) begin
                    xs  <= sat18(acc_n >>> 1);                  // 全通链的输入
                    xap <= sat18(acc_n >>> 1);
                end
                k     <= k + 4'd1;
                state <= S_ADDR;
            end
            S_A1: begin                                         // 全通：写 x + b/2，输出 b - x
                we     <= 1'b1;
                wa     <= ra;
                wd     <= sat18(a_w);
                ptr[k] <= (p_k == llast(k)) ? 11'd0 : p_k + 11'd1;
                if (k == 4'd11) begin
                    xl  <= sat18(a_y);
                    xap <= xs;                                  // 右声道全通链从头开始
                end else begin
                    xap <= sat18(a_y);
                end
                if (k == 4'd15) begin
                    xr    <= sat18(a_y);
                    state <= S_OUT;
                end else begin
                    k     <= k + 4'd1;
                    state <= S_ADDR;
                end
            end
            default: begin                                      // S_OUT（上一拍的写入在这一拍完成）
                dout_l    <= sat18(o_l);
                dout_r    <= sat18(o_r);
                out_valid <= 1'b1;
                if (en && m != 9'd256)
                    m <= m + 9'd1;
                else if (!en && m != 9'd0)
                    m <= m - 9'd1;
                state     <= S_IDLE;
            end
            endcase
        end
    end

endmodule
