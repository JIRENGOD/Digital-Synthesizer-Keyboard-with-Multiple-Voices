`timescale 1ns/1ps
// =============================================================================
// spectrum.v —— 实时频谱分析：1024 点 FFT → 32 根频谱柱（给 OLED 显示）
// -----------------------------------------------------------------------------
// 1. 采集：每个音频采样（48.828 kHz）把单声道样本写进 1024 点环形缓冲（最近 21 ms 的声音）。
// 2. start 来一拍后依次：
//    a. 拷贝：从最旧到最新取 1024 个样本，乘汉宁窗，按位反转地址写进 FFT 存储器（虚部为 0）；
//    b. FFT：基 2 时间抽取，10 级 × 512 个蝶形，每个蝶形 8 拍；每级结果除以 2，不会溢出；
//    c. 幅度：每个频点 |X| ≈ max(|实部|, |虚部|) + 0.375 × min(|实部|, |虚部|)，
//       按 fft_band_rom 归到 32 根柱子（100 Hz ~ 12 kHz 对数分布），每根取最大值；
//    d. 柱高 = 4 × (log2(幅度) - L0X8/8)，每格 1.5 dB，显示 48 dB 范围，限制在 0 ~ 31；
//       显示的柱子上升立即跟上、下降每次最多 FALL 格；峰值点停 PK_HOLD 次后每次降 1 格。
//    全部约 4.3 万拍（0.9 ms），完成后 done 来一拍，bar_h / bar_pk 更新（显示期间不变，画面不撕裂）。
// 满幅正弦（±32767）对应幅度约 8192（2^13），柱高 32（显示为 31）。
// 所有存储器读数据都晚 2 拍：地址寄存 1 拍 + 存储器输出寄存 1 拍。
// =============================================================================
module spectrum #(
    parameter integer L0X8    = 40,      // 柱高 0 对应的 8 × log2(幅度)（40 = 幅度 32，约满幅 -48 dB）
    parameter integer FALL    = 2,       // 柱子每次最多下落几格
    parameter integer PK_HOLD = 12       // 峰值点停留次数（≤ 15）
) (
    input  wire               clk,
    input  wire               rst_n,       // 同步复位，低有效
    input  wire               in_valid,    // 每个音频采样 1 拍
    input  wire signed [15:0] din,         // 单声道样本
    input  wire               start,       // 1 拍：分析最近 1024 个样本（忙时忽略）
    output reg                done,        // 1 拍：结果已更新
    output reg  [159:0]       bar_h,       // 32 × 5 位：柱高 0 ~ 31（第 b 根 = [b*5 +: 5]）
    output reg  [159:0]       bar_pk       // 32 × 5 位：峰值点高度 0 ~ 31
);

    // ---------------- 环形采集缓冲 1024 × 16 ----------------
    reg signed [15:0] cbuf [0:1023];
    reg        [9:0]  cwp;               // 下一个写入位置 = 最旧样本的位置
    reg        [9:0]  cra;
    reg signed [15:0] crd;

    always @(posedge clk) begin
        if (in_valid)
            cbuf[cwp] <= din;
        crd <= cbuf[cra];
    end

    always @(posedge clk) begin
        if (!rst_n)
            cwp <= 10'd0;
        else if (in_valid)
            cwp <= cwp + 10'd1;
    end

    // ---------------- FFT 数据存储器 1024 × 36（实部 18 位 + 虚部 18 位） ----------------
    reg  [35:0] dmem [0:1023];
    reg  [9:0]  dra, dwa;
    reg         dwe;
    reg  [35:0] dwd, drd;

    always @(posedge clk) begin
        if (dwe)
            dmem[dwa] <= dwd;
        drd <= dmem[dra];
    end

    // ---------------- 只读表（fft_roms.v，输出寄存） ----------------
    reg  [9:0]  win_a;
    reg  [8:0]  tw_a, band_a;
    wire [15:0] win_q;
    wire [31:0] tw_q;
    wire [5:0]  band_q;

    fft_win_rom  u_win  (.clk (clk), .addr (win_a),  .data (win_q));
    fft_tw_rom   u_tw   (.clk (clk), .addr (tw_a),   .data (tw_q));
    fft_band_rom u_band (.clk (clk), .addr (band_a), .data (band_q));

    function [9:0] bitrev10;
        input [9:0] x;
        integer i;
        begin
            for (i = 0; i < 10; i = i + 1)
                bitrev10[i] = x[9 - i];
        end
    endfunction

    function [7:0] log2x8;                // 8 × log2(v)：整数部分 × 8 + 3 位小数；v = 0 时为 0
        input [17:0] v;
        integer i;
        reg [4:0]  p;
        reg [17:0] t;
        begin
            p = 5'd0;
            for (i = 0; i < 18; i = i + 1)
                if (v[i]) p = i;          // 最高位 1 的位置
            t = v << (5'd17 - p);         // 最高位移到第 17 位
            log2x8 = {p, t[16:14]};
        end
    endfunction

    // ---------------- 蝶形地址：第 st 级（0 ~ 9）第 b 个蝶形（0 ~ 511） ----------------
    reg  [3:0]  st;
    reg  [8:0]  b;
    wire [9:0]  lowm = (10'd1 << st) - 10'd1;
    wire [9:0]  b10  = {1'b0, b};
    wire [9:0]  i0   = ((b10 & ~lowm) << 1) | (b10 & lowm);    // 在第 st 位插入 0
    wire [9:0]  i1   = i0 | (10'd1 << st);
    wire [8:0]  twk  = (b & lowm[8:0]) << (4'd9 - st);          // 旋转因子序号 j × 2^(9-st)

    // ---------------- 蝶形运算：A' = (u + t)/2，B' = (u - t)/2，t = v × W ----------------
    reg  signed [17:0] ur, ui, vr, vi;
    reg  signed [15:0] wr, wi;
    reg  signed [33:0] prr, pii, pri, pir;
    reg  signed [18:0] tr, ti;
    wire signed [33:0] t_re = prr - pii;
    wire signed [33:0] t_im = pri + pir;
    wire signed [19:0] sa_r = ur + tr;
    wire signed [19:0] sa_i = ui + ti;
    wire signed [19:0] sb_r = ur - tr;
    wire signed [19:0] sb_i = ui - ti;

    // ---------------- 拷贝流水线（乘汉宁窗） ----------------
    reg  [10:0]        n;                 // 下一个要取的样本（0 ~ 1023，1024 = 取完）
    reg  [9:0]         base;
    reg                c1_v, c2_v, c3_v;
    reg  [9:0]         c1_t, c2_t, c3_t;
    reg  signed [32:0] wprod;             // 样本 × 窗（无符号 16 位）
    wire signed [32:0] wprod_n = crd * $signed({1'b0, win_q});

    // ---------------- 幅度流水线 ----------------
    reg  [9:0]         k;                 // 下一个频点（0 ~ 511，512 = 取完）
    reg                m1_v, m2_v, m3_v;
    reg  [17:0]        mag3;
    reg  [4:0]         bar3;
    wire signed [17:0] xr  = drd[35:18];
    wire signed [17:0] xi  = drd[17:0];
    wire [17:0]        ar  = xr[17] ? (18'd0 - xr) : xr;
    wire [17:0]        ai  = xi[17] ? (18'd0 - xi) : xi;
    wire [17:0]        amx = (ar > ai) ? ar : ai;
    wire [17:0]        amn = (ar > ai) ? ai : ar;
    wire [18:0]        mag = amx + (amn >> 2) + (amn >> 3);
    reg  [17:0]        barmax [0:31];

    // ---------------- 柱高换算 ----------------
    reg  [4:0]         lb;                // 正在换算的柱子
    reg  [7:0]         lg8;
    reg  [3:0]         hold [0:31];
    wire [7:0]         lg_d  = lg8 - L0X8;
    wire [4:0]         hn    = (lg8 <= L0X8) ? 5'd0 : (lg_d[7:1] > 7'd31) ? 5'd31 : lg_d[5:1];
    wire [4:0]         h_cur = bar_h[lb*5 +: 5];
    wire [4:0]         p_cur = bar_pk[lb*5 +: 5];
    wire [4:0]         h_upd = (hn >= h_cur) ? hn : ((h_cur > FALL) ? h_cur - FALL : 5'd0);

    localparam [3:0] A_IDLE = 4'd0,  A_COPY = 4'd1,  A_RD0 = 4'd2,  A_RD1 = 4'd3,
                     A_GETU = 4'd4,  A_GETV = 4'd5,  A_MUL = 4'd6,  A_ADD = 4'd7,
                     A_WR0  = 4'd8,  A_WR1  = 4'd9,  A_MAG = 4'd10, A_LOG0 = 4'd11,
                     A_LOG1 = 4'd12, A_DONE = 4'd13;

    reg  [3:0] state;
    integer    q;

    always @(posedge clk) begin
        if (!rst_n) begin
            state  <= A_IDLE;
            done   <= 1'b0;
            dwe    <= 1'b0;
            dra    <= 10'd0;
            dwa    <= 10'd0;
            dwd    <= 36'd0;
            cra    <= 10'd0;
            win_a  <= 10'd0;
            tw_a   <= 9'd0;
            band_a <= 9'd0;
            n      <= 11'd0;
            base   <= 10'd0;
            c1_v   <= 1'b0;
            c2_v   <= 1'b0;
            c3_v   <= 1'b0;
            m1_v   <= 1'b0;
            m2_v   <= 1'b0;
            m3_v   <= 1'b0;
            st     <= 4'd0;
            b      <= 9'd0;
            k      <= 10'd0;
            lb     <= 5'd0;
            bar_h  <= 160'd0;
            bar_pk <= 160'd0;
            for (q = 0; q < 32; q = q + 1)
                hold[q] <= 4'd0;
        end else begin
            done <= 1'b0;
            dwe  <= 1'b0;

            case (state)
            A_IDLE: if (start) begin
                base  <= cwp;                         // 最旧的样本
                n     <= 11'd0;
                state <= A_COPY;
            end

            A_COPY: begin                             // 每拍取一个样本，3 拍后写进 FFT 存储器
                if (n != 11'd1024) begin
                    cra   <= base + n[9:0];
                    win_a <= n[9:0];
                    n     <= n + 11'd1;
                end
                c1_v  <= (n != 11'd1024);
                c1_t  <= n[9:0];
                c2_v  <= c1_v;
                c2_t  <= c1_t;
                c3_v  <= c2_v;                        // 样本、窗系数此时有效
                c3_t  <= c2_t;
                wprod <= wprod_n;
                if (c3_v) begin
                    dwe <= 1'b1;
                    dwa <= bitrev10(c3_t);
                    dwd <= {wprod[32], wprod[32:16], 18'd0};   // 样本 × 窗 / 65536（±32767），实部；虚部 0
                end
                if (n == 11'd1024 && !c1_v && !c2_v && !c3_v) begin
                    st    <= 4'd0;
                    b     <= 9'd0;
                    state <= A_RD0;
                end
            end

            A_RD0: begin dra <= i0; tw_a <= twk; state <= A_RD1;  end
            A_RD1: begin dra <= i1;              state <= A_GETU; end
            A_GETU: begin                             // x[i0]、旋转因子到了
                ur    <= drd[35:18];
                ui    <= drd[17:0];
                wr    <= tw_q[31:16];
                wi    <= tw_q[15:0];
                state <= A_GETV;
            end
            A_GETV: begin                             // x[i1] 到了
                vr    <= drd[35:18];
                vi    <= drd[17:0];
                state <= A_MUL;
            end
            A_MUL: begin
                prr   <= vr * wr;
                pii   <= vi * wi;
                pri   <= vr * wi;
                pir   <= vi * wr;
                state <= A_ADD;
            end
            A_ADD: begin
                tr    <= t_re >>> 15;
                ti    <= t_im >>> 15;
                state <= A_WR0;
            end
            A_WR0: begin
                dwe   <= 1'b1;
                dwa   <= i0;
                dwd   <= {sa_r[18:1], sa_i[18:1]};
                state <= A_WR1;
            end
            A_WR1: begin
                dwe <= 1'b1;
                dwa <= i1;
                dwd <= {sb_r[18:1], sb_i[18:1]};
                if (b != 9'd511) begin
                    b     <= b + 9'd1;
                    state <= A_RD0;
                end else if (st != 4'd9) begin
                    b     <= 9'd0;
                    st    <= st + 4'd1;
                    state <= A_RD0;
                end else begin                        // FFT 完成，开始算幅度
                    k     <= 10'd0;
                    for (q = 0; q < 32; q = q + 1)
                        barmax[q] <= 18'd0;
                    state <= A_MAG;
                end
            end

            A_MAG: begin                              // 每拍一个频点，3 拍后归到柱子
                if (k != 10'd512) begin
                    dra    <= k;
                    band_a <= k[8:0];
                    k      <= k + 10'd1;
                end
                m1_v <= (k != 10'd512);
                m2_v <= m1_v;
                m3_v <= m2_v && (band_q != 6'd63);    // 频点数据、柱号此时有效
                mag3 <= (mag > 19'd262143) ? 18'd262143 : mag[17:0];
                bar3 <= band_q[4:0];
                if (m3_v && mag3 > barmax[bar3])
                    barmax[bar3] <= mag3;
                if (k == 10'd512 && !m1_v && !m2_v && !m3_v) begin
                    lb    <= 5'd0;
                    state <= A_LOG0;
                end
            end

            A_LOG0: begin
                lg8   <= log2x8(barmax[lb]);
                state <= A_LOG1;
            end

            A_LOG1: begin                             // 柱子下落、峰值点
                bar_h[lb*5 +: 5] <= h_upd;
                if (h_upd >= p_cur) begin
                    bar_pk[lb*5 +: 5] <= h_upd;
                    hold[lb]          <= PK_HOLD;
                end else if (hold[lb] != 4'd0) begin
                    hold[lb] <= hold[lb] - 4'd1;
                end else begin
                    bar_pk[lb*5 +: 5] <= p_cur - 5'd1;
                end
                if (lb == 5'd31) begin
                    state <= A_DONE;
                end else begin
                    lb    <= lb + 5'd1;
                    state <= A_LOG0;
                end
            end

            default: begin                            // A_DONE
                done  <= 1'b1;
                state <= A_IDLE;
            end
            endcase
        end
    end

endmodule
