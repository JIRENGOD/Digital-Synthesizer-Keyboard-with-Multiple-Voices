`timescale 1ns/1ps
// =============================================================================
// guitar_voice.v —— 电吉他单音：Karplus-Strong 拨弦物理模型（参数来自 guitar_params.v）
// -----------------------------------------------------------------------------
// 每个采样（48.828 kHz）算一次：
//   d0 = v[n-M]，d1 = v[n-M-1]，dp = v[n-P]          从延迟线读出
//   a  = (d0 + d1) / 2                                两点平均：低通，高次谐波衰减更快
//   ap = C·(a - ap[n-1]) + a[n-1]                     一阶全通：补足小数延迟，音高精确
//   v  = g × ap                                       环路增益：按住用 g_sus，抬键后用 g_rel（手掌制音）
//   拨弦后的 M+1 个采样里 v 改用低通噪声（相当于拨片把弦拨乱），之后进入上面的循环
//   v 写回延迟线；输出 = (v - dp) / 2                 拾音器梳状效果
// 同一个声部再次拨弦：直接用新噪声替换（拨片碰到弦，原来的振动被打断），换音时延迟长度跟着变。
// 数值：延迟线和中间结果为 18 位有符号数，噪声峰值 ±32768，留 4 倍余量；乘法用 DSP。
// 时序：sample_en 后 11 个时钟算完（每个采样 1024 个时钟），与 piano_voice 同时开始。
// =============================================================================
module guitar_voice #(
    parameter [15:0] SEED = 16'hACE1          // 噪声初值（各声部不同，不能为 0）
) (
    input  wire               clk,             // 50 MHz
    input  wire               rst_n,           // 同步复位，低有效
    input  wire               sample_en,       // 每个采样 1 拍
    input  wire               note_on,         // 拨弦：1 拍
    input  wire               note_off,        // 制音：1 拍
    input  wire [5:0]         note,            // 音序号 0 ~ 45，与 note_on 同时给出
    output reg  signed [15:0] sample,
    output reg                sample_valid
);

    // ---------------- 参数表 ----------------
    reg  [5:0]         note_q;
    wire [9:0]         p_m, p_p;
    wire signed [15:0] p_c;
    wire [15:0]        p_gs, p_gr;

    guitar_params u_guitar_params (
        .note  (note_q),
        .m     (p_m),
        .c     (p_c),
        .p     (p_p),
        .g_sus (p_gs),
        .g_rel (p_gr)
    );

    // ---------------- 延迟线：1024 × 18，一个写口、一个读口，读数据晚 1 拍 ----------------
    reg  signed [17:0] dl [0:1023];
    reg  [9:0]         wp;                 // 本采样的 v[n] 写到这里
    reg  [9:0]         ra;                 // 读地址
    reg  [9:0]         wa;                 // 写地址
    reg                dl_we;
    reg  signed [17:0] dl_wd;
    reg  signed [17:0] dl_rd;

    always @(posedge clk) begin
        if (dl_we)
            dl[wa] <= dl_wd;
        dl_rd <= dl[ra];
    end

    // ---------------- 饱和 ----------------
    function signed [17:0] sat18;
        input signed [20:0] x;
        begin
            if (x > 21'sd131071)
                sat18 = 18'sd131071;
            else if (x < -21'sd131072)
                sat18 = -18'sd131072;
            else
                sat18 = x[17:0];
        end
    endfunction

    function signed [15:0] sat16;
        input signed [17:0] x;
        begin
            if (x > 18'sd32767)
                sat16 = 16'sd32767;
            else if (x < -18'sd32768)
                sat16 = -16'sd32768;
            else
                sat16 = x[15:0];
        end
    endfunction

    // ---------------- 运算 ----------------
    localparam [3:0] S_IDLE = 4'd0,   // 等下一个采样
                     S_PAR  = 4'd1,   // 参数就绪，发第 1 个读地址
                     S_RD1  = 4'd2,
                     S_RD2  = 4'd3,
                     S_RD3  = 4'd4,
                     S_AVG  = 4'd5,   // 两点平均
                     S_MULC = 4'd6,   // 全通：C × (a - ap[n-1])
                     S_AP   = 4'd7,
                     S_MULG = 4'd8,   // 环路增益
                     S_WR   = 4'd9;   // 写回、输出

    reg  [3:0]         state;
    reg                start_pend, on_pend, off_pend;
    reg  [5:0]         note_l;          // note_on 时锁存的音
    reg                ev_on;           // 本采样有拨弦
    reg                held;            // 1 = 按住
    reg  [9:0]         exc_cnt;         // 还要灌多少个采样的噪声
    reg  [9:0]         r_m, r_p;
    reg  signed [15:0] r_c;
    reg  [15:0]        r_g;
    reg  signed [17:0] d0, d1, dp;
    reg  signed [17:0] a_n, a1, ap1, ap_n;
    reg  signed [34:0] prod_c, prod_g;
    reg  [15:0]        lfsr;
    reg  signed [17:0] exc;             // 低通噪声（拨弦激励）

    wire [15:0]        lfsr_nx = lfsr[0] ? ((lfsr >> 1) ^ 16'hB400) : (lfsr >> 1);
    wire signed [17:0] noise   = {{2{lfsr[15]}}, lfsr};
    wire signed [18:0] exc_dif = noise - exc;
    wire signed [18:0] exc_nx  = exc + (exc_dif >>> 1);          // 一阶低通，截止约 5 kHz
    wire signed [18:0] sum01   = d0 + d1;
    wire signed [18:0] t_ap    = a_n - ap1;
    wire signed [20:0] ap_sum  = (prod_c >>> 15) + a1;
    wire signed [17:0] v_loop  = prod_g[33:16];                  // g < 1，不会溢出
    wire signed [17:0] v_new   = (exc_cnt != 10'd0) ? exc : v_loop;
    wire signed [18:0] o_dif   = v_new - dp;

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            start_pend   <= 1'b0;
            on_pend      <= 1'b0;
            off_pend     <= 1'b0;
            note_l       <= 6'd0;
            note_q       <= 6'd0;
            ev_on        <= 1'b0;
            held         <= 1'b0;
            exc_cnt      <= 10'd0;
            r_m          <= 10'd185;
            r_p          <= 10'd37;
            r_c          <= 16'sd0;
            r_g          <= 16'd0;
            d0           <= 18'sd0;
            d1           <= 18'sd0;
            dp           <= 18'sd0;
            a_n          <= 18'sd0;
            a1           <= 18'sd0;
            ap1          <= 18'sd0;
            ap_n         <= 18'sd0;
            prod_c       <= 35'sd0;
            prod_g       <= 35'sd0;
            lfsr         <= SEED;
            exc          <= 18'sd0;
            wp           <= 10'd0;
            ra           <= 10'd0;
            wa           <= 10'd0;
            dl_we        <= 1'b0;
            dl_wd        <= 18'sd0;
            sample       <= 16'sd0;
            sample_valid <= 1'b0;
        end else begin
            dl_we        <= 1'b0;
            sample_valid <= 1'b0;
            if (sample_en) start_pend <= 1'b1;
            if (note_on)   on_pend    <= 1'b1;
            if (note_off)  off_pend   <= 1'b1;
            if (note_on)   note_l     <= note;

            case (state)
            S_IDLE: if (start_pend) begin
                start_pend <= 1'b0;
                ev_on      <= on_pend | note_on;
                if (on_pend | note_on) begin
                    note_q <= note_on ? note : note_l;
                    held   <= 1'b1;
                end else if (off_pend | note_off) begin
                    held   <= 1'b0;
                end
                on_pend    <= 1'b0;
                off_pend   <= 1'b0;
                state      <= S_PAR;
            end

            S_PAR: begin                              // note_q 已更新，参数有效
                r_m <= p_m;
                r_p <= p_p;
                r_c <= p_c;
                r_g <= held ? p_gs : p_gr;
                ra  <= wp - p_m;                      // 读 v[n-M]
                if (ev_on)
                    exc_cnt <= p_m + 10'd1;           // 拨弦：灌 M+1 个采样的噪声
                state <= S_RD1;
            end

            S_RD1: begin
                ra    <= wp - r_m - 10'd1;            // 读 v[n-M-1]
                state <= S_RD2;
            end

            S_RD2: begin
                d0    <= dl_rd;
                ra    <= wp - r_p;                    // 读 v[n-P]
                state <= S_RD3;
            end

            S_RD3: begin
                d1    <= dl_rd;
                state <= S_AVG;
            end

            S_AVG: begin
                dp    <= dl_rd;
                a_n   <= sum01[18:1];                 // (d0 + d1) / 2
                state <= S_MULC;
            end

            S_MULC: begin
                prod_c <= r_c * t_ap;
                state  <= S_AP;
            end

            S_AP: begin
                ap_n  <= sat18(ap_sum);
                state <= S_MULG;
            end

            S_MULG: begin
                prod_g <= ap_n * $signed({1'b0, r_g});
                state  <= S_WR;
            end

            default: begin                            // S_WR
                dl_we  <= 1'b1;
                wa     <= wp;
                dl_wd  <= v_new;
                wp     <= wp + 10'd1;
                a1     <= a_n;                        // 全通一直按延迟线内容运算，换回环路时没有跳变
                ap1    <= ap_n;
                if (exc_cnt != 10'd0)
                    exc_cnt <= exc_cnt - 10'd1;
                lfsr   <= lfsr_nx;
                exc    <= exc_nx[17:0];
                sample       <= sat16(o_dif[18:1]);   // (v - dp) / 2
                sample_valid <= 1'b1;
                state  <= S_IDLE;
            end
            endcase
        end
    end

endmodule
