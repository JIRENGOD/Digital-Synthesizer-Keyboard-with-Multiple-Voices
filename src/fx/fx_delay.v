`timescale 1ns/1ps
// =============================================================================
// fx_delay.v —— 延迟（回声）效果器，乒乓式立体声
// -----------------------------------------------------------------------------
//   左延迟线 ──LEN──► 左声道回声；它的输出再送进右延迟线 ──LEN──► 右声道回声；
//   右延迟线的输出经过低通、乘以反馈系数，再和新的输入一起送回左延迟线。
//   回声依次是：250 ms 左、500 ms 右、750 ms 左、1 s 右……一次比一次轻、一次比一次暗（像磁带回声）。
//   输出 = 输入（原声，左右各自）+ 回声 × WET。
// 关掉时只停止往延迟线里送新声音（256 个采样渐变），已经发出的回声自然消失，不会突然断掉。
// 每个采样 in_valid 来一次，6 个时钟后 out_valid（输出保持到下一次）。
// =============================================================================
module fx_delay #(
    parameter integer LEN   = 12207,   // 回声间隔（采样）= 250 ms，最大 16384
    parameter integer FB_LR = 45875,   // 左 → 右 0.7（× 65536）
    parameter integer FB_RL = 42598,   // 右 → 左 0.65（× 65536，经过低通）
    parameter integer WET   = 32768    // 回声音量 0.5（× 65536）
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

    localparam signed [17:0] K_LR  = FB_LR;
    localparam signed [17:0] K_RL  = FB_RL;
    localparam signed [17:0] K_WET = WET;

    // ---------------- 两条延迟线，各 LEN × 18（简单双口：在 ptr 读出最老的值，下一拍在 wa = ptr 写入新值） ----------------
    reg  signed [17:0] mem_l [0:LEN-1];
    reg  signed [17:0] mem_r [0:LEN-1];
    reg  [13:0]        ptr, wa;
    reg                we;
    reg  signed [17:0] wd_l, wd_r, rd_l, rd_r;

    always @(posedge clk) begin
        if (we) begin
            mem_l[wa] <= wd_l;
            mem_r[wa] <= wd_r;
        end
        rd_l <= mem_l[ptr];
        rd_r <= mem_r[ptr];
    end

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

    reg  signed [17:0] in_l, in_r, el, er, lp, feed;
    reg  [8:0]         m;                                    // 送入量 0 ~ 256

    wire signed [18:0] mono   = in_l + in_r;                  // (L + R)，下面再除以 2
    wire signed [28:0] feed_p = mono * $signed({1'b0, m});    // × m / 256 / 2
    wire signed [18:0] lp_dif = er - lp;
    wire signed [36:0] fb_rl  = lp * K_RL;
    wire signed [36:0] fb_lr  = el * K_LR;
    wire signed [36:0] wet_l  = el * K_WET;
    wire signed [36:0] wet_r  = er * K_WET;
    wire signed [36:0] w_l    = feed + (fb_rl >>> 16);
    wire signed [36:0] o_l    = in_l + (wet_l >>> 16);
    wire signed [36:0] o_r    = in_r + (wet_r >>> 16);

    localparam [2:0] S_IDLE = 3'd0, S_RD = 3'd1, S_GET = 3'd2, S_LP = 3'd3, S_WR = 3'd4;
    reg  [2:0] state;

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            ptr       <= 14'd0;
            wa        <= 14'd0;
            we        <= 1'b0;
            wd_l      <= 18'sd0;
            wd_r      <= 18'sd0;
            in_l      <= 18'sd0;
            in_r      <= 18'sd0;
            el        <= 18'sd0;
            er        <= 18'sd0;
            lp        <= 18'sd0;
            feed      <= 18'sd0;
            m         <= 9'd0;
            dout_l    <= 18'sd0;
            dout_r    <= 18'sd0;
            out_valid <= 1'b0;
        end else begin
            we        <= 1'b0;
            out_valid <= 1'b0;
            case (state)
            S_IDLE: if (in_valid) begin
                in_l  <= din_l;
                in_r  <= din_r;
                state <= S_RD;                                // ptr 不变：这一拍 RAM 读出 ptr 处
            end
            S_RD: begin
                feed  <= feed_p[26:9];                        // (L+R)/2 × m/256
                state <= S_GET;
            end
            S_GET: begin                                      // rd = LEN 个采样以前写入的值
                el    <= rd_l;
                er    <= rd_r;
                state <= S_LP;
            end
            S_LP: begin
                lp    <= lp + (lp_dif >>> 1);                 // 反馈低通，截止约 5 kHz
                state <= S_WR;
            end
            default: begin                                    // S_WR
                we        <= 1'b1;                            // 下一拍写入 ptr 处
                wa        <= ptr;
                wd_l      <= sat18(w_l);
                wd_r      <= fb_lr[33:16];                    // 0.7 × 左回声
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
            if (we)                                           // 写完再移到下一个位置
                ptr <= (ptr == LEN - 1) ? 14'd0 : ptr + 14'd1;
        end
    end

endmodule
