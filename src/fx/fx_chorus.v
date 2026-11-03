`timescale 1ns/1ps
// =============================================================================
// fx_chorus.v —— 合唱效果器（单声道进、立体声出）
// -----------------------------------------------------------------------------
// 原理：把声音延迟 8 ~ 15 ms 再和原声混合，延迟时间用 0.6 Hz 的三角波慢慢来回变，
//   延迟变短 / 变长时音高略升 / 略降（约 ±15 音分），和原声叠在一起就像好几个人同时在弹（唱）。
//   左右声道的三角波相差 90°，声场变宽；单声道音箱听也没有问题（左右同相相加）。
// 小数延迟用线性插值：y = x[n-i] + (x[n-i-1] - x[n-i]) × f。
// 混合：输出 = 原声 × (1 - m/4) + 合唱声 × 3m/4，m 在开 / 关时用 256 个采样（约 5 ms）从 0 渐变到 1
//   或从 1 渐变到 0，切换时不会“咔”一声；关掉时输出就是原声。
// 每个采样 in_valid 来一次，9 个时钟后 out_valid（输出保持到下一次）。
// =============================================================================
module fx_chorus #(
    parameter integer BASE  = 400,     // 最短延迟（采样）≈ 8.2 ms
    parameter integer DEPTH = 340,     // 延迟变化范围（采样）≈ 7 ms，最长 BASE + DEPTH < 1023
    parameter integer RATE  = 206      // LFO 相位增量 = 0.6 Hz × 2^24 / 48828
) (
    input  wire               clk,
    input  wire               rst_n,       // 同步复位，低有效
    input  wire               en,          // 1 = 打开
    input  wire               in_valid,
    input  wire signed [17:0] din,         // 单声道输入（1.0 = 32768）
    output reg                out_valid,
    output reg  signed [17:0] dout_l,
    output reg  signed [17:0] dout_r
);

    localparam [17:0] BASE_Q8  = BASE * 256;
    localparam [16:0] DEPTH_Q8 = DEPTH * 256;

    // ---------------- 延迟线 1024 × 18 ----------------
    reg  signed [17:0] mem [0:1023];
    reg  [9:0]         wp, ra, wa;
    reg                we;
    reg  signed [17:0] wd, rd;

    always @(posedge clk) begin
        if (we)
            mem[wa] <= wd;
        rd <= mem[ra];
    end

    function signed [17:0] sat18;
        input signed [28:0] x;
        begin
            if (x > 29'sd131071)
                sat18 = 18'sd131071;
            else if (x < -29'sd131072)
                sat18 = -18'sd131072;
            else
                sat18 = x[17:0];
        end
    endfunction

    // ---------------- LFO 与延迟时间（Q8：低 8 位是小数） ----------------
    reg  [23:0] ph;
    wire [23:0] ph_r   = ph + 24'h400000;                     // 右声道相位 +90°
    wire [15:0] tri_l  = ph[23]   ? ~ph[22:7]   : ph[22:7];   // 三角波 0 → 65535 → 0
    wire [15:0] tri_r  = ph_r[23] ? ~ph_r[22:7] : ph_r[22:7];
    wire [32:0] off_l  = tri_l * DEPTH_Q8;
    wire [32:0] off_r  = tri_r * DEPTH_Q8;

    reg  [17:0] dly_l, dly_r;
    reg  signed [17:0] din_q, x0l, x1l, x0r, x1r, yl, yr;
    reg  [8:0]  m;                                            // 合唱量 0 ~ 256

    wire signed [18:0] dfl  = x1l - x0l;
    wire signed [18:0] dfr  = x1r - x0r;
    wire signed [28:0] pfl  = dfl * $signed({1'b0, dly_l[7:0]});
    wire signed [28:0] pfr  = dfr * $signed({1'b0, dly_r[7:0]});
    wire signed [28:0] yl_n = x0l + (pfl >>> 8);
    wire signed [28:0] yr_n = x0r + (pfr >>> 8);

    wire [9:0]  dry_g = 10'd256 - {3'd0, m[8:2]};             // 256 → 192
    wire [9:0]  wet_g = ({1'b0, m} + {m, 1'b0}) >> 2;         // 0 → 192（= 3m/4）
    wire signed [28:0] mix_l = din_q * $signed({1'b0, dry_g}) + yl * $signed({1'b0, wet_g});
    wire signed [28:0] mix_r = din_q * $signed({1'b0, dry_g}) + yr * $signed({1'b0, wet_g});

    localparam [3:0] S_IDLE = 4'd0, S_A0 = 4'd1, S_A1 = 4'd2, S_A2 = 4'd3, S_A3 = 4'd4,
                     S_A4   = 4'd5, S_A5 = 4'd6, S_INT = 4'd7, S_MIX = 4'd8;
    reg  [3:0] state;

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            wp        <= 10'd0;
            ra        <= 10'd0;
            wa        <= 10'd0;
            we        <= 1'b0;
            wd        <= 18'sd0;
            ph        <= 24'd0;
            dly_l     <= BASE_Q8;
            dly_r     <= BASE_Q8;
            din_q     <= 18'sd0;
            x0l       <= 18'sd0;
            x1l       <= 18'sd0;
            x0r       <= 18'sd0;
            x1r       <= 18'sd0;
            yl        <= 18'sd0;
            yr        <= 18'sd0;
            m         <= 9'd0;
            dout_l    <= 18'sd0;
            dout_r    <= 18'sd0;
            out_valid <= 1'b0;
        end else begin
            we        <= 1'b0;
            out_valid <= 1'b0;
            case (state)
            S_IDLE: if (in_valid) begin
                we    <= 1'b1;                                // 写入 x[n]
                wa    <= wp;
                wd    <= din;
                din_q <= din;
                dly_l <= BASE_Q8 + off_l[32:16];
                dly_r <= BASE_Q8 + off_r[32:16];
                state <= S_A0;
            end
            S_A0: begin ra <= wp - dly_l[17:8];         state <= S_A1; end   // x[n-i]
            S_A1: begin ra <= wp - dly_l[17:8] - 10'd1; state <= S_A2; end   // x[n-i-1]
            S_A2: begin x0l <= rd; ra <= wp - dly_r[17:8];         state <= S_A3; end
            S_A3: begin x1l <= rd; ra <= wp - dly_r[17:8] - 10'd1; state <= S_A4; end
            S_A4: begin x0r <= rd; state <= S_A5; end
            S_A5: begin x1r <= rd; state <= S_INT; end
            S_INT: begin
                yl    <= yl_n[17:0];
                yr    <= yr_n[17:0];
                state <= S_MIX;
            end
            default: begin                                    // S_MIX
                dout_l    <= sat18(mix_l >>> 8);
                dout_r    <= sat18(mix_r >>> 8);
                out_valid <= 1'b1;
                wp        <= wp + 10'd1;
                ph        <= ph + RATE;
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
