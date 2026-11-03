`timescale 1ns/1ps
// =============================================================================
// oled_frame.v —— OLED 画面内容：按字节序号给出显存数据（晚 2 拍）
// -----------------------------------------------------------------------------
// 画面 128 × 64，8 页（每页 8 行像素），字节序号 idx = 页 × 128 + 列，字节最低位 = 这一页最上面一行。
// 布局（文字为宋体 12 像素，点阵在 oled_text_rom.v / oled_digit_rom.v，由 scripts/gen_display_roms.py 生成）：
//   第 0 ~ 11 行：  音色（第 0 ~ 41 列）  音区（第 42 ~ 71 列）  音量（第 72 ~ 127 列）
//   第 12 ~ 23 行： 效果：干声 / 混响 / 延迟 / 合唱 / 三个全开
//   第 24 ~ 35 行： 录音状态；录音时长 / 发送进度从第 80 列起，每个字符 8 列
//   第 36 ~ 63 行： 频谱（28 行），32 根柱子（每根 3 列亮 + 1 列空），从左到右 100 Hz ~ 12 kHz；
//     柱高 h 格（0 ~ 31，按 28 行显示时 h > 28 算满格）= 最下面 h 行亮；峰值点 = 从下往上第 pk 行亮一个点。
// =============================================================================
module oled_frame (
    input  wire         clk,
    input  wire [9:0]   idx,         // 页 × 128 + 列
    input  wire [1:0]   tone,        // 0 钢琴，1 电吉他，2 大提琴
    input  wire [1:0]   group,       // 0 低音，1 中音，2 高音
    input  wire [1:0]   vol,         // 0 = 0 dB … 3 = -18 dB
    input  wire [2:0]   fx,          // 0 干声，1 混响，2 延迟，3 合唱，4 三个全开
    input  wire [2:0]   rec_st,      // 录音状态：0 空闲，1 录音中，2 等电脑，3 发送中，4 发送完
    input  wire [15:0]  t_bcd,       // 录音时长 mm:ss（BCD，4 位数字）
    input  wire [11:0]  p_bcd,       // 发送进度（BCD 百分比，3 位数字）
    input  wire         blink,       // 录音时“●”闪烁
    input  wire [159:0] bar_h,       // 32 × 5 位柱高（spectrum.v）
    input  wire [159:0] bar_pk,      // 32 × 5 位峰值点
    output reg  [7:0]   data
);

    wire [2:0] page = idx[9:7];
    wire [6:0] col  = idx[6:0];

    // ---------------- 动态数字（BCD） ----------------
    wire [3:0]  mm1 = t_bcd[15:12], mm0 = t_bcd[11:8];
    wire [3:0]  ss1 = t_bcd[7:4],   ss0 = t_bcd[3:0];
    wire [3:0]  pc2 = p_bcd[11:8],  pc1 = p_bcd[7:4], pc0 = p_bcd[3:0];

    // 第 3 行第 80 列起的 5 个字符（12 = 空格）
    wire        show_t = (rec_st == 3'd1) || (rec_st == 3'd4);
    wire        show_p = (rec_st == 3'd3);
    wire [2:0]  dpos   = col[5:3] - 3'd2;              // 第 80 列 = 第 0 个字符
    reg  [3:0]  dch;

    always @(*) begin
        dch = 4'd12;
        if (col >= 7'd80 && col < 7'd120) begin
            if (show_t)
                case (dpos)
                3'd0: dch = mm1;
                3'd1: dch = mm0;
                3'd2: dch = 4'd10;                     // “:”
                3'd3: dch = ss1;
                default: dch = ss0;
                endcase
            else if (show_p)
                case (dpos)
                3'd0: dch = (pc2 == 4'd0) ? 4'd12 : pc2;
                3'd1: dch = (pc2 == 4'd0 && pc1 == 4'd0) ? 4'd12 : pc1;
                3'd2: dch = pc0;
                3'd3: dch = 4'd11;                     // “%”
                default: dch = 4'd12;
                endcase
        end
    end

    // ---------------- 选文字条：按页和列 ----------------
    // 页 0 = 第 0 ~ 7 行（第 1 行字的 0 ~ 7 行）
    // 页 1 = 第 8 ~ 15 行（第 1 行字的 8 ~ 11 行 + 第 2 行字的 0 ~ 3 行）
    // 页 2 = 第 16 ~ 23 行（第 2 行字的 4 ~ 11 行）
    // 页 3 = 第 24 ~ 31 行（第 3 行字的 0 ~ 7 行）
    // 页 4 = 第 32 ~ 39 行（第 3 行字的 8 ~ 11 行 + 频谱最上面 4 行）
    reg  [4:0] s_a, s_b;              // 这一页上半部分 / 下半部分用的文字条
    wire [4:0] s_top = (col < 7'd42) ? {3'd0, tone} : (col < 7'd72) ? (5'd3 + group) : (5'd6 + vol);
    wire [4:0] s_fx  = 5'd10 + fx;
    wire [4:0] s_rec = (rec_st == 3'd0) ? 5'd15 : (rec_st == 3'd1) ? 5'd16 : (rec_st == 3'd2) ? 5'd17 :
                       (rec_st == 3'd3) ? 5'd18 : 5'd19;

    always @(*) begin
        case (page)
        3'd0, 3'd1: s_a = s_top;
        3'd2, 3'd3: s_a = (page == 3'd2) ? s_fx : s_rec;
        default:    s_a = s_rec;
        endcase
        s_b = (page == 3'd1) ? s_fx : s_a;
    end

    wire [11:0] trom_a, trom_b;

    oled_text_rom u_text_a (
        .clk  (clk),
        .addr ({s_a, col}),
        .data (trom_a)
    );

    oled_text_rom u_text_b (
        .clk  (clk),
        .addr ({s_b, col}),
        .data (trom_b)
    );

    wire [11:0] dig;

    oled_digit_rom u_digit (
        .addr ({dch, col[2:0]}),
        .data (dig)
    );

    // ---------------- 频谱（第 36 ~ 63 行）：这一页第 i 位 = 第 page×8 + i 行，从下往上第 63 - 行号 行 ----------------
    wire [4:0] bi  = col[6:2];
    wire       gap = (col[1:0] == 2'b11);
    wire [4:0] h   = bar_h[bi*5 +: 5];
    wire [4:0] pk  = bar_pk[bi*5 +: 5];
    reg  [7:0] sb;
    reg  [5:0] y;
    reg  [5:0] r;
    integer    i;

    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            y     = {page, i[2:0]};
            r     = 6'd63 - y;                         // 从下往上第 r 行（0 = 最下面）
            sb[i] = (y >= 6'd36) && !gap && ((r < h) || (r == pk));
        end
    end

    // ---------------- 拼成这一页的字节（第 2 拍输出） ----------------
    reg  [7:0] sb_q;
    reg  [2:0] page_q;
    reg        dig_on_q;
    reg [11:0] dig_q;

    always @(posedge clk) begin
        sb_q     <= sb;
        page_q   <= page;
        dig_on_q <= (dch != 4'd12);
        dig_q    <= dig;
    end

    reg         dot_q;                // 这一列是“●”（第 0 ~ 11 列），录音时闪烁

    always @(posedge clk)
        dot_q <= (rec_st == 3'd1) && (col < 7'd12) && !blink;

    wire [11:0] rec_line = (dot_q ? 12'd0 : trom_a) | (dig_on_q ? dig_q : 12'd0);   // 第 3 行：文字 + 动态数字

    always @(posedge clk) begin
        case (page_q)
        3'd0:    data <= trom_a[7:0];
        3'd1:    data <= {trom_b[3:0], trom_a[11:8]};
        3'd2:    data <= trom_a[11:4];
        3'd3:    data <= rec_line[7:0];
        3'd4:    data <= {sb_q[7:4], rec_line[11:8]};
        default: data <= sb_q;
        endcase
    end

endmodule
