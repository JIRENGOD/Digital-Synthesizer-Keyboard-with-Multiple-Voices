`timescale 1ns/1ps
// =============================================================================
// top_touch_synth.v —— 触摸键盘电子琴：钢琴 / 电吉他 / 大提琴三种音色 + 混响 / 延迟 / 合唱效果器，8 复音，
//                      三组音区，OLED 显示状态和实时频谱，HC-05 蓝牙把演奏录音传到电脑
//   板卡：Tang Mega 138K Pro 核心板（GW5AST-LV138FPG676AC1/I0）+ Tang Mega 138K Pro Dock 底板
//   键盘：16 键电容触摸钢琴模块（TTP229 芯片，两线串行 SCL / SDO），接 PMOD2
//   屏幕：0.96 寸 128×64 OLED（SSD1306 / SSD1315，I2C 4 针），接 PMOD1
//   蓝牙：HC-05（串口 9600 波特），接 PMOD0
//   三个 PMOD 口接法相同，两排都能接：从 3V3 那一端数第 1 列 3V3、第 2 列 GND、第 3 列 FPGA 输出、第 4 列 FPGA 输入
// -----------------------------------------------------------------------------
// 信号通路：
//   触摸键盘 ─SDO─► ttp229_rd（约 3.3 ms 读一次 16 个键）→ touch_keys（三组音区，KEY16 换音区，按下 / 松开事件）
//   → voice_alloc（分配到 8 个声部）→ 每个声部：piano_voice（加法合成：钢琴 / 大提琴）+ guitar_voice（拨弦物理模型）
//   → 按力度混音：钢琴与大提琴总线 + 电吉他总线 → gtr_amp（过载失真 + 音箱模拟，只处理电吉他）
//   → fx_chorus（合唱）→ fx_delay（乒乓回声）→ fx_reverb（混响）→ 音量 → i2s_dac_tx → PT8211 → 3.5 mm
//   混响输出（音量调节前）→ spectrum（1024 点 FFT，32 根频谱柱）→ oled_frame → oled_i2c → OLED
//   琴键事件 + 效果器 / 音量变化 → recorder（录音）→ uart → HC-05 → 电脑（scripts/bt_recorder.py 渲染成 WAV）
// 延迟：读到按键到开始出声 < 7 ms（连续两次读数确认 + 1 个采样），满足 ≤ 10 ms。
//
// 按键（编号按 tang_mega_138k-pro_pins.cst）：
//   key[0]（K16）：效果器切换：干声 → 混响 → 延迟 → 合唱 → 三个全开 → 干声 …（上电为干声）
//   key[1]（G15）：音量，每按一次减 6 dB：-12 → -18 → 0 → -6 → -12 dB …（上电 -12 dB）
//   key[2]（F15）：录音 开始 / 停止；停止后电脑端程序在线就自动通过蓝牙发送（见 recorder.v）
//   key[3]（G16）：音色切换：钢琴 → 电吉他 → 大提琴 → 钢琴 …（上电为钢琴；已经按住的音保持原来的音色）
//     大提琴比钢琴低一个八度，按住持续发声，带颤音
//   触摸键盘 KEY16（最右边带钢琴图标的那块）：音区切换 中音 → 高音 → 低音 → 中音 …（上电为中音）
//     低音 A2 ~ B3、中音 C4 ~ D5、高音 E5 ~ F#6（白键分别从低音“拉”、中音“哆”、高音“咪”开始）
// 指示灯 led[0] ~ led[5]（R26 J14 L20 M25 N21 N23，低电平亮）：
//   平时：led[1:0] 音色（钢琴 = led0，电吉他 = led1，大提琴 = led0 + led1）  led[2] 混响  led[3] 延迟  led[4] 合唱
//         led[5]：录音时常亮；发送时快闪；否则灭 = 正常，慢闪 = 触摸键盘一直读到错误数据 / OLED 没有应答
//   摸着触摸键盘时：led[5] 亮，led[4:0] 显示被摸的键号 KEY1 ~ KEY16（二进制，亮的灯相加就是键号）
//
// 复位：*** 同步复位，低电平有效 ***，由内部上电复位计数器产生（约 1.3 ms），不需要按键。
// =============================================================================
module top_touch_synth #(
    parameter integer NV          = 8,      // 复音数 = 并行声部数
    parameter integer KEY_REVERSE = 1,      // 1 = 从正面看 KEY1 在最左边（10-31 上板确认），见 touch_keys.v
    parameter integer TOUCH_HIGH  = 0,      // 0 = 触摸键盘有触摸为低电平（TTP229 默认），1 = 高电平
    parameter integer POR_BITS    = 16,     // 上电复位：2^16 个时钟 ≈ 1.3 ms
    parameter integer TOUCH_GAIN  = 220,    // 触摸键盘的力度增益（键盘没有力度，固定值；255 = 满）
    parameter integer DEBOUNCE_MS = 20,     // 按键消抖时间（ms）
    parameter integer VOL_INIT    = 2,      // 上电音量：0 = 0 dB，1 = -6 dB，2 = -12 dB，3 = -18 dB
    parameter         LUT_FILE    = "sine_lut.hex"
) (
    input  wire       i_clk_50m,      // P16   clk：核心板 50 MHz 晶振
    input  wire       i_key_fx_n,     // K16   key[0]，按下为 0：效果器切换
    input  wire       i_key_vol_n,    // G15   key[1]，按下为 0：音量
    input  wire       i_key_rec_n,    // F15   key[2]，按下为 0：录音开始 / 停止
    input  wire       i_key_tone_n,   // G16   key[3]，按下为 0：音色切换
    output wire       o_i2s_bck,      // E22   HP_BCK → U6 PT8211 BCK
    output wire       o_i2s_lrck,     // F24   HP_WS  → U6 PT8211 WS
    output wire       o_i2s_sdata,    // G24   HP_DIN → U6 PT8211 DIN
    output wire       o_pa_en,        // H17   PA_EN  → R248 → U32 XPT4809 EN，常为 1
    output wire [5:0] o_led_n,        // led[0..5] = R26 J14 L20 M25 N21 N23，低电平亮
    // 触摸键盘：PMOD2（J26），两排都能接。第 3 列 → 键盘 SCL、第 4 列 ← 键盘 SDO
    output wire [1:0] o_tp_scl,       // 第 3 列（第 5、6 针）：A17 A18
    input  wire [1:0] i_tp_sdo,       // 第 4 列（第 7、8 针）：B19 A19
    // 蓝牙 HC-05：PMOD0（J22），两排都能接。第 3 列 → HC-05 RXD、第 4 列 ← HC-05 TXD
    output wire [1:0] o_bt_tx,        // 第 3 列（第 5、6 针）：R17 T24
    input  wire [1:0] i_bt_rx,        // 第 4 列（第 7、8 针）：R16 L24
    // OLED：PMOD1（J24），两排都能接。从 3V3 那一端数：第 1 列 3V3、第 2 列 GND、第 3 列 SCL、第 4 列 SDA
    output wire [1:0] o_oled_scl,     // 第 3 列（第 5、6 针）：H14 H15
    inout  wire [1:0] io_oled_sda     // 第 4 列（第 7、8 针）：L17 L18
);

    localparam integer FS_KHZ   = 49;                 // 1 ms ≈ 49 个采样
    localparam integer DEB_SMP  = DEBOUNCE_MS * FS_KHZ;
    localparam [1:0]   VOL0     = VOL_INIT;
    localparam [7:0]   TGAIN    = TOUCH_GAIN;
    localparam [1:0]   T_PIANO  = 2'd0,
                       T_GUITAR = 2'd1,
                       T_CELLO  = 2'd2;

    // 与官方例程一样，功放常开
    assign o_pa_en = 1'b1;

    function signed [17:0] sat18;
        input signed [31:0] x;
        begin
            if (x > 32'sd131071)
                sat18 = 18'sd131071;
            else if (x < -32'sd131072)
                sat18 = -18'sd131072;
            else
                sat18 = x[17:0];
        end
    endfunction

    function signed [15:0] sat16;
        input signed [18:0] x;
        begin
            if (x > 19'sd32767)
                sat16 = 16'sd32767;
            else if (x < -19'sd32768)
                sat16 = -16'sd32768;
            else
                sat16 = x[15:0];
        end
    endfunction

    // -------------------------------------------------------------------------
    // 时钟（单一 50 MHz 时钟域）与上电复位
    // -------------------------------------------------------------------------
    wire clk;
    wire tick_bck4;               // 4×BCK = 6.25 MHz 时钟使能
    wire sample_tick;             // 每个采样 1 拍（48.828 kHz），来自 i2s_dac_tx
    reg  rst_n;                   // 内部同步复位，低有效
    reg  [POR_BITS-1:0] por_cnt;

    pll_audio #(
        .TICK_NUM (1),            // 50 MHz / 8 = 6.25 MHz，整数分频
        .TICK_DEN (8)
    ) u_pll_audio (
        .i_clk_50m   (i_clk_50m),
        .i_rst_n     (rst_n),
        .o_clk_audio (clk),
        .o_tick      (tick_bck4)
    );

    // 不用 initial：FPGA 配置完成后寄存器为 0，rst_n = 0（复位有效），计满后释放
    always @(posedge clk) begin
        if (por_cnt != {POR_BITS{1'b1}})
            por_cnt <= por_cnt + 1'b1;
        rst_n <= (por_cnt == {POR_BITS{1'b1}});
    end

    // -------------------------------------------------------------------------
    // 板上按键 key[0] ~ key[3]
    // -------------------------------------------------------------------------
    wire fx_press, vol_press, rec_press, tone_press;

    key_debounce #(.STABLE (DEB_SMP)) u_key_fx (
        .clk (clk), .rst_n (rst_n), .tick (sample_tick), .key_n (i_key_fx_n),
        .down (), .press (fx_press), .rls ()
    );
    key_debounce #(.STABLE (DEB_SMP)) u_key_vol (
        .clk (clk), .rst_n (rst_n), .tick (sample_tick), .key_n (i_key_vol_n),
        .down (), .press (vol_press), .rls ()
    );
    key_debounce #(.STABLE (DEB_SMP)) u_key_rec (
        .clk (clk), .rst_n (rst_n), .tick (sample_tick), .key_n (i_key_rec_n),
        .down (), .press (rec_press), .rls ()
    );
    key_debounce #(.STABLE (DEB_SMP)) u_key_tone (
        .clk (clk), .rst_n (rst_n), .tick (sample_tick), .key_n (i_key_tone_n),
        .down (), .press (tone_press), .rls ()
    );

    reg [2:0] fx_mode;            // 0 干声，1 混响，2 延迟，3 合唱，4 三个全开
    reg [1:0] vol;                // 音量：右移位数，0 = 0 dB … 3 = -18 dB
    reg [1:0] tone;               // 0 = 钢琴，1 = 电吉他，2 = 大提琴

    always @(posedge clk) begin
        if (!rst_n) begin
            fx_mode <= 3'd0;
            vol     <= VOL0;
            tone    <= T_PIANO;
        end else begin
            if (fx_press)
                fx_mode <= (fx_mode == 3'd4) ? 3'd0 : fx_mode + 3'd1;
            if (vol_press)
                vol <= vol + 2'd1;                // 再小 6 dB；-18 dB 之后回到 0 dB
            if (tone_press)
                tone <= (tone == T_CELLO) ? T_PIANO : tone + 2'd1;
        end
    end

    wire rev_en   = (fx_mode == 3'd1) || (fx_mode == 3'd4);
    wire dly_en   = (fx_mode == 3'd2) || (fx_mode == 3'd4);
    wire cho_en   = (fx_mode == 3'd3) || (fx_mode == 3'd4);
    wire tone_gtr = (tone == T_GUITAR);
    wire tone_cel = (tone == T_CELLO);

    // -------------------------------------------------------------------------
    // 触摸键盘：SCL 同时送到两个针；SDO 的两个针都有内部上拉，没接的针读到 1，
    //   所以两个针相与 = 接了键盘那个针的电平（蓝牙的 TXD 两个针同理）
    // -------------------------------------------------------------------------
    wire        tp_scl;
    wire [15:0] tp_keys;
    wire        tp_err;

    assign o_tp_scl = {2{tp_scl}};

    ttp229_rd #(
        .ACTIVE_HIGH (TOUCH_HIGH)
    ) u_ttp229_rd (
        .clk   (clk),
        .rst_n (rst_n),
        .o_scl (tp_scl),
        .i_sdo (&i_tp_sdo),
        .keys  (tp_keys),
        .err   (tp_err)
    );

    wire        tk_valid, tk_on, tk_ready;
    wire [5:0]  tk_note;
    wire [1:0]  group;

    touch_keys #(
        .KEY_REVERSE (KEY_REVERSE)
    ) u_touch_keys (
        .clk      (clk),
        .rst_n    (rst_n),
        .keys     (tp_keys),
        .ev_valid (tk_valid),
        .ev_on    (tk_on),
        .ev_note  (tk_note),
        .ev_ready (tk_ready),
        .group    (group)
    );

    // -------------------------------------------------------------------------
    // 复音分配 + NV 个声部（每个声部：加法合成引擎（钢琴 / 大提琴）+ 电吉他，击键时按当前音色选用）
    // -------------------------------------------------------------------------
    wire [NV-1:0]    v_on, v_off;
    wire [6*NV-1:0]  v_note;
    wire [8*NV-1:0]  v_gain;
    wire [16*NV-1:0] v_pbus;          // 钢琴 / 大提琴样本
    wire [16*NV-1:0] v_gbus;          // 电吉他样本
    wire [NV-1:0]    v_valid;

    voice_alloc #(
        .NV (NV)
    ) u_voice_alloc (
        .clk      (clk),
        .rst_n    (rst_n),
        .ev_valid (tk_valid),
        .ev_on    (tk_on),
        .ev_note  (tk_note),
        .ev_gain  (TGAIN),
        .ev_ready (tk_ready),
        .v_on     (v_on),
        .v_off    (v_off),
        .v_note   (v_note),
        .v_gain   (v_gain)
    );

    // 击键送给当前音色的发声单元，同时让另一个单元止音（声部被换成另一种音色时，旧音不会一直响）
    genvar gv;
    generate
        for (gv = 0; gv < NV; gv = gv + 1) begin : g_voice
            piano_voice #(
                .LUT_ADDR_W (12),
                .LUT_FILE   (LUT_FILE),
                .VIB_INC    (112 + gv * 2)            // 各声部颤音 5.2 ~ 5.9 Hz，几个音一起拉时更自然
            ) u_piano_voice (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_en    (sample_tick),
                .note_on      (v_on[gv] & ~tone_gtr),
                .note_off     (v_off[gv] | (v_on[gv] & tone_gtr)),
                .note         (v_note[gv*6 +: 6]),
                .patch        (tone_cel),
                .pure         (1'b0),
                .sample       (v_pbus[gv*16 +: 16]),
                .sample_valid (v_valid[gv])
            );

            guitar_voice #(
                .SEED (16'hACE1 ^ (gv * 16'h1357))
            ) u_guitar_voice (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_en    (sample_tick),
                .note_on      (v_on[gv] & tone_gtr),
                .note_off     (v_off[gv] | (v_on[gv] & ~tone_gtr)),
                .note         (v_note[gv*6 +: 6]),
                .sample       (v_gbus[gv*16 +: 16]),
                .sample_valid ()
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // 混音：钢琴 / 大提琴总线 = Σ 样本 × 力度 / 256，电吉他总线同理（1.0 = 32768，18 位留 4 倍余量）
    //   加法合成声部约 450 个时钟算完（电吉他 11 个时钟），之后 NV + 2 拍混完。
    //   增益每个采样最多变 4（0 → 255 约 1.3 ms），重弹正在响的声部时音量平滑过渡，不会“咔”一声。
    // -------------------------------------------------------------------------
    localparam [1:0] M_IDLE = 2'd0, M_RUN = 2'd1, M_FLUSH = 2'd2, M_OUT = 2'd3;

    reg  [1:0]         mst;
    reg  [3:0]         mi;
    reg  [8*NV-1:0]    g_cur;         // 每个声部当前使用的增益（向 v_gain 渐变）
    reg  signed [24:0] prod_p, prod_g;
    reg                prod_v;
    reg  signed [31:0] acc_p, acc_g;
    reg  signed [17:0] bus_p, bus_g;
    reg                mix_valid;

    wire signed [15:0] m_vp = v_pbus[mi*16 +: 16];
    wire signed [15:0] m_vg = v_gbus[mi*16 +: 16];
    wire        [7:0]  m_gc = g_cur[mi*8 +: 8];
    wire        [7:0]  m_gt = v_gain[mi*8 +: 8];
    wire        [7:0]  m_gn = (m_gt > m_gc) ? ((m_gt - m_gc > 8'd4) ? m_gc + 8'd4 : m_gt)
                                            : ((m_gc - m_gt > 8'd4) ? m_gc - 8'd4 : m_gt);
    wire signed [31:0] sh_p = acc_p >>> 8;
    wire signed [31:0] sh_g = acc_g >>> 8;

    always @(posedge clk) begin
        if (!rst_n) begin
            mst       <= M_IDLE;
            mi        <= 4'd0;
            g_cur     <= {(8*NV){1'b0}};
            prod_p    <= 25'sd0;
            prod_g    <= 25'sd0;
            prod_v    <= 1'b0;
            acc_p     <= 32'sd0;
            acc_g     <= 32'sd0;
            bus_p     <= 18'sd0;
            bus_g     <= 18'sd0;
            mix_valid <= 1'b0;
        end else begin
            prod_v    <= 1'b0;
            mix_valid <= 1'b0;
            if (prod_v) begin
                acc_p <= acc_p + prod_p;
                acc_g <= acc_g + prod_g;
            end
            case (mst)
            M_IDLE: if (v_valid[0]) begin
                mi    <= 4'd0;
                acc_p <= 32'sd0;
                acc_g <= 32'sd0;
                mst   <= M_RUN;
            end
            M_RUN: begin
                prod_p           <= m_vp * $signed({1'b0, m_gc});
                prod_g           <= m_vg * $signed({1'b0, m_gc});
                prod_v           <= 1'b1;
                g_cur[mi*8 +: 8] <= m_gn;
                if (mi == NV - 1)
                    mst <= M_FLUSH;
                else
                    mi <= mi + 4'd1;
            end
            M_FLUSH: mst <= M_OUT;                    // 最后一个乘积在这一拍累加
            default: begin                            // M_OUT
                bus_p     <= sat18(sh_p);
                bus_g     <= sat18(sh_g);
                mix_valid <= 1'b1;
                mst       <= M_IDLE;
            end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 电吉他音箱（过载 + 音箱模拟）→ 和钢琴 / 大提琴相加 → 合唱 → 延迟 → 混响
    // -------------------------------------------------------------------------
    wire               amp_valid;
    wire signed [17:0] amp_out;

    gtr_amp u_gtr_amp (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (mix_valid),
        .din       (bus_g),
        .out_valid (amp_valid),
        .dout      (amp_out)
    );

    wire signed [18:0] dry_sum = bus_p + amp_out;
    wire signed [17:0] dry     = sat18(dry_sum);

    wire               cho_valid, dly_valid, rev_valid;
    wire signed [17:0] cho_l, cho_r, dly_l, dly_r, rev_l, rev_r;

    fx_chorus u_fx_chorus (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (cho_en),
        .in_valid  (amp_valid),
        .din       (dry),
        .out_valid (cho_valid),
        .dout_l    (cho_l),
        .dout_r    (cho_r)
    );

    fx_delay u_fx_delay (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (dly_en),
        .in_valid  (cho_valid),
        .din_l     (cho_l),
        .din_r     (cho_r),
        .out_valid (dly_valid),
        .dout_l    (dly_l),
        .dout_r    (dly_r)
    );

    fx_reverb u_fx_reverb (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (rev_en),
        .in_valid  (dly_valid),
        .din_l     (dly_l),
        .din_r     (dly_r),
        .out_valid (rev_valid),
        .dout_l    (rev_l),
        .dout_r    (rev_r)
    );

    // 音量（右移）并限幅到 16 位；下一帧开头由 i2s_dac_tx 取走
    reg  signed [15:0] out_l, out_r;
    wire signed [18:0] vol_l = rev_l >>> vol;
    wire signed [18:0] vol_r = rev_r >>> vol;

    always @(posedge clk) begin
        if (!rst_n) begin
            out_l <= 16'sd0;
            out_r <= 16'sd0;
        end else if (rev_valid) begin
            out_l <= sat16(vol_l);
            out_r <= sat16(vol_r);
        end
    end

    // -------------------------------------------------------------------------
    // 录音 + 蓝牙
    // -------------------------------------------------------------------------
    wire        bt_tx;
    wire [2:0]  rec_st;
    wire [15:0] t_bcd;
    wire [11:0] p_bcd;
    wire        rec_blink, pc_alive;

    recorder u_recorder (
        .clk       (clk),
        .rst_n     (rst_n),
        .rec_press (rec_press),
        .ev_valid  (tk_valid & tk_ready),       // 事件被声部分配接收的那一拍，每个事件只录一次
        .ev_on     (tk_on),
        .ev_note   (tk_note),
        .tone      (tone),
        .fx        (fx_mode),
        .vol       (vol),
        .o_tx      (bt_tx),
        .i_rx      (&i_bt_rx),
        .rec_st    (rec_st),
        .t_bcd     (t_bcd),
        .p_bcd     (p_bcd),
        .blink     (rec_blink),
        .pc_alive  (pc_alive)
    );

    assign o_bt_tx = {2{bt_tx}};

    // -------------------------------------------------------------------------
    // 实时频谱 + OLED：分析混响输出（左右平均，音量调节前，所以画面不随音量变）
    // -------------------------------------------------------------------------
    wire signed [18:0] mono_sum = rev_l + rev_r;
    wire signed [15:0] mono     = sat16(mono_sum >>> 1);
    wire               ana_start, ana_done, oled_ok;
    wire [159:0]       bar_h, bar_pk;

    spectrum u_spectrum (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (rev_valid),
        .din      (mono),
        .start    (ana_start),
        .done     (ana_done),
        .bar_h    (bar_h),
        .bar_pk   (bar_pk)
    );

    wire [9:0] pix_idx;
    wire [7:0] pix_data;
    wire       oled_scl, oled_sda_o, oled_sda_oe;

    oled_frame u_oled_frame (
        .clk    (clk),
        .idx    (pix_idx),
        .tone   (tone),
        .group  (group),
        .vol    (vol),
        .fx     (fx_mode),
        .rec_st (rec_st),
        .t_bcd  (t_bcd),
        .p_bcd  (p_bcd),
        .blink  (rec_blink),
        .bar_h  (bar_h),
        .bar_pk (bar_pk),
        .data   (pix_data)
    );

    oled_i2c u_oled_i2c (
        .clk       (clk),
        .rst_n     (rst_n),
        .scl       (oled_scl),
        .sda_o     (oled_sda_o),
        .sda_oe    (oled_sda_oe),
        .sda_i     (&io_oled_sda),          // 没接的那个针被上拉为 1，相与 = 接了屏幕那个针
        .pix_idx   (pix_idx),
        .pix_data  (pix_data),
        .ana_start (ana_start),
        .ana_done  (ana_done),
        .ack_ok    (oled_ok)
    );

    assign o_oled_scl  = {2{oled_scl}};
    assign io_oled_sda = oled_sda_oe ? {2{oled_sda_o}} : 2'bzz;

    // -------------------------------------------------------------------------
    // 指示灯（板上 LED 低电平亮，输出取反）
    //   摸着触摸键盘时 led[4:0] 显示被摸的键号（最小的那个，1 ~ 16），led[5] 亮；否则显示音色 / 效果器
    // -------------------------------------------------------------------------
    reg  [24:0] blink_cnt;            // 不复位：FPGA 配置完成后为 0
    reg  [4:0]  key_id;               // 被摸的最小键号 KEY1 ~ KEY16，0 = 没有
    integer     kb;

    always @(posedge clk)
        blink_cnt <= blink_cnt + 25'd1;

    always @(*) begin
        key_id = 5'd0;
        for (kb = 15; kb >= 0; kb = kb - 1)
            if (tp_keys[kb])
                key_id = kb + 1;
    end

    wire       touching   = !tp_err && (tp_keys != 16'd0);
    wire       led5       = (rec_st == 3'd1) ? 1'b1 :                         // 录音中：常亮
                            (rec_st == 3'd3) ? blink_cnt[21] :                // 发送中：快闪
                            (tp_err || !oled_ok) ? blink_cnt[24] : 1'b0;      // 出错：慢闪
    wire [1:0] tone_led   = tone + 2'd1;          // 钢琴 01，电吉他 10，大提琴 11
    wire [5:0] led_status = {led5, cho_en, dly_en, rev_en, tone_led};
    wire [5:0] led_on     = touching ? {1'b1, key_id} : led_status;

    assign o_led_n = ~led_on;

    // -------------------------------------------------------------------------
    // 串行音频输出到 PT8211（立体声），格式与官方例程 audio_drive 相同
    // -------------------------------------------------------------------------
    i2s_dac_tx u_i2s_dac_tx (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick_bck4),
        .data_l     (out_l),
        .data_r     (out_r),
        .sample_req (sample_tick),
        .o_bck      (o_i2s_bck),
        .o_lrck     (o_i2s_lrck),
        .o_sdata    (o_i2s_sdata)
    );

endmodule
