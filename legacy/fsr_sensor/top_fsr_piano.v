`timescale 1ns/1ps
// =============================================================================
// top_fsr_piano.v —— 压力传感器钢琴：16 个压力点 = 16 个琴键，按下发声，压得越重越响，8 复音
//   板卡：Tang Mega 138K Pro 核心板（GW5AST-LV138FPG676AC1/I0）+ Tang Mega 138K Pro Dock 底板
//   传感器：4×4 薄膜压力传感器（IMM0092 小量程）+ 16 路通道扩展模块（S0~S3 选通，AO 输出电压）
// -----------------------------------------------------------------------------
// 信号通路：
//   压力点 ─对插─► 16 路模块 ─AO─► 电阻分压 ÷4 ─► 底板 J3 的 ADCV_P ─► FPGA 片内 ADC（0 ~ 1 V）
//   FPGA ─S0~S3（PMOD2）─► 16 路模块：依次选通 16 个压力点，约 2.4 ms 扫完一遍
//   → fsr_keys 判断击键 / 抬键、算力度 → voice_alloc 分配声部 → NV 个 piano_voice 并行合成
//   → 按力度加权混音 → 音量 → i2s_dac_tx → U6 PT8211 → 3.5 mm 耳机座
// 延迟：按下到发出击键 ≤ 2 遍扫描（约 5 ms），再加 1 个采样（20 µs）就开始出声，满足 ≤ 10 ms。
//
// 琴键（压力点 = 16 路模块的 0 ~ 15 号接口，音表见 piano_params.v）：
//    0 哆 C4    1 来 D4    2 咪 E4    3 发 F4    4 唆 G4    5 拉 A4    6 西 B4    7 高音哆 C5
//    8 来 D5    9 咪 E5   10 发 F5   11 唆 G5   12 拉 A5   13 西 B5   14 哆 C6   15 来 D6
//
// 模块层次：
//   top_fsr_piano
//     ├─ pll_audio     50 MHz / 8 → 6.25 MHz 时钟使能（= 4 × BCK，不用 PLL）
//     ├─ key_debounce  ×3  按键 key[0]/key[1]/key[2] 同步 + 消抖
//     ├─ fsr_scan      S0~S3 选通 + 片内 ADC 读数（src/sensor/，ADC 由 gowin_adc.v 例化）
//     ├─ fsr_keys      基线校准、击键 / 抬键判断、力度
//     ├─ voice_alloc   复音分配（同键重击、空闲优先、全忙时抢最早的）
//     ├─ piano_voice   ×NV  钢琴音色合成（src/piano/），每个声部 21 个振荡器，NV 个同时运算
//     └─ i2s_dac_tx    PT8211 串行发送器（格式与官方 audio_drive 相同）
//
// 复位：*** 同步复位，低电平有效 ***，由内部上电复位计数器产生（约 1.3 ms），不需要按键。
//
// 按键和指示灯（编号按 tang_mega_138k-pro_pins.cst）：
//   key[0]（K16）：弹一遍演示音阶 哆 ~ 高音哆（不经过传感器，用来检查声音通路）；上电 0.3 s 后也自动弹一遍
//   key[1]（G15）：音量，每按一次减 6 dB：-12 → -18 → 0 → -6 → -12 dB …（上电 -12 dB）
//   key[2]（F15）：重新校准传感器零点（先松开所有压力点再按，约 0.1 s 完成；上电时自动校准一次）
//   led[0] ~ led[5]（R26 J14 L20 M25 N21 N23）：
//     没有按压：流水灯（说明比特流在运行）；
//     有按压：  亮灯个数表示最重的那个点的压力（从 led[0] 起亮 1 ~ 6 个，每多一个约重一倍）；
//     6 个灯一起闪：ADC 没有响应。
// =============================================================================
module top_fsr_piano #(
    parameter integer NV            = 8,      // 复音数 = 并行的 piano_voice 个数
    parameter integer POR_BITS      = 16,     // 上电复位：2^16 个时钟 ≈ 1.3 ms
    parameter integer FIRST_NOTE_MS = 300,    // 上电后多久弹演示音阶（ms）
    parameter integer DEMO_STEP_MS  = 500,    // 演示音阶：相邻两个音的间隔（ms）
    parameter integer DEMO_HOLD_MS  = 400,    // 演示音阶：每个音按住多久（ms）
    parameter integer DEMO_LAST_MS  = 1500,   // 演示音阶：最后的高音哆按住多久（ms）
    parameter integer DEMO_GAIN     = 200,    // 演示音阶的力度增益（255 = 满）
    parameter integer DEBOUNCE_MS   = 20,     // 按键消抖时间（ms）
    parameter integer VOL_INIT      = 2,      // 上电音量：0 = 0 dB，1 = -6 dB，2 = -12 dB，3 = -18 dB
    parameter integer TH_ON         = 48,     // 击键阈值（ADC 计数，见 fsr_keys.v）
    parameter integer TH_OFF        = 24,     // 抬键阈值
    parameter         LUT_FILE      = "sine_lut.hex"
) (
    input  wire       i_clk_50m,      // P16   clk：核心板 50 MHz 晶振
    input  wire       i_key_demo_n,   // K16   key[0]，按下为 0：弹演示音阶
    input  wire       i_key_vol_n,    // G15   key[1]，按下为 0：音量
    input  wire       i_key_cal_n,    // F15   key[2]，按下为 0：重新校准传感器
    output wire       o_i2s_bck,      // E22   HP_BCK → U6 PT8211 BCK
    output wire       o_i2s_lrck,     // F24   HP_WS  → U6 PT8211 WS
    output wire       o_i2s_sdata,    // G24   HP_DIN → U6 PT8211 DIN
    output wire       o_pa_en,        // H17   PA_EN  → R248 → U32 XPT4809 EN，常为 1
    output wire [5:0] o_led_n,        // led[0..5] = R26 J14 L20 M25 N21 N23，低电平亮
    output wire [3:0] o_mux_s_a,      // PMOD2 一排：S0~S3 = A17 B19 B20 C21
    output wire [3:0] o_mux_s_b       // PMOD2 另一排：S0~S3 = A18 A19 A20 B21（与上一排相同，接哪排都行）
);

    // 时间参数换算成采样数（Fs = 48.828 kHz，1 ms ≈ 49 个采样）；20 位计数器最长约 21 s
    localparam integer FS_KHZ    = 49;
    localparam [19:0]  FIRST_SMP = FIRST_NOTE_MS * FS_KHZ;
    localparam [19:0]  STEP_SMP  = DEMO_STEP_MS  * FS_KHZ;
    localparam [19:0]  HOLD_SMP  = DEMO_HOLD_MS  * FS_KHZ;
    localparam [19:0]  LAST_SMP  = DEMO_LAST_MS  * FS_KHZ;
    localparam integer DEB_SMP   = DEBOUNCE_MS   * FS_KHZ;
    localparam [22:0]  LED_LAST  = 23'd4999999;    // 流水灯：50 MHz 下每 0.1 s 走一步
    localparam [1:0]   VOL0      = VOL_INIT;
    localparam [7:0]   DGAIN     = DEMO_GAIN;
    localparam [11:0]  BAR1      = TH_ON;          // 指示灯：第 n 个灯亮的压力 = TH_ON × 2^(n-1)

    // 与官方例程一样，功放常开
    assign o_pa_en = 1'b1;

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
    // 按键：key[0] 演示音阶，key[1] 音量，key[2] 重新校准
    // -------------------------------------------------------------------------
    wire demo_press, vol_press, cal_press;

    key_debounce #(
        .STABLE (DEB_SMP)
    ) u_key_demo (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (sample_tick),
        .key_n (i_key_demo_n),
        .down  (),
        .press (demo_press),
        .rls   ()
    );

    key_debounce #(
        .STABLE (DEB_SMP)
    ) u_key_vol (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (sample_tick),
        .key_n (i_key_vol_n),
        .down  (),
        .press (vol_press),
        .rls   ()
    );

    key_debounce #(
        .STABLE (DEB_SMP)
    ) u_key_cal (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (sample_tick),
        .key_n (i_key_cal_n),
        .down  (),
        .press (cal_press),
        .rls   ()
    );

    // -------------------------------------------------------------------------
    // 压力传感器：扫描 16 个点 → 击键 / 抬键事件
    // -------------------------------------------------------------------------
    wire [3:0]  mux_sel;
    wire [3:0]  scan_ch;
    wire [11:0] scan_val;
    wire        scan_valid, scan_last, adc_err;

    fsr_scan u_fsr_scan (
        .clk     (clk),
        .rst_n   (rst_n),
        .o_sel   (mux_sel),
        .ch      (scan_ch),
        .value   (scan_val),
        .valid   (scan_valid),
        .last    (scan_last),
        .adc_err (adc_err)
    );

    assign o_mux_s_a = mux_sel;
    assign o_mux_s_b = mux_sel;

    wire        key_ev_valid, key_ev_on, key_ev_ready;
    wire [3:0]  key_ev_key;
    wire [7:0]  key_ev_gain;
    wire [11:0] scan_max;
    wire        calibrating;

    fsr_keys #(
        .TH_ON  (TH_ON),
        .TH_OFF (TH_OFF)
    ) u_fsr_keys (
        .clk         (clk),
        .rst_n       (rst_n),
        .cal_start   (cal_press),
        .ch          (scan_ch),
        .value       (scan_val),
        .valid       (scan_valid),
        .last        (scan_last),
        .ev_valid    (key_ev_valid),
        .ev_on       (key_ev_on),
        .ev_key      (key_ev_key),
        .ev_gain     (key_ev_gain),
        .ev_ready    (key_ev_ready),
        .scan_max    (scan_max),
        .calibrating (calibrating)
    );

    // -------------------------------------------------------------------------
    // 演示音阶：哆 来 咪 发 唆 拉 西 高音哆（键号 16 ~ 23 → 音 0 ~ 7）
    //   每 STEP 弹一个音，按住 HOLD 后抬键，最后的高音哆按住 LAST；上电后弹一遍，之后按 key[0] 再弹
    // -------------------------------------------------------------------------
    localparam [2:0] D_BOOT = 3'd0,   // 上电后等 FIRST_NOTE_MS
                     D_IDLE = 3'd1,   // 等 key[0]
                     D_ON   = 3'd2,   // 击键事件等待接收
                     D_HOLD = 3'd3,   // 按住
                     D_OFF  = 3'd4,   // 抬键事件等待接收
                     D_GAP  = 3'd5;   // 等下一个音

    reg  [2:0]  dseq;
    reg  [19:0] dcnt;                 // 采样计数：D_HOLD / D_GAP 从本音击键算起
    reg  [2:0]  dnote;
    reg         d_valid, d_on;
    wire        d_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            dseq    <= D_BOOT;
            dcnt    <= 20'd0;
            dnote   <= 3'd0;
            d_valid <= 1'b0;
            d_on    <= 1'b0;
        end else begin
            if (d_ready)
                d_valid <= 1'b0;
            case (dseq)
            D_BOOT: if (sample_tick) begin
                if (dcnt >= FIRST_SMP) begin
                    dnote   <= 3'd0;
                    d_valid <= 1'b1;
                    d_on    <= 1'b1;
                    dseq    <= D_ON;
                end else begin
                    dcnt <= dcnt + 20'd1;
                end
            end
            D_IDLE: if (demo_press) begin
                dnote   <= 3'd0;
                d_valid <= 1'b1;
                d_on    <= 1'b1;
                dseq    <= D_ON;
            end
            D_ON: if (d_ready) begin
                dcnt <= 20'd0;
                dseq <= D_HOLD;
            end
            D_HOLD: if (sample_tick) begin
                dcnt <= dcnt + 20'd1;
                if (dcnt >= ((dnote == 3'd7) ? LAST_SMP : HOLD_SMP)) begin
                    d_valid <= 1'b1;
                    d_on    <= 1'b0;
                    dseq    <= D_OFF;
                end
            end
            D_OFF: if (d_ready) begin
                dseq <= (dnote == 3'd7) ? D_IDLE : D_GAP;
            end
            D_GAP: if (sample_tick) begin
                dcnt <= dcnt + 20'd1;
                if (dcnt >= STEP_SMP) begin
                    dnote   <= dnote + 3'd1;
                    d_valid <= 1'b1;
                    d_on    <= 1'b1;
                    dseq    <= D_ON;
                end
            end
            default: dseq <= D_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 复音分配 + NV 个钢琴声部并行合成
    // -------------------------------------------------------------------------
    wire [NV-1:0]    v_on, v_off;
    wire [4*NV-1:0]  v_note;
    wire [8*NV-1:0]  v_gain;
    wire [16*NV-1:0] v_bus;           // 每个声部的 16 位样本
    wire [NV-1:0]    v_valid;

    voice_alloc #(
        .NV (NV)
    ) u_voice_alloc (
        .clk     (clk),
        .rst_n   (rst_n),
        .a_valid (key_ev_valid),
        .a_on    (key_ev_on),
        .a_id    ({1'b0, key_ev_key}),
        .a_gain  (key_ev_gain),
        .a_ready (key_ev_ready),
        .b_valid (d_valid),
        .b_on    (d_on),
        .b_id    ({2'b10, dnote}),
        .b_gain  (DGAIN),
        .b_ready (d_ready),
        .v_on    (v_on),
        .v_off   (v_off),
        .v_note  (v_note),
        .v_gain  (v_gain)
    );

    genvar gv;
    generate
        for (gv = 0; gv < NV; gv = gv + 1) begin : g_voice
            piano_voice #(
                .LUT_ADDR_W (12),
                .LUT_FILE   (LUT_FILE)
            ) u_piano_voice (
                .clk          (clk),
                .rst_n        (rst_n),
                .sample_en    (sample_tick),
                .note_on      (v_on[gv]),
                .note_off     (v_off[gv]),
                .note         (v_note[gv*4 +: 4]),
                .pure         (1'b0),
                .sample       (v_bus[gv*16 +: 16]),
                .sample_valid (v_valid[gv])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // 混音：Σ 声部样本 × 力度增益 / 256，再按音量右移并限幅
    //   所有声部时序相同，同时给出 sample_valid；之后 NV + 2 拍算完，下一帧开头由 i2s_dac_tx 取走。
    //   增益每个采样最多变 4（0 → 255 约 1.3 ms），重击正在响的声部时音量平滑过渡，不会“咔”一声。
    // -------------------------------------------------------------------------
    localparam [1:0] M_IDLE = 2'd0, M_RUN = 2'd1, M_FLUSH = 2'd2, M_OUT = 2'd3;

    reg  [1:0]          mst;
    reg  [3:0]          mi;
    reg  [8*NV-1:0]     g_cur;        // 每个声部当前使用的增益（向 v_gain 渐变）
    reg  signed [24:0]  prod;
    reg                 prod_v;
    reg  signed [31:0]  macc;
    reg  signed [15:0]  mix_out;
    reg  [1:0]          vol;          // 音量：右移位数，0 = 0 dB … 3 = -18 dB

    wire signed [15:0] m_v   = v_bus[mi*16 +: 16];
    wire        [7:0]  m_gc  = g_cur[mi*8 +: 8];
    wire        [7:0]  m_gt  = v_gain[mi*8 +: 8];
    wire        [7:0]  m_gn  = (m_gt > m_gc) ? ((m_gt - m_gc > 8'd4) ? m_gc + 8'd4 : m_gt)
                                             : ((m_gc - m_gt > 8'd4) ? m_gc - 8'd4 : m_gt);
    wire signed [31:0] m_sh  = macc >>> ({3'd0, vol} + 5'd8);

    always @(posedge clk) begin
        if (!rst_n) begin
            mst     <= M_IDLE;
            mi      <= 4'd0;
            g_cur   <= {(8*NV){1'b0}};
            prod    <= 25'sd0;
            prod_v  <= 1'b0;
            macc    <= 32'sd0;
            mix_out <= 16'sd0;
        end else begin
            prod_v <= 1'b0;
            if (prod_v)
                macc <= macc + prod;
            case (mst)
            M_IDLE: if (v_valid[0]) begin
                mi   <= 4'd0;
                macc <= 32'sd0;
                mst  <= M_RUN;
            end
            M_RUN: begin
                prod              <= m_v * $signed({1'b0, m_gc});
                prod_v            <= 1'b1;
                g_cur[mi*8 +: 8]  <= m_gn;
                if (mi == NV - 1)
                    mst <= M_FLUSH;
                else
                    mi <= mi + 4'd1;
            end
            M_FLUSH: mst <= M_OUT;                    // 最后一个乘积在这一拍累加
            default: begin                            // M_OUT
                if (m_sh > 32'sd32767)
                    mix_out <= 16'sd32767;
                else if (m_sh < -32'sd32768)
                    mix_out <= -16'sd32768;
                else
                    mix_out <= m_sh[15:0];
                mst <= M_IDLE;
            end
            endcase
        end
    end

    // 音量（key[1]）
    always @(posedge clk) begin
        if (!rst_n)
            vol <= VOL0;
        else if (vol_press)
            vol <= vol + 2'd1;                        // 再小 6 dB；-18 dB 之后回到 0 dB
    end

    // -------------------------------------------------------------------------
    // 指示灯：流水灯 / 压力条 / ADC 出错时全闪
    //   流水灯是约翰逊计数器，初值 0（FPGA 配置完成后寄存器为 0），不受复位影响：
    //   000000 → 000001 → 000011 → … → 111111 → 111110 → … → 000000。板上 LED 低电平亮，输出取反。
    // -------------------------------------------------------------------------
    reg [22:0] led_cnt;
    reg [5:0]  led_flow;
    reg [23:0] blink_cnt;

    always @(posedge clk) begin
        blink_cnt <= blink_cnt + 24'd1;
        if (led_cnt == LED_LAST) begin
            led_cnt  <= 23'd0;
            led_flow <= {led_flow[4:0], ~led_flow[5]};
        end else begin
            led_cnt <= led_cnt + 23'd1;
        end
    end

    wire [5:0] led_bar = (scan_max >= (BAR1 << 5)) ? 6'b111111 :
                         (scan_max >= (BAR1 << 4)) ? 6'b011111 :
                         (scan_max >= (BAR1 << 3)) ? 6'b001111 :
                         (scan_max >= (BAR1 << 2)) ? 6'b000111 :
                         (scan_max >= (BAR1 << 1)) ? 6'b000011 :
                         (scan_max >= BAR1)        ? 6'b000001 : 6'b000000;
    wire [5:0] led_on  = adc_err            ? {6{blink_cnt[23]}} :   // 约 3 Hz 全闪
                         (led_bar != 6'd0)  ? led_bar : led_flow;

    assign o_led_n = ~led_on;

    // -------------------------------------------------------------------------
    // 串行音频输出到 PT8211（左右声道相同），格式与官方例程 audio_drive 相同
    // -------------------------------------------------------------------------
    i2s_dac_tx u_i2s_dac_tx (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick_bck4),
        .data_l     (mix_out),
        .data_r     (mix_out),
        .sample_req (sample_tick),
        .o_bck      (o_i2s_bck),
        .o_lrck     (o_i2s_lrck),
        .o_sdata    (o_i2s_sdata)
    );

endmodule
