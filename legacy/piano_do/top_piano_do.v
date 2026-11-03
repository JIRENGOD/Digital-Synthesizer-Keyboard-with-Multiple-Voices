`timescale 1ns/1ps
// =============================================================================
// top_piano_do.v —— 上电后用钢琴音色自动弹 C 大调音阶“哆 来 咪 发 唆 拉 西 哆”（C4 ~ C5），
//                   从底板 3.5 mm 耳机座输出
//   板卡：Tang Mega 138K Pro 核心板（GW5AST-LV138FPG676AC1/I0）+ Tang Mega 138K Pro Dock 底板
//   引脚：按 tang_mega_138k-pro_pins.cst；音频输出方式参考官方 audio 例程（见 i2s_dac_tx.v）
// -----------------------------------------------------------------------------
// 声音通路（Dock 原理图 FPGA_AUDIO 页）：
//   FPGA ─串行音频→ U6 PT8211（DAC）→ LMV321（低通滤波）→ U32 XPT4809（耳机功放）
//        → J29 3.5 mm 耳机座 → 音频线 → 外接有源音箱（左右声道内容相同）
//   o_pa_en（H17，引脚表里没有，按 Dock 原理图）经 R248 接 XPT4809 的 EN 脚：
//   与官方例程一样常为 1（功放常开）。
//   3.5 mm 口电平比普通线路输出高，信号太满会让音箱输入过载、声音发糊，
//   所以上电音量默认 -12 dB，可用 key[1] 调。
//
// 模块层次：
//   top_piano_do
//     ├─ pll_audio     50 MHz / 8 → 6.25 MHz 时钟使能（= 4 × BCK，不用 PLL）
//     ├─ key_debounce  ×3  按键 key[0]/key[1]/key[2] 同步 + 消抖
//     ├─ piano_voice   钢琴音色合成（src/piano/）；pure = 1 时只出基音纯正弦（对照用）
//     │    ├─ piano_params  钢琴音色参数表：8 个音 C4 ~ C5（采样率 48 828.125 Hz）
//     │    └─ sine_lut      4096 点正弦表
//     └─ i2s_dac_tx    PT8211 串行发送器：每帧 32 BCK、每声道 16 位，格式与官方 audio_drive 相同
//
// 复位：*** 同步复位，低电平有效 ***，由内部上电复位计数器产生（约 1.3 ms），不需要按键。
//
// 上电后的过程：
//   0 ~ 300 ms  发送静音
//   300 ms 起   依次弹 哆 来 咪 发 唆 拉 西 高音哆：每 0.5 s 弹一个音，每个音按住 0.4 s 后抬键
//               （制音器止音），最后的高音哆按住 1.5 s
//   之后：AUTO_PLAY = 1 时停 1.5 s 再从哆弹下一遍，循环；AUTO_PLAY = 0 时只弹一遍
// 按键和指示灯（编号按 tang_mega_138k-pro_pins.cst）：
//   key[0]（K16）：按一下 = 立刻从哆开始重新弹一遍
//   key[1]（G15）：音量，每按一次减 6 dB：-12 → -18 → 0 → -6 → -12 dB …（上电 -12 dB）
//   key[2]（F15）：切换音源：钢琴 ↔ 纯正弦（同样按音阶变调、音不断，用来核对音高、检查通路是否干净）
//   led[0] ~ led[5]（R26 J14 L20 M25 N21 N23）：流水灯 —— 每 0.1 s 从 led[0] 到 led[5] 依次点亮，
//     全亮后再从 led[0] 起依次熄灭，循环。灯在跑 = 比特流已经烧进 FPGA、50 MHz 时钟正常。
// =============================================================================
module top_piano_do #(
    parameter integer POR_BITS      = 16,     // 上电复位：2^16 个时钟 ≈ 1.3 ms
    parameter integer FIRST_NOTE_MS = 300,    // 复位后多久弹第一个音（ms）
    parameter integer AUTO_PLAY     = 1,      // 1 = 弹完一遍停 REST_MS 再弹，循环；0 = 只弹一遍
    parameter integer STEP_MS       = 500,    // 相邻两个音的击键间隔（ms）
    parameter integer HOLD_MS       = 400,    // 每个音按住多久后抬键（ms），应小于 STEP_MS
    parameter integer LAST_HOLD_MS  = 1500,   // 最后的高音哆按住多久（ms）
    parameter integer REST_MS       = 1500,   // 一遍弹完（抬键）后停多久再弹下一遍（ms）
    parameter integer KEY_EN        = 1,      // 1 = 按 key[0] 从哆重新弹
    parameter integer DEBOUNCE_MS   = 20,     // 按键消抖时间（ms）
    parameter integer VOL_INIT      = 2,      // 上电音量：0 = 0 dB，1 = -6 dB，2 = -12 dB，3 = -18 dB
    parameter integer SINE_TEST     = 0,      // 上电音源：0 = 钢琴，1 = 纯正弦
    parameter         LUT_FILE      = "sine_lut.hex"
) (
    input  wire       i_clk_50m,      // P16   clk：核心板 50 MHz 晶振
    input  wire       i_key_n,        // K16   key[0]，按下为 0：从哆重新弹
    input  wire       i_key_vol_n,    // G15   key[1]，按下为 0：音量
    input  wire       i_key_mode_n,   // F15   key[2]，按下为 0：钢琴 / 纯正弦
    output wire       o_i2s_bck,      // E22   HP_BCK → U6 PT8211 BCK
    output wire       o_i2s_lrck,     // F24   HP_WS  → U6 PT8211 WS
    output wire       o_i2s_sdata,    // G24   HP_DIN → U6 PT8211 DIN
    output wire       o_pa_en,        // H17   PA_EN  → R248 → U32 XPT4809 EN，常为 1
    output wire [5:0] o_led_n         // led[0..5] = R26 J14 L20 M25 N21 N23，低电平亮：流水灯
);

    // 时间参数换算成采样数（Fs = 48.828 kHz，1 ms ≈ 49 个采样）；20 位计数器最长约 21 s
    localparam integer FS_KHZ    = 49;
    localparam [19:0]  FIRST_SMP = FIRST_NOTE_MS * FS_KHZ;
    localparam [19:0]  STEP_SMP  = STEP_MS       * FS_KHZ;
    localparam [19:0]  HOLD_SMP  = HOLD_MS       * FS_KHZ;
    localparam [19:0]  LAST_SMP  = LAST_HOLD_MS  * FS_KHZ;
    localparam [19:0]  REST_SMP  = REST_MS       * FS_KHZ;
    localparam integer DEB_SMP   = DEBOUNCE_MS   * FS_KHZ;
    localparam [22:0]  LED_LAST  = 23'd4999999;    // 流水灯：50 MHz 下每 0.1 s 走一步
    localparam [1:0]   VOL0      = VOL_INIT;
    localparam [2:0]   NOTE_LAST = 3'd7;           // 第 8 个音 = 高音哆（piano_params.v 的音 7）

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
    // 按键：key[0] 从哆重新弹，key[1] 音量，key[2] 切换音源
    // -------------------------------------------------------------------------
    wire key_press;               // key[0] 确认按下，1 拍
    wire vol_press;               // key[1] 确认按下，1 拍
    wire mode_press;              // key[2] 确认按下，1 拍

    key_debounce #(
        .STABLE (DEB_SMP)
    ) u_key_play (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (sample_tick),
        .key_n (i_key_n),
        .down  (),
        .press (key_press),
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
    ) u_key_mode (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (sample_tick),
        .key_n (i_key_mode_n),
        .down  (),
        .press (mode_press),
        .rls   ()
    );

    // -------------------------------------------------------------------------
    // 自动弹音阶（按采样计数）：哆 来 咪 发 唆 拉 西 高音哆
    //   Q_WAIT ──到时间──► 弹哆 ─► Q_HOLD ──按住到时间──► 抬键 ─► Q_GAP ──到击键间隔──► 弹下一个音 ─► Q_HOLD …
    //   高音哆按住到时间后抬键 ─► Q_WAIT（AUTO_PLAY = 1，停 REST_MS 再弹）或 Q_STOP（AUTO_PLAY = 0）
    //   note_idx 与 auto_on 在同一拍更新，piano_voice 击键时锁存 note_idx
    // -------------------------------------------------------------------------
    localparam [1:0] Q_WAIT = 2'd0,   // 等待开弹：上电延时 / 两遍之间的停顿
                     Q_HOLD = 2'd1,   // 按住当前音
                     Q_GAP  = 2'd2,   // 已抬键，等下一个音
                     Q_STOP = 2'd3;   // 弹完一遍后停住，等 key[0]

    reg  [1:0]  seq;
    reg  [19:0] t_cnt;            // 采样计数：Q_WAIT 从进入算起；Q_HOLD/Q_GAP 从本音击键算起
    reg  [2:0]  note_idx;         // 当前音：0 = 哆 … 7 = 高音哆
    reg         started;          // 第一次击键之后为 1
    reg         auto_on, auto_off;
    wire        last_note = (note_idx == NOTE_LAST);
    wire        restart   = started && (KEY_EN != 0) && key_press;   // 第一次自动击键之前不响应按键

    always @(posedge clk) begin
        if (!rst_n) begin
            seq      <= Q_WAIT;
            t_cnt    <= 20'd0;
            note_idx <= 3'd0;
            started  <= 1'b0;
            auto_on  <= 1'b0;
            auto_off <= 1'b0;
        end else begin
            auto_on  <= 1'b0;
            auto_off <= 1'b0;
            if (restart) begin                        // key[0]：立刻从哆重新弹一遍
                seq      <= Q_HOLD;
                t_cnt    <= 20'd0;
                note_idx <= 3'd0;
                auto_on  <= 1'b1;
            end else if (sample_tick) begin
                t_cnt <= t_cnt + 20'd1;
                case (seq)
                Q_WAIT: if (t_cnt >= (started ? REST_SMP : FIRST_SMP)) begin
                    seq      <= Q_HOLD;
                    t_cnt    <= 20'd0;
                    note_idx <= 3'd0;
                    started  <= 1'b1;
                    auto_on  <= 1'b1;                 // 弹哆
                end
                Q_HOLD: if (t_cnt >= (last_note ? LAST_SMP : HOLD_SMP)) begin
                    auto_off <= 1'b1;                 // 抬键（制音器止音）
                    if (!last_note) begin
                        seq <= Q_GAP;                 // t_cnt 接着数，到 STEP_SMP 弹下一个音
                    end else begin
                        seq   <= (AUTO_PLAY != 0) ? Q_WAIT : Q_STOP;
                        t_cnt <= 20'd0;
                    end
                end
                Q_GAP: if (t_cnt >= STEP_SMP) begin
                    seq      <= Q_HOLD;
                    t_cnt    <= 20'd0;
                    note_idx <= note_idx + 3'd1;
                    auto_on  <= 1'b1;                 // 弹下一个音
                end
                default: ;                            // Q_STOP：等 key[0]
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // 音量、音源选择
    // -------------------------------------------------------------------------
    reg  [1:0]  vol;              // 音量：样本右移位数，0 = 0 dB … 3 = -18 dB
    reg         mode_sine;        // 1 = 输出纯正弦，0 = 钢琴

    always @(posedge clk) begin
        if (!rst_n) begin
            vol       <= VOL0;
            mode_sine <= (SINE_TEST != 0);
        end else begin
            if (vol_press)
                vol <= vol + 2'd1;                // 再小 6 dB；-18 dB 之后回到 0 dB
            if (mode_press)
                mode_sine <= ~mode_sine;
        end
    end

    // -------------------------------------------------------------------------
    // LED 流水灯：只用 50 MHz 时钟，不受复位和音频部分影响
    //   约翰逊计数器，初值 0（FPGA 配置完成后寄存器为 0），1 = 点亮：
    //   000000 → 000001 → 000011 → … → 111111 → 111110 → … → 000000
    //   即 led[0] 到 led[5] 依次点亮，再从 led[0] 起依次熄灭。板上 LED 低电平亮，输出取反。
    // -------------------------------------------------------------------------
    reg [22:0] led_cnt;
    reg [5:0]  led_on;

    always @(posedge clk) begin
        if (led_cnt == LED_LAST) begin
            led_cnt <= 23'd0;
            led_on  <= {led_on[4:0], ~led_on[5]};
        end else begin
            led_cnt <= led_cnt + 23'd1;
        end
    end

    assign o_led_n = ~led_on;

    // -------------------------------------------------------------------------
    // 音源：钢琴；mode_sine = 1 时引擎只输出当前音的基音纯正弦（同一个振荡器、同一张正弦表，对照用）
    // -------------------------------------------------------------------------
    wire signed [15:0] voice;

    piano_voice #(
        .LUT_ADDR_W (12),
        .LUT_FILE   (LUT_FILE)
    ) u_piano_voice (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_tick),
        .note_on      (auto_on),
        .note_off     (auto_off),
        .note         ({1'b0, note_idx}),
        .pure         (mode_sine),
        .sample       (voice),
        .sample_valid ()
    );

    wire signed [15:0] out = voice >>> vol;

    // -------------------------------------------------------------------------
    // 串行音频输出到 PT8211（左右声道相同），格式与官方例程 audio_drive 相同
    // -------------------------------------------------------------------------
    i2s_dac_tx u_i2s_dac_tx (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick_bck4),
        .data_l     (out),
        .data_r     (out),
        .sample_req (sample_tick),
        .o_bck      (o_i2s_bck),
        .o_lrck     (o_i2s_lrck),
        .o_sdata    (o_i2s_sdata)
    );

endmodule
