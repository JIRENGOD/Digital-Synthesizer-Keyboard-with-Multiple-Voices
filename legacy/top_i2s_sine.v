`timescale 1ns/1ps
// =============================================================================
// top_i2s_sine.v —— 里程碑 1：上电即响的 I2S 正弦测试音（Tang Mega 60K / GW5AT-LV60PG484A）
// -----------------------------------------------------------------------------
// 模块层次：
//   top_i2s_sine
//     ├─ pll_audio   50 MHz -> 512*Fs 时钟使能（分数分频；本版本不例化 PLL，原因见该文件）
//     ├─ nco_sine    32 位相位累加器，sample_en（=Fs）时前进一步
//     │    └─ sine_lut   4096 x 16 位有符号正弦表（$readmemh 加载 sine_lut.hex）
//     └─ i2s_master  产生 MCLK / BCK / LRCK / SDATA，每帧发出 sample_req
//
// 时钟方案：全部逻辑在 i_clk_50m 单一时钟域，用时钟使能（不是分频出来的新时钟）驱动：
//     CLK_MODE=0：Fs = 48 kHz（平均精确），MCLK 12.288 MHz，BCK 3.072 MHz，LRCK 48 kHz
//     CLK_MODE=1：Fs = 48.828125 kHz（50 MHz 整数分频，零附加抖动），MCLK 12.5 MHz
//
// 复位方案：*** 同步复位，低电平有效（rst_n = 0 时复位） ***
//     rst_n 由内部上电复位计数器产生，不需要外部复位按键：
//     GOWIN FPGA 配置完成后，未指定初值的普通触发器上电为 0，于是 rst_n=0（复位有效），
//     por_cnt 从 0 计满 2^POR_BITS-1（默认约 1.3 ms）后 rst_n 变 1 并一直保持。
//
// 上电时序（避免爆音）：
//     1. 复位期间：所有输出为 0。
//     2. 复位释放：I2S 时钟开始运行，SDATA 全 0（静音帧），让 DAC 锁定时钟、输出稳定。
//     3. 静音满 STARTUP_MUTE_FRAMES/2 帧：功放使能（仅当打开 TOP_USE_PA_EN）。
//     4. 静音满 STARTUP_MUTE_FRAMES 帧（默认 9600 帧 = 200 ms）：tone_en=1，
//        NCO 从相位 0 起振，第一个样本 sin(0)=0，波形连续，无直流台阶。
//
// 相位增量：PHASE_INC = round(F_out * 2^32 / Fs)，scripts/gen_sine_lut.py 会打印：
//     F_out     CLK_MODE=0 (48 kHz)    CLK_MODE=1 (48.828125 kHz)
//     440 Hz    32'd39370534           32'd38702809
//     1 kHz     32'd89478485           32'd87960930
// =============================================================================

// ---- 可选端口开关（改动后，constraints/top_i2s_sine.cst 中对应的约束也要同步增删）----
// DAC 有 MCLK（SCK/SYSCLK）引脚时保留；没有 MCLK 引脚就注释掉这一行。
`define TOP_USE_MCLK
// 原理图上 DAC 静音脚 / 功放使能脚接到了 FPGA 时打开，并在 CST 中约束 o_pa_en。
// `define TOP_USE_PA_EN

module top_i2s_sine #(
    parameter integer CLK_MODE            = 0,      // 0 = 48 kHz 分数分频；1 = 48.828125 kHz 整数分频
    parameter [31:0]  PHASE_INC           = (CLK_MODE == 1) ? 32'd38702809   // 440 Hz
                                                            : 32'd39370534,  // 440 Hz
    parameter integer I2S_FORMAT          = 0,      // 0 = 标准 I2S，1 = 左对齐，2 = 右对齐 16 位（PT8211 类）
    parameter integer VOLUME_SHIFT        = 2,      // 音量：样本算术右移位数，0 = 满幅，1 = -6 dB，2 = -12 dB
    parameter integer TEST_TONE_EN        = 1,      // 1 = 上电即响；0 = 只输出静音帧
    parameter integer STARTUP_MUTE_FRAMES = 9600,   // 起振前的静音帧数（1..65535），9600 帧 ≈ 200 ms
    parameter integer POR_BITS            = 16,     // 上电复位计数器位宽：2^16 拍 ≈ 1.3 ms
    parameter integer LUT_ADDR_W          = 12,     // 查找表深度 2^12 = 4096，须与 sine_lut.hex 行数一致
    parameter         LUT_FILE            = "sine_lut.hex",
    parameter         PA_EN_ACTIVE        = 1'b1    // o_pa_en 的有效电平（功放 SD/MUTE 脚低有效时改为 1'b0）
) (
    input  wire i_clk_50m,        // 板载 50 MHz 晶振
`ifdef TOP_USE_MCLK
    output wire o_i2s_mclk,       // 主时钟 256*Fs
`endif
    output wire o_i2s_bck,        // 位时钟 64*Fs
    output wire o_i2s_lrck,       // 帧时钟 / 声道选择，频率 Fs
    output wire o_i2s_sdata       // 串行数据，MSB 先发
`ifdef TOP_USE_PA_EN
   ,output wire o_pa_en           // 功放 / DAC 使能（可选）
`endif
);

    localparam integer TICK_NUM = (CLK_MODE == 1) ? 1 : 1536;
    localparam integer TICK_DEN = (CLK_MODE == 1) ? 2 : 3125;
    localparam [15:0]  MUTE_LAST  = STARTUP_MUTE_FRAMES - 1;
    localparam [15:0]  PA_ON_LAST = STARTUP_MUTE_FRAMES / 2;

    // -------------------------------------------------------------------------
    // 时钟
    // -------------------------------------------------------------------------
    wire clk_audio;      // = i_clk_50m
    wire tick_512fs;     // 512*Fs 时钟使能
    reg  rst_n;          // 内部同步复位，低有效

    pll_audio #(
        .TICK_NUM (TICK_NUM),
        .TICK_DEN (TICK_DEN)
    ) u_pll_audio (
        .i_clk_50m   (i_clk_50m),
        .i_rst_n     (rst_n),
        .o_clk_audio (clk_audio),
        .o_tick      (tick_512fs)
    );

    // -------------------------------------------------------------------------
    // 上电复位（不使用 initial；依赖 FPGA 上电后寄存器为 0）
    // -------------------------------------------------------------------------
    reg [POR_BITS-1:0] por_cnt;

    always @(posedge clk_audio) begin
        if (por_cnt != {POR_BITS{1'b1}})
            por_cnt <= por_cnt + 1'b1;
        rst_n <= (por_cnt == {POR_BITS{1'b1}});
    end

    // -------------------------------------------------------------------------
    // 启动静音计数：按帧（sample_tick）计数
    // -------------------------------------------------------------------------
    wire        sample_tick;      // 每帧 1 拍，频率 = Fs
    reg  [15:0] mute_cnt;
    reg         startup_done;
    reg         pa_on;

    always @(posedge clk_audio) begin
        if (!rst_n) begin
            mute_cnt     <= 16'd0;
            startup_done <= 1'b0;
            pa_on        <= 1'b0;
        end else if (sample_tick && !startup_done) begin
            if (mute_cnt == PA_ON_LAST)
                pa_on <= 1'b1;
            if (mute_cnt == MUTE_LAST)
                startup_done <= 1'b1;
            else
                mute_cnt <= mute_cnt + 16'd1;
        end
    end

    wire tone_en = startup_done && (TEST_TONE_EN != 0);

    // -------------------------------------------------------------------------
    // 振荡器
    // -------------------------------------------------------------------------
    wire signed [15:0] nco_sample;

    nco_sine #(
        .PHASE_W  (32),
        .ADDR_W   (LUT_ADDR_W),
        .OUT_W    (16),
        .LUT_FILE (LUT_FILE)
    ) u_nco_sine (
        .clk          (clk_audio),
        .rst_n        (rst_n),
        .sample_en    (sample_tick),
        .enable       (tone_en),
        .phase_inc    (PHASE_INC),
        .sample       (nco_sample),
        .sample_valid ()
    );

    // 16 位样本左对齐到 24 位（低 8 位补 0），再按 VOLUME_SHIFT 做算术右移衰减
    wire signed [23:0] sample_24  = {nco_sample, 8'h00};
    wire signed [23:0] sample_out = sample_24 >>> VOLUME_SHIFT;

    // -------------------------------------------------------------------------
    // I2S 发送（左右声道相同的单声道测试音）
    // -------------------------------------------------------------------------
    wire mclk_int;

    i2s_master #(
        .DATA_W (24),
        .FORMAT (I2S_FORMAT)
    ) u_i2s_master (
        .clk        (clk_audio),
        .rst_n      (rst_n),
        .tick       (tick_512fs),
        .data_l     (sample_out),
        .data_r     (sample_out),
        .sample_req (sample_tick),
        .o_mclk     (mclk_int),
        .o_bck      (o_i2s_bck),
        .o_lrck     (o_i2s_lrck),
        .o_sdata    (o_i2s_sdata)
    );

`ifdef TOP_USE_MCLK
    assign o_i2s_mclk = mclk_int;
`endif

`ifdef TOP_USE_PA_EN
    assign o_pa_en = pa_on ? PA_EN_ACTIVE : ~PA_EN_ACTIVE;
`endif

endmodule
