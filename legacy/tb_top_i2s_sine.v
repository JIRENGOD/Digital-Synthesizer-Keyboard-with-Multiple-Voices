`timescale 1ns/1ps
// =============================================================================
// tb_top_i2s_sine.v —— top_i2s_sine 仿真测试台
// -----------------------------------------------------------------------------
// 编译顺序：src/top_i2s_sine.v 必须排在本文件之前（端口开关宏 TOP_USE_MCLK /
//          TOP_USE_PA_EN 在那里定义）。在 sim/ 目录下运行，hex 路径默认为
//          ../src/sine_lut.hex，可用 -DLUT_HEX=... 覆盖。
//
// 自动检查项（全部通过时打印 RESULT: PASS）：
//   1. 相位增量：DUT 的 PHASE_INC == round(440 * 2^32 / Fs)，且每次 sample_en
//      相位累加器恰好前进 PHASE_INC（证明 NCO 按采样率而不是 50 MHz 前进）。
//   2. LRCK 周期 ≈ 20.833 us（48 kHz），每个 LRCK 周期恰好 64 个 BCK、256 个 MCLK。
//   3. SDATA / LRCK 只在 BCK 下降沿附近变化：相对 BCK 上升沿的建立、保持时间 >= 100 ns。
//   4. 按标准 I2S 解码 SDATA，左右声道一致，且每个样本与参考模型（同一份 hex +
//      同样的相位累加 + 音量移位）逐位相等。
//   5. 起振前至少有 MUTE_FRAMES 帧全零（上电静音），起振从 sin(0)=0 开始。
//   6. 用过零点插值测出音调频率 ≈ 440 Hz。
// 波形输出：tb_top_i2s_sine.vcd（用 GTKWave 打开）。
// =============================================================================

`ifndef LUT_HEX
`define LUT_HEX "../src/sine_lut.hex"
`endif

module tb_top_i2s_sine;

    // -------------------------------------------------------------------------
    // 仿真参数
    // -------------------------------------------------------------------------
    localparam integer CLK_MODE    = 0;        // 与 DUT 相同：0 = 48 kHz
    localparam real    F_TONE      = 440.0;    // DUT 默认 PHASE_INC 对应 440 Hz
    localparam integer VOL_SHIFT   = 2;
    localparam integer ADDR_W      = 12;
    localparam integer MUTE_FRAMES = 8;        // 把上板用的 9600 帧（200 ms）静音缩短为 8 帧
    localparam integer RUN_FRAMES  = 2400;     // 起振后检查 2400 帧（50 ms，约 22 个周期）

    localparam real F_CLK    = 50.0e6;
    localparam real FS       = (CLK_MODE == 1) ? (F_CLK / 2.0 / 512.0)
                                               : (F_CLK * 1536.0 / 3125.0 / 512.0);
    localparam real T_FRAME  = 1.0e9 / FS;     // ns：48 kHz -> 20833.333 ns
    localparam real T_MARGIN = 100.0;          // ns：建立/保持最小要求

    // -------------------------------------------------------------------------
    // 时钟与 DUT
    // -------------------------------------------------------------------------
    reg clk = 1'b0;
    always #10 clk = ~clk;                     // 50 MHz

    wire bck, lrck, sdata;
    wire mclk;

    top_i2s_sine #(
        .CLK_MODE            (CLK_MODE),
        .VOLUME_SHIFT        (VOL_SHIFT),
        .STARTUP_MUTE_FRAMES (MUTE_FRAMES),
        .POR_BITS            (4),              // 上电复位缩短为 16 拍
        .LUT_ADDR_W          (ADDR_W),
        .LUT_FILE            (`LUT_HEX)
    ) dut (
        .i_clk_50m   (clk),
`ifdef TOP_USE_MCLK
        .o_i2s_mclk  (mclk),
`endif
        .o_i2s_bck   (bck),
        .o_i2s_lrck  (lrck),
        .o_i2s_sdata (sdata)
`ifdef TOP_USE_PA_EN
       ,.o_pa_en     ()
`endif
    );

`ifndef TOP_USE_MCLK
    assign mclk = 1'b0;
`endif

    // 模拟 FPGA 上电：芯片里寄存器上电即为 0，而仿真器里初值是 X
    initial begin
        dut.por_cnt = 0;
        dut.rst_n   = 1'b0;
    end

    // -------------------------------------------------------------------------
    // 参考模型数据
    // -------------------------------------------------------------------------
    reg [15:0] ref_lut [0:(1<<ADDR_W)-1];
    reg [31:0] inc_exp;
    integer    errors = 0;

    initial begin
        $readmemh(`LUT_HEX, ref_lut);
        inc_exp = $rtoi(F_TONE * 4294967296.0 / FS + 0.5);
    end

    // -------------------------------------------------------------------------
    // 检查 1：NCO 每次 sample_en 恰好前进 PHASE_INC
    // -------------------------------------------------------------------------
    reg [31:0] ph_before  = 32'd0;
    reg [31:0] inc_seen   = 32'd0;         // 实测：NCO 每个采样周期前进的相位
    reg        ph_pending = 1'b0;
    integer    n_phase_steps = 0;
    integer    n_sample_en   = 0;

    always @(posedge clk) begin
        if (ph_pending) begin
            if (n_phase_steps == 0)
                inc_seen = dut.u_nco_sine.phase_acc - ph_before;
            if (dut.u_nco_sine.phase_acc !== ph_before + inc_exp) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("[%0t] ERROR: phase step %h -> %h, expected +%0d",
                             $time, ph_before, dut.u_nco_sine.phase_acc, inc_exp);
            end
            n_phase_steps = n_phase_steps + 1;
        end
        if (dut.rst_n === 1'b1 && dut.u_nco_sine.sample_en === 1'b1)
            n_sample_en = n_sample_en + 1;
        ph_pending = (dut.rst_n === 1'b1) && (dut.u_nco_sine.sample_en === 1'b1)
                     && (dut.u_nco_sine.enable === 1'b1);
        ph_before  = dut.u_nco_sine.phase_acc;
    end

    // -------------------------------------------------------------------------
    // 检查 2：LRCK 周期、每帧 BCK / MCLK 个数
    // -------------------------------------------------------------------------
    real    t_lr_first = 0.0, t_lr_last = 0.0, per = 0.0;
    real    per_min = 1.0e12, per_max = 0.0;
    integer n_lr = 0;
    integer bck_cnt = 0, mclk_cnt = 0;

    always @(posedge bck)  bck_cnt  = bck_cnt + 1;
    always @(posedge mclk) mclk_cnt = mclk_cnt + 1;

    always @(negedge lrck) begin               // 标准 I2S：LRCK 下降沿 = 新帧（左声道）开始
        if (dut.rst_n === 1'b1) begin
            if (n_lr > 0) begin
                per = $realtime - t_lr_last;
                if (per < per_min) per_min = per;
                if (per > per_max) per_max = per;
                if (bck_cnt != 64) begin
                    errors = errors + 1;
                    $display("[%0t] ERROR: %0d BCK rising edges in one LRCK period (expected 64)", $time, bck_cnt);
                end
`ifdef TOP_USE_MCLK
                if (mclk_cnt != 256) begin
                    errors = errors + 1;
                    $display("[%0t] ERROR: %0d MCLK rising edges in one LRCK period (expected 256)", $time, mclk_cnt);
                end
`endif
            end else begin
                t_lr_first = $realtime;
            end
            t_lr_last = $realtime;
            n_lr      = n_lr + 1;
        end
        bck_cnt  = 0;
        mclk_cnt = 0;
    end

    // -------------------------------------------------------------------------
    // 检查 3：SDATA / LRCK 相对 BCK 上升沿的建立、保持时间
    // -------------------------------------------------------------------------
    real    t_sd_chg = -1.0e9, t_lr_chg = -1.0e9, t_bck_rise = -1.0e9;
    real    min_setup = 1.0e12, min_hold = 1.0e12, dt = 0.0;
    integer n_sd_toggle = 0;
    wire    timing_active = (n_lr >= 1);

    always @(sdata) begin
        if (timing_active) begin
            dt = $realtime - t_bck_rise;
            if (dt < min_hold) min_hold = dt;
            n_sd_toggle = n_sd_toggle + 1;
        end
        t_sd_chg = $realtime;
    end

    always @(lrck) begin
        if (timing_active) begin
            dt = $realtime - t_bck_rise;
            if (dt < min_hold) min_hold = dt;
        end
        t_lr_chg = $realtime;
    end

    always @(posedge bck) begin
        if (timing_active) begin
            dt = $realtime - t_sd_chg;
            if (dt < min_setup) min_setup = dt;
            dt = $realtime - t_lr_chg;
            if (dt < min_setup) min_setup = dt;
        end
        t_bck_rise = $realtime;
    end

    // -------------------------------------------------------------------------
    // 检查 4/5/6：I2S 解码 + 参考模型逐样本比对 + 过零测频
    // -------------------------------------------------------------------------
    reg               lr_q = 1'b0, lr_q_vld = 1'b0, have_left = 1'b0;
    reg        [31:0] rx_sh = 32'd0;
    integer           rx_bits = 0;
    reg signed [23:0] rx_left = 24'sd0;
    reg signed [23:0] rx_sample = 24'sd0;      // 解码出的样本（波形里看这个）

    reg               tone_started = 1'b0;
    integer           k = 0, n_frames = 0, n_zero_before = 0, n_match = 0;
    reg        [31:0] ref_phase = 32'd0;
    reg signed [23:0] ref_word  = 24'sd0;
    reg signed [23:0] prev_s    = 24'sd0;
    integer           zc_n = 0, ip = 0, ic = 0;
    real              zc_first = 0.0, zc_last = 0.0, zc_t = 0.0;

    task compare_sample;
        input signed [23:0] got;
        begin
            ref_word = $signed({ref_lut[ref_phase[31 -: ADDR_W]], 8'h00}) >>> VOL_SHIFT;
            if (got !== ref_word) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("[%0t] ERROR: sample #%0d = %0d, reference = %0d", $time, k, got, ref_word);
            end else begin
                n_match = n_match + 1;
            end
        end
    endtask

    task check_frame;
        input signed [23:0] l;
        input signed [23:0] r;
        begin
            n_frames  = n_frames + 1;
            rx_sample = l;
            if (l !== r) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("[%0t] ERROR: left %0d != right %0d", $time, l, r);
            end
            if (!tone_started) begin
                if (l == 24'sd0) begin
                    n_zero_before = n_zero_before + 1;
                end else begin
                    // 第一个非零样本应是样本 #1（样本 #0 = sin(0) = 0）
                    tone_started = 1'b1;
                    k         = 1;
                    ref_phase = inc_exp;
                    compare_sample(l);
                end
            end else begin
                k         = k + 1;
                ref_phase = ref_phase + inc_exp;
                compare_sample(l);
                // 正向过零（prev < 0 <= cur），线性插值求过零时刻（单位：样本）
                if (prev_s < 0 && l >= 0) begin
                    ip   = prev_s;
                    ic   = l;
                    zc_t = (k - 1) + $itor(-ip) / $itor(ic - ip);
                    if (zc_n == 0) zc_first = zc_t;
                    zc_last = zc_t;
                    zc_n    = zc_n + 1;
                end
            end
            prev_s = l;
        end
    endtask

    always @(posedge bck) begin                // DAC 视角：BCK 上升沿采样
        if (dut.rst_n === 1'b1) begin
            if (lr_q_vld && (lrck !== lr_q)) begin
                // LRCK 已翻转：这是新声道槽的第 0 位，上一槽的 32 位已收齐
                if (rx_bits == 32) begin
                    if (lr_q == 1'b0) begin                // 标准 I2S：LRCK=0 为左声道
                        rx_left   = rx_sh[30:7];           // 槽内第 1..24 位
                        have_left = 1'b1;
                    end else if (have_left) begin
                        check_frame(rx_left, rx_sh[30:7]);
                        have_left = 1'b0;
                    end
                end
                rx_bits = 0;
            end
            rx_sh    = {rx_sh[30:0], sdata};
            rx_bits  = rx_bits + 1;
            lr_q     = lrck;
            lr_q_vld = 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // 波形、结束判定与汇总
    // -------------------------------------------------------------------------
    real f_meas = 0.0, f_exact = 0.0, t_avg = 0.0;

    initial begin
        $dumpfile("tb_top_i2s_sine.vcd");
`ifdef VERILATOR
        $dumpvars(1, tb_top_i2s_sine);
`else
        // 不导出 50 MHz clk 和 MCLK，以控制 VCD 大小
        $dumpvars(0, bck, lrck, sdata, rx_sample, tone_started,
                  dut.rst_n, dut.sample_tick, dut.tone_en,
                  dut.u_nco_sine.phase_acc, dut.u_nco_sine.sample);
`endif

        wait (tone_started === 1'b1);
        $display("[%0t] tone started after %0d all-zero frames", $time, n_zero_before);
        wait (k >= RUN_FRAMES);

        t_avg   = (t_lr_last - t_lr_first) / (n_lr - 1);
        f_exact = inc_exp * FS / 4294967296.0;
        if (zc_n >= 2)
            f_meas = (zc_n - 1) / ((zc_last - zc_first) * t_avg * 1.0e-9);

        if (inc_seen !== inc_exp) begin
            errors = errors + 1;
            $display("ERROR: NCO phase step = %0d, expected %0d", inc_seen, inc_exp);
        end
        if (n_phase_steps < RUN_FRAMES) begin
            errors = errors + 1;
            $display("ERROR: only %0d NCO phase steps checked", n_phase_steps);
        end
        if (t_avg < T_FRAME * 0.9999 || t_avg > T_FRAME * 1.0001
            || per_min < T_FRAME - 25.0 || per_max > T_FRAME + 25.0) begin
            errors = errors + 1;
            $display("ERROR: LRCK period out of range");
        end
        if (min_setup < T_MARGIN || min_hold < T_MARGIN) begin
            errors = errors + 1;
            $display("ERROR: SDATA/LRCK setup or hold vs BCK rising edge < %0.0f ns", T_MARGIN);
        end
        if (n_sd_toggle == 0) begin
            errors = errors + 1;
            $display("ERROR: SDATA never toggles");
        end
        if (n_zero_before < MUTE_FRAMES) begin
            errors = errors + 1;
            $display("ERROR: only %0d silent frames before the tone (need >= %0d)", n_zero_before, MUTE_FRAMES);
        end
        if (n_match != k) begin
            errors = errors + 1;
            $display("ERROR: %0d of %0d samples match the reference", n_match, k);
        end
        if (f_meas < F_TONE * 0.999 || f_meas > F_TONE * 1.001) begin
            errors = errors + 1;
            $display("ERROR: measured tone %f Hz", f_meas);
        end

        $display("");
        $display("==================== tb_top_i2s_sine summary ====================");
        $display("LRCK period  : avg %0.3f ns (expect %0.3f), min %0.1f, max %0.1f  -> Fs = %0.3f Hz",
                 t_avg, T_FRAME, per_min, per_max, 1.0e9 / t_avg);
`ifdef TOP_USE_MCLK
        $display("Clock ratio  : %0d LRCK periods checked, each = 64 BCK and 256 MCLK", n_lr - 1);
`else
        $display("Clock ratio  : %0d LRCK periods checked, each = 64 BCK", n_lr - 1);
`endif
        $display("SDATA/LRCK   : %0d SDATA toggles, worst setup %0.1f ns, worst hold %0.1f ns (BCK rising edge)",
                 n_sd_toggle, min_setup, min_hold);
        $display("PHASE_INC    : NCO steps %0d per sample, round(%0.1f*2^32/%0.3f) = %0d",
                 inc_seen, F_TONE, FS, inc_exp);
        $display("NCO stepping : %0d sample_en pulses, %0d phase steps all +PHASE_INC",
                 n_sample_en, n_phase_steps);
        $display("Startup      : %0d all-zero frames before first non-zero sample (>= %0d)",
                 n_zero_before, MUTE_FRAMES);
        $display("Samples      : %0d / %0d decoded I2S samples equal the reference model", n_match, k);
        $display("Tone         : %0.4f Hz measured from %0d zero crossings (exact %0.6f Hz)",
                 f_meas, zc_n, f_exact);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $display("=================================================================");
        $finish;
    end

    initial begin
        #80_000_000;                           // 80 ms 看门狗
        $display("RESULT: FAIL (timeout, tone_started=%0d, frames=%0d)", tone_started, n_frames);
        $finish;
    end

endmodule
