`timescale 1ns/1ps
// =============================================================================
// tb_top_piano_do.v —— top_piano_do（钢琴“哆”）仿真测试台
// -----------------------------------------------------------------------------
// 为缩短仿真时间，时间参数改小：2 ms 打开功放、5 ms 第一次击键、按住 600 ms、
// 抬键后停 300 ms 再击键（905 ms）；抬键后的安静间隔里，700 ms 按下底板按键、800 ms 松开；
// 共仿真 1.0 s。
//
// 自动检查（全部通过时最后打印 RESULT: PASS）。打印信息是英文，含义如下：
//   LRCK period   LRCK 平均周期 ≈ 20833 ns（48 kHz），每帧 34 个 BCK
//   SDATA/LRCK    数据只在 BCK 下降沿变化，相对上升沿的建立/保持时间 ≥ 250 ns
//   Slot format   每个声道正好 17 位、第 0 位 = 符号位（I2S/右对齐/左对齐 DAC 都能正确解码），左右相同
//   PA_EN         功放在第一个非零样本之前打开（o_pa_en 变 0）
//   Silence       第一次击键前全部为 0
//   Decay         击键后音量逐渐变小（dB）
//   Damper        自动抬键后音量快速变小（dB）
//   Key press     按下按键（击键）后音量回升（dB）；Key release 松开按键后快速变小
//   Restrike      自动再次击键后音量回升（dB）
//   Peak          最大样本值（不削顶，且足够响）
// 加 +dump 运行时，把解码出的样本写到 piano_dump.txt（每行一个整数，48 kHz），供频谱分析。
//
// 运行（在 sim 目录）：
//   iverilog -g2012 -o tb_piano.vvp ../src/top_piano_do.v ../src/pll_audio.v ../src/i2s_dac_tx.v ../src/key_debounce.v \
//            ../src/piano/piano_voice.v ../src/piano/piano_params.v ../src/sine_lut.v \
//            tb_top_piano_do.v
//   vvp tb_piano.vvp
// =============================================================================

`ifndef LUT_HEX
`define LUT_HEX "../src/sine_lut.hex"
`endif

module tb_top_piano_do;

    localparam integer PA_MS       = 2;
    localparam integer FIRST_MS    = 5;
    localparam integer HOLD_MS     = 600;
    localparam integer GAP_MS      = 300;
    localparam integer KEY_DOWN_MS = 700;
    localparam integer KEY_UP_MS   = 800;
    localparam integer END_MS      = 1000;
    localparam real    T_FRAME     = 1.0e9 / 48000.0;     // ns
    localparam real    T_MARGIN    = 250.0;               // ns

    // -------------------------------------------------------------------------
    // 时钟、按键、被测设计
    // -------------------------------------------------------------------------
    reg clk = 1'b0;
    always #10 clk = ~clk;                                // 50 MHz

    reg  key_n = 1'b1;
    wire bck, lrck, sdata, pa_en;

    top_piano_do #(
        .POR_BITS      (4),
        .PA_ON_MS      (PA_MS),
        .FIRST_NOTE_MS (FIRST_MS),
        .HOLD_MS       (HOLD_MS),
        .GAP_MS        (GAP_MS),
        .DEBOUNCE_MS   (2),
        .VOL_INIT      (0),
        .LUT_FILE      (`LUT_HEX)
    ) dut (
        .i_clk_50m    (clk),
        .i_key_n      (key_n),
        .i_key_vol_n  (1'b1),
        .i_key_mode_n (1'b1),
        .o_i2s_bck    (bck),
        .o_i2s_lrck   (lrck),
        .o_i2s_sdata  (sdata),
        .o_pa_en      (pa_en),
        .o_led_n      ()
    );

    initial begin                                         // 模拟 FPGA 上电后寄存器为 0
        dut.por_cnt = 0;
        dut.rst_n   = 1'b0;
    end

    initial begin                                         // 底板按键：按下、松开
        #(KEY_DOWN_MS * 1.0e6) key_n = 1'b0;
        #((KEY_UP_MS - KEY_DOWN_MS) * 1.0e6) key_n = 1'b1;
    end

    integer errors = 0;

    // -------------------------------------------------------------------------
    // 按 DAC 的方式在 BCK 上升沿接收数据并解码
    // -------------------------------------------------------------------------
    reg  signed [15:0] smp [0:65535];                     // 解码出的样本（48 kHz）
    integer            nfr    = 0;
    integer            nb     = 0;
    reg         [16:0] bits   = 17'd0;
    reg                lr_prev = 1'b0, lr_vld = 1'b0, synced = 1'b0;
    reg  signed [15:0] val_l  = 16'sd0;
    real               t0     = -1.0;                     // 第一个完整帧的时间

    always @(posedge bck) begin
        if (dut.rst_n === 1'b1) begin
            if (lr_vld && lrck !== lr_prev) begin         // LRCK 变化：上一个声道收完
                if (synced) begin
                    if (nb != 17) begin
                        errors = errors + 1;
                        if (errors <= 10) $display("[%0t] ERROR: slot has %0d bits (expected 17)", $time, nb);
                    end else if (bits[16] !== bits[15]) begin
                        errors = errors + 1;
                        if (errors <= 10) $display("[%0t] ERROR: bit0 is not the sign bit", $time);
                    end else if (lr_prev == 1'b0) begin
                        val_l = bits[15:0];
                    end else begin
                        if (bits[15:0] !== val_l) begin
                            errors = errors + 1;
                            if (errors <= 10) $display("[%0t] ERROR: left %0d != right %0d", $time, val_l, $signed(bits[15:0]));
                        end
                        if (nfr == 0) t0 = $realtime;
                        if (nfr < 65536) smp[nfr] = val_l;
                        nfr = nfr + 1;
                    end
                end
                synced = 1'b1;
                nb     = 0;
            end
            bits    = {bits[15:0], sdata};
            nb      = nb + 1;
            lr_prev = lrck;
            lr_vld  = 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // LRCK 周期、每帧 BCK 数
    // -------------------------------------------------------------------------
    real    t_lr_first = 0.0, t_lr_last = 0.0, per = 0.0, per_min = 1.0e12, per_max = 0.0;
    integer n_lr = 0, bck_cnt = 0;

    always @(posedge bck) bck_cnt = bck_cnt + 1;

    always @(negedge lrck) begin                          // LRCK 下降沿 = 新一帧（左声道）开始
        if (dut.rst_n === 1'b1) begin
            if (n_lr > 0) begin
                per = $realtime - t_lr_last;
                if (per < per_min) per_min = per;
                if (per > per_max) per_max = per;
                if (bck_cnt != 34) begin
                    errors = errors + 1;
                    if (errors <= 10) $display("[%0t] ERROR: %0d BCK in one frame (expected 34)", $time, bck_cnt);
                end
            end else begin
                t_lr_first = $realtime;
            end
            t_lr_last = $realtime;
            n_lr      = n_lr + 1;
        end
        bck_cnt = 0;
    end

    // -------------------------------------------------------------------------
    // SDATA / LRCK 相对 BCK 上升沿的建立、保持时间
    // -------------------------------------------------------------------------
    real t_sd_chg = -1.0e9, t_lr_chg = -1.0e9, t_bck_rise = -1.0e9;
    real min_setup = 1.0e12, min_hold = 1.0e12, dt = 0.0;

    always @(sdata) begin
        if (n_lr >= 1) begin
            dt = $realtime - t_bck_rise;
            if (dt < min_hold) min_hold = dt;
        end
        t_sd_chg = $realtime;
    end

    always @(lrck) begin
        if (n_lr >= 1) begin
            dt = $realtime - t_bck_rise;
            if (dt < min_hold) min_hold = dt;
        end
        t_lr_chg = $realtime;
    end

    always @(posedge bck) begin
        if (n_lr >= 1) begin
            dt = $realtime - t_sd_chg;
            if (dt < min_setup) min_setup = dt;
            dt = $realtime - t_lr_chg;
            if (dt < min_setup) min_setup = dt;
        end
        t_bck_rise = $realtime;
    end

    // -------------------------------------------------------------------------
    // 功放使能
    // -------------------------------------------------------------------------
    real t_pa_on = -1.0;
    always @(negedge pa_en) if (t_pa_on < 0.0) t_pa_on = $realtime;

    // -------------------------------------------------------------------------
    // 辅助函数：某时间窗内的平均幅度、两个幅度之比（dB）
    // -------------------------------------------------------------------------
    function real level;                                  // start_ms 起 len_ms 长的平均 |样本|
        input real start_ms;
        input real len_ms;
        integer i, i0, i1;
        real    s, x;
        begin
            i0 = $rtoi(start_ms * 48.0);
            i1 = $rtoi((start_ms + len_ms) * 48.0);
            s  = 0.0;
            for (i = i0; i < i1; i = i + 1) begin
                x = $itor(smp[i]);
                s = s + ((x < 0.0) ? -x : x);
            end
            level = s / (i1 - i0) + 1.0e-9;
        end
    endfunction

    function real db;
        input real a;
        input real b;
        db = 20.0 * $log10(a / b);
    endfunction

    task expect_db;
        input [8*24-1:0] name;
        input real       value;
        input real       min_db;
        begin
            $display("%0s: %6.1f dB (need >= %0.1f dB)", name, value, min_db);
            if (value < min_db) errors = errors + 1;
        end
    endtask

    // -------------------------------------------------------------------------
    // 仿真结束：汇总检查
    // -------------------------------------------------------------------------
    integer i, fd, first_nz, peak;
    real    t_avg, t_nz;
    real    t_rel1, t_hit2, t_key;

    initial begin
        #(PA_MS * 0.5e6);
        if (pa_en !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: o_pa_en should be 1 (amplifier off) during startup");
        end

        #(END_MS * 1.0e6 - PA_MS * 0.5e6);

        // 时间轴：样本 i 的时刻 ≈ t0 + i × T_FRAME；以下窗口位置按 ms 换算为样本序号
        first_nz = -1;
        peak     = 0;
        for (i = 0; i < nfr && i < 65536; i = i + 1) begin
            if (first_nz < 0 && smp[i] != 0) first_nz = i;
            if (smp[i] > peak)  peak = smp[i];
            if (-smp[i] > peak) peak = -smp[i];
        end
        t_avg  = (t_lr_last - t_lr_first) / (n_lr - 1);
        t_nz   = t0 + first_nz * T_FRAME;
        t_rel1 = FIRST_MS + HOLD_MS;                     // 自动抬键（ms）
        t_hit2 = t_rel1 + GAP_MS;                        // 自动再次击键（ms）
        t_key  = KEY_DOWN_MS;

        $display("");
        $display("==================== tb_top_piano_do summary ====================");
        $display("LRCK period : avg %0.3f ns (expect %0.3f), min %0.1f, max %0.1f -> Fs = %0.2f Hz",
                 t_avg, T_FRAME, per_min, per_max, 1.0e9 / t_avg);
        if (t_avg < T_FRAME * 0.9999 || t_avg > T_FRAME * 1.0001) errors = errors + 1;
        $display("Frames      : %0d frames checked, each 34 BCK, 2 x 17-bit slots", n_lr - 1);
        $display("SDATA/LRCK  : worst setup %0.1f ns, worst hold %0.1f ns (need >= %0.0f)",
                 min_setup, min_hold, T_MARGIN);
        if (min_setup < T_MARGIN || min_hold < T_MARGIN) errors = errors + 1;
        $display("PA_EN       : amplifier on at %0.3f ms, first sound at %0.3f ms",
                 t_pa_on / 1.0e6, t_nz / 1.0e6);
        if (t_pa_on < 0.0 || first_nz < 0 || t_pa_on > t_nz) errors = errors + 1;
        $display("Silence     : first non-zero sample #%0d (note expected at %0d ms)", first_nz, FIRST_MS);
        if (first_nz < (FIRST_MS - 1) * 48) errors = errors + 1;

        expect_db("Decay (0.5 s held)", db(level(FIRST_MS + 20, 40), level(FIRST_MS + 500, 40)), 3.0);
        expect_db("Damper (auto key up)", db(level(t_rel1 - 40, 40), level(t_rel1 + 50, 40)), 6.0);
        expect_db("Key press",          db(level(t_key + 8, 30),     level(t_key - 40, 38)), 8.0);
        expect_db("Key release",        db(level(KEY_UP_MS - 40, 38), level(KEY_UP_MS + 60, 38)), 6.0);
        expect_db("Restrike",           db(level(t_hit2 + 10, 40),   level(t_hit2 - 40, 38)), 8.0);

        $display("Peak        : %0d (must be < 32767 and > 4000)", peak);
        if (peak >= 32767 || peak <= 4000) errors = errors + 1;

        if ($test$plusargs("dump")) begin
            fd = $fopen("piano_dump.txt", "w");
            for (i = 0; i < nfr && i < 65536; i = i + 1)
                $fdisplay(fd, "%0d", smp[i]);
            $fclose(fd);
            $display("Dump        : %0d samples written to piano_dump.txt", nfr);
        end

        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $display("=================================================================");
        $finish;
    end

endmodule
