`timescale 1ns/1ps
// =============================================================================
// recorder.v —— 演奏录音 + 蓝牙回传（HC-05，9600 波特）
// -----------------------------------------------------------------------------
// 为什么不直接传声音：9600 波特每秒只能传约 960 字节，而 1 秒的声音有约 10 万字节（要传 100 多秒）。
// 所以这里录的是“演奏动作”：每个琴键按下 / 松开、效果器和音量的变化，都带毫秒时间。电脑端程序
// scripts/bt_recorder.py 收到后，用和 FPGA 一样的合成算法、一样的参数把它渲染成 WAV（另存一份 MIDI）。
// 1 分钟的演奏一般只有几 KB，几秒钟就传完。
//
// 录音状态 rec_st（OLED 上显示）：
//   0 空闲          按 key[2] 开始录音
//   1 录音中        再按 key[2] 停止；存储器满（16384 个事件）或录满约 69 分钟也会自动停止
//   2 等电脑        电脑端程序在线（2 秒内收到过 'H'）就自动开始发送
//   3 发送中        进度 p_bcd；电脑掉线就回到 2，电脑回来后从头重发
//   4 已发送        t_bcd 是这次录音的时长；电脑发 'R' 可以再发一遍
//   状态 2、4 时按 key[2] 开始新的录音（旧的录音被覆盖），状态 3 时按 key[2] 不起作用。
//
// 电脑 → FPGA：'H'（每 0.5 秒一次，表示电脑在线；FPGA 不在发送时回一个 'k'）、'R'（重发上一次录音）
// FPGA → 电脑（大端字节序）：
//   头 16 字节：'M' 'P' 'R' '1'、事件数 N（4 字节）、时长（ms，4 字节）、开始时的效果器、音量、0、0
//   N × 4 字节事件（下面的 32 位字），最后 4 字节：前面所有字节之和（16 位）、'E' 'D'
// 事件字：[31:30] 类型（0 按下，1 松开，2 控制）  [29:8] 时间（ms）
//   按下：[7:6] 音色（0 钢琴，1 电吉他，2 大提琴）[5:0] 音序号；松开：[5:0] 音序号
//   控制：[7:5] 0 = 效果器 / 1 = 音量，[4:0] 新的值
// =============================================================================
module recorder #(
    parameter integer AW         = 14,        // 事件存储器地址位宽：2^14 = 16384 个事件
    parameter integer MS_CYC     = 50000,     // 1 ms 的时钟数
    parameter integer BAUD_DIV   = 5208,      // 50 MHz / 9600
    parameter integer PC_TIMEOUT = 2000       // 这么多 ms 没收到 'H' 就当电脑断开
) (
    input  wire        clk,
    input  wire        rst_n,           // 同步复位，低有效
    input  wire        rec_press,       // key[2] 按下，1 拍
    input  wire        ev_valid,        // 琴键事件，1 拍
    input  wire        ev_on,           // 1 = 按下，0 = 松开
    input  wire [5:0]  ev_note,
    input  wire [1:0]  tone,            // 当前音色
    input  wire [2:0]  fx,              // 当前效果器模式
    input  wire [1:0]  vol,             // 当前音量
    output wire        o_tx,            // → HC-05 RXD
    input  wire        i_rx,            // ← HC-05 TXD
    output reg  [2:0]  rec_st,
    output reg  [15:0] t_bcd,           // 录音时长：分十位、分个位、秒十位、秒个位（BCD）
    output reg  [11:0] p_bcd,           // 发送进度：百位、十位、个位（BCD 百分比）
    output wire        blink,           // 录音时每秒前一半为 1（让“●”闪烁）
    output wire        pc_alive         // 1 = 电脑端程序在线
);

    localparam [2:0] R_IDLE = 3'd0, R_REC = 3'd1, R_WAIT = 3'd2, R_SEND = 3'd3, R_DONE = 3'd4;
    localparam [AW:0] N_MAX  = (1 << AW);

    // ---------------- 事件存储器 2^AW × 32（简单双口） ----------------
    reg [31:0]   mem [0:(1<<AW)-1];
    reg [AW-1:0] wa, ra;
    reg          we;
    reg [31:0]   wd, rd;

    always @(posedge clk) begin
        if (we)
            mem[wa] <= wd;
        rd <= mem[ra];
    end

    // ---------------- 串口 ----------------
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;
    wire       rx_valid;
    wire [7:0] rx_data;

    uart_tx #(.DIV (BAUD_DIV)) u_tx (
        .clk (clk), .rst_n (rst_n), .start (tx_start), .data (tx_data), .busy (tx_busy), .tx (o_tx)
    );
    uart_rx #(.DIV (BAUD_DIV)) u_rx (
        .clk (clk), .rst_n (rst_n), .rx (i_rx), .valid (rx_valid), .data (rx_data)
    );

    // ---------------- 毫秒节拍 ----------------
    reg  [15:0] ms_cnt;
    wire        ms_tick = (ms_cnt == MS_CYC - 1);

    always @(posedge clk) begin
        if (!rst_n)
            ms_cnt <= 16'd0;
        else
            ms_cnt <= ms_tick ? 16'd0 : ms_cnt + 16'd1;
    end

    // 电脑在线：收到 'H' 后 PC_TIMEOUT ms 内算在线
    reg [11:0] pc_ms;
    assign pc_alive = (pc_ms != 12'd0);

    // ---------------- BCD 加 1 ----------------
    function [15:0] time_inc;                 // mm:ss 加 1 秒（到 99:59 不再加）
        input [15:0] v;
        begin
            time_inc = v;
            if (v[3:0] != 4'd9)
                time_inc[3:0] = v[3:0] + 4'd1;
            else begin
                time_inc[3:0] = 4'd0;
                if (v[7:4] != 4'd5)
                    time_inc[7:4] = v[7:4] + 4'd1;
                else begin
                    time_inc[7:4] = 4'd0;
                    if (v[11:8] != 4'd9)
                        time_inc[11:8] = v[11:8] + 4'd1;
                    else if (v[15:12] != 4'd9) begin
                        time_inc[11:8]  = 4'd0;
                        time_inc[15:12] = v[15:12] + 4'd1;
                    end else
                        time_inc = v;
                end
            end
        end
    endfunction

    function [11:0] pct_inc;                  // 百分比加 1
        input [11:0] v;
        begin
            pct_inc = v;
            if (v[3:0] != 4'd9)
                pct_inc[3:0] = v[3:0] + 4'd1;
            else if (v[7:4] != 4'd9) begin
                pct_inc[3:0] = 4'd0;
                pct_inc[7:4] = v[7:4] + 4'd1;
            end else
                pct_inc = 12'h100;
        end
    endfunction

    // ---------------- 录音 ----------------
    reg [21:0] t_ms;                          // 录音开始以来的毫秒数
    reg [9:0]  ms_in_sec;
    reg [AW:0] n_ev;                          // 已录事件数
    reg [21:0] dur_ms;                        // 录音时长
    reg [2:0]  init_fx, fx_last;
    reg [1:0]  init_vol, vol_last;
    reg        has_rec;

    assign blink = (ms_in_sec < 10'd500);

    // ---------------- 发送 ----------------
    localparam [1:0] T_IDLE = 2'd0, T_ADDR = 2'd1, T_WAIT = 2'd2, T_PUT = 2'd3;
    reg  [1:0]  tst;
    reg  [16:0] bi;                           // 正在发第几个字节
    reg  [16:0] tot;                          // 总字节数 = 16 + 4N + 4
    reg  [15:0] csum;
    reg  [17:0] pacc;                         // 进度累加器：每发一个字节加 100，满 tot 进 1%
    reg         reply_k;                      // 要回 'k'
    reg         req_r;                        // 电脑要求重发

    wire [16:0] be      = bi - 17'd16;        // 事件区内的字节偏移
    wire [16:0] tl      = bi - (tot - 17'd4); // 尾部字节序号
    wire        in_hdr  = (bi < 17'd16);
    wire        in_trl  = (bi >= tot - 17'd4);
    reg  [7:0]  byte_out;

    always @(*) begin
        if (in_hdr) begin
            case (bi[3:0])
            4'd0:  byte_out = 8'h4D;                       // 'M'
            4'd1:  byte_out = 8'h50;                       // 'P'
            4'd2:  byte_out = 8'h52;                       // 'R'
            4'd3:  byte_out = 8'h31;                       // '1'
            4'd4:  byte_out = 8'h00;
            4'd5:  byte_out = 8'h00;
            4'd6:  byte_out = {{(15-AW){1'b0}}, n_ev[AW:8]};
            4'd7:  byte_out = n_ev[7:0];
            4'd8:  byte_out = 8'h00;
            4'd9:  byte_out = {2'b00, dur_ms[21:16]};
            4'd10: byte_out = dur_ms[15:8];
            4'd11: byte_out = dur_ms[7:0];
            4'd12: byte_out = {5'd0, init_fx};
            4'd13: byte_out = {6'd0, init_vol};
            default: byte_out = 8'h00;
            endcase
        end else if (in_trl) begin
            case (tl[1:0])
            2'd0:    byte_out = csum[15:8];
            2'd1:    byte_out = csum[7:0];
            2'd2:    byte_out = 8'h45;                     // 'E'
            default: byte_out = 8'h44;                     // 'D'
            endcase
        end else begin
            case (be[1:0])
            2'd0:    byte_out = rd[31:24];
            2'd1:    byte_out = rd[23:16];
            2'd2:    byte_out = rd[15:8];
            default: byte_out = rd[7:0];
            endcase
        end
    end

    wire put_now  = (rec_st == R_SEND) && (tst == T_PUT) && !tx_busy && !tx_start;
    wire pct_step = (rec_st == R_SEND) && (pacc >= {1'b0, tot}) && (p_bcd != 12'h100);
    wire full     = (n_ev == N_MAX) || (t_ms == 22'h3FFFFF);
    wire start_rec = rec_press && (rec_st != R_SEND) && (rec_st != R_REC);
    wire stop_rec  = (rec_st == R_REC) && (rec_press || full);
    wire start_tx  = has_rec && ((rec_st == R_WAIT && pc_alive) ||
                                 (req_r && (rec_st == R_WAIT || rec_st == R_DONE)));

    always @(posedge clk) begin
        if (!rst_n) begin
            rec_st    <= R_IDLE;
            t_bcd     <= 16'h0000;
            p_bcd     <= 12'h000;
            t_ms      <= 22'd0;
            ms_in_sec <= 10'd0;
            n_ev      <= {(AW+1){1'b0}};
            dur_ms    <= 22'd0;
            init_fx   <= 3'd0;
            init_vol  <= 2'd0;
            fx_last   <= 3'd0;
            vol_last  <= 2'd0;
            has_rec   <= 1'b0;
            we        <= 1'b0;
            wa        <= {AW{1'b0}};
            wd        <= 32'd0;
            ra        <= {AW{1'b0}};
            tst       <= T_IDLE;
            bi        <= 17'd0;
            tot       <= 17'd20;
            csum      <= 16'd0;
            pacc      <= 18'd0;
            reply_k   <= 1'b0;
            req_r     <= 1'b0;
            pc_ms     <= 12'd0;
            tx_start  <= 1'b0;
            tx_data   <= 8'd0;
        end else begin
            we       <= 1'b0;
            tx_start <= 1'b0;

            // ---- 电脑发来的命令 ----
            if (rx_valid && rx_data == 8'h48) begin             // 'H'
                pc_ms   <= PC_TIMEOUT;
                reply_k <= 1'b1;
            end else if (ms_tick && pc_ms != 12'd0) begin
                pc_ms <= pc_ms - 12'd1;
            end
            if (rx_valid && rx_data == 8'h52)                   // 'R'
                req_r <= 1'b1;

            // ---- 不在发送时回 'k' ----
            if (reply_k && rec_st != R_SEND && tst == T_IDLE && !tx_busy && !tx_start) begin
                tx_data  <= 8'h6B;                              // 'k'
                tx_start <= 1'b1;
                reply_k  <= 1'b0;
            end

            // ---- 录音 ----
            if (start_rec) begin
                rec_st    <= R_REC;
                t_ms      <= 22'd0;
                ms_in_sec <= 10'd0;
                t_bcd     <= 16'h0000;
                n_ev      <= {(AW+1){1'b0}};
                init_fx   <= fx;
                init_vol  <= vol;
                fx_last   <= fx;
                vol_last  <= vol;
                has_rec   <= 1'b0;
                req_r     <= 1'b0;
            end else if (stop_rec) begin
                rec_st  <= R_WAIT;
                dur_ms  <= t_ms;
                has_rec <= 1'b1;
            end else if (rec_st == R_REC) begin
                if (ms_tick) begin
                    t_ms <= t_ms + 22'd1;
                    if (ms_in_sec == 10'd999) begin
                        ms_in_sec <= 10'd0;
                        t_bcd     <= time_inc(t_bcd);
                    end else begin
                        ms_in_sec <= ms_in_sec + 10'd1;
                    end
                end
                // 一拍最多写一个事件：琴键优先，效果器、音量的变化比较出来，不会漏
                if (ev_valid) begin
                    we   <= 1'b1;
                    wa   <= n_ev[AW-1:0];
                    wd   <= ev_on ? {2'd0, t_ms, tone, ev_note} : {2'd1, t_ms, 2'd0, ev_note};
                    n_ev <= n_ev + 1'b1;
                end else if (fx != fx_last) begin
                    we      <= 1'b1;
                    wa      <= n_ev[AW-1:0];
                    wd      <= {2'd2, t_ms, 3'd0, 2'd0, fx};
                    n_ev    <= n_ev + 1'b1;
                    fx_last <= fx;
                end else if (vol != vol_last) begin
                    we       <= 1'b1;
                    wa       <= n_ev[AW-1:0];
                    wd       <= {2'd2, t_ms, 3'd1, 3'd0, vol};
                    n_ev     <= n_ev + 1'b1;
                    vol_last <= vol;
                end
            end

            // ---- 发送 ----
            if (start_tx && !start_rec) begin
                rec_st <= R_SEND;
                tst    <= T_ADDR;
                bi     <= 17'd0;
                tot    <= 17'd20 + {n_ev, 2'b00};
                csum   <= 16'd0;
                pacc   <= 18'd0;
                p_bcd  <= 12'h000;
                req_r  <= 1'b0;
            end else if (rec_st == R_SEND) begin
                if (!pc_alive) begin                            // 电脑掉线：回到等待，回来后从头重发
                    rec_st <= R_WAIT;
                    tst    <= T_IDLE;
                end else begin
                    case (tst)
                    T_ADDR: begin
                        ra  <= be[AW+1:2];
                        tst <= T_WAIT;
                    end
                    T_WAIT: tst <= T_PUT;                       // 存储器读出
                    T_PUT: if (put_now) begin
                        tx_data  <= byte_out;
                        tx_start <= 1'b1;
                        if (!in_trl)
                            csum <= csum + {8'd0, byte_out};
                        if (bi == tot - 17'd1) begin            // 最后一个字节
                            rec_st <= R_DONE;
                            tst    <= T_IDLE;
                        end else begin
                            bi  <= bi + 17'd1;
                            tst <= T_ADDR;
                        end
                    end
                    default: tst <= T_ADDR;
                    endcase
                end
            end

            // ---- 发送进度（开始发送的那一拍已在上面清零，这里不能再改它）----
            if (!(start_tx && !start_rec)) begin
                if (rec_st == R_DONE)
                    p_bcd <= 12'h100;
                else if (pct_step)
                    p_bcd <= pct_inc(p_bcd);
            end
            if (!(start_tx && !start_rec))
                pacc <= pacc + (put_now ? 18'd100 : 18'd0) - (pct_step ? {1'b0, tot} : 18'd0);
        end
    end

endmodule
