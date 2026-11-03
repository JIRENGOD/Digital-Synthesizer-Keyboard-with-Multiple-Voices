`timescale 1ns/1ps
// =============================================================================
// ttp229_rd.v —— 16 键电容触摸钢琴键盘（比特树 A025，TTP229 芯片，两线串行 SCL / SDO）读取
// -----------------------------------------------------------------------------
// 时序（FPGA 产生 SCL，芯片从 SDO 送出数据），和常用的 TTP229 Arduino 例程一样：
//   SCL 平时为高；读一次 = 16 个时钟：SCL 拉低 → 在低电平中间读 SDO → SCL 拉高。
//   第 1 个时钟读到 KEY1（bit0），……，第 16 个读到 KEY16（bit15）。
//   SCL 半周期 10 µs（50 kHz，芯片允许 1 ~ 512 kHz）；两次读数之间 SCL 保持高 3 ms
//   （芯片要求 ≥ 2 ms，之后才会重新从 KEY1 开始送），约 3.3 ms 读一次。
// 电平：TTP229 芯片两线串行输出默认是“有触摸 = 0（低电平）、无触摸 = 1（高电平）”，
//   常用 Arduino 例程也是 `!digitalRead(SDO)`，所以 ACTIVE_HIGH 默认为 0；
//   只有芯片选项改成高电平有效的模块才设成 1。
// 检查：一次读数里被触摸的键超过 MAX_KEYS 个，视为数据错误（例如 SDO 一直是低电平），
//   这一次不采用；连续 ERR_FRMS 次（约 0.5 s）都是错误数据时 err = 1，并把 keys 清零（防止音一直响）。
// 去抖：连续两次读数相同才更新 keys。
// =============================================================================
module ttp229_rd #(
    parameter integer ACTIVE_HIGH = 0,        // 0 = 有触摸为低电平（TTP229 默认）
    parameter integer HALF_CYC    = 500,      // SCL 半周期（时钟数）：10 µs
    parameter integer GAP_CYC     = 150000,   // 两次读数之间的间隔（时钟数）：3 ms
    parameter integer MAX_KEYS    = 12,       // 一次读数最多认可几个键同时被触摸
    parameter integer ERR_FRMS    = 150       // 连续这么多次错误数据（约 0.5 s）才报 err，≤ 255
) (
    input  wire        clk,          // 50 MHz
    input  wire        rst_n,        // 同步复位，低有效
    output reg         o_scl,        // → 模块 SCL
    input  wire        i_sdo,        // ← 模块 SDO
    output reg  [15:0] keys,         // 1 = 被触摸；bit0 = KEY1 … bit15 = KEY16
    output reg         err           // 1 = 一直读到错误数据（SDO 一直是有效电平）
);

    localparam [1:0] S_GAP  = 2'd0,
                     S_LOW  = 2'd1,
                     S_HIGH = 2'd2,
                     S_DONE = 2'd3;
    localparam       POL    = (ACTIVE_HIGH != 0) ? 1'b1 : 1'b0;

    reg  [1:0]  state;
    reg  [1:0]  sdo_s;                // SDO 两级同步
    reg  [17:0] cnt;
    reg  [3:0]  bitn;
    reg  [15:0] cur;                  // 这一次读到的 16 个键（1 = 被触摸）
    reg  [4:0]  n_on;                 // 这一次被触摸的键数
    reg  [15:0] last;                 // 上一次的有效读数
    reg  [7:0]  n_err;                // 连续错误次数

    wire        touched = (sdo_s[1] == POL);
    wire        frame_ok = (n_on <= MAX_KEYS);

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_GAP;
            sdo_s <= 2'b00;
            o_scl <= 1'b1;
            cnt   <= 18'd0;
            bitn  <= 4'd0;
            cur   <= 16'd0;
            n_on  <= 5'd0;
            last  <= 16'd0;
            n_err <= 8'd0;
            keys  <= 16'd0;
            err   <= 1'b0;
        end else begin
            sdo_s <= {sdo_s[0], i_sdo};
            case (state)
            S_GAP: begin                              // SCL 高，等两次读数之间的间隔
                o_scl <= 1'b1;
                if (cnt == GAP_CYC - 1) begin
                    cnt   <= 18'd0;
                    bitn  <= 4'd0;
                    n_on  <= 5'd0;
                    o_scl <= 1'b0;                    // 第 1 个时钟的下降沿
                    state <= S_LOW;
                end else begin
                    cnt <= cnt + 18'd1;
                end
            end

            S_LOW: begin                              // SCL 低：中间读 SDO
                if (cnt == HALF_CYC / 2 - 1) begin
                    cur[bitn] <= touched;
                    n_on      <= n_on + {4'd0, touched};
                end
                if (cnt == HALF_CYC - 1) begin
                    cnt   <= 18'd0;
                    o_scl <= 1'b1;
                    state <= S_HIGH;
                end else begin
                    cnt <= cnt + 18'd1;
                end
            end

            S_HIGH: begin                             // SCL 高
                if (cnt == HALF_CYC - 1) begin
                    cnt <= 18'd0;
                    if (bitn == 4'd15) begin
                        state <= S_DONE;              // 16 位读完，SCL 保持高
                    end else begin
                        bitn  <= bitn + 4'd1;
                        o_scl <= 1'b0;
                        state <= S_LOW;
                    end
                end else begin
                    cnt <= cnt + 18'd1;
                end
            end

            default: begin                            // S_DONE：处理这一次读数
                if (frame_ok) begin
                    n_err <= 8'd0;
                    err   <= 1'b0;
                    if (cur == last)                  // 去抖：连续两次相同
                        keys <= cur;
                    last <= cur;
                end else if (n_err == ERR_FRMS - 1) begin
                    err  <= 1'b1;                     // 一直是错误数据
                    keys <= 16'd0;
                    last <= 16'd0;
                end else begin
                    n_err <= n_err + 8'd1;
                end
                state <= S_GAP;
            end
            endcase
        end
    end

endmodule
