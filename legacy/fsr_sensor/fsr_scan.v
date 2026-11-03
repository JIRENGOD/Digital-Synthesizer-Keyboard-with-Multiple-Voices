`timescale 1ns/1ps
// =============================================================================
// fsr_scan.v —— 16 路压力传感器扫描：控制 16 路通道扩展模块的 S0~S3 选通，
//               用 FPGA 片内 ADC（ADCLRC，专用输入脚 ADCV_P / ADCV_N）逐个读电压
// -----------------------------------------------------------------------------
// 每个通道（clk = 50 MHz）：
//   1. o_sel（→ 模块 S3..S0）给出通道号，等 SETTLE_CYC 个时钟（默认 100 µs），让模块 AO 电压稳定；
//   2. ADCREQI 拉高 REQ_CYC 个时钟，上升沿启动一次转换；
//   3. 从请求起至少等 GUARD_CYC 个时钟（ADC 在请求后 10 个 ADC 时钟内先把 ADCRDY 拉低），
//      再等 ADCRDY 变 1（两级同步后），然后等 2 拍读 ADCVALUE（ADCRDY 为 1 期间数值不变）；
//   4. 输出 value 和 1 拍 valid，切到下一个通道。
//   从请求起超过 TIMEOUT_CYC 还没有 ADCRDY：value 输出 0，adc_err = 1，照常换下一个通道。
//
// ADC 设置（gowin_adc.v，云源 IP 生成器生成，配置在 gowin_adc.mod，可用 IDE 的 IP Core Generator 打开）：
//   电压模式，输入 = 专用差分脚 ADCV_P(N12) - ADCV_N(P11)，即底板 J3 的 1 脚与 3 脚，量程 0 ~ 1 V；
//   ADC 时钟 = clk 50 MHz ÷ 8 = 6.25 MHz（IP 内部分频）；每次转换 256 个采样，约 41 µs；
//   ADCVALUE = 电压(V) × 2048，1 计数 ≈ 0.49 mV。
// 一个通道约 150 µs，16 个通道一遍约 2.4 ms。
// =============================================================================
module fsr_scan #(
    parameter integer SETTLE_CYC  = 5000,    // 换通道后等待的时钟数（5000 = 100 µs）
    parameter integer REQ_CYC     = 64,      // ADCREQI 高电平宽度（64 = 8 个 ADC 时钟）
    parameter integer GUARD_CYC   = 400,     // 从请求起至少等这么久才看 ADCRDY（= 50 个 ADC 时钟）
    parameter integer TIMEOUT_CYC = 40000    // 从请求起 800 µs 没有 ADCRDY 视为出错（正常约 41 µs）
) (
    input  wire        clk,          // 50 MHz
    input  wire        rst_n,        // 同步复位，低有效
    output reg  [3:0]  o_sel,        // → 16 路模块 S3..S0（o_sel[0] = S0）
    output reg  [3:0]  ch,           // 本次 value 对应的通道号
    output reg  [11:0] value,        // ADC 读数，0 ~ 4095（限幅；1 V = 2048）
    output reg         valid,        // 1 拍：value 有效
    output reg         last,         // 1 拍：与 valid 同时，表示 0 ~ 15 号通道这一遍读完
    output reg         adc_err       // 1 = 最近一次转换超时（ADC 没有响应）
);

    // ---------------- 片内 ADC ----------------
    wire        adc_rdy;
    wire [13:0] adc_val;
    reg         adc_req;

    Gowin_ADC u_adc (
        .adcrdy       (adc_rdy),
        .adcvalue     (adc_val),
        .clk          (clk),
        .drstn        (rst_n),
        .adcreqi      (adc_req),
        .fscal_value  (10'd653),     // 电压模式增益校准典型值（UG299）
        .offset_value (12'd0)        // 电压模式偏移典型值
    );

    // ---------------- 扫描流程 ----------------
    localparam [1:0] S_SETTLE = 2'd0,   // 等模拟电压稳定
                     S_REQ    = 2'd1,   // 发请求，等 GUARD_CYC
                     S_WAIT   = 2'd2,   // 等 ADCRDY
                     S_READ   = 2'd3;   // ADCRDY 已到，等 2 拍后读数

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [1:0]  rd_cnt;
    reg [1:0]  rdy_s;                   // ADCRDY 两级同步

    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= S_SETTLE;
            cnt     <= 16'd0;
            rd_cnt  <= 2'd0;
            rdy_s   <= 2'b00;
            adc_req <= 1'b0;
            o_sel   <= 4'd0;
            ch      <= 4'd0;
            value   <= 12'd0;
            valid   <= 1'b0;
            last    <= 1'b0;
            adc_err <= 1'b0;
        end else begin
            rdy_s <= {rdy_s[0], adc_rdy};
            valid <= 1'b0;
            last  <= 1'b0;

            case (state)
            S_SETTLE: begin
                if (cnt == SETTLE_CYC - 1) begin
                    cnt     <= 16'd0;
                    adc_req <= 1'b1;                  // 上升沿启动转换
                    state   <= S_REQ;
                end else begin
                    cnt <= cnt + 16'd1;
                end
            end

            S_REQ: begin
                cnt <= cnt + 16'd1;
                if (cnt == REQ_CYC - 1)
                    adc_req <= 1'b0;
                if (cnt == GUARD_CYC - 1)
                    state <= S_WAIT;
            end

            S_WAIT: begin
                cnt <= cnt + 16'd1;
                if (rdy_s[1]) begin
                    rd_cnt <= 2'd0;
                    state  <= S_READ;
                end else if (cnt == TIMEOUT_CYC - 1) begin   // ADC 没有响应：本通道记 0
                    value   <= 12'd0;
                    adc_err <= 1'b1;
                    ch      <= o_sel;
                    valid   <= 1'b1;
                    last    <= (o_sel == 4'd15);
                    o_sel   <= o_sel + 4'd1;
                    cnt     <= 16'd0;
                    state   <= S_SETTLE;
                end
            end

            default: begin                            // S_READ
                rd_cnt <= rd_cnt + 2'd1;
                if (rd_cnt == 2'd2) begin
                    value   <= (adc_val > 14'd4095) ? 12'd4095 : adc_val[11:0];
                    adc_err <= 1'b0;
                    ch      <= o_sel;
                    valid   <= 1'b1;
                    last    <= (o_sel == 4'd15);
                    o_sel   <= o_sel + 4'd1;          // 切到下一个通道，开始等电压稳定
                    cnt     <= 16'd0;
                    state   <= S_SETTLE;
                end
            end
            endcase
        end
    end

endmodule
