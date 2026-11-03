`timescale 1ns/1ps
// =============================================================================
// key_debounce.v —— 机械按键同步 + 消抖（按下为低电平）
// -----------------------------------------------------------------------------
// 先用两级触发器把按键电平同步到 clk；之后每个 tick 检查一次：
// 电平与已确认的状态不同、并且连续保持 STABLE 个 tick，才确认翻转，
// 同时发出 1 拍 press（确认按下）或 rls（确认松开）。
// =============================================================================
module key_debounce #(
    parameter integer STABLE = 960          // 需连续稳定的 tick 数（20 ms × 48 kHz），1 ~ 2^20
) (
    input  wire clk,
    input  wire rst_n,      // 同步复位，低有效
    input  wire tick,       // 计数节拍（这里用 48 kHz 采样节拍）
    input  wire key_n,      // 按键原始电平：按下为 0
    output reg  down,       // 消抖后的状态：1 = 按住
    output reg  press,      // 确认按下，1 拍
    output reg  rls         // 确认松开，1 拍
);

    reg  [1:0]  sync;
    reg  [19:0] cnt;
    wire        now_down = ~sync[1];

    always @(posedge clk) begin
        if (!rst_n) begin
            sync  <= 2'b11;
            cnt   <= 20'd0;
            down  <= 1'b0;
            press <= 1'b0;
            rls   <= 1'b0;
        end else begin
            sync  <= {sync[0], key_n};
            press <= 1'b0;
            rls   <= 1'b0;
            if (tick) begin
                if (now_down == down) begin
                    cnt <= 20'd0;
                end else if (cnt == STABLE - 1) begin
                    cnt   <= 20'd0;
                    down  <= now_down;
                    press <= now_down;
                    rls   <= ~now_down;
                end else begin
                    cnt <= cnt + 20'd1;
                end
            end
        end
    end

endmodule
