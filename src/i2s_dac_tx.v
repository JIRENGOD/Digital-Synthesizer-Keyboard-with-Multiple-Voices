`timescale 1ns/1ps
// =============================================================================
// i2s_dac_tx.v —— PT8211 串行音频发送器
// -----------------------------------------------------------------------------
// =============================================================================
module i2s_dac_tx (
    input  wire               clk,          // 50 MHz
    input  wire               rst_n,        // 同步复位，低有效
    input  wire               tick,         // 时钟使能，4 × BCK = 6.25 MHz
    input  wire signed [15:0] data_l,
    input  wire signed [15:0] data_r,
    output reg                sample_req,   // 每帧 1 拍（48.828 kHz）
    output reg                o_bck,
    output reg                o_lrck,       // 0 = 左声道
    output reg                o_sdata
);

    reg [6:0]  q;             // 帧内位置（单位：tick）
    reg [15:0] shreg;         // 当前声道还没送出的位
    reg [15:0] hold_r;        // 本帧右声道数据

    always @(posedge clk) begin
        if (!rst_n) begin
            q          <= 7'd0;
            shreg      <= 16'd0;
            hold_r     <= 16'd0;
            o_bck      <= 1'b0;
            o_lrck     <= 1'b0;
            o_sdata    <= 1'b0;
            sample_req <= 1'b0;
        end else begin
            sample_req <= 1'b0;
            if (tick) begin
                q <= q + 7'd1;
                case (q[1:0])
                2'd0: o_bck <= 1'b1;                          // BCK ↑
                2'd1: begin                                   // 送出第 q[6:2] 位
                    if (q[6:2] == 5'd0) begin                 // 新的一帧：左声道 MSB
                        o_lrck     <= 1'b0;
                        o_sdata    <= data_l[15];
                        shreg      <= {data_l[14:0], 1'b0};
                        hold_r     <= data_r;
                        sample_req <= 1'b1;
                    end else if (q[6:2] == 5'd16) begin       // 右声道 MSB
                        o_lrck  <= 1'b1;
                        o_sdata <= hold_r[15];
                        shreg   <= {hold_r[14:0], 1'b0};
                    end else begin
                        o_sdata <= shreg[15];
                        shreg   <= {shreg[14:0], 1'b0};
                    end
                end
                2'd2: o_bck <= 1'b0;                          // BCK ↓
                default: ;
                endcase
            end
        end
    end

endmodule
