`timescale 1ns/1ps
// =============================================================================
// oled_i2c.v —— 0.96 寸 128×64 OLED（SSD1306 / SSD1315，I2C 4 针）控制器
// -----------------------------------------------------------------------------
// 流程：上电等 100 ms → 发初始化命令 → 循环 { 发显存窗口命令（同时请频谱模块分析）
//       → 等分析完 → 一次发完整屏 1024 字节显存 }，约 25 帧/秒。
//   每发 REINIT 帧重发一次初始化命令（不关显示）：屏幕后插上、接触不好掉电后都能自己恢复。
//   地址字节没有应答时：换另一个常用地址（0x3C ↔ 0x3D），等 50 ms 后从初始化重来，ack_ok = 0。
// I2C 时序：SCL 250 kHz（QTR = 50 个时钟为 1/4 位）。每一位：
//   第 1 个 1/4：SCL 拉低；第 2 个：改 SDA；第 3、4 个：SCL 高（屏幕在这期间读 SDA）。
//   SDA 只在第 9 位（应答位）松开，由屏幕拉低应答，其余时间 FPGA 直接驱动高 / 低电平，
//   上升沿不依赖上拉电阻（屏幕模块没焊上拉也能用）。SCL 一直由 FPGA 驱动（SSD1306 不会拉住 SCL）。
// 显存格式：水平寻址，页 0 ~ 7（每页 8 行），每页 128 列，一个字节 = 一列 8 个点（最低位在上）。
//   第 n 个字节 = 第 n/128 页第 n%128 列，由 oled_frame.v 按 pix_idx 给出。
// =============================================================================
module oled_i2c #(
    parameter integer QTR       = 50,          // I2C 1/4 位时间（时钟数）：50 → SCL 250 kHz
    parameter integer PWR_CYC   = 5_000_000,   // 上电后等 100 ms 再初始化屏幕
    parameter integer RETRY_CYC = 2_500_000,   // 屏幕没应答时等 50 ms 再试
    parameter integer ANA_WAIT  = 500_000,     // 等频谱分析最多 10 ms
    parameter integer REINIT    = 63           // 每发这么多帧重发一次初始化命令，≤ 63
) (
    input  wire       clk,
    input  wire       rst_n,         // 同步复位，低有效
    output reg        scl,           // SCL 输出（推挽）
    output reg        sda_o,         // SDA 输出电平
    output reg        sda_oe,        // 1 = 驱动 SDA，0 = 松开（应答位，由屏幕拉低）
    input  wire       sda_i,         // SDA 实际电平
    output reg  [9:0] pix_idx,       // 下一个要发的显存字节：页 × 128 + 列
    input  wire [7:0] pix_data,      // pix_idx 变化后 2 拍有效
    output reg        ana_start,     // 1 拍：请频谱模块分析
    input  wire       ana_done,      // 1 拍：频谱结果已更新
    output reg        ack_ok         // 1 = 屏幕有应答
);

    // ---------------- 命令表：0 ~ 25 初始化，26 ~ 31 显存窗口（整屏） ----------------
    function [7:0] cmd_byte;
        input [4:0] i;
        begin
            case (i)
            5'd0:  cmd_byte = 8'hAE;   // 关显示
            5'd1:  cmd_byte = 8'hD5;   // 时钟分频
            5'd2:  cmd_byte = 8'h80;
            5'd3:  cmd_byte = 8'hA8;   // 64 行
            5'd4:  cmd_byte = 8'h3F;
            5'd5:  cmd_byte = 8'hD3;   // 显示偏移 0
            5'd6:  cmd_byte = 8'h00;
            5'd7:  cmd_byte = 8'h40;   // 起始行 0
            5'd8:  cmd_byte = 8'h8D;   // 电荷泵：开
            5'd9:  cmd_byte = 8'h14;
            5'd10: cmd_byte = 8'h20;   // 水平寻址
            5'd11: cmd_byte = 8'h00;
            5'd12: cmd_byte = 8'hA1;   // 列反转 + 行反转：排针在上方时画面是正的
            5'd13: cmd_byte = 8'hC8;
            5'd14: cmd_byte = 8'hDA;   // COM 引脚配置（128×64）
            5'd15: cmd_byte = 8'h12;
            5'd16: cmd_byte = 8'h81;   // 对比度
            5'd17: cmd_byte = 8'hCF;
            5'd18: cmd_byte = 8'hD9;   // 预充电
            5'd19: cmd_byte = 8'hF1;
            5'd20: cmd_byte = 8'hDB;   // VCOMH
            5'd21: cmd_byte = 8'h40;
            5'd22: cmd_byte = 8'hA4;   // 按显存显示
            5'd23: cmd_byte = 8'hA6;   // 不反色
            5'd24: cmd_byte = 8'h2E;   // 关滚动
            5'd25: cmd_byte = 8'hAF;   // 开显示
            5'd26: cmd_byte = 8'h21;   // 列地址 0 ~ 127
            5'd27: cmd_byte = 8'h00;
            5'd28: cmd_byte = 8'h7F;
            5'd29: cmd_byte = 8'h22;   // 页地址 0 ~ 7
            5'd30: cmd_byte = 8'h00;
            default: cmd_byte = 8'h07;
            endcase
        end
    endfunction

    localparam [1:0]  P_INIT = 2'd0, P_WIN = 2'd1, P_DATA = 2'd2;      // 要发的传输
    localparam [2:0]  O_PWR  = 3'd0, O_NEXT = 3'd1, O_START = 3'd2, O_BIT = 3'd3,
                      O_ACK  = 3'd4, O_STOP = 3'd5, O_END   = 3'd6, O_WAIT = 3'd7;

    reg  [2:0]  ost;
    reg  [1:0]  phase;
    reg  [1:0]  q;                    // 1/4 位序号
    reg  [5:0]  qc;                   // 1/4 位计时
    wire        qt = (qc == QTR - 1);
    reg  [22:0] wcnt;                 // 等待计时
    reg  [1:0]  sda_s;                // SDA 同步
    reg  [7:0]  sh;                   // 正在发的字节
    reg  [2:0]  bitn;
    reg  [10:0] bcnt;                 // 本次传输第几个字节：0 地址、1 控制字、2.. 内容
    reg  [10:0] blast;                // 最后一个字节的序号
    reg  [4:0]  cbase;                // 命令内容从命令表第几项开始
    reg         addr_sel;             // 0 = 0x3C，1 = 0x3D
    reg         init_full;            // 1 = 初始化从“关显示”开始
    reg         nack;
    reg  [5:0]  fcnt;                 // 上次初始化以来发了几帧
    reg         ana_rdy;

    wire [10:0] kn       = bcnt + 11'd1;                       // 下一个字节的序号
    wire [4:0]  cidx     = cbase + kn[4:0] - 5'd2;
    wire [7:0]  ctrl     = (phase == P_DATA) ? 8'h40 : 8'h00;  // 0x40 = 后面是显存数据，0x00 = 后面是命令
    wire [7:0]  next_pay = (phase == P_DATA) ? pix_data : cmd_byte(cidx);

    always @(posedge clk) begin
        if (!rst_n) begin
            ost       <= O_PWR;
            phase     <= P_INIT;
            q         <= 2'd0;
            qc        <= 6'd0;
            wcnt      <= 23'd0;
            sda_s     <= 2'b11;
            sh        <= 8'd0;
            bitn      <= 3'd0;
            bcnt      <= 11'd0;
            blast     <= 11'd0;
            cbase     <= 5'd0;
            addr_sel  <= 1'b0;
            init_full <= 1'b1;
            nack      <= 1'b0;
            fcnt      <= 6'd0;
            ana_rdy   <= 1'b0;
            scl       <= 1'b1;
            sda_o     <= 1'b1;
            sda_oe    <= 1'b1;
            pix_idx   <= 10'd0;
            ana_start <= 1'b0;
            ack_ok    <= 1'b0;
        end else begin
            sda_s     <= {sda_s[0], sda_i};
            qc        <= qt ? 6'd0 : qc + 6'd1;
            ana_start <= 1'b0;
            if (ana_done)
                ana_rdy <= 1'b1;

            case (ost)
            O_PWR: begin                              // 上电等屏幕稳定
                if (wcnt == PWR_CYC - 1) begin
                    wcnt <= 23'd0;
                    ost  <= O_NEXT;
                end else begin
                    wcnt <= wcnt + 23'd1;
                end
            end

            O_NEXT: begin                             // 准备下一次传输
                nack <= 1'b0;
                bcnt <= 11'd0;
                case (phase)
                P_INIT: begin
                    cbase <= init_full ? 5'd0 : 5'd1;
                    blast <= init_full ? 11'd27 : 11'd26;
                    ost   <= O_START;
                end
                P_WIN: begin
                    cbase     <= 5'd26;
                    blast     <= 11'd7;
                    ana_start <= 1'b1;               // 发窗口命令的同时分析频谱
                    ana_rdy   <= 1'b0;
                    wcnt      <= 23'd0;
                    ost       <= O_START;
                end
                default: begin                        // P_DATA：等频谱分析完（最多 10 ms）
                    blast <= 11'd1025;
                    if (ana_rdy || wcnt >= ANA_WAIT) begin
                        pix_idx <= 10'd0;
                        ost     <= O_START;
                    end else begin
                        wcnt <= wcnt + 23'd1;
                    end
                end
                endcase
                q <= 2'd0;
            end

            O_START: if (qt) begin                    // 起始条件：SCL 高时 SDA 由高变低
                case (q)
                2'd0: begin scl <= 1'b1; sda_o <= 1'b1; sda_oe <= 1'b1; end
                2'd1: sda_o <= 1'b0;
                2'd2: ;
                default: begin
                    sh   <= {(addr_sel ? 7'h3D : 7'h3C), 1'b0};   // 地址字节（写）
                    bitn <= 3'd7;
                    ost  <= O_BIT;
                end
                endcase
                q <= q + 2'd1;
            end

            O_BIT: if (qt) begin                      // 发 1 位：拉低 SCL → 改 SDA → SCL 高 2 个 1/4
                case (q)
                2'd0: scl <= 1'b0;
                2'd1: begin sda_o <= sh[7]; sda_oe <= 1'b1; end
                2'd2: scl <= 1'b1;
                default: begin
                    sh <= {sh[6:0], 1'b0};
                    if (bitn == 3'd0)
                        ost <= O_ACK;
                    else
                        bitn <= bitn - 3'd1;
                end
                endcase
                q <= q + 2'd1;
            end

            O_ACK: if (qt) begin                      // 第 9 位：松开 SDA，屏幕拉低 = 应答
                case (q)
                2'd0: scl <= 1'b0;
                2'd1: sda_oe <= 1'b0;
                2'd2: scl <= 1'b1;
                default: begin
                    if (bcnt == 11'd0 && sda_s[1]) begin          // 地址没有应答
                        nack <= 1'b1;
                        ost  <= O_STOP;
                    end else if (bcnt == blast) begin
                        ost  <= O_STOP;
                    end else begin
                        sh      <= (bcnt == 11'd0) ? ctrl : next_pay;
                        pix_idx <= kn[9:0] - 10'd1;              // 预取再下一个字节（2 拍后有效）
                        bcnt    <= kn;
                        bitn    <= 3'd7;
                        ost     <= O_BIT;
                    end
                end
                endcase
                q <= q + 2'd1;
            end

            O_STOP: if (qt) begin                     // 停止条件：SCL 高时 SDA 由低变高
                case (q)
                2'd0: scl <= 1'b0;
                2'd1: begin sda_o <= 1'b0; sda_oe <= 1'b1; end
                2'd2: scl <= 1'b1;
                default: begin
                    sda_o <= 1'b1;
                    ost   <= O_END;
                end
                endcase
                q <= q + 2'd1;
            end

            O_END: if (qt) begin                      // 一次传输结束
                wcnt <= 23'd0;
                if (nack) begin                       // 没应答：换地址，等一会儿从初始化重来
                    ack_ok    <= 1'b0;
                    addr_sel  <= ~addr_sel;
                    phase     <= P_INIT;
                    init_full <= 1'b1;
                    fcnt      <= 6'd0;
                    ost       <= O_WAIT;
                end else begin
                    ack_ok <= 1'b1;
                    case (phase)
                    P_INIT: phase <= P_WIN;
                    P_WIN:  phase <= P_DATA;
                    default: begin                    // 一帧发完
                        if (fcnt == REINIT - 1) begin
                            fcnt      <= 6'd0;
                            init_full <= 1'b0;        // 定期重发初始化（不关显示）
                            phase     <= P_INIT;
                        end else begin
                            fcnt  <= fcnt + 6'd1;
                            phase <= P_WIN;
                        end
                    end
                    endcase
                    ost <= O_NEXT;
                end
            end

            default: begin                            // O_WAIT：没应答后的等待
                if (wcnt == RETRY_CYC - 1) begin
                    wcnt <= 23'd0;
                    ost  <= O_NEXT;
                end else begin
                    wcnt <= wcnt + 23'd1;
                end
            end
            endcase
        end
    end

endmodule
