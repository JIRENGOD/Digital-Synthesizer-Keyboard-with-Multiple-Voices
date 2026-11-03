`timescale 1ns/1ps
// =============================================================================
// uart.v —— 串口收发（HC-05 蓝牙模块默认格式：9600 波特、8 位数据、无校验、1 位停止位）
// -----------------------------------------------------------------------------
//   uart_tx：start 来 1 拍时发送 data（先发最低位），busy = 1 期间不接受新数据
//   uart_rx：收到一个字节后 valid 来 1 拍；在每一位的中间采样，停止位不是 1 的字节丢掉
// 位时间 = DIV 个时钟：50 MHz / 9600 = 5208.3 → 5208，误差 0.006%。
// =============================================================================
module uart_tx #(
    parameter integer DIV = 5208
) (
    input  wire       clk,
    input  wire       rst_n,        // 同步复位，低有效
    input  wire       start,        // 1 拍：发送 data
    input  wire [7:0] data,
    output reg        busy,
    output reg        tx            // 空闲为 1
);

    reg [9:0]  sh;                  // {停止位, 数据, 起始位}，每发一位右移一次
    reg [3:0]  nb;
    reg [12:0] cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0;
            tx   <= 1'b1;
            sh   <= 10'h3FF;
            nb   <= 4'd0;
            cnt  <= 13'd0;
        end else if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                sh   <= {1'b1, data, 1'b0};
                nb   <= 4'd0;
                cnt  <= 13'd0;
                busy <= 1'b1;
                tx   <= 1'b0;               // 起始位
            end
        end else if (cnt == DIV - 1) begin
            cnt <= 13'd0;
            if (nb == 4'd9) begin           // 停止位发完
                busy <= 1'b0;
                tx   <= 1'b1;
            end else begin
                nb <= nb + 4'd1;
                sh <= {1'b1, sh[9:1]};
                tx <= sh[1];                // 下一位
            end
        end else begin
            cnt <= cnt + 13'd1;
        end
    end

endmodule


module uart_rx #(
    parameter integer DIV = 5208
) (
    input  wire       clk,
    input  wire       rst_n,        // 同步复位，低有效
    input  wire       rx,           // 空闲为 1
    output reg        valid,        // 1 拍：data 有效
    output reg  [7:0] data
);

    reg [1:0]  s;                   // 两级同步
    reg        busy;
    reg [3:0]  nb;                  // 0 = 起始位，1 ~ 8 = 数据位，9 = 停止位
    reg [12:0] cnt;
    reg [7:0]  sh;

    always @(posedge clk) begin
        if (!rst_n) begin
            s     <= 2'b11;
            busy  <= 1'b0;
            nb    <= 4'd0;
            cnt   <= 13'd0;
            sh    <= 8'd0;
            valid <= 1'b0;
            data  <= 8'd0;
        end else begin
            s     <= {s[0], rx};
            valid <= 1'b0;
            if (!busy) begin
                if (!s[1]) begin            // 起始位的下降沿
                    busy <= 1'b1;
                    nb   <= 4'd0;
                    cnt  <= 13'd0;
                end
            end else if (cnt == ((nb == 4'd0) ? (DIV / 2 - 1) : (DIV - 1))) begin
                cnt <= 13'd0;
                if (nb == 4'd0) begin       // 起始位中间：还是 0 才算数
                    if (s[1])
                        busy <= 1'b0;
                    else
                        nb <= 4'd1;
                end else if (nb != 4'd9) begin
                    sh <= {s[1], sh[7:1]};  // 先收到的是最低位
                    nb <= nb + 4'd1;
                end else begin              // 停止位中间
                    busy <= 1'b0;
                    if (s[1]) begin
                        valid <= 1'b1;
                        data  <= sh;
                    end
                end
            end else begin
                cnt <= cnt + 13'd1;
            end
        end
    end

endmodule
