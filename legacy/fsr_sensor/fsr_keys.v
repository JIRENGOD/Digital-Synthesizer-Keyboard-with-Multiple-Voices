`timescale 1ns/1ps
// =============================================================================
// fsr_keys.v —— 16 个压力点 → 击键 / 抬键事件 + 力度
// -----------------------------------------------------------------------------
// 输入：fsr_scan 逐个送来的通道读数（每个点约 2.4 ms 读一次）。
//
// 零点（基线）：上电后和按 key[2] 时自动校准 —— 先丢掉 CAL_SKIP 遍扫描（等手离开、电压稳定），
//   再把 16 遍读数的平均值作为每个点的基线。平时没按的点还会慢慢跟踪基线：
//   读数低于基线时每遍减 1，略高于基线时每 64 遍加 1（应对传感器蠕变和漂移）。
//
// 每个点的状态：
//   空闲 ──读数 ≥ 基线+TH_ON──► 起音（记下读数）──下一遍仍 ≥ 基线+TH_OFF──► 发出击键，进入按住
//                                              └─下一遍已 < 基线+TH_OFF──► 当作干扰，回到空闲
//   按住 ──读数 < 基线+TH_OFF──► 发出抬键，回到空闲
//   连续两遍才确认击键，可滤掉单次干扰；第二遍时压力往往还在上升，取两遍中较大的读数算力度。
//   从按下到发出击键最多 2 遍扫描（约 5 ms）。
//
// 力度：增益 = (峰值 - 基线) >> VEL_SHIFT，限制在 VEL_MIN ~ 255（255 = 满音量）。
//   薄膜压力传感器的电导与压力近似成正比，压得越重读数越大，声音越响。
// 基线高于 BASE_MAX 的点（没接好，或校准时一直被压着）不发声，基线降下来后自动恢复。
// =============================================================================
module fsr_keys #(
    parameter integer TH_ON     = 48,    // 击键阈值：比基线高多少（ADC 计数，1 计数 ≈ 0.49 mV）
    parameter integer TH_OFF    = 24,    // 抬键阈值（低于击键阈值，形成迟滞，按住时不会抖动）
    parameter integer BASE_MAX  = 600,   // 基线上限（ADC 计数）
    parameter integer CAL_SKIP  = 32,    // 校准前先丢掉的扫描遍数（约 80 ms）
    parameter integer VEL_SHIFT = 2,     // 力度增益 = (峰值 - 基线) >> VEL_SHIFT
    parameter integer VEL_MIN   = 48     // 最轻的力度增益（48/255 ≈ -14.5 dB）
) (
    input  wire        clk,
    input  wire        rst_n,         // 同步复位，低有效；复位释放后自动校准一次
    input  wire        cal_start,     // 1 拍：重新校准
    input  wire [3:0]  ch,            // ← fsr_scan
    input  wire [11:0] value,
    input  wire        valid,
    input  wire        last,
    output reg         ev_valid,      // 事件有效，保持到 ev_ready
    output reg         ev_on,         // 1 = 击键，0 = 抬键
    output reg  [3:0]  ev_key,        // 压力点编号 0 ~ 15
    output reg  [7:0]  ev_gain,       // 力度增益（击键时有效）
    input  wire        ev_ready,      // 1 拍：事件已被接收
    output reg  [11:0] scan_max,      // 上一遍扫描中最大的 (读数 - 基线)，给指示灯用
    output reg         calibrating    // 1 = 正在校准（不发声）
);

    localparam [1:0]   K_IDLE  = 2'd0,
                       K_ATK   = 2'd1,
                       K_HELD  = 2'd2;
    localparam integer CAL_END = CAL_SKIP + 15;   // 最后一遍校准扫描的序号
    localparam [7:0]   VMIN    = VEL_MIN;

    reg [11:0] base [0:15];       // 基线
    reg [11:0] pk   [0:15];       // 起音那一遍的读数
    reg [1:0]  st   [0:15];       // 状态
    reg [5:0]  upc  [0:15];       // 基线上调计数
    reg [15:0] sum  [0:15];       // 校准累加

    reg [6:0]  cal_n;             // 校准中已扫完的遍数
    reg [11:0] smax;              // 本遍到目前为止最大的 (读数 - 基线)

    // 第 1 拍锁存读数，第 2 拍（p_go）处理
    reg        p_go, p_last;
    reg [3:0]  p_ch;
    reg [11:0] p_v;

    wire [11:0]        b      = base[p_ch];
    wire [11:0]        pkv    = pk[p_ch];
    wire [1:0]         s      = st[p_ch];
    wire [5:0]         u      = upc[p_ch];
    wire [15:0]        sm     = sum[p_ch];
    wire signed [12:0] d      = $signed({1'b0, p_v}) - $signed({1'b0, b});
    wire               ch_ok  = (b <= BASE_MAX);
    wire [11:0]        pkmax  = (p_v > pkv) ? p_v : pkv;
    wire [11:0]        vel    = pkmax - b;                 // 只在 d ≥ TH_OFF 时使用，不会为负
    wire [11:0]        g_raw  = vel >> VEL_SHIFT;
    wire [7:0]         gain   = (g_raw > 12'd255) ? 8'd255 :
                                (g_raw < VEL_MIN) ? VMIN   : g_raw[7:0];
    wire [16:0]        sum_n  = {1'b0, sm} + {5'd0, p_v};
    wire [11:0]        smax_n = (ch_ok && d > 0 && d[11:0] > smax) ? d[11:0] : smax;

    integer k;

    always @(posedge clk) begin
        if (!rst_n) begin
            p_go        <= 1'b0;
            p_last      <= 1'b0;
            p_ch        <= 4'd0;
            p_v         <= 12'd0;
            ev_valid    <= 1'b0;
            ev_on       <= 1'b0;
            ev_key      <= 4'd0;
            ev_gain     <= 8'd0;
            scan_max    <= 12'd0;
            smax        <= 12'd0;
            calibrating <= 1'b1;                       // 上电先校准
            cal_n       <= 7'd0;
            for (k = 0; k < 16; k = k + 1) begin
                st[k]   <= K_IDLE;
                upc[k]  <= 6'd0;
                base[k] <= 12'd0;
            end
        end else begin
            p_go   <= valid;
            p_last <= last;
            if (valid) begin
                p_ch <= ch;
                p_v  <= value;
            end
            if (ev_ready)
                ev_valid <= 1'b0;

            if (cal_start) begin
                calibrating <= 1'b1;
                cal_n       <= 7'd0;
            end

            if (p_go) begin
                if (calibrating) begin
                    // 校准中：按住的点先抬键；丢掉前 CAL_SKIP 遍，再累加 16 遍求平均
                    if (s == K_HELD) begin
                        ev_valid <= 1'b1;
                        ev_on    <= 1'b0;
                        ev_key   <= p_ch;
                    end
                    st[p_ch] <= K_IDLE;
                    if (cal_n >= CAL_SKIP) begin
                        sum[p_ch] <= (cal_n == CAL_SKIP) ? {4'd0, p_v} : sum_n[15:0];
                        if (cal_n == CAL_END)
                            base[p_ch] <= sum_n[15:4];         // 16 遍的平均
                    end
                end else begin
                    case (s)
                    K_IDLE: begin
                        if (ch_ok && d >= TH_ON) begin
                            st[p_ch] <= K_ATK;
                            pk[p_ch] <= p_v;
                        end else if (d < 0) begin
                            base[p_ch] <= b - 12'd1;           // 读数低于基线：基线下调
                        end else if (d > 0) begin
                            upc[p_ch] <= u + 6'd1;
                            if (u == 6'd63)
                                base[p_ch] <= b + 12'd1;       // 长时间略高于基线：基线上调
                        end
                    end
                    K_ATK: begin
                        if (d < TH_OFF) begin
                            st[p_ch] <= K_IDLE;                // 只有一遍超过阈值：当作干扰
                        end else begin
                            st[p_ch] <= K_HELD;                // 确认击键
                            ev_valid <= 1'b1;
                            ev_on    <= 1'b1;
                            ev_key   <= p_ch;
                            ev_gain  <= gain;
                        end
                    end
                    default: begin                             // K_HELD
                        if (d < TH_OFF) begin
                            st[p_ch] <= K_IDLE;                // 抬键
                            ev_valid <= 1'b1;
                            ev_on    <= 1'b0;
                            ev_key   <= p_ch;
                        end
                    end
                    endcase
                end

                if (p_last) begin                              // 一遍扫描结束
                    scan_max <= calibrating ? 12'd0 : smax_n;
                    smax     <= 12'd0;
                    if (calibrating && !cal_start) begin
                        if (cal_n == CAL_END)
                            calibrating <= 1'b0;
                        cal_n <= cal_n + 7'd1;
                    end
                end else begin
                    smax <= calibrating ? 12'd0 : smax_n;
                end
            end
        end
    end

endmodule
