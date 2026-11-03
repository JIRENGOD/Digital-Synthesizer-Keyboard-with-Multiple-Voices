`timescale 1ns/1ps
// =============================================================================
// piano_voice.v —— 加法合成单音引擎：钢琴 / 大提琴（音色参数来自 piano_params.v）
// -----------------------------------------------------------------------------
// 原理：每个采样（48.828 kHz）把参数表里的全部振荡器依次算一遍再相加：
//     输出 = Σ 正弦表[相位_k] × 包络_k        （噪声那一项用低通噪声代替正弦）
//   相位_k：32 位相位累加器，每个采样加 inc_k（数控振荡器 NCO 原理）；
//   包络_k：24 位无符号数，按下面的状态机变化：
//
//     note_on ──► 起音 ATT ──到达 peak──► 衰减1 DEC1 ──低于 knee──► 衰减2 DEC2
//     任何状态下 note_on 都重新起音（相位不清零，波形连续，不会“咔”一声）；
//     note_off 进入抬键 REL，按 m_rel 衰减（钢琴：制音器；大提琴：弓离开弦）。
//
//   指数衰减用“移位减法”：每隔 m 个采样，包络 -= 包络 >> 8，即 ×(1-1/256)，
//   衰减速率 = 0.0339 dB × 48828 / m（dB/s）。
//
// 音色（patch，与 note_on 同时给出，击键时锁存）：0 = 钢琴，1 = 大提琴。
//   大提琴另加颤音：每个声部一个约 5.5 Hz 的低频振荡器（各声部频率略有不同，几个音一起拉时更自然），
//   击键后约 0.35 s 内由浅变深，最深 ±18 音分；所有谐波的频率按同一比例变化（相位增量 × (1 + 颤音量)）。
//
// 相位增量 = 基音增量 base（按 {音色, 音} 查表）× 频率倍数 ratio（按 {音色, 振荡器} 查表），用 DSP 乘法器算；
//   高于约 20 kHz 的分音（高音区钢琴的最高几个分音）静音，防止混叠成难听的低频。
//
// 时序（clk = 50 MHz，每个采样 1024 个时钟）：
//   每个振荡器 21 拍：PREP 1 拍（参数表、本振荡器的状态读出并寄存）
//                     + MULF 1 拍（相位增量 = base × ratio）
//                     + VIB 1 拍（算颤音：相位增量 × 颤音量）
//                     + FETCH 1 拍（写回新相位/包络，同时发出正弦表地址）
//                     + LOAD 1 拍 + 16 拍移位相加乘法（正弦 16 位 × 包络高 16 位）。
//   21 个振荡器约 440 拍。多个 piano_voice 并行工作即为多复音（见 top_touch_synth.v），时序完全相同。
//
// 复位：同步复位，低有效。复位释放后先用 32 拍把全部振荡器状态清零（INIT），
//       不依赖 initial 或上电初值。
//
// note：音序号 0 ~ 45（见 piano_params.v 的音表）。在有击键的那个采样开始时锁存，
//       所以要和 note_on 同时给出并保持到下一次击键；换音时相位不清零，不会“咔”一声。
//
// pure = 1：对照用的纯正弦 —— 只输出当前音的第 0 个振荡器（基音），幅度固定为半满幅，持续发声，
//           其余分音和噪声静音（包络照常在后台更新，切回时无需重新初始化）。
// =============================================================================
module piano_voice #(
    parameter integer LUT_ADDR_W = 12,               // 正弦表深度 2^12，须与 sine_lut.hex 一致
    parameter         LUT_FILE   = "sine_lut.hex",
    parameter integer VIB_INC    = 118,              // 颤音频率 = VIB_INC × 48828 / 2^20 Hz（118 ≈ 5.5 Hz）
    parameter integer VIB_DEPTH  = 683,              // 最深颤音 = ± VIB_DEPTH / 65536（683 ≈ ±18 音分）
    parameter integer VIB_STEP   = 10                // 颤音加深速度：每个采样深度 += VIB_STEP / 256（约 0.35 s 到最深）
) (
    input  wire               clk,           // 50 MHz
    input  wire               rst_n,         // 同步复位，低有效
    input  wire               sample_en,     // 每个采样 1 拍（48 kHz）
    input  wire               note_on,       // 击键：1 拍脉冲，任意时刻
    input  wire               note_off,      // 抬键：1 拍脉冲
    input  wire [5:0]         note,          // 音序号 0 ~ 45（见 piano_params.v），与 note_on 同时给出
    input  wire               patch,         // 音色：0 = 钢琴，1 = 大提琴，与 note_on 同时给出
    input  wire               pure,          // 1 = 只输出基音纯正弦（对照用）
    output reg  signed [15:0] sample,        // 16 位有符号输出
    output reg                sample_valid   // 新样本有效，1 拍
);

    // 包络状态（复位清零后为 ST_REL 且包络为 0，即静音）
    localparam [1:0] ST_REL  = 2'd0,   // 抬键 / 静止：按 m_rel 衰减
                     ST_ATT  = 2'd1,   // 起音：线性上升到 peak
                     ST_DEC1 = 2'd2,   // 衰减 1
                     ST_DEC2 = 2'd3;   // 衰减 2

    // 计算流程
    localparam [3:0] S_INIT  = 4'd0,
                     S_IDLE  = 4'd1,
                     S_PREP  = 4'd2,
                     S_MULF  = 4'd3,
                     S_VIB   = 4'd4,
                     S_FETCH = 4'd5,
                     S_LOAD  = 4'd6,
                     S_MUL   = 4'd7,
                     S_OUT   = 4'd8;

    localparam [17:0] DEP_MAX  = VIB_DEPTH * 256;
    localparam [31:0] INC_MUTE = 32'h68DB8BAC;    // 20 kHz 对应的相位增量，分音频率高于它就静音

    reg [3:0] state;
    reg [4:0] idx;                     // 当前振荡器序号
    reg [5:0] note_q;                  // 当前音序号（击键时更新）
    reg       patch_l;                 // note_on 时锁存的音色
    reg       patch_q;                 // 当前音色（击键时更新）
    reg       frame_on, frame_off;     // 本采样内是否有击键 / 抬键

    // ---------------- 音色参数表（输出晚 1 拍：PREP 拍给地址，MULF 拍有效，寄存到 r_*） ----------------
    wire [26:0] p_base;
    wire [24:0] p_ratio;
    wire [23:0] p_peak, p_att, p_knee;
    wire [11:0] p_m1, p_m2, p_mrel;
    wire        p_noise, p_last;
    wire [3:0]  p_lpf;

    piano_params u_piano_params (
        .clk       (clk),
        .patch     (patch_q),
        .note      (note_q),
        .idx       (idx),
        .base      (p_base),
        .ratio     (p_ratio),
        .peak      (p_peak),
        .att       (p_att),
        .knee      (p_knee),
        .m1        (p_m1),
        .m2        (p_m2),
        .is_noise  (p_noise),
        .is_last   (p_last),
        .m_rel     (p_mrel),
        .noise_lpf (p_lpf)
    );

    reg [31:0] r_inc;
    reg [23:0] r_peak, r_att, r_knee;
    reg [11:0] r_m1, r_m2;
    reg        r_noise, r_last;
    wire [51:0] fprod = p_base * p_ratio;         // 基音增量 × 频率倍数（Q5.20）

    always @(posedge clk) begin
        if (state == S_MULF) begin
            r_inc   <= fprod[51:20];
            r_peak  <= p_peak;
            r_att   <= p_att;
            r_knee  <= p_knee;
            r_m1    <= p_m1;
            r_m2    <= p_m2;
            r_noise <= p_noise;
            r_last  <= p_last;
        end
    end

    // ---------------- 每个振荡器的状态（32 项，实际用几项由参数表决定） ----------------
    reg [31:0] ph_mem  [0:31];         // 相位
    reg [23:0] env_mem [0:31];         // 包络
    reg [1:0]  stg_mem [0:31];         // 包络状态
    reg [11:0] cnt_mem [0:31];         // 衰减步进计数

    // PREP 拍先把本振荡器的状态读出来寄存，FETCH 拍再计算并写回
    // （读、算、写分在不同的拍，时序路径短；FETCH 拍用的仍是更新前的值，功能不变）
    reg [31:0] ph_rd;
    reg [23:0] env_rd;
    reg [1:0]  stg_rd;
    reg [11:0] cnt_rd;

    always @(posedge clk) begin
        if (state == S_PREP) begin
            ph_rd  <= ph_mem[idx];
            env_rd <= env_mem[idx];
            stg_rd <= stg_mem[idx];
            cnt_rd <= cnt_mem[idx];
        end
    end

    // ---------------- 大提琴颤音（钢琴时颤音量恒为 0） ----------------
    reg  [19:0]        vib_ph;         // 颤音低频振荡器相位
    reg  [17:0]        dep_acc;        // 颤音深度（低 8 位是小数），击键时清零
    wire signed [16:0] vu   = $signed({vib_ph[19], vib_ph[19:4]});   // 相位 -0.5 ~ 0.5 周期 → -32768 ~ 32767
    wire        [16:0] vabs = vu[16] ? (17'd0 - vu) : vu;             // |相位|：0 ~ 32768
    wire signed [17:0] vfac = $signed({1'b0, 17'd32768 - vabs});      // 0 ~ 32768
    reg  signed [34:0] vsq;            // 相位 × (0.5 周期 - |相位|)：抛物线近似正弦
    reg  signed [15:0] vsin;           // ≈ sin(2π × 相位) × 16384
    reg  signed [27:0] vdep;
    reg  signed [11:0] vib;            // 本采样的颤音量：± VIB_DEPTH（/ 65536）

    always @(posedge clk) begin        // 每拍都算一遍，相位 / 深度每个采样才变一次
        vsq  <= vu * vfac;
        vsin <= vsq >>> 14;
        vdep <= vsin * $signed({1'b0, dep_acc[17:8]});
        vib  <= patch_q ? (vdep >>> 14) : 12'sd0;
    end

    reg  signed [33:0] vprod;          // (相位增量 >> 12) × 颤音量
    wire signed [31:0] vadd = vprod >>> 4;   // = 相位增量 × 颤音量 / 65536

    always @(posedge clk) begin
        if (state == S_VIB)
            vprod <= $signed({1'b0, r_inc[31:12]}) * vib;
    end

    // ---------------- 包络下一状态（组合逻辑） ----------------
    reg  [1:0]  stg_nx;
    reg  [23:0] env_nx;
    reg  [11:0] cnt_nx;
    reg  [11:0] period;
    wire [24:0] att_sum = {1'b0, env_rd} + {1'b0, r_att};
    wire [23:0] env_dec = env_rd - (env_rd >> 8);       // ×(1 - 1/256)

    always @(*) begin
        stg_nx = stg_rd;
        cnt_nx = cnt_rd;
        env_nx = env_rd;
        if (frame_on) begin
            stg_nx = ST_ATT;
            cnt_nx = 12'd0;
        end else if (frame_off && stg_rd != ST_REL) begin
            stg_nx = ST_REL;
            cnt_nx = 12'd0;
        end

        case (stg_nx)
            ST_DEC1: period = r_m1;
            ST_DEC2: period = r_m2;
            default: period = p_mrel;
        endcase

        if (stg_nx == ST_ATT) begin
            if (att_sum >= {1'b0, r_peak}) begin
                env_nx = r_peak;
                stg_nx = ST_DEC1;
                cnt_nx = 12'd0;
            end else begin
                env_nx = att_sum[23:0];
            end
        end else begin
            if (cnt_nx + 12'd1 >= period) begin             // 每 period 个采样衰减一步
                cnt_nx = 12'd0;
                env_nx = env_dec;
            end else begin
                cnt_nx = cnt_nx + 12'd1;
            end
            if (stg_nx == ST_DEC1 && env_nx < r_knee)
                stg_nx = ST_DEC2;
        end
    end

    // 状态存储器写入：INIT 时清零，FETCH 时写回本振荡器的新状态
    always @(posedge clk) begin
        if (state == S_INIT) begin
            ph_mem[idx]  <= 32'd0;
            env_mem[idx] <= 24'd0;
            stg_mem[idx] <= ST_REL;
            cnt_mem[idx] <= 12'd0;
        end else if (state == S_FETCH) begin
            ph_mem[idx]  <= ph_rd + r_inc + vadd;
            env_mem[idx] <= env_nx;
            stg_mem[idx] <= stg_nx;
            cnt_mem[idx] <= cnt_nx;
        end
    end

    // ---------------- 正弦表（FETCH 拍给地址，LOAD 拍得到数据） ----------------
    wire signed [15:0] lut_q;

    sine_lut #(
        .ADDR_W   (LUT_ADDR_W),
        .DATA_W   (16),
        .LUT_FILE (LUT_FILE)
    ) u_sine_lut (
        .clk   (clk),
        .rd_en (state == S_FETCH),
        .addr  (ph_rd[31 -: LUT_ADDR_W]),
        .data  (lut_q)
    );

    // ---------------- 噪声：16 位 LFSR 白噪声 → 一阶低通 ----------------
    reg  [15:0]        lfsr;
    reg  signed [15:0] lpf;
    wire [15:0]        lfsr_nx  = lfsr[0] ? ((lfsr >> 1) ^ 16'hB400) : (lfsr >> 1);
    wire signed [16:0] lpf_diff = $signed({lfsr[15], lfsr}) - $signed({lpf[15], lpf});
    wire signed [16:0] lpf_nx   = $signed({lpf[15], lpf}) + (lpf_diff >>> p_lpf);

    // ---------------- 计算流程 ----------------
    reg                start_pend, on_pend, off_pend;
    reg  [15:0]        e16;            // 当前振荡器包络高 16 位（乘数）
    reg  signed [31:0] mcand;          // 被乘数（正弦或噪声），每拍左移
    reg  [15:0]        mplier;         // 乘数（包络），每拍右移
    reg  [3:0]         bitn;
    reg  signed [35:0] acc;            // 累加器
    wire signed [35:0] acc_sh = acc >>> 16;

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_INIT;
            idx          <= 5'd0;
            note_q       <= 6'd0;
            patch_l      <= 1'b0;
            patch_q      <= 1'b0;
            frame_on     <= 1'b0;
            frame_off    <= 1'b0;
            start_pend   <= 1'b0;
            on_pend      <= 1'b0;
            off_pend     <= 1'b0;
            e16          <= 16'd0;
            mcand        <= 32'sd0;
            mplier       <= 16'd0;
            bitn         <= 4'd0;
            acc          <= 36'sd0;
            lfsr         <= 16'hACE1;
            lpf          <= 16'sd0;
            vib_ph       <= 20'd0;
            dep_acc      <= 18'd0;
            sample       <= 16'sd0;
            sample_valid <= 1'b0;
        end else begin
            sample_valid <= 1'b0;
            if (sample_en) start_pend <= 1'b1;
            if (note_on)   on_pend    <= 1'b1;
            if (note_off)  off_pend   <= 1'b1;
            if (note_on)   patch_l    <= patch;

            case (state)
            S_INIT: begin                         // 清零全部 32 项状态
                idx <= idx + 5'd1;
                if (idx == 5'd31)
                    state <= S_IDLE;
            end

            S_IDLE: begin                         // 等待下一个采样
                if (start_pend) begin
                    start_pend <= 1'b0;
                    frame_on   <= on_pend  | note_on;
                    frame_off  <= off_pend | note_off;
                    if (on_pend | note_on) begin  // 有击键：本采样起换成新的音、新的音色，颤音从浅开始
                        note_q  <= note;
                        patch_q <= note_on ? patch : patch_l;
                        dep_acc <= 18'd0;
                    end else if (dep_acc < DEP_MAX) begin
                        dep_acc <= dep_acc + VIB_STEP;
                    end
                    vib_ph     <= vib_ph + VIB_INC;
                    on_pend    <= 1'b0;
                    off_pend   <= 1'b0;
                    lfsr       <= lfsr_nx;        // 噪声每个采样更新一次
                    lpf        <= lpf_nx[15:0];
                    acc        <= 36'sd0;
                    idx        <= 5'd0;
                    state      <= S_PREP;
                end
            end

            S_PREP: state <= S_MULF;              // 参数表给地址、读出本振荡器的状态

            S_MULF: state <= S_VIB;               // 相位增量在上面的 always 块中算出并寄存

            S_VIB: state <= S_FETCH;              // 颤音乘法在上面的 always 块中寄存

            S_FETCH: begin                        // 新状态在上面的存储器 always 块中写回
                if (pure)                         // 纯正弦：只留基音，固定半满幅
                    e16 <= (idx == 5'd0) ? 16'h8000 : 16'h0000;
                else if (r_inc >= INC_MUTE)       // 高于 20 kHz 的分音静音
                    e16 <= 16'h0000;
                else
                    e16 <= env_rd[23:8];          // 用更新前的包络
                state <= S_LOAD;
            end

            S_LOAD: begin                         // 正弦表数据此时有效
                mcand  <= r_noise ? {{16{lpf[15]}}, lpf} : {{16{lut_q[15]}}, lut_q};
                mplier <= e16;
                bitn   <= 4'd0;
                state  <= S_MUL;
            end

            S_MUL: begin                          // 16 拍移位相加：acc += mcand × mplier
                if (mplier[0])
                    acc <= acc + {{4{mcand[31]}}, mcand};
                mcand  <= mcand <<< 1;
                mplier <= mplier >> 1;
                bitn   <= bitn + 4'd1;
                if (bitn == 4'd15) begin
                    if (r_last) begin
                        state <= S_OUT;
                    end else begin
                        idx   <= idx + 5'd1;
                        state <= S_PREP;
                    end
                end
            end

            default: begin                        // S_OUT：缩放到 16 位并限幅（参数表已留余量，正常不会触发）
                if (acc_sh > 36'sd32767)
                    sample <= 16'sd32767;
                else if (acc_sh < -36'sd32768)
                    sample <= -16'sd32768;
                else
                    sample <= acc_sh[15:0];
                sample_valid <= 1'b1;
                frame_on     <= 1'b0;
                frame_off    <= 1'b0;
                state        <= S_IDLE;
            end
            endcase
        end
    end

endmodule
