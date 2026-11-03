`timescale 1ns/1ps
// =============================================================================
// voice_alloc.v —— 复音分配：把击键 / 抬键事件分配给 NV 个声部（每个声部里有加法合成和电吉他两个发声单元）
// -----------------------------------------------------------------------------
// 事件来源：触摸键盘（touch_keys.v），音序号 0 ~ 45 同时作为“键号”——同一个音不会同时属于两个键
// 击键时选声部（依次）：
//   1. 正在弹或刚松开同一个音的声部 —— 同一根弦再弹一次（不会叠出两个同样的音）；
//   2. 已抬键的声部中最早抬键的（它的余音最弱）；
//   3. 全部都按住时，抢占最早按下的声部。
// 抬键时：找到按住这个音的声部，发 note_off；找不到（已被抢占）就忽略。
// 每个事件用 NV + 2 拍处理完；来源保持 valid，直到收到 1 拍 ready。
// =============================================================================
module voice_alloc #(
    parameter integer NV = 8                  // 声部数，2 ~ 16
) (
    input  wire            clk,
    input  wire            rst_n,             // 同步复位，低有效
    input  wire            ev_valid,
    input  wire            ev_on,             // 1 = 击键，0 = 抬键
    input  wire [5:0]      ev_note,           // 音序号
    input  wire [7:0]      ev_gain,           // 力度增益（击键时有效）
    output reg             ev_ready,
    // 给每个声部
    output reg  [NV-1:0]   v_on,              // 击键脉冲（1 拍）
    output reg  [NV-1:0]   v_off,             // 抬键脉冲（1 拍）
    output reg  [6*NV-1:0] v_note,            // 每个声部的音序号
    output reg  [8*NV-1:0] v_gain             // 每个声部的力度增益
);

    localparam [1:0] A_IDLE  = 2'd0,
                     A_SCAN  = 2'd1,
                     A_APPLY = 2'd2;
    localparam [6:0] NO_KEY  = 7'h7F;         // 声部还没用过（音序号只有 0 ~ 45）

    reg [6:0]    vid   [0:NV-1];              // 每个声部当前的音
    reg [15:0]   stamp [0:NV-1];              // 最近一次击键 / 抬键的事件序号
    reg [NV-1:0] held;                        // 1 = 按住中
    reg [15:0]   now;                         // 事件序号（每个事件加 1）

    reg [1:0]  st;
    reg        e_on;
    reg [6:0]  e_id;
    reg [7:0]  e_gain;
    reg [3:0]  i;
    reg        have_match, have_free, have_held;
    reg [3:0]  match_i, free_i, held_i;
    reg [15:0] free_age, held_age;

    wire [15:0] age = now - stamp[i];         // 越大越早
    wire [3:0]  tgt = have_match ? match_i : (have_free ? free_i : held_i);

    integer k;

    always @(posedge clk) begin
        if (!rst_n) begin
            st         <= A_IDLE;
            e_on       <= 1'b0;
            e_id       <= 7'd0;
            e_gain     <= 8'd0;
            i          <= 4'd0;
            have_match <= 1'b0;
            have_free  <= 1'b0;
            have_held  <= 1'b0;
            match_i    <= 4'd0;
            free_i     <= 4'd0;
            held_i     <= 4'd0;
            free_age   <= 16'd0;
            held_age   <= 16'd0;
            held       <= {NV{1'b0}};
            now        <= 16'd0;
            ev_ready   <= 1'b0;
            v_on       <= {NV{1'b0}};
            v_off      <= {NV{1'b0}};
            v_note     <= {(6*NV){1'b0}};
            v_gain     <= {(8*NV){1'b0}};
            for (k = 0; k < NV; k = k + 1) begin
                vid[k]   <= NO_KEY;
                stamp[k] <= 16'd0;
            end
        end else begin
            ev_ready <= 1'b0;
            v_on     <= {NV{1'b0}};
            v_off    <= {NV{1'b0}};

            case (st)
            A_IDLE: begin
                have_match <= 1'b0;
                have_free  <= 1'b0;
                have_held  <= 1'b0;
                i          <= 4'd0;
                if (ev_valid) begin
                    e_on     <= ev_on;
                    e_id     <= {1'b0, ev_note};
                    e_gain   <= ev_gain;
                    ev_ready <= 1'b1;
                    st       <= A_SCAN;
                end
            end

            A_SCAN: begin                             // 每拍看一个声部
                if (vid[i] == e_id && (e_on || held[i]) && !have_match) begin
                    have_match <= 1'b1;
                    match_i    <= i;
                end
                if (!held[i]) begin
                    if (!have_free || age > free_age) begin
                        have_free <= 1'b1;
                        free_i    <= i;
                        free_age  <= age;
                    end
                end else begin
                    if (!have_held || age > held_age) begin
                        have_held <= 1'b1;
                        held_i    <= i;
                        held_age  <= age;
                    end
                end
                if (i == NV - 1)
                    st <= A_APPLY;
                else
                    i <= i + 4'd1;
            end

            default: begin                            // A_APPLY
                if (e_on) begin
                    vid[tgt]           <= e_id;
                    held[tgt]          <= 1'b1;
                    stamp[tgt]         <= now;
                    now                <= now + 16'd1;
                    v_on[tgt]          <= 1'b1;
                    v_note[tgt*6 +: 6] <= e_id[5:0];
                    v_gain[tgt*8 +: 8] <= e_gain;
                end else if (have_match) begin
                    held[match_i]      <= 1'b0;
                    stamp[match_i]     <= now;
                    now                <= now + 16'd1;
                    v_off[match_i]     <= 1'b1;
                end
                st <= A_IDLE;
            end
            endcase
        end
    end

endmodule
