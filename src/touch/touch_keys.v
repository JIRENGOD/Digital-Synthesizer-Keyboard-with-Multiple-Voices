`timescale 1ns/1ps
// =============================================================================
// touch_keys.v —— 触摸键盘的键位映射、三组音区、按下 / 松开事件
// -----------------------------------------------------------------------------
// 键位（10-31 上板确认，KEY_REVERSE = 1：从正面看 KEY1 在最左边）：
//   白键 从左到右 KEY1 ~ KEY9，黑键 从左到右 KEY10 ~ KEY15；KEY16（最右边带钢琴图标的那块）是功能键。
//   KEY_REVERSE = 0 时左右反过来。
// 三组音区（KEY16 每摸一下切换：中音 → 高音 → 低音 → 中音 …，上电为中音）：
//   组 0 低音：白键 A2 B2 C3 D3 E3 F3 G3 A3 B3（最高是低音“西”），  黑键 A#2 C#3 D#3 F#3 G#3 A#3
//   组 1 中音：白键 C4 D4 E4 F4 G4 A4 B4 C5 D5（哆 ~ 高音“来”），   黑键 C#4 D#4 F#4 G#4 A#4 C#5
//   组 2 高音：白键 E5 F5 G5 A5 B5 C6 D6 E6 F6（从高音“咪”开始），  黑键 F#5 G#5 A#5 C#6 D#6 F#6
//   音序号 = MIDI 音号 - 45（A2 = 0 … F#6 = 45），见 piano_params.v。三组的音互不重叠。
// 事件：逐个比较 15 个琴键的当前状态和已报告的状态，有变化就发一个事件（保持到 ev_ready），
//   和弦（同时按几个键）会在几十个时钟内依次发完。
//   按下时记住这个键发的音，松开时报同一个音 —— 按住一个键时换音区，松开仍能正确止音。
// =============================================================================
module touch_keys #(
    parameter integer KEY_REVERSE = 1
) (
    input  wire        clk,
    input  wire        rst_n,         // 同步复位，低有效
    input  wire [15:0] keys,          // ← ttp229_rd，bit0 = KEY1
    output reg         ev_valid,      // 事件有效，保持到 ev_ready
    output reg         ev_on,         // 1 = 按下，0 = 松开
    output reg  [5:0]  ev_note,       // 音序号 0 ~ 45
    input  wire        ev_ready,      // 1 拍：事件已被接收
    output reg  [1:0]  group          // 当前音区：0 低音，1 中音，2 高音
);

    // 第 g 组第 pos 个键的音序号；pos 0 ~ 8 = 白键从左到右，9 ~ 14 = 黑键从左到右
    function [5:0] key_note;
        input [1:0] g;
        input [3:0] pos;
        begin
            case ({g, pos})
            // 低音组：A2 B2 C3 D3 E3 F3 G3 A3 B3 / A#2 C#3 D#3 F#3 G#3 A#3
            {2'd0, 4'd0}:  key_note = 6'd0;   {2'd0, 4'd1}:  key_note = 6'd2;
            {2'd0, 4'd2}:  key_note = 6'd3;   {2'd0, 4'd3}:  key_note = 6'd5;
            {2'd0, 4'd4}:  key_note = 6'd7;   {2'd0, 4'd5}:  key_note = 6'd8;
            {2'd0, 4'd6}:  key_note = 6'd10;  {2'd0, 4'd7}:  key_note = 6'd12;
            {2'd0, 4'd8}:  key_note = 6'd14;
            {2'd0, 4'd9}:  key_note = 6'd1;   {2'd0, 4'd10}: key_note = 6'd4;
            {2'd0, 4'd11}: key_note = 6'd6;   {2'd0, 4'd12}: key_note = 6'd9;
            {2'd0, 4'd13}: key_note = 6'd11;  {2'd0, 4'd14}: key_note = 6'd13;
            // 中音组：C4 D4 E4 F4 G4 A4 B4 C5 D5 / C#4 D#4 F#4 G#4 A#4 C#5
            {2'd1, 4'd0}:  key_note = 6'd15;  {2'd1, 4'd1}:  key_note = 6'd17;
            {2'd1, 4'd2}:  key_note = 6'd19;  {2'd1, 4'd3}:  key_note = 6'd20;
            {2'd1, 4'd4}:  key_note = 6'd22;  {2'd1, 4'd5}:  key_note = 6'd24;
            {2'd1, 4'd6}:  key_note = 6'd26;  {2'd1, 4'd7}:  key_note = 6'd27;
            {2'd1, 4'd8}:  key_note = 6'd29;
            {2'd1, 4'd9}:  key_note = 6'd16;  {2'd1, 4'd10}: key_note = 6'd18;
            {2'd1, 4'd11}: key_note = 6'd21;  {2'd1, 4'd12}: key_note = 6'd23;
            {2'd1, 4'd13}: key_note = 6'd25;  {2'd1, 4'd14}: key_note = 6'd28;
            // 高音组：E5 F5 G5 A5 B5 C6 D6 E6 F6 / F#5 G#5 A#5 C#6 D#6 F#6
            {2'd2, 4'd0}:  key_note = 6'd31;  {2'd2, 4'd1}:  key_note = 6'd32;
            {2'd2, 4'd2}:  key_note = 6'd34;  {2'd2, 4'd3}:  key_note = 6'd36;
            {2'd2, 4'd4}:  key_note = 6'd38;  {2'd2, 4'd5}:  key_note = 6'd39;
            {2'd2, 4'd6}:  key_note = 6'd41;  {2'd2, 4'd7}:  key_note = 6'd43;
            {2'd2, 4'd8}:  key_note = 6'd44;
            {2'd2, 4'd9}:  key_note = 6'd33;  {2'd2, 4'd10}: key_note = 6'd35;
            {2'd2, 4'd11}: key_note = 6'd37;  {2'd2, 4'd12}: key_note = 6'd40;
            {2'd2, 4'd13}: key_note = 6'd42;  {2'd2, 4'd14}: key_note = 6'd45;
            default:       key_note = 6'd15;
            endcase
        end
    endfunction

    // KEY(k+1) 在键盘上的位置：0 ~ 8 白键从左到右，9 ~ 14 黑键从左到右
    function [3:0] key_pos;
        input [3:0] k;                // 0 = KEY1 … 14 = KEY15
        begin
            if (KEY_REVERSE != 0)
                key_pos = k;
            else if (k < 4'd9)
                key_pos = 4'd8 - k;
            else
                key_pos = 4'd9 + (4'd14 - k);   // KEY10 → 黑键 5（最右）… KEY15 → 黑键 0（最左）
        end
    endfunction

    reg [14:0] cur;                   // 已经报告出去的 15 个琴键状态
    reg [5:0]  held_note [0:14];      // 每个琴键按下时发的音
    reg [3:0]  si;                    // 正在比较的键
    reg        fn_prev;               // KEY16 上一次的状态

    always @(posedge clk) begin
        if (!rst_n) begin
            cur      <= 15'd0;
            si       <= 4'd0;
            fn_prev  <= 1'b0;
            group    <= 2'd1;             // 上电：中音组
            ev_valid <= 1'b0;
            ev_on    <= 1'b0;
            ev_note  <= 6'd0;
        end else begin
            // 功能键 KEY16：摸下时切换音区 中 → 高 → 低 → 中
            fn_prev <= keys[15];
            if (keys[15] && !fn_prev)
                group <= (group == 2'd1) ? 2'd2 : (group == 2'd2) ? 2'd0 : 2'd1;

            // 琴键 KEY1 ~ KEY15
            if (ev_valid) begin
                if (ev_ready) begin
                    ev_valid <= 1'b0;
                    si       <= (si == 4'd14) ? 4'd0 : si + 4'd1;
                end
            end else if (keys[si] != cur[si]) begin
                ev_valid <= 1'b1;
                ev_on    <= keys[si];
                cur[si]  <= keys[si];
                if (keys[si]) begin
                    ev_note       <= key_note(group, key_pos(si));
                    held_note[si] <= key_note(group, key_pos(si));
                end else begin
                    ev_note <= held_note[si];
                end
            end else begin
                si <= (si == 4'd14) ? 4'd0 : si + 4'd1;
            end
        end
    end

endmodule
