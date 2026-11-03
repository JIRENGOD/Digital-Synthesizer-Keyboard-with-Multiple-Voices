#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 src/guitar/guitar_params.v —— 电吉他音色参数表（供 src/guitar/guitar_voice.v 使用）。

合成方法：Karplus-Strong 拨弦算法（物理建模）。一根弦 = 一条延迟线 + 环路滤波器：
    v[n] = g × 全通( (v[n-M] + v[n-M-1]) / 2 )
  拨弦：先往延迟线里灌一个周期（M+1 个采样）的噪声，相当于拨片把弦拨乱；之后信号在环路里来回走：
    · 两点平均是低通，高次谐波每走一圈损失更多 → 声音由亮变暗，和真实琴弦一样；
    · 环路总延迟 M + 0.5 + d = Fs / f0 决定音高：整数部分 M 用延迟线，小数部分 d（0.5 ~ 1.5）
      用一阶全通滤波器 A(z) = (C + z^-1) / (1 + C·z^-1) 实现，音高精确；
    · g 略小于 1，决定余音长短；抬键后换成小得多的 g，模拟手掌制音。
  拾音器：输出 = v[n] - v[n-P]，P = 拾音器位置 × 一个周期，产生电吉他特有的梳状音色。
  过载失真和音箱模拟在 src/guitar/gtr_amp.v。

音表：与 scripts/gen_piano_params.py 的钢琴音序号相同（0 ~ 45 = A2 ~ F#6，直接从那里读取），
  同一个键钢琴和电吉他音高一样。电脑端的录音渲染程序 scripts/bt_recorder.py 也调用这里的 build()。

用法（在工程根目录执行）：
    python scripts/gen_guitar_params.py
改完后在云源里重新综合即可。
"""
import argparse
import cmath
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_piano_params import FS, NOTE_COUNT, NOTE_MIDI0, midi_hz, midi_name   # noqa: E402

DEPTH = 1024                # 延迟线深度（guitar_voice.v），M + 2 不能超过它
# ============================== 音色旋钮（可改） ==============================
SUSTAIN_DBPS = 6.0          # 按住时基音的衰减速率（dB/s）：6 dB/s ≈ 10 s 衰减 60 dB
RELEASE_DBPS = 150.0        # 抬键（手掌制音）后的衰减速率（dB/s）
PICKUP = 0.2                # 拾音器位置：P = 0.2 个周期，第 5、10 … 次谐波处有凹陷
# ==============================================================================


def allpass_delay(c, w):
    """一阶全通 (c + z^-1) / (1 + c z^-1) 在角频率 w 处的相位延迟（采样）。"""
    z1 = cmath.exp(-1j * w)
    return -cmath.phase((c + z1) / (1 + c * z1)) / w


def solve_c(d, w):
    """求全通系数 c，使它在基音频率处的相位延迟正好为 d（相位延迟随 c 单调减小，用二分法）。"""
    lo, hi = -0.9, 0.9
    for _ in range(60):
        mid = (lo + hi) / 2
        if allpass_delay(mid, w) > d:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def gain_for(dbps, f0, w):
    """每走一圈的环路增益 g：两点平均本身每圈已经衰减 cos(w/2)，g 只补不足的部分。"""
    avg_db = -20 * math.log10(math.cos(w / 2))
    need_db = dbps / f0 - avg_db
    g = 10 ** (-max(0.0, need_db) / 20)
    return min(65535, round(g * 65536)), avg_db * f0


def build():
    rows = []
    for k in range(NOTE_COUNT):
        name = midi_name(NOTE_MIDI0 + k)
        f0 = midi_hz(NOTE_MIDI0 + k)
        w = 2 * math.pi * f0 / FS
        D = FS / f0                       # 环路总延迟（采样）
        M = math.floor(D - 1.0)           # 延迟线整数延迟，使 d 落在 0.5 ~ 1.5
        d = D - 0.5 - M
        c = solve_c(d, w)
        cq = round(c * 32768)
        P = round(PICKUP * D)
        if M + 2 > DEPTH or P < 2 or P >= M:
            raise SystemExit(f"{name}：M = {M}、P = {P} 超出范围（延迟线深度 {DEPTH}）")
        gs, avg_dbps = gain_for(SUSTAIN_DBPS, f0, w)
        gr, _ = gain_for(RELEASE_DBPS, f0, w)
        # 量化后的实际音高（相位延迟按量化后的 c 重新算）
        f_real = FS / (M + 0.5 + allpass_delay(cq / 32768, w))
        cents = 1200 * math.log2(f_real / f0)
        rows.append(dict(name=name, f0=f0, D=D, M=M, d=d, c=cq, P=P,
                         gs=gs, gr=gr, avg=avg_dbps, cents=cents))
    return rows


def render(rows):
    lines = []
    w = lines.append
    w("`timescale 1ns/1ps")
    w("// =============================================================================")
    w(f"// guitar_params.v —— 电吉他（Karplus-Strong 拨弦模型）参数表：{len(rows)} 个音 "
      f"{rows[0]['name']} ~ {rows[-1]['name']}")
    w("// 由 scripts/gen_guitar_params.py 自动生成。不要手改下面的数字：改音色请改脚本顶部的“音色旋钮”。")
    w("// -----------------------------------------------------------------------------")
    w("//   m     延迟线整数延迟 M（采样）；环路总延迟 = M + 0.5（两点平均）+ d（全通）= Fs / f0")
    w("//   c     一阶全通系数 × 32768（有符号），实现小数延迟 d")
    w("//   p     拾音器抽头：输出 = v[n] - v[n-p]")
    w(f"//   g_sus 按住时每圈环路增益 × 65536（基音衰减 {SUSTAIN_DBPS:g} dB/s）")
    w(f"//   g_rel 抬键后每圈环路增益 × 65536（衰减 {RELEASE_DBPS:g} dB/s）")
    w("//")
    w("// 序号 音名   基音(Hz)   总延迟     M      d       c      P   音高误差(音分)")
    for k, r in enumerate(rows):
        w(f"//  {k:2d}  {r['name']:<4s} {r['f0']:8.2f}  {r['D']:8.3f}  {r['M']:4d}"
          f"  {r['d']:6.3f}  {r['c']:6d}  {r['P']:3d}  {r['cents']:+8.3f}")
    w("// =============================================================================")
    w("module guitar_params (")
    w("    input  wire [5:0]         note,     // 音序号 0 ~ 45（与 piano_params.v 的钢琴相同）")
    w("    output reg  [9:0]         m,        // 延迟线整数延迟")
    w("    output reg  signed [15:0] c,        // 全通系数 × 32768")
    w("    output reg  [9:0]         p,        // 拾音器抽头")
    w("    output reg  [15:0]        g_sus,    // 按住时环路增益 × 65536")
    w("    output reg  [15:0]        g_rel     // 抬键后环路增益 × 65536")
    w(");")
    w("")
    w("    always @(*) begin")
    w("        case (note)")

    def field(name, value, width):
        return f"{name} = {value};".ljust(width)

    for k, r in enumerate(rows):
        cs = f"-16'sd{-r['c']}" if r["c"] < 0 else f"16'sd{r['c']}"
        w(f"        6'd{k:<2d}: begin " + field("m", f"10'd{r['M']}", 13) + field("c", cs, 17)
          + field("p", f"10'd{r['P']}", 12) + field("g_sus", f"16'd{r['gs']}", 20)
          + field("g_rel", f"16'd{r['gr']}", 20) + f"end  // {r['name']}")
    w("        default: begin m = 10'd185; c = 16'sd0; p = 10'd37; g_sus = 16'd65000; g_rel = 16'd60000; end")
    w("        endcase")
    w("    end")
    w("")
    w("endmodule")
    return "\n".join(lines) + "\n"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="生成电吉他参数表 src/guitar/guitar_params.v")
    ap.add_argument("--out", default=os.path.join(here, "..", "src", "guitar", "guitar_params.v"))
    args = ap.parse_args()
    rows = build()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(render(rows))
    print(f"已写入 {os.path.normpath(args.out)}：{len(rows)} 个音")
    for k, r in enumerate(rows):
        print(f"  {k:2d} {r['name']:<4s} f0={r['f0']:8.2f}  M={r['M']:3d}  d={r['d']:.3f}  c={r['c']:6d}"
              f"  P={r['P']:3d}  g_sus={r['gs']}  g_rel={r['gr']}  平均滤波衰减={r['avg']:.2f} dB/s"
              f"  误差={r['cents']:+.3f} 音分")


if __name__ == "__main__":
    main()
