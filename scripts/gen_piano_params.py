#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 src/piano/piano_params.v —— 加法合成音色参数表（供 src/piano/piano_voice.v 使用），含两种音色：
  音色 0 = 钢琴，音色 1 = 大提琴（同一个合成引擎，只是参数不同；大提琴另有颤音，在 piano_voice.v 里）

合成方法（加法合成）：
  一个音 = 若干个正弦“分音”叠加，每个分音各有：频率、音量、起音时间、两段指数衰减。
  钢琴：
    频率 —— 第 n 分音约为基音的 n 倍，钢琴弦有“非谐性”，分音越高越偏高；
    衰减 —— 先快（即发音）后慢（余音），高分音衰减得更快；
    同度失谐弦 —— 中音区一个键有 3 根弦，彼此略有失谐，产生缓慢“拍音”；
    琴槌敲击声 —— 击弦瞬间十几毫秒的短促“嗒”（低通滤波的噪声）。
  大提琴（弓弦乐器，持续音）：
    频率 —— 拉弓时弦振动被锁定成严格的整数倍谐波（没有非谐性）；
    音量 —— 弓弦本身接近锯齿波（第 n 谐波约 1/n），再乘上琴身共鸣（几个共鸣峰 + 高频滚降），
            所以每个音的谐波强弱都不一样（按 {音, 谐波} 查表）；
    包络 —— 拉弓起音约 90 ms，起音后稍微回落，然后按住多久就响多久（几乎不衰减），抬键后约 1 s 消失；
    弓毛噪声 —— 起音时稍强的“沙沙”声，之后保持很轻的摩擦声。

音：音序号 0 ~ 45，共 46 个半音。钢琴、电吉他：A2 ~ F#6（MIDI 45 ~ 90）；大提琴低一个八度：A1 ~ F#5。
  触摸键盘分三组音区（src/touch/touch_keys.v）：低音 A2 ~ B3、中音 C4 ~ D5（上电默认）、高音 E5 ~ F#6，
  每组 9 个白键 + 6 个黑键。电吉他参数表（gen_guitar_params.py）用同一份音序号。

相位增量（频率）= 基音增量 base × 频率倍数 ratio，在 piano_voice.v 里用乘法器算，
  这样 46 个音只需要存 46 个 base，而不是 46 × 21 个相位增量。
  钢琴的包络参数与音高无关（按振荡器查表）；大提琴的谐波强弱随音高变（按 {音, 振荡器} 查表，放在 BSRAM 里）。

用法（在工程根目录执行）：
    python scripts/gen_piano_params.py
改完后要同时重新生成 gen_guitar_params.py（它读这里的音表），然后在云源里重新综合。
电脑端的录音渲染程序 scripts/bt_recorder.py 也直接调用这里的函数，保证和 FPGA 发出的声音一致。
"""
import argparse
import math
import os

FS = 50_000_000 / 1024    # 采样率 48 828.125 Hz，必须与 i2s_dac_tx 的时序一致（50 MHz / 32 / 32）
ENV_MAX = (1 << 24) - 1   # 包络满幅（piano_voice.v 中包络为 24 位）
ENV_SHIFT = 8         # 衰减一步：包络 -= 包络 >> 8，即乘以 (1 - 1/256)
CNT_MAX = 4095        # 衰减周期计数器为 12 位
STEP_DB = -20 * math.log10(1 - 2 ** -ENV_SHIFT)   # 每一步衰减约 0.0339 dB
NOTE_COUNT = 46       # 音序号 0 ~ 45
NOTE_MIDI0 = 45       # 音序号 0 = MIDI 45 = A2（钢琴、电吉他）
CELLO_SHIFT = -12     # 大提琴比钢琴低一个八度
BASE_BITS = 27        # 基音相位增量位宽
RATIO_FRAC = 20       # 频率倍数 Q5.20
RATIO_BITS = 25
MUTE_HZ = 20000.0     # piano_voice.v 把高于约 20 kHz 的分音静音（防止混叠）

# ============================== 钢琴音色旋钮（可改） ==============================
A4_HZ = 440.0               # 标准音高
INHARMONICITY_B = 0.00035   # 非谐性系数 B（三角钢琴中音区约 0.0002~0.0005，越大越“金属”）
# 击弦瞬间第 1..16 分音相对基音的电平（dB）。第 8、16 分音很弱：
# 琴槌大约敲在弦长 1/8 处，正好是这两个分音的波节。
LEVEL_DB = [0, -1, -4, -6, -8, -10, -13, -24, -16, -18, -21, -23, -26, -29, -32, -42]
ATTACK_MS = 2.0             # 基音起音时间；第 n 分音为 ATTACK_MS / sqrt(n)
DECAY1_DBPS = 12.0          # 第一段（即发音）衰减：基音 12 dB/s，第 n 分音乘以 n^DECAY_EXP
DECAY_EXP = 0.9             # 越大高分音消失得越快：击弦瞬间明亮、随后变柔和
DECAY2_RATIO = 3.5          # 第二段（余音）速率 = 第一段速率 / 3.5
KNEE_DB = 12.0              # 包络比峰值低 12 dB 后，从第一段切到第二段
UNISON_PARTIALS = 4         # 前 4 个分音各加一根同度失谐弦
UNISON_CENTS = 1.6          # 失谐量（音分）；基音拍频约 0.24 Hz
UNISON_SHARE = 0.3          # 失谐弦占该分音音量的比例
HAMMER_LEVEL = 0.12         # 琴槌噪声包络峰值（相对基音）
HAMMER_ATTACK_MS = 0.5
HAMMER_DECAY1_DBPS = 1600.0 # 约 12 ms 衰减 20 dB：短促的“嗒”，不拖成低沉的“轰”
HAMMER_KNEE_DB = 20.0
HAMMER_DECAY2_DBPS = 400.0
HAMMER_LPF_SHIFT = 2        # 噪声一阶低通 y += (x - y) >> 2，截止约 2.2 kHz
RELEASE_DBPS = 120.0        # 抬键后制音器（止音毡）的衰减速率
HEADROOM = 0.9              # 所有振荡器峰值之和 = 0.9 满幅，任何相位组合都不会削顶

# ============================= 大提琴音色旋钮（可改） =============================
CELLO_HARMONICS = 20        # 谐波个数（再加 1 个弓毛噪声 = 21 个振荡器，与钢琴相同）
CELLO_SLOPE = 1.0           # 弓弦频谱：第 n 谐波 -20·log10(n) dB（接近锯齿波）
# 琴身共鸣：(中心频率 Hz, 增益 dB, 宽度 倍频程)。频率固定，所以不同音高的谐波落在不同的共鸣上
CELLO_BODY = [(110, 3.0, 0.40),     # 空腔共鸣
              (210, 5.0, 0.35),     # 面板主共鸣，温暖的“木头”声
              (420, 3.0, 0.35),
              (1100, -3.0, 0.40),   # 中频略凹，鼻音少一点
              (2300, 3.0, 0.55)]    # 琴马共鸣，声音“亮”
CELLO_HICUT = 3000.0        # 高于此频率每倍频程 -12 dB（声音温暖，不刺耳）
CELLO_LOCUT = 80.0          # 低于此频率每倍频程 -12 dB
CELLO_ATTACK_MS = 90.0      # 基音起音（拉弓）时间；第 n 谐波为 CELLO_ATTACK_MS / n^0.3（高次谐波稍快）
CELLO_DECAY1_DBPS = 6.0     # 起音后稍微回落……
CELLO_KNEE_DB = 2.0         # ……回落 2 dB 后进入持续段
CELLO_SUSTAIN_DBPS = 0.4    # 按住时几乎不衰减（持续音）
CELLO_RELEASE_DBPS = 60.0   # 抬键（弓离开弦）后的衰减速率
CELLO_NOISE_DB = -24.0      # 弓毛噪声峰值（相对基音）
CELLO_NOISE_ATTACK_MS = 8.0
CELLO_NOISE_DECAY1_DBPS = 60.0   # 起音的“沙”很快变轻……
CELLO_NOISE_KNEE_DB = 12.0       # ……降 12 dB 后保持
CELLO_NOISE_LPF_SHIFT = 1   # 噪声一阶低通 >>1，截止约 5 kHz（比钢琴琴槌声亮）
CELLO_HEADROOM = 0.9
# ==============================================================================

NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
PATCH_NAMES = ["钢琴", "大提琴"]


def midi_hz(m):
    return A4_HZ * 2 ** ((m - 69) / 12)


def midi_name(m):
    return f"{NAMES[m % 12]}{m // 12 - 1}"


def pad_cjk(text, width, right=False):
    """按等宽字体的显示宽度补空格（一个汉字占两列），让注释表格对齐。"""
    fill = " " * max(0, width - sum(2 if ord(c) > 0x2E80 else 1 for c in text))
    return fill + text if right else text + fill


def period_for(dbps):
    """衰减速率（dB/s）-> 每隔多少个采样走一步；同时返回量化后的实际速率。"""
    m = min(CNT_MAX, max(1, round(STEP_DB * FS / dbps)))
    return m, STEP_DB * FS / m


def finalize(entries, headroom):
    """把电平、时间换算成 piano_voice.v 用的整数参数；所有振荡器峰值之和 = headroom 满幅。"""
    scale = headroom / sum(e["amp"] for e in entries)
    for e in entries:
        e["ratio_q"] = round(e["ratio"] * 2 ** RATIO_FRAC)
        e["peak"] = round(e["amp"] * scale * ENV_MAX)
        e["att"] = max(1, math.ceil(e["peak"] / (e["att_ms"] / 1000 * FS)))
        e["knee"] = round(e["peak"] * 10 ** (-e["knee_db"] / 20))
        e["m1"], e["d1q"] = period_for(e["d1"])
        e["m2"], e["d2q"] = period_for(e["d2"])
    return entries


def piano_entries():
    """钢琴：与音高无关的全部振荡器参数（频率用相对基音的倍数表示）。"""
    def ratio(n):
        return n * math.sqrt((1 + INHARMONICITY_B * n * n) / (1 + INHARMONICITY_B))

    entries = []
    for n, level_db in enumerate(LEVEL_DB, start=1):
        d1 = DECAY1_DBPS * n ** DECAY_EXP
        amp = 10 ** (level_db / 20)
        share = 1 - UNISON_SHARE if n <= UNISON_PARTIALS else 1.0
        entries.append(dict(kind=f"第{n}分音", ratio=ratio(n), amp=amp * share, level_db=level_db,
                            att_ms=ATTACK_MS / math.sqrt(n), d1=d1, d2=d1 / DECAY2_RATIO,
                            knee_db=KNEE_DB, noise=0))
    for n in range(1, UNISON_PARTIALS + 1):
        main = entries[n - 1]
        entries.append(dict(main, kind=f"第{n}分音失谐弦", ratio=main["ratio"] * 2 ** (UNISON_CENTS / 1200),
                            amp=10 ** (LEVEL_DB[n - 1] / 20) * UNISON_SHARE))
    entries.append(dict(kind="琴槌噪声", ratio=0.0, amp=HAMMER_LEVEL, level_db=20 * math.log10(HAMMER_LEVEL),
                        att_ms=HAMMER_ATTACK_MS, d1=HAMMER_DECAY1_DBPS, d2=HAMMER_DECAY2_DBPS,
                        knee_db=HAMMER_KNEE_DB, noise=1))
    return finalize(entries, HEADROOM)


def cello_body_db(f):
    db = 0.0
    for fc, g, w in CELLO_BODY:
        x = math.log2(f / fc) / w
        db += g * math.exp(-0.5 * x * x)
    if f > CELLO_HICUT:
        db -= 12 * math.log2(f / CELLO_HICUT)
    if f < CELLO_LOCUT:
        db -= 12 * math.log2(CELLO_LOCUT / f)
    return db


def cello_entries(f0):
    """大提琴：基音为 f0 的一个音的全部振荡器参数（谐波强弱随音高变，见 CELLO_BODY）。"""
    entries = []
    for n in range(1, CELLO_HARMONICS + 1):
        level_db = -20 * math.log10(n) * CELLO_SLOPE + cello_body_db(n * f0) - cello_body_db(f0)
        entries.append(dict(kind=f"第{n}谐波", ratio=float(n), amp=10 ** (level_db / 20), level_db=level_db,
                            att_ms=CELLO_ATTACK_MS / n ** 0.3, d1=CELLO_DECAY1_DBPS, d2=CELLO_SUSTAIN_DBPS,
                            knee_db=CELLO_KNEE_DB, noise=0))
    entries.append(dict(kind="弓毛噪声", ratio=0.0, amp=10 ** (CELLO_NOISE_DB / 20), level_db=CELLO_NOISE_DB,
                        att_ms=CELLO_NOISE_ATTACK_MS, d1=CELLO_NOISE_DECAY1_DBPS, d2=CELLO_SUSTAIN_DBPS,
                        knee_db=CELLO_NOISE_KNEE_DB, noise=1))
    return finalize(entries, CELLO_HEADROOM)


def base_inc(f0):
    v = round(f0 * 2 ** 32 / FS)
    if v >= 1 << BASE_BITS:
        raise SystemExit(f"基音 {f0:.1f} Hz 的相位增量超过 {BASE_BITS} 位")
    return v


def build():
    """返回 (钢琴 MIDI 音号表, 大提琴 MIDI 音号表, 钢琴振荡器参数, 大提琴每个音的振荡器参数)。"""
    piano = piano_entries()
    pmidi = [NOTE_MIDI0 + k for k in range(NOTE_COUNT)]
    cmidi = [m + CELLO_SHIFT for m in pmidi]
    cello = [cello_entries(midi_hz(m)) for m in cmidi]
    if len(piano) != len(cello[0]):
        raise SystemExit("两种音色的振荡器个数必须相同")
    if len(piano) > 32:
        raise SystemExit("振荡器数量超过 32（piano_voice.v 的上限）")
    for e in piano + cello[0]:
        if e["ratio_q"] >= 1 << RATIO_BITS:
            raise SystemExit(f"{e['kind']} 的频率倍数超出 Q5.{RATIO_FRAC} 范围")
    return pmidi, cmidi, piano, cello


def render(pmidi, cmidi, piano, cello):
    lines = []
    w = lines.append
    w("`timescale 1ns/1ps")
    w("// =============================================================================")
    w(f"// piano_params.v —— 加法合成音色参数表：音色 0 = 钢琴（{midi_name(pmidi[0])} ~ {midi_name(pmidi[-1])}），"
      f"音色 1 = 大提琴（{midi_name(cmidi[0])} ~ {midi_name(cmidi[-1])}）")
    w("// 由 scripts/gen_piano_params.py 自动生成。不要手改下面的数字：改音色请改脚本顶部的“音色旋钮”，")
    w("// 然后重新生成。")
    w("// -----------------------------------------------------------------------------")
    w(f"// 音序号 note = 0 ~ {NOTE_COUNT - 1}：钢琴 MIDI {pmidi[0]} + note，大提琴 MIDI {cmidi[0]} + note（低一个八度）")
    w("// 每个振荡器（一个正弦分音）的参数，piano_voice.v 每个采样依次计算全部振荡器再相加：")
    w(f"//   base  基音相位增量 = round(基音频率 × 2^32 / {FS:g})，按 {{音色, 音}} 查表")
    w(f"//   ratio 频率倍数（Q5.{RATIO_FRAC}，噪声为 0），按 {{音色, 振荡器}} 查表；相位增量 = base × ratio")
    w("//   peak  包络峰值（24 位，满幅 16777215）；每个音所有振荡器的 peak 之和 = 0.9 满幅，不会削顶")
    w("//   att   起音：每个采样包络加 att，直到 peak（线性上升）")
    w("//   m1    衰减 1：每 m1 个采样，包络 ×(1-1/256)，即 -0.0339 dB")
    w("//   knee  包络低于 knee 后切换到衰减 2")
    w("//   m2    衰减 2：同 m1")
    w("//   抬键（note_off）后改用 m_rel 衰减")
    w("// peak、att、knee：钢琴按振荡器查表（与音高无关）；大提琴按 {音, 振荡器} 查表（随音高变，放在 BSRAM 里）")
    w("// 除 m_rel、noise_lpf 外，所有输出都晚 1 拍（输出寄存）。")
    mid = NOTE_COUNT // 2
    for p, (pname, ent, rel, lpf) in enumerate([("钢琴", piano, RELEASE_DBPS, HAMMER_LPF_SHIFT),
                                                 ("大提琴", cello[mid], CELLO_RELEASE_DBPS, CELLO_NOISE_LPF_SHIFT)]):
        _, rel_q = period_for(rel)
        w("//")
        note_txt = "" if p == 0 else f"（电平是音序号 {mid} = {midi_name(cmidi[mid])} 的）"
        w(f"// 音色 {p}：{pname}（抬键 {rel_q:.1f} dB/s，噪声低通 >>{lpf}）{note_txt}")
        w("//  序号 振荡器        频率倍数  电平(dB)  起音(ms)  衰减1(dB/s)  拐点(dB)  衰减2(dB/s)")
        for i, e in enumerate(ent):
            ratio = "噪声" if e["noise"] else f"{e['ratio']:.4f}"
            w(f"//   {i:2d}  {pad_cjk(e['kind'], 14)}{pad_cjk(ratio, 9, right=True)}  {e['level_db']:7.1f}"
              f"  {e['att_ms']:7.2f}  {e['d1q']:10.1f}  {-e['knee_db']:8.0f}  {e['d2q']:10.1f}")
    w("// =============================================================================")
    w("module piano_params (")
    w("    input  wire        clk,")
    w("    input  wire        patch,      // 音色：0 = 钢琴，1 = 大提琴")
    w("    input  wire [5:0]  note,       // 音序号 0 ~ 45")
    w("    input  wire [4:0]  idx,        // 振荡器序号")
    w(f"    output reg  [{BASE_BITS - 1}:0] base,       // 基音相位增量")
    w(f"    output reg  [{RATIO_BITS - 1}:0] ratio,      // 频率倍数 Q5.{RATIO_FRAC}")
    w("    output wire [23:0] peak,       // 包络峰值")
    w("    output wire [23:0] att,        // 起音每采样增量")
    w("    output wire [23:0] knee,       // 衰减 1 -> 衰减 2 的拐点")
    w("    output reg  [11:0] m1,         // 衰减 1 周期（采样数）")
    w("    output reg  [11:0] m2,         // 衰减 2 周期（采样数）")
    w("    output reg         is_noise,   // 1 = 噪声（不查正弦表）")
    w("    output reg         is_last,    // 1 = 最后一个振荡器")
    w("    output wire [11:0] m_rel,      // 抬键后的衰减周期（组合输出）")
    w("    output wire [3:0]  noise_lpf   // 噪声一阶低通的右移位数（组合输出）")
    w(");")
    w("")
    rel0, rq0 = period_for(RELEASE_DBPS)
    rel1, rq1 = period_for(CELLO_RELEASE_DBPS)
    w(f"    assign m_rel     = patch ? 12'd{rel1} : 12'd{rel0};   // 大提琴 {rq1:.1f} dB/s，钢琴 {rq0:.1f} dB/s")
    w(f"    assign noise_lpf = patch ? 4'd{CELLO_NOISE_LPF_SHIFT} : 4'd{HAMMER_LPF_SHIFT};")
    w("")
    w("    reg        patch_q;")
    w("    reg [71:0] pia_q;              // 钢琴 {peak, att, knee}")
    w("    reg [71:0] cel_q;              // 大提琴 {peak, att, knee}")
    w("    assign {peak, att, knee} = patch_q ? cel_q : pia_q;")
    w("")
    w("    always @(posedge clk) begin")
    w("        patch_q <= patch;")
    w("")
    w("        // 基音相位增量：按 {音色, 音序号} 查表")
    w("        case ({patch, note})")
    for p, mids in enumerate([pmidi, cmidi]):
        for k, m in enumerate(mids):
            f0 = midi_hz(m)
            v = base_inc(f0)
            w(f"        {{1'b{p}, 6'd{k:<2d}}}: base <= {BASE_BITS}'d{v};{' ' * (11 - len(str(v)))}"
              f"// {PATCH_NAMES[p]} {midi_name(m):<4s}{f0:8.2f} Hz")
    w(f"        default: base <= {BASE_BITS}'d0;")
    w("        endcase")
    w("")
    w("        // 频率倍数、衰减周期、噪声标志：按 {音色, 振荡器序号} 查表")
    w("        case ({patch, idx})")
    for p, ent in enumerate([piano, cello[0]]):
        last = len(ent) - 1
        for i, e in enumerate(ent):
            w(f"        {{1'b{p}, 5'd{i:<2d}}}: begin ratio <= {RATIO_BITS}'d{e['ratio_q']};"
              f"{' ' * (9 - len(str(e['ratio_q'])))}m1 <= 12'd{e['m1']};{' ' * (5 - len(str(e['m1'])))}"
              f"m2 <= 12'd{e['m2']};{' ' * (5 - len(str(e['m2'])))}is_noise <= 1'b{e['noise']}; "
              f"is_last <= 1'b{int(i == last)}; end  // {PATCH_NAMES[p]} {e['kind']}")
    w(f"        default: begin ratio <= {RATIO_BITS}'d0; m1 <= 12'd1; m2 <= 12'd1; is_noise <= 1'b0; "
      "is_last <= 1'b1; end")
    w("        endcase")
    w("")
    w("        // 钢琴 {peak, att, knee}：按振荡器序号查表（与音高无关）")
    w("        case (idx)")
    for i, e in enumerate(piano):
        w(f"        5'd{i:<2d}: pia_q <= {{24'd{e['peak']}, 24'd{e['att']}, 24'd{e['knee']}}};  // {e['kind']}")
    w("        default: pia_q <= 72'd0;")
    w("        endcase")
    w("    end")
    w("")
    w("    // 大提琴 {peak, att, knee}：按 {音序号, 振荡器序号} 查表（同步只读，综合成 BSRAM）")
    w("    always @(posedge clk) begin")
    w("        case ({note, idx})")
    for k, ent in enumerate(cello):
        w(f"        // 音 {k}：{midi_name(cmidi[k])}  {midi_hz(cmidi[k]):.2f} Hz")
        for i, e in enumerate(ent):
            w(f"        {{6'd{k:<2d}, 5'd{i:<2d}}}: cel_q <= {{24'd{e['peak']}, 24'd{e['att']}, 24'd{e['knee']}}};")
    w("        default: cel_q <= 72'd0;")
    w("        endcase")
    w("    end")
    w("")
    w("endmodule")
    return "\n".join(lines) + "\n"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="生成音色参数表 src/piano/piano_params.v（钢琴 + 大提琴）")
    ap.add_argument("--out", default=os.path.join(here, "..", "src", "piano", "piano_params.v"))
    args = ap.parse_args()

    pmidi, cmidi, piano, cello = build()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(render(pmidi, cmidi, piano, cello))
    print(f"已写入 {os.path.normpath(args.out)}")
    print(f"  钢琴：{midi_name(pmidi[0])} ~ {midi_name(pmidi[-1])}，大提琴：{midi_name(cmidi[0])} ~ "
          f"{midi_name(cmidi[-1])}，每个音 {len(piano)} 个振荡器")
    for k in (0, NOTE_COUNT // 2, NOTE_COUNT - 1):
        f0 = midi_hz(pmidi[k])
        muted = sum(1 for e in piano if not e["noise"] and e["ratio"] * f0 >= MUTE_HZ)
        lv = " ".join(f"{e['level_db']:5.1f}" for e in cello[k][:8])
        print(f"  音 {k:2d}：钢琴 {midi_name(pmidi[k]):<4s}（{muted} 个分音高于 20 kHz，硬件里静音）"
              f"  大提琴 {midi_name(cmidi[k]):<4s} 前 8 个谐波电平(dB): {lv}")


if __name__ == "__main__":
    main()
