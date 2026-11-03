#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_figures.py —— 生成技术文档和 README 用的框图、流程图和数据图（输出到 docs/images/）

框图、流程图按工程源码画；数据图（包络曲线、分音电平、示范录音的波形与频谱）直接调用
scripts/ 下的参数生成脚本和电脑端渲染程序计算，和 FPGA 用的参数完全一致。

用法（在工程根目录执行）：
    python docs/gen_figures.py
需要 matplotlib、numpy、scipy、Pillow。示范录音图需要先运行一次 python scripts/bt_recorder.py --demo。
"""
import os
import sys
import wave

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.patches import FancyBboxPatch, Polygon, Ellipse   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
OUT = os.path.join(HERE, "images")
sys.path.insert(0, os.path.join(ROOT, "scripts"))

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
DPI = 220
FS = 6.8                    # 默认字号（磅）；图按 14.5 cm 宽画，插入文档时不放大，字号就是实际大小
W = 14.5

C_IN = ("#FFF4E5", "#C05621")     # 输入设备
C_OUT = ("#E6FFFA", "#2C7A7B")    # 输出设备
C_LOG = ("#EBF4FF", "#2B6CB0")    # FPGA 逻辑
C_SYN = ("#F0FFF4", "#2F855A")    # 合成
C_FX = ("#FAF5FF", "#6B46C1")     # 效果器
C_PC = ("#F7FAFC", "#4A5568")     # 电脑 / 其他
C_GEN = ("#FFFBEB", "#B7791F")    # 脚本生成
INK = "#222222"


# ================================ 画图小工具 ================================
def canvas(w, h):
    fig = plt.figure(figsize=(w / 2.54, h / 2.54), dpi=DPI)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.axis("off")
    return fig, ax


def box(ax, x, y, w, h, text, col=C_LOG, fs=FS, bold=False, ls="-", lw=0.9, ha="center", rs=0.12):
    fc, ec = col
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={rs}",
                                fc=fc, ec=ec, lw=lw, ls=ls, zorder=2))
    tx = x + w / 2 if ha == "center" else x + 0.15
    ax.text(tx, y + h / 2, text, ha=ha, va="center", fontsize=fs, color=INK,
            fontweight="bold" if bold else "normal", linespacing=1.35, zorder=3)
    return dict(l=x, r=x + w, b=y, t=y + h, cx=x + w / 2, cy=y + h / 2)


def group(ax, x, y, w, h, title, col=C_PC, fs=FS):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.18",
                                fc="none", ec=col[1], lw=1.0, ls=(0, (5, 2.5)), zorder=1))
    if title:
        ax.text(x + 0.15, y + h - 0.1, title, ha="left", va="top", fontsize=fs, color=col[1],
                fontweight="bold", zorder=3)


def diamond(ax, cx, cy, w, h, text, col=C_LOG, fs=FS):
    fc, ec = col
    ax.add_patch(Polygon([(cx - w / 2, cy), (cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2)],
                         closed=True, fc=fc, ec=ec, lw=0.9, zorder=2))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=INK, zorder=3, linespacing=1.3)
    return dict(l=cx - w / 2, r=cx + w / 2, b=cy - h / 2, t=cy + h / 2, cx=cx, cy=cy)


def oval(ax, cx, cy, w, h, text, col=C_PC, fs=FS):
    ax.add_patch(Ellipse((cx, cy), w, h, fc=col[0], ec=col[1], lw=0.9, zorder=2))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=INK, zorder=3)
    return dict(l=cx - w / 2, r=cx + w / 2, b=cy - h / 2, t=cy + h / 2, cx=cx, cy=cy)


def path(ax, pts, color=INK, lw=0.9, ls="-", arrow=True, both=False, z=4):
    """折线；最后一段带箭头（both=True 时第一段也带箭头）。"""
    pts = list(pts)
    if len(pts) > 2:
        xs, ys = zip(*pts[:-1])
        ax.plot(xs, ys, color=color, lw=lw, ls=ls, solid_capstyle="butt", zorder=z)
    style = ("<|-|>" if both and len(pts) == 2 else "-|>") if arrow else "-"
    ax.annotate("", xy=pts[-1], xytext=pts[-2],
                arrowprops=dict(arrowstyle=style, color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0,
                                mutation_scale=7), zorder=z)
    if both and len(pts) > 2:
        ax.annotate("", xy=pts[0], xytext=pts[1],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, shrinkA=0, shrinkB=0,
                                    mutation_scale=7), zorder=z)


def label(ax, x, y, text, fs=6.5, ha="left", va="bottom", color="#444444", bold=False):
    ax.text(x, y, text, fontsize=fs, ha=ha, va=va, color=color, zorder=5, linespacing=1.25,
            fontweight="bold" if bold else "normal")


def dot(ax, x, y, color=INK):
    ax.plot([x], [y], "o", ms=2.6, color=color, zorder=6)


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=DPI, facecolor="white", bbox_inches="tight", pad_inches=0.03)
    plt.close(fig)
    print("已生成", os.path.relpath(p, ROOT))


# ================================ 图 1-1 设计流程 ================================
def fig_design_flow():
    fig, ax = canvas(W, 4.7)
    bw, bh, gap, x0 = 3.05, 1.15, 0.55, 0.3
    xs = [x0 + i * (bw + gap) for i in range(4)]
    top = [("① 需求分析\n方案论证", C_PC), ("② 声学 / 算法建模\n参数与 ROM 生成（Python）", C_GEN),
           ("③ Verilog RTL\n模块化设计", C_LOG), ("④ 静态检查（Verilator）\n模型对照验证（Python）", C_LOG)]
    bot = [("⑤ 综合 / 布局布线\n时序分析（云源软件）", C_LOG), ("⑥ 上板调试\n听感 / LED / OLED", C_OUT),
           ("⑦ 迭代优化\n时序 / 资源 / 交互", C_SYN), ("⑧ 定型\n比特流 + 电脑端程序", C_PC)]
    yt, yb = 3.2, 0.45
    T = [box(ax, xs[i], yt, bw, bh, t, c) for i, (t, c) in enumerate(top)]
    B = [box(ax, xs[3 - i], yb, bw, bh, t, c) for i, (t, c) in enumerate(bot)]
    for i in range(3):
        path(ax, [(T[i]["r"], T[i]["cy"]), (T[i + 1]["l"], T[i + 1]["cy"])])
    path(ax, [(T[3]["cx"], T[3]["b"]), (B[0]["cx"], B[0]["t"])])
    for i in range(3):
        path(ax, [(B[i]["l"], B[i]["cy"]), (B[i + 1]["r"], B[i + 1]["cy"])])
    # ⑦ → ③ 回到 RTL 修改
    path(ax, [(B[2]["cx"] + 0.6, B[2]["t"]), (B[2]["cx"] + 0.6, 2.55), (T[2]["cx"], 2.55), (T[2]["cx"], T[2]["b"])],
         ls=(0, (3, 2)), color="#2F855A")
    label(ax, B[2]["cx"] + 0.75, 2.6, "定位问题后修改 RTL / 参数", color="#2F855A")
    save(fig, "fig1_design_flow.png")


# ================================ 图 2-1 系统整体框图 ================================
def fig_system():
    H = 11.0
    fig, ax = canvas(W, H)
    group(ax, 3.15, 0.2, 8.45, 10.6, "FPGA（GW5AST-LV138，Tang Mega 138K Pro）· 50 MHz 单时钟域", C_LOG, fs=7)
    # 左侧：输入设备、蓝牙、电脑
    kb = box(ax, 0.1, 8.9, 2.75, 1.35, "16 键电容触摸键盘\nTTP229（PMOD2）\nSCL / SDO", C_IN)
    btn = box(ax, 0.1, 7.75, 2.75, 0.75, "板载按键 key0~key3", C_IN)
    led = box(ax, 0.1, 6.75, 2.75, 0.75, "6 个板载 LED", C_OUT)
    hc = box(ax, 0.1, 3.5, 2.75, 1.1, "HC-05 蓝牙模块\n（PMOD0，UART）", C_OUT)
    pc = box(ax, 0.1, 0.9, 2.75, 1.7, "笔记本电脑\nbt_recorder.py\n→ WAV / MIDI", C_PC)
    # FPGA 左列
    ttp = box(ax, 3.35, 9.1, 1.5, 1.0, "触摸读取\nttp229_rd")
    tk = box(ax, 5.05, 9.1, 1.5, 1.0, "键位 / 音区\ntouch_keys")
    ctl = box(ax, 3.35, 6.75, 3.2, 1.75, "按键消抖 + 控制寄存器\n音色、效果器、音量、录音\nLED 指示")
    rec = box(ax, 3.35, 4.95, 3.2, 1.15, "录音 recorder\n事件 + 毫秒时标\n最多 16384 条")
    ua = box(ax, 3.35, 3.65, 3.2, 0.8, "UART 收发  9600 bps")
    # FPGA 右列：音频通路
    va = box(ax, 6.95, 9.1, 4.45, 1.0, "复音分配 voice_alloc（8 声部）\n同音重弹 / 最早释放优先")
    group(ax, 6.95, 6.6, 4.45, 2.15, "8 个并行声部", C_SYN)
    add = box(ax, 7.1, 6.72, 2.05, 1.55, "加法合成引擎\n钢琴 / 大提琴\n21 个振荡器", C_SYN)
    ks = box(ax, 9.3, 6.72, 1.95, 1.55, "Karplus-Strong\n物理模型\n电吉他", C_SYN)
    mix = box(ax, 6.95, 5.35, 4.45, 0.85, "混音（力度增益）\n电吉他：过载失真 + 音箱模拟", C_SYN)
    fx = box(ax, 6.95, 4.1, 4.45, 0.85, "效果器：合唱 → 延迟 → 混响\n（立体声）", C_FX)
    i2s = box(ax, 6.95, 2.95, 4.45, 0.75, "音量 → I2S 发送（16 位，48.8 kHz）")
    sp = box(ax, 6.95, 1.75, 4.45, 0.75, "频谱分析：1024 点 FFT → 32 频带")
    od = box(ax, 6.95, 0.45, 4.45, 0.85, "OLED 画面生成（字库 + 频谱）\nI2C 控制（250 kHz）")
    # 右侧：输出设备
    dac = box(ax, 11.85, 2.6, 2.6, 1.45, "PT8211 DAC\n→ 低通 → 功放\n→ 3.5 mm → 音箱", C_OUT)
    ol = box(ax, 11.85, 0.35, 2.6, 1.05, "0.96 寸 OLED\n128×64（PMOD1）", C_OUT)
    # 连线
    path(ax, [(kb["r"], 9.6), (ttp["l"], 9.6)])
    path(ax, [(ttp["r"], 9.6), (tk["l"], 9.6)])
    path(ax, [(tk["r"], 9.6), (va["l"], 9.6)])
    dot(ax, 6.75, 9.6)
    path(ax, [(6.75, 9.6), (6.75, 5.525), (rec["r"], 5.525)])
    label(ax, 6.72, 6.25, "琴键事件", ha="right", fs=6)
    path(ax, [(va["cx"], va["b"]), (va["cx"], 8.75)])
    path(ax, [(btn["r"], btn["cy"]), (ctl["l"], btn["cy"])])
    path(ax, [(ctl["l"], led["cy"]), (led["r"], led["cy"])])
    path(ax, [(ctl["r"], 7.75), (6.95, 7.75)], ls=(0, (3, 2)), color="#2B6CB0")
    label(ax, 6.75, 7.86, "音色", fs=5.6, color="#2B6CB0", ha="center")
    path(ax, [(4.15, ctl["b"]), (4.15, rec["t"])])
    label(ax, 4.25, 6.25, "录音键", fs=6)
    path(ax, [(4.95, rec["b"]), (4.95, ua["t"])])
    path(ax, [(hc["r"], ua["cy"]), (ua["l"], ua["cy"])], both=True)
    path(ax, [(1.475, hc["b"]), (1.475, pc["t"])], both=True, ls=(0, (3, 2)))
    label(ax, 1.6, 2.85, "蓝牙串口", fs=6)
    path(ax, [(va["cx"], 6.6), (va["cx"], mix["t"])])
    path(ax, [(va["cx"], mix["b"]), (va["cx"], fx["t"])])
    path(ax, [(va["cx"], fx["b"]), (va["cx"], i2s["t"])])
    path(ax, [(fx["l"], fx["cy"]), (6.8, fx["cy"]), (6.8, sp["cy"]), (sp["l"], sp["cy"])])
    path(ax, [(va["cx"], sp["b"]), (va["cx"], od["t"])])
    path(ax, [(i2s["r"], dac["cy"]), (dac["l"], dac["cy"])])
    path(ax, [(od["r"], ol["cy"]), (ol["l"], ol["cy"])])
    label(ax, 11.62, dac["cy"] + 0.05, "I2S", fs=6)
    label(ax, 11.62, ol["cy"] + 0.05, "I2C", fs=6)
    save(fig, "fig2_system.png")


# ================================ 图 2-2 硬件连接 ================================
def fig_hardware():
    H = 8.6
    fig, ax = canvas(W, H)
    group(ax, 3.2, 0.25, 8.1, 6.05, "Tang Mega 138K Pro 核心板 + Dock 底板", C_PC)
    p0 = box(ax, 3.55, 5.15, 2.1, 0.62, "PMOD0（J22）", C_PC)
    p1 = box(ax, 6.2, 5.15, 2.1, 0.62, "PMOD1（J24）", C_PC)
    p2 = box(ax, 8.85, 5.15, 2.1, 0.62, "PMOD2（J26）", C_PC)
    fp = box(ax, 3.75, 2.75, 7.0, 1.55, "FPGA  GW5AST-LV138FPG676AC1/I0\n50 MHz 晶振（P16）", C_LOG)
    kb = box(ax, 3.45, 0.5, 2.45, 1.2, "按键 S0~S3\nK16 G15 F15 G16", C_IN)
    ld = box(ax, 6.2, 0.5, 2.0, 1.2, "LED0~5\n低电平点亮", C_OUT)
    au = box(ax, 8.5, 0.5, 2.6, 1.6, "音频电路\nPT8211 → LMV321\n→ XPT4809 → J29", C_OUT)
    hc = box(ax, 3.55, 7.05, 2.1, 1.3, "HC-05 蓝牙\nVCC GND\nRXD TXD", C_OUT)
    ol = box(ax, 6.2, 7.05, 2.1, 1.3, "0.96 寸 OLED\nVCC GND\nSCL SDA", C_OUT)
    tp = box(ax, 8.85, 7.05, 2.1, 1.3, "触摸键盘\nVCC GND\nSCL SDO", C_IN)
    for m, p, t in ((hc, p0, "UART"), (ol, p1, "I2C"), (tp, p2, "两线串行")):
        path(ax, [(m["cx"], m["b"]), (p["cx"], p["t"])], both=True)
        label(ax, m["cx"] + 0.1, 6.45, t, fs=6)
    for p in (p0, p1, p2):
        path(ax, [(p["cx"], p["b"]), (p["cx"], fp["t"])], both=True, lw=0.8)
    path(ax, [(kb["cx"], kb["t"]), (kb["cx"], fp["b"])])
    path(ax, [(ld["cx"], fp["b"]), (ld["cx"], ld["t"])])
    path(ax, [(9.8, fp["b"]), (9.8, au["t"])])
    label(ax, 9.9, 2.25, "I2S + PA_EN", fs=5.8)
    sk = box(ax, 11.85, 0.7, 2.5, 1.2, "有源音箱", C_OUT)
    path(ax, [(au["r"], sk["cy"]), (sk["l"], sk["cy"])])
    label(ax, 11.48, sk["cy"] + 0.08, "3.5 mm", fs=5.8, ha="center")
    pc = box(ax, 0.15, 3.0, 2.6, 1.8, "笔记本电脑\n云源下载程序\n蓝牙接收录音", C_PC)
    path(ax, [(pc["r"], 3.55), (3.2, 3.55)], both=True)
    label(ax, 2.98, 3.62, "USB", fs=5.8, ha="center")
    path(ax, [(1.45, pc["t"]), (1.45, 7.7), (hc["l"], 7.7)], both=True, ls=(0, (3, 2)))
    label(ax, 1.55, 7.76, "蓝牙（SPP 串口）", fs=6)
    label(ax, 11.5, 4.3, "三个 PMOD 口接法相同：\n第 1 列 3V3\n第 2 列 GND\n第 3 列 FPGA 输出\n第 4 列 FPGA 输入", fs=6, va="bottom")
    save(fig, "fig2_hardware.png")


# ================================ 图 2-3 音频输出电路 ================================
def fig_audio():
    H = 3.6
    fig, ax = canvas(W, H)
    y, h = 1.35, 1.15
    f = box(ax, 0.1, y, 2.7, h, "FPGA（i2s_dac_tx）\nBCK:E22  WS:F24\nDIN:G24", C_LOG, fs=6.5)
    d = box(ax, 3.3, y, 2.15, h, "U6 PT8211\n16 位立体声 DAC", C_OUT)
    l = box(ax, 5.95, y, 2.0, h, "LMV321\n低通滤波 / 缓冲", C_OUT)
    x = box(ax, 8.45, y, 2.1, h, "U32 XPT4809\n耳机功放", C_OUT)
    j = box(ax, 11.05, y, 1.5, h, "J29\n3.5 mm", C_OUT)
    s = box(ax, 13.05, y, 1.35, h, "有源\n音箱", C_OUT)
    for a, b in ((f, d), (d, l), (l, x), (x, j), (j, s)):
        path(ax, [(a["r"], a["cy"]), (b["l"], b["cy"])])
    path(ax, [(1.45, y), (1.45, 0.55), (x["cx"], 0.55), (x["cx"], y)])
    label(ax, 2.0, 0.6, "PA_EN（H17）经 R248 接 EN：输出 1 = 功放工作（常开）", fs=6.3)
    label(ax, 0.1, 2.95, "串行音频格式：每帧 32 个 BCK，左右声道各 16 位，MSB 先发；"
                         "BCK = 50 MHz ÷ 32 = 1.5625 MHz，采样率 = 48.828 kHz", fs=6.3)
    save(fig, "fig2_audio.png")


# ================================ 图 2-4 FPGA 模块层次 ================================
def fig_hierarchy():
    rows = [("pll_audio", "时钟使能（4 × BCK）", []),
            ("key_debounce ×4", "按键同步、消抖", []),
            ("ttp229_rd", "触摸键盘读取", []),
            ("touch_keys", "键位映射 / 三组音区", []),
            ("voice_alloc", "复音分配（8 声部）", []),
            ("g_voice[0..7]", "8 个并行声部", [("piano_voice：加法合成", True, "含 piano_params、sine_lut"),
                                               ("guitar_voice：Karplus-Strong 电吉他", True, "含 guitar_params")]),
            ("gtr_amp", "电吉他过载 / 音箱模拟", []),
            ("fx_chorus", "合唱", []),
            ("fx_delay", "乒乓延迟", []),
            ("fx_reverb", "混响（Freeverb）", []),
            ("i2s_dac_tx", "I2S 发送", []),
            ("spectrum", "1024 点 FFT 频谱", [("fft_win / tw / band_rom：窗、旋转因子、频带表", True, "")]),
            ("oled_frame", "OLED 画面生成", [("oled_text_rom ×2、oled_digit_rom：点阵", True, "")]),
            ("oled_i2c", "OLED I2C 控制", []),
            ("recorder", "录音 / 蓝牙协议", [("uart_tx / uart_rx：串口 9600 bps", False, "")])]
    n = sum(max(1, len(c)) for _, _, c in rows)
    rh = 0.56
    H = n * rh + 1.0
    fig, ax = canvas(W, H)
    y_first = H - 0.45
    y_last = y_first - (n - 1) * rh
    root = box(ax, 0.15, (y_first + y_last) / 2 - 0.55, 2.3, 1.1, "top_touch_synth\n顶层", C_LOG, bold=True)
    xb, bw = 2.95, 4.75
    ax.plot([2.7, 2.7], [y_first, y_last], color=INK, lw=0.8, zorder=1)
    ax.plot([root["r"], 2.7], [root["cy"], root["cy"]], color=INK, lw=0.8)
    y = y_first
    for name, desc, kids in rows:
        k = max(1, len(kids))
        cy = y - (k - 1) * rh / 2
        b = box(ax, xb, cy - 0.21, bw, 0.42, f"{name}：{desc}", C_SYN if "voice" in name else C_LOG,
                fs=6.4, ha="left", rs=0.08)
        ax.plot([2.7, xb], [cy, cy], color=INK, lw=0.8, zorder=1)
        for i, (txt, gen, note) in enumerate(kids):
            ky = y - i * rh
            box(ax, 8.1, ky - 0.21, 6.3, 0.42, txt + (f"（{note}）" if note else ""), C_GEN if gen else C_LOG,
                fs=6.2, ha="left", rs=0.08)
            ax.plot([b["r"], 7.9, 7.9, 8.1], [cy, cy, ky, ky], color=INK, lw=0.8, zorder=1)
        y -= k * rh
    ax.add_patch(FancyBboxPatch((8.1, 0.08), 0.4, 0.26, boxstyle="round,pad=0,rounding_size=0.05",
                                fc=C_GEN[0], ec=C_GEN[1], lw=0.8))
    label(ax, 8.6, 0.09, "含 Python 脚本生成的参数表 / ROM", fs=6, va="bottom")
    save(fig, "fig2_hierarchy.png")


# ================================ 图 2-5 加法合成声部流程 + 包络 ================================
def fig_voice():
    import bt_recorder as br
    H = 10.6
    fig, ax = canvas(W, H)
    x0, bw, bh = 0.55, 6.25, 0.6
    steps = [("等待采样节拍 sample_en（每 1024 个时钟一次）", C_PC),
             ("锁存击键 / 抬键、音序号、音色；idx ← 0", C_LOG),
             ("PREP：查参数表，读出第 idx 个振荡器的状态", C_LOG),
             ("MULF：相位增量 = 基音增量 × 频率倍数", C_LOG),
             ("VIB：相位增量 ×（1 + 颤音量）（大提琴）", C_LOG),
             ("FETCH：更新包络状态，写回相位与包络", C_LOG),
             ("LOAD：取正弦表输出（噪声项取低通噪声）", C_LOG),
             ("MUL：16 拍移位相加，acc += 波形 × 包络", C_SYN)]
    ys = [9.75 - i * 0.84 for i in range(len(steps))]
    B = [box(ax, x0, yy, bw, bh, t, c, fs=6.6) for (t, c), yy in zip(steps, ys)]
    for i in range(len(B) - 1):
        path(ax, [(B[i]["cx"], B[i]["b"]), (B[i + 1]["cx"], B[i + 1]["t"])])
    d = diamond(ax, x0 + bw / 2, B[-1]["b"] - 0.75, 4.0, 0.9, "idx 是最后一个振荡器？", fs=6.6)
    path(ax, [(B[-1]["cx"], B[-1]["b"]), (d["cx"], d["t"])])
    out = box(ax, x0, d["b"] - 1.0, bw, bh, "OUT：缩放、限幅，输出 16 位样本", C_SYN, fs=6.6)
    path(ax, [(d["cx"], d["b"]), (out["cx"], out["t"])])
    label(ax, d["cx"] + 0.1, d["b"] - 0.45, "是", fs=6.3)
    path(ax, [(d["l"], d["cy"]), (0.25, d["cy"]), (0.25, B[2]["cy"]), (B[2]["l"], B[2]["cy"])])
    label(ax, 0.3, d["cy"] + 0.05, "否：idx + 1", fs=6.3)
    path(ax, [(out["r"], out["cy"]), (7.05, out["cy"]), (7.05, B[0]["cy"]), (B[0]["r"], B[0]["cy"])])
    label(ax, 0.55, out["b"] - 0.08, "每个振荡器 21 拍；21 个振荡器约 450 个时钟\n（每个采样周期有 1024 个时钟）", fs=6.1, va="top")
    # 包络状态机
    label(ax, 7.55, 10.15, "包络状态机（每个振荡器各一个）", fs=6.8, bold=True, color="#2B6CB0")
    rl = box(ax, 7.55, 8.6, 2.75, 1.0, "REL  释放 / 静止\n指数衰减（m_rel）", fs=6.5)
    at = box(ax, 11.55, 8.6, 2.75, 1.0, "ATT  起音\n线性上升到 peak", fs=6.5)
    d1 = box(ax, 11.55, 6.65, 2.75, 1.0, "DEC1  衰减 1\n指数衰减（m1）", fs=6.5)
    d2 = box(ax, 7.55, 6.65, 2.75, 1.0, "DEC2  衰减 2 / 持续\n指数衰减（m2）", fs=6.5)
    path(ax, [(rl["r"], rl["cy"]), (at["l"], at["cy"])])
    label(ax, 10.45, rl["cy"] + 0.05, "note_on", fs=6.1)
    path(ax, [(at["cx"], at["b"]), (d1["cx"], d1["t"])])
    label(ax, at["cx"] + 0.1, 8.05, "到达 peak", fs=6.1)
    path(ax, [(d1["l"], d1["cy"]), (d2["r"], d2["cy"])])
    label(ax, 10.45, d1["cy"] + 0.05, "低于 knee", fs=6.1)
    path(ax, [(d2["cx"], d2["t"]), (rl["cx"], rl["b"])])
    label(ax, d2["cx"] + 0.1, 8.05, "note_off", fs=6.1)
    label(ax, 7.55, 6.5, "指数衰减：每 m 个采样，包络 × 255/256（−0.034 dB）\n"
                          "任意状态收到 note_on 回到 ATT（相位不清零）；\nATT、DEC1 收到 note_off 也直接进入 REL", fs=6.1, va="top")
    # 实际参数算出的基音包络
    ax2 = fig.add_axes([8.45 / W, 0.75 / H, 5.9 / W, 3.9 / H])
    n = int(3.0 * br.FS)
    t = np.arange(n) / br.FS
    off = int(2.0 * br.FS)
    for e, mrel, lab, c in ((br.PIANO[0], br.MREL[0], "钢琴 C4（第 1 分音）", "#2B6CB0"),
                            (br.CELLO[27][0], br.MREL[1], "大提琴 C4（第 1 谐波）", "#C05621")):
        env = br.env_curve(n, 0.0, e["peak"], e["att"], e["knee"], e["m1"], e["m2"], off, mrel)
        ax2.plot(t, 20 * np.log10(np.maximum(env, 1) / e["peak"]), lw=1.2, color=c, label=lab)
    ax2.axvline(2.0, color="#888", lw=0.8, ls="--")
    ax2.text(2.04, -6, "抬键", fontsize=6, color="#555")
    ax2.set_xlim(0, 3)
    ax2.set_ylim(-60, 3)
    ax2.set_xlabel("时间 / s", fontsize=6.3)
    ax2.set_ylabel("相对峰值 / dB", fontsize=6.3)
    ax2.tick_params(labelsize=5.8)
    ax2.grid(alpha=0.3, lw=0.5)
    ax2.legend(fontsize=5.8, loc="lower left", frameon=False)
    ax2.set_title("按参数表计算的基音包络（0 s 击键，2 s 抬键）", fontsize=6.3)
    save(fig, "fig2_voice.png")


# ================================ 图 2-6 分音 / 谐波电平 ================================
def fig_partials():
    import bt_recorder as br
    import gen_piano_params as gp
    fig = plt.figure(figsize=(W / 2.54, 5.2 / 2.54), dpi=DPI)
    a1 = fig.add_axes([0.07, 0.2, 0.38, 0.66])
    a2 = fig.add_axes([0.57, 0.2, 0.40, 0.66])
    n = np.arange(1, 17)
    a1.bar(n, gp.LEVEL_DB, color="#2B6CB0", width=0.7)
    a1.set_xticks(n)
    a1.set_ylim(-45, 2)
    a1.set_title("钢琴：16 个分音击弦电平（与音高无关）", fontsize=6.8)
    a1.set_xlabel("分音序号", fontsize=6.5)
    a1.set_ylabel("相对基音 / dB", fontsize=6.5)
    a1.text(4.2, -36, "第 8、16 分音位于\n琴槌敲击点的波节", fontsize=6, ha="center", color="#555")
    m = np.arange(1, 21)
    for idx, c, lab in ((15, "#C05621", "大提琴 C3"), (27, "#2F855A", "大提琴 C4"), (39, "#6B46C1", "大提琴 C5")):
        lv = [e["level_db"] for e in br.CELLO[idx][:20]]
        a2.plot(m, lv, "o-", ms=2.5, lw=1.0, color=c, label=lab)
    a2.set_xticks(m[::2])
    a2.set_ylim(-45, 8)
    a2.set_title("大提琴：20 个谐波电平（锯齿波频谱 × 琴身共鸣）", fontsize=6.8)
    a2.set_xlabel("谐波序号", fontsize=6.5)
    a2.legend(fontsize=6, frameon=False, loc="upper right")
    for a in (a1, a2):
        a.tick_params(labelsize=6)
        a.grid(alpha=0.3, lw=0.5)
    save(fig, "fig2_partials.png")


# ================================ 图 2-7 电吉他模型 ================================
def fig_guitar():
    H = 5.6
    fig, ax = canvas(W, H)
    yb, hb = 3.65, 1.0
    ex = box(ax, 0.1, 3.5, 2.3, 1.3, "拨弦激励\n低通白噪声\n（前 M+1 个采样）", C_IN, fs=6.5)
    ax.add_patch(Ellipse((3.05, yb + hb / 2), 0.55, 0.55, fc="white", ec=INK, lw=0.9, zorder=3))
    ax.text(3.05, yb + hb / 2, "v[n]", fontsize=6, ha="center", va="center", zorder=4)
    dl = box(ax, 3.75, yb, 1.9, hb, "延迟线\nz^-M", C_SYN, fs=6.5)
    av = box(ax, 6.2, yb, 2.2, hb, "两点平均\n(x[n]+x[n-1]) / 2", C_SYN, fs=6.5)
    ap = box(ax, 8.95, yb, 2.15, hb, "一阶全通 A(z)\n补偿小数延迟 d", C_SYN, fs=6.5)
    g = box(ax, 11.65, yb, 1.65, hb, "× g\ng_sus / g_rel", C_SYN, fs=6.5)
    cy = yb + hb / 2
    path(ax, [(ex["r"], cy), (2.78, cy)])
    path(ax, [(3.32, cy), (dl["l"], cy)])
    path(ax, [(dl["r"], cy), (av["l"], cy)])
    path(ax, [(av["r"], cy), (ap["l"], cy)])
    path(ax, [(ap["r"], cy), (g["l"], cy)])
    path(ax, [(g["r"], cy), (13.75, cy), (13.75, 2.85), (3.05, 2.85), (3.05, cy - 0.28)])
    label(ax, 6.3, 2.92, "反馈：新样本写回延迟线；环路总延迟 M + 0.5 + d = Fs / f0", fs=6.1)
    label(ax, 0.1, 5.05, "按住用 g_sus（基音约 6 dB/s 衰减）；抬键改用 g_rel（约 150 dB/s，相当于手掌制音）", fs=6.1)
    y2, h2 = 0.4, 1.05
    pk = box(ax, 2.6, y2, 2.4, h2, "拾音器\n(v[n] − v[n−P]) / 2", C_SYN, fs=6.4)
    dot(ax, 3.4, 2.85)
    path(ax, [(3.4, 2.85), (3.4, pk["t"])])
    label(ax, 3.48, 2.05, "v[n]", fs=6)
    path(ax, [(4.6, yb), (4.6, pk["t"])])
    label(ax, 4.68, 2.05, "v[n−P]", fs=6)
    mx = box(ax, 5.45, y2, 1.9, h2, "8 声部求和\n（× 力度）", C_SYN, fs=6.4)
    group(ax, 7.6, 0.15, 6.85, 1.95, "gtr_amp（电吉他音箱）", C_FX, fs=6.2)
    a1 = box(ax, 7.75, y2, 2.4, h2 - 0.2, "×4 → 软削波\n1.5u − 0.5u³", C_FX, fs=6.4)
    a2 = box(ax, 10.55, y2, 1.65, h2 - 0.2, "两级低通\n≈ 4 kHz", C_FX, fs=6.4)
    a3 = box(ax, 12.6, y2, 1.7, h2 - 0.2, "× 0.3\n→ 混音", C_FX, fs=6.4)
    path(ax, [(pk["r"], pk["cy"]), (mx["l"], mx["cy"])])
    path(ax, [(mx["r"], mx["cy"]), (a1["l"], a1["cy"])])
    path(ax, [(a1["r"], a1["cy"]), (a2["l"], a2["cy"])])
    path(ax, [(a2["r"], a2["cy"]), (a3["l"], a3["cy"])])
    save(fig, "fig2_guitar.png")


# ================================ 图 2-8 效果器链 ================================
def fig_fx():
    H = 5.7
    fig, ax = canvas(W, H)
    y, h = 3.3, 1.7
    i0 = box(ax, 0.1, y, 1.7, h, "干声\n单声道\n（18 位）", C_PC, fs=6.5)
    ch = box(ax, 2.3, y, 2.75, h, "合唱 fx_chorus\n调制延迟 8~15 ms\n0.6 Hz 三角波\n左右相差 90°", C_FX, fs=6.5)
    dl = box(ax, 5.55, y, 2.8, h, "延迟 fx_delay\n乒乓回声 250 ms\n左→右 0.7\n右→左 0.65（含低通）", C_FX, fs=6.5)
    rv = box(ax, 8.85, y, 2.85, h, "混响 fx_reverb\nFreeverb 结构\n8 梳状并联 → 4 全通\n反馈 0.88，阻尼 0.25", C_FX, fs=6.5)
    vo = box(ax, 12.2, y, 2.2, h, "音量 4 档\n（右移 0~3 位）\n→ I2S", C_LOG, fs=6.5)
    path(ax, [(i0["r"], i0["cy"]), (ch["l"], ch["cy"])])
    for a, b in ((ch, dl), (dl, rv), (rv, vo)):
        path(ax, [(a["r"], a["cy"] + 0.25), (b["l"], b["cy"] + 0.25)])
        path(ax, [(a["r"], a["cy"] - 0.25), (b["l"], b["cy"] - 0.25)])
        label(ax, a["r"] + 0.05, a["cy"] + 0.3, "L", fs=5.8)
        label(ax, a["r"] + 0.05, a["cy"] - 0.2, "R", fs=5.8)
    label(ax, 0.1, 5.25, "效果器关掉时直通；开关时用 256 个采样（约 5 ms）渐变，关闭后已有的回声、余响自然衰减", fs=6.1)
    group(ax, 1.9, 0.1, 11.0, 2.75, "混响内部：16 条延迟线共用一块 16K × 18 存储器，每个采样依次处理约 90 个时钟", C_FX, fs=6.2)
    inn = box(ax, 2.15, 0.6, 1.45, 0.95, "输入\n(L+R) / 2", C_PC, fs=6.3)
    cb = box(ax, 4.15, 0.35, 3.6, 1.45, "8 个并联梳状滤波器\n延迟 25~37 ms\n反馈支路含阻尼低通", C_FX, fs=6.3)
    ax.add_patch(Ellipse((8.35, 1.07), 0.5, 0.5, fc="white", ec=INK, lw=0.9, zorder=3))
    ax.text(8.35, 1.07, "Σ", fontsize=7, ha="center", va="center", zorder=4)
    al = box(ax, 9.15, 1.35, 2.3, 0.62, "4 级全通（左）", C_FX, fs=6.3)
    ar = box(ax, 9.15, 0.3, 2.3, 0.62, "4 级全通（右）", C_FX, fs=6.3)
    path(ax, [(inn["r"], inn["cy"]), (cb["l"], 1.07)])
    path(ax, [(cb["r"], 1.07), (8.1, 1.07)])
    path(ax, [(8.6, 1.07), (8.9, 1.07), (8.9, al["cy"]), (al["l"], al["cy"])])
    path(ax, [(8.9, 1.07), (8.9, ar["cy"]), (ar["l"], ar["cy"])])
    path(ax, [(al["r"], al["cy"]), (12.2, al["cy"])])
    path(ax, [(ar["r"], ar["cy"]), (12.2, ar["cy"])])
    label(ax, 11.55, al["cy"] + 0.07, "湿声 L", fs=5.8)
    label(ax, 11.55, ar["cy"] + 0.07, "湿声 R", fs=5.8)
    label(ax, 4.15, 2.05, "右声道 4 级全通的延迟各比左声道长 25 个采样（立体声展宽）", fs=5.8, va="bottom")
    path(ax, [(rv["cx"], rv["b"]), (rv["cx"], 2.85)], ls=(0, (2, 2)), color="#6B46C1", arrow=False)
    save(fig, "fig2_fx.png")


# ================================ 图 2-9 频谱分析与显示流程 ================================
def fig_spectrum():
    H = 4.4
    fig, ax = canvas(W, H)
    bw, bh, gap, x0 = 2.55, 1.25, 0.35, 0.15
    xs = [x0 + i * (bw + gap) for i in range(5)]
    r1 = ["混响输出\n左右平均", "1024 点环形缓冲\n（最近 21 ms）", "汉宁窗\n+ 位反转写入", "基 2 FFT\n10 级 × 512 蝶形\n每级 ÷ 2",
          "幅度近似\nmax + 3/8 · min"]
    r2 = ["32 个对数频带\n100 Hz~12 kHz\n每带取最大", "柱高 = 4·log2(幅度)−20\n1.5 dB / 格\n下落 + 峰值保持", "oled_frame\n画面生成",
          "oled_i2c\nI2C 250 kHz", "OLED\n约 25 帧 / 秒"]
    T = [box(ax, xs[i], 2.85, bw, bh, t, C_LOG, fs=6.4) for i, t in enumerate(r1)]
    B = [box(ax, xs[4 - i], 0.35, bw, bh, t, C_OUT if i == 4 else C_LOG, fs=6.2 if i == 1 else 6.4)
         for i, t in enumerate(r2)]
    for i in range(4):
        path(ax, [(T[i]["r"], T[i]["cy"]), (T[i + 1]["l"], T[i + 1]["cy"])])
        path(ax, [(B[i]["l"], B[i]["cy"]), (B[i + 1]["r"], B[i + 1]["cy"])])
    path(ax, [(T[4]["cx"], T[4]["b"]), (B[0]["cx"], B[0]["t"])])
    label(ax, 0.15, 2.25, "一次分析约 4.3 万个时钟（0.9 ms），在每帧 OLED 刷新开始时启动；结果在发送显存期间保持不变，画面不撕裂", fs=6.1)
    save(fig, "fig2_spectrum.png")


# ================================ 图 2-10 OLED 界面布局 ================================
def fig_oled():
    from PIL import Image, ImageFont
    from matplotlib.colors import ListedColormap
    import gen_display_roms as gd
    tmp = os.path.join(OUT, "_oled_tmp.png")
    os.makedirs(OUT, exist_ok=True)
    gd.preview(tmp, ImageFont.truetype(gd.FONT[0], gd.FONT[2], index=gd.FONT[1]))
    img = np.array(Image.open(tmp).convert("L").resize((128, 64), Image.NEAREST)) > 127
    os.remove(tmp)
    H = 4.6
    fig, ax = canvas(W, H)
    ax2 = fig.add_axes([0.55 / W, 0.4 / H, 7.4 / W, 3.7 / H])
    ax2.imshow(img.astype(float), cmap=ListedColormap(["#101010", "#BFF3FF"]), interpolation="nearest")
    ax2.set_xticks([0, 42, 72, 127])
    ax2.set_yticks([0, 12, 24, 36, 63])
    ax2.tick_params(labelsize=5.5, length=2)
    ax2.set_title("128 × 64 点阵（按显示 ROM 渲染的示例画面）", fontsize=6.2)
    notes = [(3.75, "第 0~11 行：音色（第 0~41 列）、音区\n（第 42~71 列）、音量（第 72~127 列）"),
             (2.95, "第 12~23 行：效果器"),
             (2.45, "第 24~35 行：录音状态；录音中“●”闪烁\n并显示 mm:ss，发送时显示百分比"),
             (1.45, "第 36~63 行：32 段实时频谱，左低右高\n（100 Hz~12 kHz），柱顶带下落的峰值点")]
    for yy, t in notes:
        label(ax, 8.55, yy, t, fs=6.1, va="center")
    label(ax, 8.55, 0.55, "文字：宋体 12 像素点阵（gen_display_roms.py 生成）", fs=5.8, va="center", color="#777")
    save(fig, "fig2_oled.png")


# ================================ 图 2-11 录音状态机 ================================
def fig_recorder():
    H = 5.4
    fig, ax = canvas(W, H)
    bw, bh, gap, x0, y = 2.2, 1.15, 0.72, 0.3, 2.55
    st = [("0 空闲", "录音：按key2开始"), ("1 录音中", "●录音中 mm:ss"), ("2 等待电脑", "已录好，等待电脑连接"),
          ("3 发送中", "蓝牙发送 xx%"), ("4 已发送", "已发送 时长 mm:ss")]
    S = []
    for i, (a, b) in enumerate(st):
        x = x0 + i * (bw + gap)
        S.append(box(ax, x, y, bw, bh, "", C_LOG))
        ax.text(x + bw / 2, y + bh * 0.66, a, fontsize=7, ha="center", va="center", fontweight="bold", zorder=4)
        ax.text(x + bw / 2, y + bh * 0.3, b, fontsize=5.6, ha="center", va="center", color="#555", zorder=4)
    def mark(x, yy, s):
        ax.text(x, yy, s, fontsize=7, ha="center", va="center", color="#C05621", zorder=6,
                bbox=dict(boxstyle="circle,pad=0.08", fc="white", ec="none"))
    for i in range(4):
        path(ax, [(S[i]["r"], S[i]["cy"]), (S[i + 1]["l"], S[i + 1]["cy"])])
        mark((S[i]["r"] + S[i + 1]["l"]) / 2, S[i]["cy"] + 0.28, "①②③④"[i])
    # ⑤ 发送中 → 等待电脑（下方）
    path(ax, [(S[3]["cx"], S[3]["b"]), (S[3]["cx"], 1.85), (S[2]["cx"], 1.85), (S[2]["cx"], S[2]["b"])])
    mark((S[3]["cx"] + S[2]["cx"]) / 2, 1.85, "⑤")
    # ⑥ 已发送 → 发送中（上方）
    path(ax, [(S[4]["cx"], S[4]["t"]), (S[4]["cx"], 4.15), (S[3]["cx"] + 0.3, 4.15), (S[3]["cx"] + 0.3, S[3]["t"])])
    mark((S[4]["cx"] + S[3]["cx"] + 0.3) / 2, 4.15, "⑥")
    # ⑦ 等待电脑 / 已发送 → 录音中（更上方）
    path(ax, [(S[2]["cx"] - 0.3, S[2]["t"]), (S[2]["cx"] - 0.3, 4.65), (S[1]["cx"], 4.65), (S[1]["cx"], S[1]["t"])])
    ax.plot([S[4]["cx"] + 0.5, S[4]["cx"] + 0.5, S[2]["cx"] - 0.3], [S[4]["t"], 4.65, 4.65], color=INK, lw=0.9)
    mark((S[1]["cx"] + S[2]["cx"]) / 2, 4.65, "⑦")
    txt = ("① 按 key2 开始录音   ② 再按 key2，或存储满（16384 条 / 约 69 分钟）   ③ 电脑端程序在线（2 s 内收到 'H'）\n"
           "④ 全部字节发送完   ⑤ 电脑掉线（之后从头重发）   ⑥ 电脑发 'R' 要求重发   ⑦ 按 key2 开始新录音（覆盖旧录音）")
    label(ax, 0.35, 0.25, txt, fs=6.2)
    save(fig, "fig2_recorder.png")


# ================================ 图 2-12 电脑端程序流程 ================================
def fig_pc():
    H = 11.2
    fig, ax = canvas(W, H)
    lx, lw_ = 0.55, 6.0
    s0 = oval(ax, lx + lw_ / 2, 10.55, 2.0, 0.6, "开始")
    l1 = box(ax, lx, 9.0, lw_, 1.05, "扫描串口（注册表 SERIALCOMM，蓝牙串口优先），\n逐个打开并发 'H'，收到 'k' 即连接；\n都没有回应则 10 s 后重试", fs=6.7)
    l2 = box(ax, lx, 7.6, lw_, 0.7, "每 0.5 s 发 'H' 心跳，读取串口数据", fs=6.8)
    d1 = diamond(ax, lx + lw_ / 2, 6.3, 3.8, 0.95, "收到完整录音包？", fs=6.8)
    d2 = diamond(ax, lx + lw_ / 2, 4.75, 3.8, 0.95, "校验和、包尾正确？", fs=6.8)
    l5 = box(ax, lx, 3.2, lw_, 0.7, "保存原始数据 .bin", fs=6.8)
    path(ax, [(s0["cx"], s0["b"]), (l1["cx"], l1["t"])])
    path(ax, [(l1["cx"], l1["b"]), (l2["cx"], l2["t"])])
    path(ax, [(l2["cx"], l2["b"]), (d1["cx"], d1["t"])])
    path(ax, [(d1["cx"], d1["b"]), (d2["cx"], d2["t"])])
    path(ax, [(d2["cx"], d2["b"]), (l5["cx"], l5["t"])])
    label(ax, d1["cx"] + 0.1, 5.5, "是", fs=6.5)
    label(ax, d2["cx"] + 0.1, 3.95, "是", fs=6.5)
    path(ax, [(d1["l"], d1["cy"]), (0.3, d1["cy"]), (0.3, l2["cy"]), (l2["l"], l2["cy"])])
    label(ax, 0.35, d1["cy"] + 0.05, "否", fs=6.5)
    path(ax, [(d2["r"], d2["cy"]), (7.05, d2["cy"]), (7.05, l2["cy"]), (l2["r"], l2["cy"])])
    label(ax, d2["r"] + 0.05, d2["cy"] - 0.62, "否：发 'R'\n请求重发", fs=6.2)
    rx, rw = 7.6, 6.6
    R = [("复音分配（规则与 FPGA 的 voice_alloc 相同）", 9.35),
         ("逐声部合成：加法合成（钢琴 / 大提琴，含颤音）、\nKarplus-Strong（电吉他）", 8.15),
         ("电吉他总线过载失真 + 音箱模拟，与另一总线相加", 6.95),
         ("效果器：合唱 → 延迟 → 混响（按录音中的开关时刻）", 5.75),
         ("音量（按录音中的切换时刻）；归一化到 −1 dBFS", 4.55),
         ("重采样 48.828 kHz → 48 kHz，保存 WAV；导出 MIDI", 3.35)]
    RB = [box(ax, rx, yy, rw, 0.85 if "\n" in t else 0.7, t, C_SYN, fs=6.7) for t, yy in R]
    for i in range(len(RB) - 1):
        path(ax, [(RB[i]["cx"], RB[i]["b"]), (RB[i + 1]["cx"], RB[i + 1]["t"])])
    path(ax, [(l5["r"], l5["cy"]), (7.3, l5["cy"]), (7.3, 10.55), (RB[0]["cx"], 10.55), (RB[0]["cx"], RB[0]["t"])])
    dot(ax, 0.3, d1["cy"])
    path(ax, [(RB[-1]["cx"], RB[-1]["b"]), (RB[-1]["cx"], 2.6), (0.3, 2.6), (0.3, d1["cy"])], arrow=False)
    label(ax, 7.75, 2.65, "继续等待下一段录音（Ctrl+C 退出）", fs=6.3)
    label(ax, 0.55, 1.6, "渲染用的音色参数直接调用 gen_piano_params.py、gen_guitar_params.py，与 FPGA 完全一致；\n"
                         "录音文件保存在工程根目录 recordings/：录音_年月日_时分秒.wav / .mid / .bin。\n"
                         "串口直接调用 Windows API（ctypes），不需要安装 pyserial。", fs=6.3)
    save(fig, "fig2_pc.png")


# ================================ 图 3-x 示范录音的波形与频谱 ================================
def fig_demo():
    from scipy.signal import spectrogram
    p = os.path.join(ROOT, "recordings", "示范.wav")
    if not os.path.exists(p):
        print("跳过示范录音图：先运行 python scripts/bt_recorder.py --demo")
        return
    with wave.open(p, "rb") as f:
        fs = f.getframerate()
        x = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16).reshape(-1, 2).astype(float)
    m = x.mean(axis=1) / 32768.0
    t = np.arange(len(m)) / fs
    fig = plt.figure(figsize=(W / 2.54, 7.0 / 2.54), dpi=DPI)
    a1 = fig.add_axes([0.075, 0.66, 0.9, 0.27])
    a2 = fig.add_axes([0.075, 0.13, 0.9, 0.48], sharex=a1)
    a1.plot(t, m, lw=0.3, color="#2B6CB0")
    a1.set_ylim(-1, 1)
    a1.set_ylabel("幅度", fontsize=6.5)
    a1.tick_params(labelsize=6, labelbottom=False)
    segs = [(0.3, 3.9, "钢琴 · 干声"), (4.5, 8.1, "电吉他 · 混响"), (8.7, 16.7, "大提琴 · 合唱（音阶）"),
            (17.3, 19.8, "大提琴和弦")]
    for s, e, txt in segs:
        a1.axvspan(s, e, color="#EBF4FF", zorder=0)
        a1.text((s + e) / 2, 0.75, txt, fontsize=6.2, ha="center", color="#333")
    f, tt, S = spectrogram(m, fs=fs, nperseg=4096, noverlap=3072, window="hann")
    keep = f <= 3000
    a2.pcolormesh(tt, f[keep], 10 * np.log10(S[keep] + 1e-12), shading="auto", cmap="magma",
                  vmin=np.percentile(10 * np.log10(S[keep] + 1e-12), 55), vmax=None)
    a2.set_ylabel("频率 / Hz", fontsize=6.5)
    a2.set_xlabel("时间 / s", fontsize=6.5)
    a2.tick_params(labelsize=6)
    a1.set_title("电脑端程序渲染的示范录音（python scripts/bt_recorder.py --demo）：波形与频谱", fontsize=6.8)
    save(fig, "fig3_demo_render.png")


def main():
    fig_design_flow()
    fig_system()
    fig_hardware()
    fig_audio()
    fig_hierarchy()
    fig_voice()
    fig_partials()
    fig_guitar()
    fig_fx()
    fig_spectrum()
    fig_oled()
    fig_recorder()
    fig_pc()
    fig_demo()


if __name__ == "__main__":
    main()
