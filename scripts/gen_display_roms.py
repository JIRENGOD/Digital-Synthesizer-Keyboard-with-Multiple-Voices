#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 OLED 显示和频谱分析用的只读表（ROM）：
  src/oled/oled_text_rom.v   —— 屏幕上三行中文的点阵条（宋体 12 像素），每条 128 列 × 12 行
  src/oled/oled_digit_rom.v  —— 动态数字“0~9 : %”的点阵（宋体 12 像素，半角 6 列，每格 8 列）
  src/spectrum/fft_roms.v    —— 1024 点 FFT 用的汉宁窗、旋转因子、频点 → 频谱柱对应表

屏幕布局（128 × 64）：
  第 0 ~ 11 行   第 1 行文字：音色（第 0 ~ 41 列）、音区（第 42 ~ 71 列）、音量（第 72 ~ 127 列）
  第 12 ~ 23 行  第 2 行文字：效果器
  第 24 ~ 35 行  第 3 行文字：录音状态（录音时长 / 发送进度这些数字在第 80 列开始，由数字表动态显示）
  第 36 ~ 63 行  实时频谱（28 行）
每一条文字（strip）都是整行 128 列宽，字只画在它自己的那一段里，其余为空；oled_frame.v 按列号选条。

用法（在工程根目录执行）：
    python scripts/gen_display_roms.py
改了显示文字（下面的 STRIPS）或频谱柱的频率范围（F_LO / F_HI）后重新生成，再在云源里重新综合。
需要 Pillow 和 Windows 自带的宋体 C:/Windows/Fonts/simsun.ttc。
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

FS = 50_000_000 / 1024      # 采样率 48 828.125 Hz（与 i2s_dac_tx 一致）
N = 1024                    # FFT 点数（spectrum.v）
NBAR = 32                   # 频谱柱数（每根 4 列：3 列亮 + 1 列空）
F_LO, F_HI = 100.0, 12000.0 # 频谱柱覆盖的频率范围（Hz），按对数均匀分成 NBAR 段
FONT = ("C:/Windows/Fonts/simsun.ttc", 0, 12)   # 宋体 12 像素（自带点阵，OLED 上最清楚）
LINE_H = 12
DIGIT_X = 80                # 第 3 行动态数字从第 80 列开始，每个字符占 8 列

# (条号, 文字, x 起点, 是否右对齐到第 127 列)。条号顺序必须和 oled_frame.v 一致。
STRIPS = [
    ("钢琴", 0, False), ("电吉他", 0, False), ("大提琴", 0, False),            # 0 ~ 2   音色
    ("低音", 42, False), ("中音", 42, False), ("高音", 42, False),             # 3 ~ 5   音区
    ("音量|0dB", 72, True), ("音量|-6dB", 72, True),                          # 6 ~ 9   音量（| 后面右对齐）
    ("音量|-12dB", 72, True), ("音量|-18dB", 72, True),
    ("效果：干声", 0, False), ("效果：混响", 0, False), ("效果：延迟", 0, False),  # 10 ~ 14 效果器
    ("效果：合唱", 0, False), ("效果：三个全开", 0, False),
    ("录音：按key2开始", 0, False),                                          # 15 没有录音
    ("●录音中", 0, False),                                                   # 16 录音中（后面是 mm:ss）
    ("已录好，等待电脑连接", 0, False),                                        # 17 等电脑连接
    ("蓝牙发送", 0, False),                                                   # 18 发送中（后面是 xx%）
    ("已发送 时长", 0, False),                                                # 19 发送完（后面是 mm:ss）
]
DIGITS = "0123456789:% "    # 数字表的字符（序号 0 ~ 12）


def render_strip(font, text, x, right):
    im = Image.new("1", (128, LINE_H), 0)
    d = ImageDraw.Draw(im)
    if right:
        left, val = text.split("|")
        d.text((x, 0), left, font=font, fill=1)
        w = font.getbbox(val)[2]
        d.text((128 - w, 0), val, font=font, fill=1)
    else:
        d.text((x, 0), text, font=font, fill=1)
    box = im.getbbox()
    full = Image.new("1", (400, LINE_H), 0)
    ImageDraw.Draw(full).text((x, 0), text.replace("|", ""), font=font, fill=1)
    if full.getbbox() and full.getbbox()[2] > 128 and not right:
        raise SystemExit(f"“{text}”太长，超出 128 像素")
    return im, box


def column_words(im, width):
    px = im.load()
    words = []
    for col in range(width):
        v = 0
        for row in range(LINE_H):
            if px[col, row]:
                v |= 1 << row                       # 第 0 位 = 最上面一行
        words.append(v)
    return words


def write_text_rom(path, font):
    lines = ["`timescale 1ns/1ps",
             "// =============================================================================",
             "// oled_text_rom.v —— OLED 文字点阵条（宋体 12 像素），由 scripts/gen_display_roms.py 自动生成，不要手改",
             "// -----------------------------------------------------------------------------",
             "// 地址 = {条号[4:0], 列[6:0]}；数据 = 这一列的 12 个点（第 0 位 = 最上面一行）。读数据晚 1 拍。",
             "// 条号："]
    rom = []
    for i, (text, x, right) in enumerate(STRIPS):
        im, _ = render_strip(font, text, x, right)
        rom.append(column_words(im, 128))
        lines.append(f"//   {i:2d}  {text.replace('|', ' ')}")
    lines += ["// =============================================================================",
              "module oled_text_rom (",
              "    input  wire        clk,",
              "    input  wire [11:0] addr,",
              "    output reg  [11:0] data",
              ");",
              "",
              "    always @(posedge clk) begin",
              "        case (addr)"]
    for s, words in enumerate(rom):
        for col, v in enumerate(words):
            if v:
                lines.append(f"        12'd{s * 128 + col}: data <= 12'h{v:03X};")
    lines += ["        default: data <= 12'h000;",
              "        endcase",
              "    end",
              "",
              "endmodule",
              ""]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    return rom


def write_digit_rom(path, font):
    lines = ["`timescale 1ns/1ps",
             "// =============================================================================",
             "// oled_digit_rom.v —— OLED 动态字符点阵（宋体 12 像素半角），由 scripts/gen_display_roms.py 自动生成，不要手改",
             "// -----------------------------------------------------------------------------",
             "// 地址 = {字符[3:0], 列[2:0]}（每格 8 列，字在左边 6 列）；数据 = 这一列的 12 个点（第 0 位在上）。",
             "// 字符：" + "  ".join(f"{i}=“{c}”" for i, c in enumerate(DIGITS)),
             "// 组合输出（无延迟）。",
             "// =============================================================================",
             "module oled_digit_rom (",
             "    input  wire [6:0]  addr,",
             "    output reg  [11:0] data",
             ");",
             "",
             "    always @(*) begin",
             "        case (addr)"]
    for g, ch in enumerate(DIGITS):
        im = Image.new("1", (8, LINE_H), 0)
        ImageDraw.Draw(im).text((0, 0), ch, font=font, fill=1)
        for col, v in enumerate(column_words(im, 8)):
            if v:
                lines.append(f"        7'd{g * 8 + col}: data = 12'h{v:03X};   // “{ch}” 第 {col} 列")
    lines += ["        default: data = 12'h000;",
              "        endcase",
              "    end",
              "",
              "endmodule",
              ""]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))


def band_table():
    df = FS / N
    edges = [F_LO * (F_HI / F_LO) ** (b / NBAR) for b in range(NBAR + 1)]
    band = [63] * (N // 2)
    ranges = []
    prev_hi = 1                                     # 第 0、1 个频点（直流附近）不用
    for b in range(NBAR):
        lo = max(prev_hi + 1, round(edges[b] / df))
        hi = max(lo, round(edges[b + 1] / df) - 1)
        for k in range(lo, hi + 1):
            band[k] = b
        ranges.append((lo, hi))
        prev_hi = hi
    return band, ranges


def write_fft_roms(path):
    win = [round(65535 * 0.5 * (1 - math.cos(2 * math.pi * n / N))) for n in range(N)]
    tw = []
    for k in range(N // 2):
        wr = round(32767 * math.cos(2 * math.pi * k / N))
        wi = round(-32767 * math.sin(2 * math.pi * k / N))
        tw.append(((wr & 0xFFFF) << 16) | (wi & 0xFFFF))
    band, ranges = band_table()
    df = FS / N
    lines = ["`timescale 1ns/1ps",
             "// =============================================================================",
             "// fft_roms.v —— 1024 点 FFT 用的只读表，由 scripts/gen_display_roms.py 自动生成，不要手改",
             "//   fft_win_rom  汉宁窗 w[n] = 65535 × (1 - cos(2πn/1024)) / 2，无符号",
             "//   fft_tw_rom   旋转因子 W^k = cos(2πk/1024) - j·sin(2πk/1024)，{实部, 虚部} 各 16 位有符号（×32767）",
             "//   fft_band_rom 频点 k（k × 47.68 Hz）属于第几根频谱柱，63 = 不显示",
             f"// 频谱柱（{NBAR} 根，{F_LO:g} ~ {F_HI:g} Hz 按对数均分；低频段每根至少 1 个频点）：",
             ]
    for b, (lo, hi) in enumerate(ranges):
        lines.append(f"//   柱 {b:2d}：频点 {lo:3d} ~ {hi:3d}（{lo * df:7.0f} ~ {(hi + 1) * df:7.0f} Hz）")
    lines += ["// 三个表都是读数据晚 1 拍（输出寄存）。",
              "// =============================================================================",
              "module fft_win_rom (",
              "    input  wire        clk,",
              "    input  wire [9:0]  addr,",
              "    output reg  [15:0] data",
              ");",
              "    always @(posedge clk) begin",
              "        case (addr)"]
    for n, v in enumerate(win):
        lines.append(f"        10'd{n}: data <= 16'd{v};")
    lines += ["        default: data <= 16'd0;",
              "        endcase",
              "    end",
              "endmodule",
              "",
              "module fft_tw_rom (",
              "    input  wire        clk,",
              "    input  wire [8:0]  addr,",
              "    output reg  [31:0] data",
              ");",
              "    always @(posedge clk) begin",
              "        case (addr)"]
    for k, v in enumerate(tw):
        lines.append(f"        9'd{k}: data <= 32'h{v:08X};")
    lines += ["        default: data <= 32'h7FFF0000;",
              "        endcase",
              "    end",
              "endmodule",
              "",
              "module fft_band_rom (",
              "    input  wire       clk,",
              "    input  wire [8:0] addr,",
              "    output reg  [5:0] data",
              ");",
              "    always @(posedge clk) begin",
              "        case (addr)"]
    for k, v in enumerate(band):
        if v != 63:
            lines.append(f"        9'd{k}: data <= 6'd{v};")
    lines += ["        default: data <= 6'd63;",
              "        endcase",
              "    end",
              "endmodule",
              ""]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    return ranges


def preview(path, font):
    """画一张示意图（4 倍放大），方便检查文字位置。"""
    im = Image.new("1", (128, 64), 0)
    d = ImageDraw.Draw(im)
    for idx, y in ((2, 0), (5, 0), (8, 0), (14, 12), (16, 24)):
        s, _ = render_strip(font, *STRIPS[idx])
        im.paste(s, (0, y), s)
    for k, ch in enumerate("01:23"):
        d.text((DIGIT_X + 8 * k, 24), ch, font=font, fill=1)
    for b in range(NBAR):
        h = int(14 + 12 * math.sin(b * 0.45))
        d.rectangle((b * 4, 63 - h + 1, b * 4 + 2, 63), fill=1)
    im.resize((512, 256), Image.NEAREST).save(path)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, "..", "src")
    os.makedirs(os.path.join(src, "oled"), exist_ok=True)
    os.makedirs(os.path.join(src, "spectrum"), exist_ok=True)
    font = ImageFont.truetype(FONT[0], FONT[2], index=FONT[1])
    write_text_rom(os.path.join(src, "oled", "oled_text_rom.v"), font)
    write_digit_rom(os.path.join(src, "oled", "oled_digit_rom.v"), font)
    ranges = write_fft_roms(os.path.join(src, "spectrum", "fft_roms.v"))
    print(f"已写入 src/oled/oled_text_rom.v（{len(STRIPS)} 条文字）、src/oled/oled_digit_rom.v、"
          f"src/spectrum/fft_roms.v（{NBAR} 根频谱柱，{F_LO:g} ~ {F_HI:g} Hz）")
    if os.environ.get("OLED_PREVIEW"):
        preview(os.environ["OLED_PREVIEW"], font)
        print("示意图：" + os.environ["OLED_PREVIEW"])


if __name__ == "__main__":
    main()
