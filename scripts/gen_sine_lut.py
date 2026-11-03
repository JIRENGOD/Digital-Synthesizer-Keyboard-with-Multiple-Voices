#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 src/sine_lut.hex —— 供 sine_lut.v 用 $readmemh 加载的整周期正弦表。

核心就一行：
    v = round(sin(2*pi*i/N) * 32767)
每行一个 4 位十六进制数（16 位二进制补码），共 N 行，不含注释（避免综合器解析问题）。

用法（在工程根目录执行）：
    python scripts/gen_sine_lut.py                 # 默认 4096 点 -> src/sine_lut.hex
    python scripts/gen_sine_lut.py --depth 1024    # 改深度后，同步修改 top_i2s_sine.v 的 LUT_ADDR_W
    python scripts/gen_sine_lut.py --tone 523.25   # 额外打印任意音调的相位增量

深度与相位截断杂散（440 Hz @ 48 kHz，numpy 实测）：
    1024 点 ≈ -61 dBc，4096 点 ≈ -73 dBc，16384 点 ≈ -84 dBc
"""
import argparse
import math
import os

F_CLK = 50_000_000
FS_TABLE = {
    "CLK_MODE=0 (48 kHz)": F_CLK * 1536 / 3125 / 512,  # = 48000.0
    "CLK_MODE=1 (48.828125 kHz)": F_CLK / 2 / 512,     # = 48828.125
}


def phase_inc(f_out, fs):
    """相位增量 = round(F_out * 2^32 / Fs)"""
    return int(round(f_out * 2**32 / fs))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="生成 sine_lut.hex（16 位有符号正弦表）")
    ap.add_argument("--depth", type=int, default=4096, help="表深度 N，2 的幂，>=1024（默认 4096）")
    ap.add_argument("--amp", type=int, default=32767, help="幅度（默认 32767 = 16 位满幅）")
    ap.add_argument("--out", default=os.path.join(here, "..", "src", "sine_lut.hex"), help="输出文件")
    ap.add_argument("--tone", type=float, action="append", default=[], help="额外打印该频率的相位增量")
    args = ap.parse_args()

    n = args.depth
    if n < 1024 or n & (n - 1):
        raise SystemExit("--depth 必须是 2 的幂且 >= 1024")
    if not 0 < args.amp <= 32767:
        raise SystemExit("--amp 必须在 1..32767 之间")

    with open(args.out, "w", newline="\n") as f:
        for i in range(n):
            v = int(round(math.sin(2 * math.pi * i / n) * args.amp))
            f.write(f"{v & 0xFFFF:04X}\n")

    addr_w = n.bit_length() - 1
    print(f"已写入 {os.path.normpath(args.out)}：{n} 点，LUT_ADDR_W = {addr_w}，幅度 ±{args.amp}")

    for name, fs in FS_TABLE.items():
        for tone in [440.0, 1000.0] + args.tone:
            inc = phase_inc(tone, fs)
            print(f"  {name:28s} F_out={tone:8.2f} Hz -> PHASE_INC = 32'd{inc:<10d}"
                  f"（实际 {inc * fs / 2**32:.6f} Hz）")


if __name__ == "__main__":
    main()
