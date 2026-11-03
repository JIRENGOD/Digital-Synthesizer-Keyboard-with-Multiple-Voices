#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bt_recorder.py —— 电脑端程序：通过蓝牙（HC-05）接收 FPGA 电子琴的演奏录音，生成 WAV 和 MIDI 文件

FPGA 录下的是“演奏动作”（每个琴键按下 / 松开的时间、音色、效果器和音量的变化，见 src/bt/recorder.v），
传到电脑后，本程序用和 FPGA 相同的合成算法、相同的参数（直接调用 gen_piano_params.py、gen_guitar_params.py）
把这段演奏重新“弹”一遍，生成 48 kHz 立体声 WAV；同时另存一份 MIDI，可以用其他音乐软件打开、编辑。

用法（在工程根目录执行）：
  1. 第一次用：在 Windows“设置 → 蓝牙和其他设备 → 添加设备 → 蓝牙”里找到 HC-05，配对密码 1234。
  2. python scripts/bt_recorder.py              自动找 HC-05 的蓝牙串口，连上后一直等录音（Ctrl+C 退出）
     python scripts/bt_recorder.py --port COM5   自动找不到时手动指定（“更多蓝牙设置 → COM 端口”里
                                                 HC-05 “传出”的那个）
  3. 在电子琴上按 key[2] 开始录音，再按一次停止；录音会自动传过来，保存在工程根目录的 recordings 文件夹：
     录音_年月日_时分秒.wav（声音）、.mid（MIDI）、.bin（原始数据）
  其他：
     python scripts/bt_recorder.py --render recordings/xxx.bin   把保存的原始数据重新渲染成 WAV / MIDI
     python scripts/bt_recorder.py --demo                        不连硬件，渲染一段示范，检查本程序能不能用
需要 numpy、scipy。
"""
import argparse
import datetime
import math
import os
import struct
import sys
import time
import wave

import numpy as np
from scipy.signal import lfilter, resample_poly

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_guitar_params as gg   # noqa: E402
import gen_piano_params as gp    # noqa: E402

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.normpath(os.path.join(HERE, "..", "recordings"))

# ============================== 与 FPGA 一致的常数 ==============================
FS = gp.FS                       # 48 828.125 Hz
NV = 8                           # 复音数（top_touch_synth.v 的 NV）
TOUCH_GAIN = 220 / 256           # 触摸键盘的力度增益
VIB_STEP, VIB_DEPTH = 10, 683    # 大提琴颤音（piano_voice.v）
DEP_MAX = VIB_DEPTH * 256
R_STEP = 255.0 / 256.0           # 包络每一步 ×(1 - 1/256)
LN_R = math.log(R_STEP)
MUTE_HZ = 20000.0                # 高于 20 kHz 的分音静音
TONE_NAMES = ["钢琴", "电吉他", "大提琴"]
FX_NAMES = ["干声", "混响", "延迟", "合唱", "三个全开"]

_PMIDI, _CMIDI, PIANO, CELLO = gp.build()
GTR = gg.build()
MREL = [gp.period_for(gp.RELEASE_DBPS)[0], gp.period_for(gp.CELLO_RELEASE_DBPS)[0]]
NLPF = [gp.HAMMER_LPF_SHIFT, gp.CELLO_NOISE_LPF_SHIFT]
NOSC = len(PIANO)


def ms2n(t_ms):
    return int(round(t_ms * FS / 1000.0))


# ================================ 数据包解析 ================================
def parse_packet(pkt):
    """解析 FPGA 发来的录音包（格式见 src/bt/recorder.v）。返回 (录音, 错误信息)。"""
    if len(pkt) < 20 or pkt[:4] != b"MPR1":
        return None, "包头不对"
    n_ev, dur_ms = struct.unpack(">II", pkt[4:12])
    total = 20 + 4 * n_ev
    if len(pkt) != total:
        return None, "长度不对"
    if pkt[total - 2:] != b"ED":
        return None, "包尾不对"
    if (sum(pkt[:total - 4]) & 0xFFFF) != struct.unpack(">H", pkt[total - 4:total - 2])[0]:
        return None, "校验和不对"
    events = []
    for k in range(n_ev):
        w = struct.unpack(">I", pkt[16 + 4 * k:20 + 4 * k])[0]
        typ, t = w >> 30, (w >> 8) & 0x3FFFFF
        if typ == 0:
            events.append((t, "on", (w >> 6) & 3, w & 63))
        elif typ == 1:
            events.append((t, "off", 0, w & 63))
        elif typ == 2:
            events.append((t, "fx" if ((w >> 5) & 7) == 0 else "vol", w & 31, 0))
    return dict(dur_ms=dur_ms, fx0=min(pkt[12], 4), vol0=min(pkt[13], 3), events=events), None


def make_packet(rec):
    """把录音编码成和 FPGA 一样的数据包（--demo 用）。"""
    words = []
    for t, kind, a, b in rec["events"]:
        if kind == "on":
            words.append((t << 8) | (a << 6) | b)
        elif kind == "off":
            words.append((1 << 30) | (t << 8) | b)
        else:
            words.append((2 << 30) | (t << 8) | ((0 if kind == "fx" else 1) << 5) | a)
    body = b"MPR1" + struct.pack(">II", len(words), rec["dur_ms"]) + bytes([rec["fx0"], rec["vol0"], 0, 0])
    body += b"".join(struct.pack(">I", w) for w in words)
    return body + struct.pack(">H", sum(body) & 0xFFFF) + b"ED"


# ================================ 合成：复音分配 ================================
def allocate(events, n_end):
    """按 voice_alloc.v 的规则把琴键事件分给 8 个声部，返回每个声部两个发声单元的事件表。"""
    vid, held, stamp, now = [None] * NV, [False] * NV, [0] * NV, 0
    add_ev = [[] for _ in range(NV)]          # 加法合成单元（钢琴 / 大提琴）
    gtr_ev = [[] for _ in range(NV)]          # 电吉他单元
    for t, kind, a, b in events:
        n = ms2n(t)
        if kind == "on":
            tone, note = a, b
            tgt = next((i for i in range(NV) if vid[i] == note), None)
            if tgt is None:
                for want_held in (False, True):
                    for i in range(NV):
                        if held[i] == want_held and (tgt is None or now - stamp[i] > now - stamp[tgt]):
                            tgt = i
                    if tgt is not None:
                        break
            vid[tgt], held[tgt], stamp[tgt] = note, True, now
            now += 1
            if tone == 1:
                add_ev[tgt].append((n, "off"))
                gtr_ev[tgt].append((n, "on", note, 0))
            else:
                add_ev[tgt].append((n, "on", note, 1 if tone == 2 else 0))
                gtr_ev[tgt].append((n, "off"))
        elif kind == "off":
            for i in range(NV):
                if vid[i] == b and held[i]:
                    held[i], stamp[i] = False, now
                    now += 1
                    add_ev[i].append((n, "off"))
                    gtr_ev[i].append((n, "off"))
                    break
    for i in range(NV):                       # 录音结束时还按着的键都抬起
        add_ev[i].append((n_end, "off"))
        gtr_ev[i].append((n_end, "off"))
    return add_ev, gtr_ev


def segments(evs):
    """一个发声单元的事件表 → 每个音一段：击键、抬键、被下一个音打断的时间。"""
    segs, cur = [], None
    for e in evs:
        if e[1] == "on":
            if cur is not None:
                cur["cut"] = e[0]
            cur = dict(on=e[0], off=None, cut=None, note=e[2], patch=e[3])
            segs.append(cur)
        elif cur is not None and cur["off"] is None:
            cur["off"] = e[0]
    return segs


# ============================ 合成：加法合成（钢琴 / 大提琴） ============================
def env_curve(n, e0, peak, att, knee, m1, m2, n_off, mrel):
    """一个振荡器 n 个采样的包络，与 piano_voice.v 的状态机一致（指数段用连续函数代替台阶）。"""
    t = np.arange(n, dtype=np.float64)
    env = np.empty(n)
    if e0 >= peak:
        ta, t_pk = 0, -1.0
    else:
        ta = int(math.ceil((peak - e0) / att))
        t_pk = ta - 1.0
        k = min(ta, n)
        env[:k] = np.minimum(e0 + att * (t[:k] + 1.0), peak)
    if ta < n:
        td = t[ta:] - t_pk
        t_k = m1 * math.log(knee / peak) / LN_R if knee < peak else 0.0
        env[ta:] = np.where(td < t_k, peak * np.exp(LN_R * td / m1), knee * np.exp(LN_R * (td - t_k) / m2))
    if n_off is not None and n_off < n:
        e_off = env[n_off - 1] if n_off > 0 else e0
        env[n_off:] = e_off * np.exp(LN_R * (t[n_off:] - (n_off - 1)) / mrel)
    return env


def render_additive(v, evs, bus, n_total, rng):
    e_st, ph_st = np.zeros(NOSC), np.zeros(NOSC)
    for seg in segments(evs):
        n0, patch, note = seg["on"], seg["patch"], seg["note"]
        if n0 >= n_total:
            break
        mrel = MREL[patch]
        off = seg["off"] if seg["off"] is not None else n_total
        n1 = min(n_total, off + 2950 * mrel)          # 抬键后衰减 100 dB 就不再算
        cut = seg["cut"] is not None and seg["cut"] <= n1
        if cut:
            n1 = seg["cut"]
        L = n1 - n0
        if L <= 0:
            continue
        n_off = None if seg["off"] is None else seg["off"] - n0
        entries = PIANO if patch == 0 else CELLO[note]
        f0 = gp.midi_hz(gp.NOTE_MIDI0 + (0 if patch == 0 else gp.CELLO_SHIFT) + note)
        if patch == 1:                                # 大提琴颤音：所有谐波频率按同一比例变化
            dep = np.minimum(np.arange(1, L + 1) * VIB_STEP, DEP_MAX) / 256.0 / 65536.0
            fv = (112 + 2 * v) * FS / 2 ** 20
            vib = dep * np.sin(2 * np.pi * fv * np.arange(n0, n1) / FS)
            S = np.concatenate(([0.0], np.cumsum(1.0 + vib)))
        else:
            S = np.arange(L + 1, dtype=np.float64)
        out = np.zeros(L)
        new_e, new_ph = np.zeros(NOSC), np.zeros(NOSC)
        for k, e in enumerate(entries):
            env = env_curve(L, e_st[k], e["peak"], e["att"], e["knee"], e["m1"], e["m2"], n_off, mrel)
            new_e[k] = env[-1]
            if e["noise"]:
                a = 2.0 ** -NLPF[patch]
                src = lfilter([a], [1.0, a - 1.0], rng.integers(-32768, 32768, L).astype(np.float64))
            else:
                w = 2 * np.pi * f0 * e["ratio"] / FS
                new_ph[k] = (ph_st[k] + w * S[L]) % (2 * np.pi)
                if f0 * e["ratio"] >= MUTE_HZ:
                    continue
                src = 32767.0 * np.sin(ph_st[k] + w * S[:L])
            out += src * env
        bus[n0:n1] += out / 2 ** 24
        if cut:                                       # 被下一个音打断：包络、相位接着用（和 FPGA 一样）
            e_st, ph_st = new_e, new_ph
        else:
            e_st, ph_st = np.zeros(NOSC), np.zeros(NOSC)


# ============================ 合成：电吉他（Karplus-Strong） ============================
def ks_note(note, L, n_off, rng):
    p = GTR[note]
    M, P, c = p["M"], p["P"], p["c"] / 32768.0
    H = M + 2
    buf = np.zeros(H + L)
    exc = lfilter([0.5], [1.0, -0.5], rng.integers(-32768, 32768, M + 1).astype(np.float64))
    g = np.full(L, p["gs"] / 65536.0)
    if n_off is not None and n_off < L:
        g[max(n_off, 0):] = p["gr"] / 65536.0
    zi = np.zeros(1)
    s = 0
    while s < L:                                      # 每次算 M 个采样：只依赖已经算好的前一圈
        e = min(s + M, L)
        a = 0.5 * (buf[H + s - M:H + e - M] + buf[H + s - M - 1:H + e - M - 1])
        ap, zi = lfilter([c, 1.0], [1.0, c], a, zi=zi)
        vv = g[s:e] * ap
        if s <= M:                                    # 拨弦：前 M+1 个采样灌噪声
            k = min(e, M + 1)
            vv[:k - s] = exc[s:k]
        buf[H + s:H + e] = vv
        s = e
    out = 0.5 * (buf[H:] - buf[H - P:H - P + L])     # 拾音器
    return np.clip(out, -32768, 32767)


def render_guitar(evs, bus, n_total, rng):
    for seg in segments(evs):
        n0 = seg["on"]
        if n0 >= n_total:
            break
        off = seg["off"] if seg["off"] is not None else n_total
        n1 = min(n_total, off + int(FS))
        cut = seg["cut"] is not None and seg["cut"] < n1
        if cut:
            n1 = seg["cut"]
        L = n1 - n0
        if L <= 0:
            continue
        out = ks_note(seg["note"], L, off - n0, rng)
        if cut:                                       # 被新的拨弦打断：最后 64 个采样淡出
            k = min(64, L)
            out[-k:] *= np.linspace(1.0, 0.0, k)
        bus[n0:n1] += out


def gtr_amp(x):
    """gtr_amp.v：×4 驱动 → 三次软削波 → 两级约 4 kHz 低通 → ×0.3。"""
    u = np.clip(x * 4.0, -32767, 32767) / 32768.0
    y = (1.5 * u - 0.5 * u ** 3) * 32768.0
    y = lfilter([0.4], [1.0, -0.6], lfilter([0.4], [1.0, -0.6], y))
    return 0.3 * y


# ================================ 效果器 ================================
def ramp(n, switches):
    """效果器开关的渐变量 m（0 ~ 256，每个采样 ±1），switches = [(采样序号, 开/关), ...]。"""
    m = np.empty(n)
    cur = 256.0 if switches[0][1] else 0.0
    for i, (s, en) in enumerate(switches):
        e = switches[i + 1][0] if i + 1 < len(switches) else n
        s, e = min(s, n), min(e, n)
        if e <= s:
            continue
        k = np.arange(1, e - s + 1, dtype=np.float64)
        m[s:e] = np.clip(cur + k if en else cur - k, 0.0, 256.0)
        cur = m[e - 1]
    return m


def fx_chorus(x, m):
    """fx_chorus.v：延迟 8 ~ 15 ms（0.6 Hz 三角波调制，左右差 90°）+ 线性插值。"""
    n = len(x)
    ph = (206 * np.arange(n, dtype=np.int64)) % (1 << 24)
    xp = np.concatenate((np.zeros(1024), x))
    outs = []
    for p in (ph, (ph + 0x400000) % (1 << 24)):
        q = (p >> 7) & 0xFFFF
        tri = np.where(p & (1 << 23), 0xFFFF - q, q)
        dly = 400 * 256 + ((tri * (340 * 256)) >> 16)
        idx = np.arange(n) - (dly >> 8) + 1024
        frac = (dly & 255) / 256.0
        y = xp[idx] + (xp[idx - 1] - xp[idx]) * frac
        outs.append((x * (256 - np.floor(m / 4)) + y * np.floor(3 * m / 4)) / 256.0)
    return outs[0], outs[1]


def fx_delay(l, r, m, LEN=12207):
    """fx_delay.v：乒乓回声 250 ms，右 → 左反馈 0.65（带低通），左 → 右 0.7，回声音量 0.5。"""
    n = len(l)
    feed = (l + r) / 2.0 * m / 256.0
    wl, wr, el, er = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    zi = np.zeros(1)
    for s in range(0, n, LEN):
        e = min(s + LEN, n)
        if s >= LEN:
            el[s:e] = wl[s - LEN:e - LEN]
            er[s:e] = wr[s - LEN:e - LEN]
        lp, zi = lfilter([0.5], [1.0, -0.5], er[s:e], zi=zi)
        wl[s:e] = np.clip(feed[s:e] + 0.65 * lp, -131072, 131071)
        wr[s:e] = 0.7 * el[s:e]
    return l + 0.5 * el, r + 0.5 * er


def fx_reverb(l, r, m):
    """fx_reverb.v（Freeverb）：8 个带阻尼的梳状滤波器 → 左右各 4 个串联全通。"""
    n = len(l)
    inp = (l + r) * m / 16384.0
    acc = np.zeros(n)
    for D in (1236, 1315, 1414, 1501, 1574, 1651, 1724, 1790):
        w = np.zeros(n)
        zi = np.zeros(1)
        for s in range(0, n, D):
            e = min(s + D, n)
            rd = w[s - D:e - D] if s >= D else np.zeros(e - s)
            fs, zi = lfilter([0.75], [1.0, -0.25], rd, zi=zi)
            w[s:e] = np.clip(inp[s:e] + (57672 / 65536) * fs, -131072, 131071)
            acc[s:e] += rd
    xs = acc / 2.0

    def chain(x, lens):
        for D in lens:
            w, y = np.zeros(n), np.zeros(n)
            for s in range(0, n, D):
                e = min(s + D, n)
                rd = w[s - D:e - D] if s >= D else np.zeros(e - s)
                w[s:e] = x[s:e] + 0.5 * rd
                y[s:e] = rd - x[s:e]
            x = y
        return x

    return l + 2.0 * chain(xs, (616, 488, 378, 249)), r + 2.0 * chain(xs, (641, 514, 403, 275))


# ================================ 整体渲染 ================================
def render(rec, say=print):
    n_end = ms2n(rec["dur_ms"])
    n_total = n_end + int(5 * FS)                     # 留 5 秒给余音、回声、混响
    bus_p, bus_g = np.zeros(n_total), np.zeros(n_total)
    add_ev, gtr_ev = allocate(rec["events"], n_end)
    rng = np.random.default_rng(20261103)
    for v in range(NV):
        say(f"  合成声部 {v + 1}/{NV} …")
        render_additive(v, add_ev[v], bus_p, n_total, rng)
        render_guitar(gtr_ev[v], bus_g, n_total, rng)
    bus_p = np.clip(bus_p * TOUCH_GAIN, -131072, 131071)
    bus_g = np.clip(bus_g * TOUCH_GAIN, -131072, 131071)
    dry = np.clip(bus_p + gtr_amp(bus_g), -131072, 131071)

    say("  效果器 …")
    modes = [(0, rec["fx0"])] + [(ms2n(t), a) for t, k, a, _ in rec["events"] if k == "fx"]
    sw = lambda ons: [(s, md in ons) for s, md in modes]   # noqa: E731
    l, r = fx_chorus(dry, ramp(n_total, sw((3, 4))))
    l, r = fx_delay(np.clip(l, -131072, 131071), np.clip(r, -131072, 131071), ramp(n_total, sw((2, 4))))
    l, r = fx_reverb(np.clip(l, -131072, 131071), np.clip(r, -131072, 131071), ramp(n_total, sw((1, 4))))

    gain = np.empty(n_total)                          # 音量：-6 dB 一档（和 FPGA 的右移一样）
    vols = [(0, rec["vol0"])] + [(ms2n(t), a) for t, k, a, _ in rec["events"] if k == "vol"]
    for i, (s, vv) in enumerate(vols):
        e = vols[i + 1][0] if i + 1 < len(vols) else n_total
        gain[min(s, n_total):min(e, n_total)] = 2.0 ** -min(vv, 3)
    x = np.stack((l * gain, r * gain), axis=1)

    a = np.abs(x).max(axis=1)
    peak = a.max()
    if peak <= 0:
        return np.zeros((int(48000 * max(rec["dur_ms"], 500) / 1000), 2), dtype=np.int16)
    loud = np.nonzero(a > peak * 10 ** (-70 / 20))[0]  # 去掉末尾 -70 dB 以下的静音
    end = min(n_total, max(n_end, int(loud[-1]) if len(loud) else 0) + int(0.3 * FS))
    x = x[:end] * (0.89 * 32767 / peak)               # 峰值 -1 dBFS
    say("  转换成 48 kHz …")
    y = resample_poly(x, 3072, 3125, axis=0)          # 48828.125 Hz → 48000 Hz
    return np.clip(np.round(y), -32768, 32767).astype(np.int16)


def write_wav(path, pcm):
    with wave.open(path, "wb") as f:
        f.setnchannels(2)
        f.setsampwidth(2)
        f.setframerate(48000)
        f.writeframes(pcm.tobytes())


def write_midi(path, rec):
    """MIDI：1 个音轨，1 tick = 1 ms；通道 0 钢琴、1 电吉他（过载）、2 大提琴；混响 / 合唱用 CC91 / CC93。"""
    ev = [(0, 0, b"\xFF\x51\x03" + (1000000).to_bytes(3, "big"))]   # 每拍 1 秒，1000 tick/拍
    vol_cc = [127, 90, 64, 45]

    def ctl(t, num, val):
        for ch in range(3):
            ev.append((t, 1, bytes([0xB0 | ch, num, val])))

    for ch, prog in enumerate((0, 29, 42)):
        ev.append((0, 0, bytes([0xC0 | ch, prog])))
    ctl(0, 7, vol_cc[rec["vol0"]])
    ctl(0, 91, 64 if rec["fx0"] in (1, 4) else 0)
    ctl(0, 93, 64 if rec["fx0"] in (3, 4) else 0)
    active = {}
    for t, kind, a, b in rec["events"]:
        if kind == "on":
            if b in active:
                ch, mn = active.pop(b)
                ev.append((t, 1, bytes([0x80 | ch, mn, 0])))
            mn = (gp.NOTE_MIDI0 + (gp.CELLO_SHIFT if a == 2 else 0) + b) & 0x7F
            ch = min(a, 2)
            active[b] = (ch, mn)
            ev.append((t, 2, bytes([0x90 | ch, mn, 100])))
        elif kind == "off" and b in active:
            ch, mn = active.pop(b)
            ev.append((t, 1, bytes([0x80 | ch, mn, 0])))
        elif kind == "fx":
            ctl(t, 91, 64 if a in (1, 4) else 0)
            ctl(t, 93, 64 if a in (3, 4) else 0)
        elif kind == "vol":
            ctl(t, 7, vol_cc[min(a, 3)])
    for ch, mn in active.values():
        ev.append((rec["dur_ms"], 1, bytes([0x80 | ch, mn, 0])))
    ev.sort(key=lambda e: (e[0], e[1]))

    def vlq(v):
        out = [v & 0x7F]
        v >>= 7
        while v:
            out.append(0x80 | (v & 0x7F))
            v >>= 7
        return bytes(reversed(out))

    trk, last = bytearray(), 0
    for t, _, data in ev:
        trk += vlq(t - last) + data
        last = t
    trk += vlq(0) + b"\xFF\x2F\x00"
    with open(path, "wb") as f:
        f.write(b"MThd" + struct.pack(">IHHH", 6, 0, 1, 1000))
        f.write(b"MTrk" + struct.pack(">I", len(trk)) + trk)


def summary(rec):
    ons = [e for e in rec["events"] if e[1] == "on"]
    tones = sorted({e[2] for e in ons})
    dur = rec["dur_ms"] / 1000
    return (f"时长 {int(dur // 60):02d}:{dur % 60:04.1f}，{len(ons)} 个音，音色 "
            f"{'、'.join(TONE_NAMES[t] for t in tones if t < 3) or '无'}，开始时效果器 {FX_NAMES[rec['fx0']]}、"
            f"音量 {[0, -6, -12, -18][rec['vol0']]} dB")


def save_and_render(pkt, rec, base):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(base + ".bin", "wb") as f:
        f.write(pkt)
    print("  " + summary(rec))
    t0 = time.time()
    pcm = render(rec)
    write_wav(base + ".wav", pcm)
    write_midi(base + ".mid", rec)
    print(f"  已保存（渲染用了 {time.time() - t0:.1f} 秒）：\n    {base}.wav\n    {base}.mid\n    {base}.bin")


# ================================ 串口（Windows） ================================
if os.name == "nt":
    import ctypes
    import winreg
    from ctypes import wintypes

    class _DCB(ctypes.Structure):
        _fields_ = [("DCBlength", wintypes.DWORD), ("BaudRate", wintypes.DWORD),
                    ("fBinary", wintypes.DWORD, 1), ("fParity", wintypes.DWORD, 1),
                    ("fOutxCtsFlow", wintypes.DWORD, 1), ("fOutxDsrFlow", wintypes.DWORD, 1),
                    ("fDtrControl", wintypes.DWORD, 2), ("fDsrSensitivity", wintypes.DWORD, 1),
                    ("fTXContinueOnXoff", wintypes.DWORD, 1), ("fOutX", wintypes.DWORD, 1),
                    ("fInX", wintypes.DWORD, 1), ("fErrorChar", wintypes.DWORD, 1),
                    ("fNull", wintypes.DWORD, 1), ("fRtsControl", wintypes.DWORD, 2),
                    ("fAbortOnError", wintypes.DWORD, 1), ("fDummy2", wintypes.DWORD, 17),
                    ("wReserved", wintypes.WORD), ("XonLim", wintypes.WORD), ("XoffLim", wintypes.WORD),
                    ("ByteSize", ctypes.c_ubyte), ("Parity", ctypes.c_ubyte), ("StopBits", ctypes.c_ubyte),
                    ("XonChar", ctypes.c_char), ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
                    ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char), ("wReserved1", wintypes.WORD)]

    class _TIMEOUTS(ctypes.Structure):
        _fields_ = [("ReadIntervalTimeout", wintypes.DWORD), ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
                    ("ReadTotalTimeoutConstant", wintypes.DWORD), ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
                    ("WriteTotalTimeoutConstant", wintypes.DWORD)]

    _K32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _K32.CreateFileW.restype = wintypes.HANDLE
    _K32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
                                 wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    _K32.ReadFile.argtypes = [wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
                              ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
    _K32.WriteFile.argtypes = [wintypes.HANDLE, wintypes.LPCVOID, wintypes.DWORD,
                               ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
    _K32.CloseHandle.argtypes = [wintypes.HANDLE]
    _K32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
    _K32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
    _K32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
    _K32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]


class SerialPort:
    """最简单的串口：9600 波特 8N1；read() 最多等 0.1 秒。"""

    def __init__(self, port):
        if os.name != "nt":
            import serial                                   # 非 Windows 系统用 pyserial
            self._s = serial.Serial(port, 9600, timeout=0.1)
            return
        self._s = None
        h = _K32.CreateFileW("\\\\.\\" + port, 0xC0000000, 0, None, 3, 0, None)
        if h is None or h == ctypes.c_void_p(-1).value:
            raise OSError(ctypes.get_last_error(), f"打不开 {port}")
        self.h = h
        dcb = _DCB()
        dcb.DCBlength = ctypes.sizeof(_DCB)
        if _K32.GetCommState(h, ctypes.byref(dcb)):
            dcb.BaudRate, dcb.ByteSize, dcb.Parity, dcb.StopBits = 9600, 8, 0, 0
            dcb.fBinary, dcb.fParity, dcb.fOutxCtsFlow, dcb.fOutxDsrFlow = 1, 0, 0, 0
            dcb.fOutX, dcb.fInX, dcb.fDtrControl, dcb.fRtsControl = 0, 0, 1, 1
            _K32.SetCommState(h, ctypes.byref(dcb))
        to = _TIMEOUTS(0xFFFFFFFF, 0xFFFFFFFF, 100, 0, 3000)
        _K32.SetCommTimeouts(h, ctypes.byref(to))
        _K32.PurgeComm(h, 0x000F)

    def read(self, n=512):
        if self._s is not None:
            return self._s.read(n)
        buf = ctypes.create_string_buffer(n)
        got = wintypes.DWORD(0)
        if not _K32.ReadFile(self.h, buf, n, ctypes.byref(got), None):
            raise OSError(ctypes.get_last_error(), "串口读出错")
        return buf.raw[:got.value]

    def write(self, data):
        if self._s is not None:
            self._s.write(data)
            return
        got = wintypes.DWORD(0)
        if not _K32.WriteFile(self.h, data, len(data), ctypes.byref(got), None):
            raise OSError(ctypes.get_last_error(), "串口写出错")

    def close(self):
        if self._s is not None:
            self._s.close()
        else:
            _K32.CloseHandle(self.h)


def list_ports():
    """[(COM 口, 是不是蓝牙串口)]，蓝牙的排在前面。"""
    ports = []
    if os.name == "nt":
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            i = 0
            while True:
                try:
                    name, val, _ = winreg.EnumValue(key, i)
                except OSError:
                    break
                ports.append((str(val), "Bth" in name or "BTH" in name.upper()))
                i += 1
        except OSError:
            pass
    ports.sort(key=lambda p: (not p[1], int("".join(c for c in p[0] if c.isdigit()) or 0)))
    return ports


def probe(port, seconds=4.0):
    """打开串口，发 'H'，收到 'k' 或录音包头说明是电子琴。返回 (串口, 已收到的数据) 或 None。"""
    try:
        s = SerialPort(port)
    except OSError as e:
        print(f"  {port}：打不开（{e.args[-1]}）")
        return None
    buf, t_end, last_h = bytearray(), time.time() + seconds, 0.0
    try:
        while time.time() < t_end:
            if time.time() - last_h > 0.3:
                s.write(b"H")
                last_h = time.time()
            buf += s.read(256)
            if b"k" in buf or b"MPR1" in buf:
                return s, buf
    except OSError:
        pass
    print(f"  {port}：没有回应")
    s.close()
    return None


def connect(port_arg):
    while True:
        cands = [port_arg] if port_arg else [p for p, bt in list_ports() if bt] or [p for p, _ in list_ports()]
        if not cands:
            print("没有找到蓝牙串口：请先在 Windows“设置 → 蓝牙和其他设备”里添加 HC-05（配对密码 1234），"
                  "或者用 --port COMx 指定。10 秒后重试……")
        for p in cands:
            print(f"正在连接 {p}（蓝牙连接可能要十几秒）……")
            r = probe(p, 8.0 if port_arg else 5.0)
            if r:
                print(f"已连接电子琴（{p}）。在电子琴上按 key[2] 开始录音，再按一次停止，录音会自动传过来。")
                return r[0], r[1]
        if cands:
            print("都没有回应：检查 HC-05 是否通电（灯在闪）、TXD/RXD 是否接对、电子琴是否烧了新程序。10 秒后重试……")
        time.sleep(10)


def receive_forever(port_arg):
    while True:
        ser, buf = connect(port_arg)
        last_h, last_rx, shown = 0.0, time.time(), ""
        try:
            while True:
                now = time.time()
                if now - last_h >= 0.5:                     # 心跳：告诉 FPGA 电脑在线
                    ser.write(b"H")
                    last_h = now
                chunk = ser.read(512)
                if chunk:
                    buf += chunk
                    last_rx = now
                i = buf.find(b"MPR1")
                if i < 0:
                    del buf[:max(0, len(buf) - 3)]
                    if now - last_rx > 6:
                        raise OSError(0, "6 秒没有收到电子琴的回应")
                    continue
                del buf[:i]
                if len(buf) < 16:
                    continue
                n_ev = struct.unpack(">I", buf[4:8])[0]
                if n_ev > 16384:                            # 不是真的包头
                    del buf[:4]
                    continue
                total = 20 + 4 * n_ev
                msg = f"\r正在接收录音：{min(100, 100 * len(buf) // total):3d}%"
                if msg != shown:
                    print(msg, end="", flush=True)
                    shown = msg
                if len(buf) >= total:
                    pkt = bytes(buf[:total])
                    del buf[:total]
                    shown = ""
                    rec, err = parse_packet(pkt)
                    if rec is None:
                        print(f"\n  数据有误（{err}），请电子琴重发……")
                        ser.write(b"R")
                        continue
                    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    print(f"\n收到录音 {stamp}")
                    save_and_render(pkt, rec, os.path.join(OUT_DIR, f"录音_{stamp}"))
                    print("继续等待下一段录音（Ctrl+C 退出）……")
                    last_rx = time.time()
                elif now - last_rx > 5:
                    print("\n  接收中断，请电子琴重发……")
                    buf.clear()
                    ser.write(b"R")
                    last_rx = now
        except OSError as e:
            print(f"\n蓝牙连接断开（{e.args[-1]}），重新连接……")
            try:
                ser.close()
            except OSError:
                pass
            time.sleep(2)


# ================================ 示范 ================================
def demo_recording():
    """中音区 C 大调音阶：钢琴（干声）→ 电吉他（混响）→ 大提琴（合唱，带和弦）。"""
    ev, t = [], 300
    scale = [15, 17, 19, 20, 22, 24, 26, 27]           # C4 D4 E4 F4 G4 A4 B4 C5（音序号）
    for tone in (0, 1, 2):
        if tone == 1:
            ev.append((t, "fx", 1, 0))
        if tone == 2:
            ev.append((t, "fx", 3, 0))
        hold = 900 if tone == 2 else 350
        for nt in scale:
            ev.append((t, "on", tone, nt))
            ev.append((t + hold, "off", 0, nt))
            t += hold + 100
        t += 600
    for nt in (15, 19, 22):                            # 大提琴和弦 C E G
        ev.append((t, "on", 2, nt))
    for nt in (15, 19, 22):
        ev.append((t + 2500, "off", 0, nt))
    return dict(dur_ms=t + 3000, fx0=0, vol0=2, events=ev)


def main():
    ap = argparse.ArgumentParser(description="接收电子琴的蓝牙录音，生成 WAV / MIDI")
    ap.add_argument("--port", help="蓝牙串口，例如 COM5（不写就自动找）")
    ap.add_argument("--render", help="把保存的 .bin 原始数据重新渲染")
    ap.add_argument("--demo", action="store_true", help="不连硬件，渲染一段示范")
    args = ap.parse_args()

    if args.demo:
        rec = demo_recording()
        pkt = make_packet(rec)
        rec2, err = parse_packet(pkt)
        assert rec2 is not None, err
        print("渲染示范录音：")
        save_and_render(pkt, rec2, os.path.join(OUT_DIR, "示范"))
    elif args.render:
        with open(args.render, "rb") as f:
            pkt = f.read()
        rec, err = parse_packet(pkt)
        if rec is None:
            raise SystemExit(f"文件不对：{err}")
        print(f"重新渲染 {args.render}：")
        save_and_render(pkt, rec, os.path.splitext(args.render)[0])
    else:
        try:
            receive_forever(args.port)
        except KeyboardInterrupt:
            print("\n已退出。")


if __name__ == "__main__":
    main()
