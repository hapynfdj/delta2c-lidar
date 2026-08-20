#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_delta2c_log.py — DELTA-2C 原始日志分析器（协议 ver 0x13）

正确解码（2026-08-20 双雷达 + 手/胸标注日志实测确认）:
    帧:  AA | len(u16 BE, 不含校验) | 0x13 | 0x61 | 0xAD | dlen(u16)
          | freq_x20 | off(i16) | start_angle(u16) | end_angle(u16)
          | N x [quality, d_hi, d_lo] | checksum(u16 BE = 前导字节和)
    采样: q=第1字节;  distance_mm = ((d_hi<<8)|d_lo) * 0.5   (2C 用 0.5mm/格)
    d==0 且 q!=0  -> 无回波;   d==0 且 q==0 -> 空槽

用法:
    python analyze_delta2c_log.py <raw.log> [raw2.log ...]

输出: 帧/校验统计、转速、全局距离直方图(米)、每 0.5s 最近/中位/最远、
      帧内"平直段"(>=5 连续采样, 距离散布<8%, 0.15-8m) —— 手/胸/墙这类面目标。
"""
import re
import sys
from collections import Counter, defaultdict

HEXLINE = re.compile(r"([0-9A-Fa-f]{2} ?)+")
TSLINE = re.compile(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]#")


def load_chunks(path):
    """Return list of (ts_seconds, bytes) preserving the log's time stamps."""
    out = []
    cur_ts = None
    cur = bytearray()
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            m = TSLINE.match(ln)
            if m:
                hms, ms = m.group(1).split(" ")[1].split(".")
                h, mi, sec = (int(x) for x in hms.split(":"))
                ts = h * 3600 + mi * 60 + sec + int(ms) / 1000.0
                cur_ts = ts
            if HEXLINE.fullmatch(s):
                if cur_ts is None:
                    cur_ts = 0.0
                cur.extend(int(x, 16) for x in s.split())
            else:
                if cur:
                    out.append((cur_ts if cur_ts is not None else 0.0, bytes(cur)))
                    cur = bytearray()
                    cur_ts = None
        if cur:
            out.append((cur_ts if cur_ts is not None else 0.0, bytes(cur)))
    return out


def parse_all(data):
    """Parse one byte blob -> (frames, bad, unknown, speed_frames).
    frames: list of dict(xx, st, en, pts=[(q, d_mm)])"""
    frames, bad, unknown, speeds = [], 0, 0, []
    i, n = 0, len(data)
    while i < n:
        if data[i] != 0xAA:
            i += 1
            continue
        if i + 3 > n:
            break
        plen = (data[i + 1] << 8) | data[i + 2]
        if not (6 <= plen <= 1024):
            i += 1
            continue
        total = plen + 2
        if i + total > n:
            break
        fr = data[i:i + total]
        i += total
        chk = (fr[plen] << 8) | fr[plen + 1]
        if sum(fr[:plen]) & 0xFFFF != chk:
            bad += 1
            continue
        if fr[3] != 0x13 or fr[4] != 0x61 or fr[5] not in (0xAD, 0xAE):
            unknown += 1
            continue
        xx = fr[8] if plen > 8 else 0
        if fr[5] == 0xAE:
            speeds.append(xx)
            continue
        if plen < 15:
            unknown += 1
            continue
        st = (fr[11] << 8) | fr[12]
        en = (fr[13] << 8) | fr[14]
        dlen = (fr[6] << 8) | fr[7]
        nmax = (dlen - 7) // 3 if dlen >= 7 else 0
        pts = []
        for k in range(nmax):
            p = 15 + 3 * k
            if p + 2 >= plen:
                break
            q = fr[p]
            d_mm = ((fr[p + 1] << 8) | fr[p + 2]) * 0.5   # 0.5mm/unit (2C)
            pts.append((q, d_mm))
        frames.append(dict(xx=xx, st=st, en=en, pts=pts))
    return frames, bad, unknown, speeds


def hz_of(xx_list):
    if not xx_list:
        return 0.0
    c = Counter(xx_list)
    return c.most_common(1)[0][0] / 20.0


def analyze(path):
    print("=" * 66)
    print("文件: %s" % path)
    chunks = load_chunks(path)
    all_frames = []
    for ts, blob in chunks:
        frames, bad, unknown, speeds = parse_all(blob)
        for f in frames:
            f["ts"] = ts
            all_frames.append(f)
    nf = len(all_frames)
    if nf == 0:
        print("  未解析到任何 0xAD 测距帧")
        return
    nbad = sum(1 for _, blob in chunks for _ in [1] if False)  # placeholder
    # recompute bad/unknown totals:
    tot_bad = tot_unk = 0
    for _, blob in chunks:
        _, b, u, _ = parse_all(blob)
        tot_bad += b
        tot_unk += u
    xs = [f["xx"] for f in all_frames]
    print("  测距帧: %d   校验失败: %d   未知帧: %d   转速: %.2f Hz"
          % (nf, tot_bad, tot_unk, hz_of(xs)))
    nq = sum(1 for f in all_frames for q, d in f["pts"] if q != 0 and d > 0)
    nz = sum(1 for f in all_frames for q, d in f["pts"] if d == 0)
    print("  有效采样(q!=0,d>0): %d   零距离采样: %d" % (nq, nz))

    # 全局距离直方图 (米, 0.25m 桶, 只统计 d>0 且 d<20m)
    hist = Counter()
    for f in all_frames:
        for q, d in f["pts"]:
            if d <= 0 or d >= 20000:
                continue
            hist[int(d / 250)] += 1
    print("  距离直方图(米):")
    for b in sorted(hist):
        if hist[b] >= max(1, nq // 200):
            lo, hi = b * 0.25, (b + 1) * 0.25
            bar = "#" * min(60, hist[b] * 60 // max(1, max(hist.values())))
            print("    %4.2f-%4.2f m %6d %s" % (lo, hi, hist[b], bar))

    # 每 0.5s 窗口: 最近/中位/最远 (有效 d)
    win = defaultdict(list)
    for f in all_frames:
        w = int(f["ts"] * 2) / 2.0
        for q, d in f["pts"]:
            if d > 0:
                win[w].append(d / 1000.0)
    if win:
        print("  时间轴(每 0.5s: 最近/中位/最远, 米):")
        for w in sorted(win):
            v = sorted(win[w])
            if not v:
                continue
            print("    t=%6.1fs  n=%4d  最近=%5.2f  中位=%5.2f  最远=%5.2f"
                  % (w, len(v), v[0], v[len(v) // 2], v[-1]))

    # 帧内平直段 (面目标: 手/胸/墙)
    print("  平直段事件 (>=5 连续采样, 散布<8%, 0.15-8m):")
    shown = 0
    for f in all_frames:
        pts = f["pts"]
        i = 0
        while i < len(pts):
            if pts[i][1] <= 0:
                i += 1
                continue
            j = i
            while j + 1 < len(pts) and pts[j + 1][1] > 0:
                j += 1
            run = [d / 1000.0 for _, d in pts[i:j + 1]]
            if len(run) >= 5 and 0.15 <= min(run) and max(run) <= 8.0 \
                    and (max(run) - min(run)) / min(run) < 0.08:
                st_a = f["st"] * 0.01
                en_a = f["en"] * 0.01
                n = len(pts)
                a0 = st_a + (en_a - st_a) * (i + 0.5) / n
                a1 = st_a + (en_a - st_a) * (j + 0.5) / n
                print("    t=%6.1fs 角度%6.1f-%-6.1f° 距离 %5.2f-%.2f m (n=%d)"
                      % (f["ts"], a0, a1, min(run), max(run), len(run)))
                shown += 1
                if shown >= 40:
                    print("    ... (更多略)")
                    return
            i = j + 1
    if shown == 0:
        print("    (无)")


if __name__ == "__main__":
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        sys.exit(1)
    for p in paths:
        analyze(p)
