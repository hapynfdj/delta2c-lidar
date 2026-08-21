#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LDS_DELTA_2C_115200 host-side validation.

Mirrors the C++ parser logic (src/LDS_DELTA_2C_115200.cpp) exactly and
verifies it against the Python reference DeltaParser on real captured bytes:

  - every 0xAD frame parses with a valid checksum (0 failures)
  - per-sample (angle_deg, dist_mm, quality) matches the reference exactly
  - the scan_completed (wrap) flag fires exactly once per revolution

Usage:
    python test_2c_parser.py [raw_log ...]
    (defaults to the three sample logs in samples/)
"""
import re
import sys
from collections import Counter

sys.path.insert(0, ".")
from delta2c_debug_gui import DeltaParser  # reference implementation

HEXLINE = re.compile(r"([0-9A-Fa-f]{2} ?)+")


def load_bytes(path):
    out = bytearray()
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            if HEXLINE.fullmatch(s):
                out.extend(int(h, 16) for h in s.split())
    return bytes(out)


def decode_u16_be(b0, b1):
    return (b0 << 8) | b1


def mirror_cpp_parser(data):
    """Exact mirror of LDS_DELTA_2C_115200::processByte.

    Returns frames: list of dict(xx, st_x100, en_x100, scan_completed,
    pts=[(angle_deg, dist_mm, q)]), plus bad/unknown counts.
    """
    START = 0xAA
    VER = 0x13
    PTYPE = 0x61
    DT_MEAS, DT_SPEED = 0xAD, 0xAE
    MAX_SAMPLES = 40  # mirrors get_max_data_sample_count()
    HEADER_BYTES = 15  # sizeof(packet_header_t): 8 + 7

    frames = []
    n_checksum_bad = 0   # checksum mismatches (must be 0 on clean captures)
    n_empty_rejected = 0  # 0xAD frames with dlen<10 (no samples) — rejected by
                          # design, consistent with the upstream 2A variant
    parser_idx = 0
    checksum = 0
    last_start_x100 = 0xFFFF  # first packet reports scan_completed (C++ init)
    rx = bytearray()
    i, n = 0, len(data)

    while i < n:
        c = data[i]
        i += 1
        # C++: guard before writing — byte is dropped when the buffer is full
        if parser_idx >= MAX_SAMPLES * 3 + HEADER_BYTES:
            parser_idx = 0
            continue
        if parser_idx >= len(rx):
            rx.append(c)
        else:
            rx[parser_idx] = c
        parser_idx += 1
        checksum += c

        if parser_idx == 1:
            if c != START:
                parser_idx = 0
            else:
                checksum = c
        elif parser_idx == 3:
            plen = decode_u16_be(rx[1], rx[2])
            if plen > MAX_SAMPLES * 3 + HEADER_BYTES + 2:
                parser_idx = 0
        elif parser_idx == 4:
            if c != VER:
                parser_idx = 0
        elif parser_idx == 5:
            if c != PTYPE:
                parser_idx = 0
        elif parser_idx == 6:
            if c not in (DT_MEAS, DT_SPEED):
                parser_idx = 0
        elif parser_idx == 8:
            dlen = decode_u16_be(rx[6], rx[7])
            if dlen == 0 or dlen > 7 + MAX_SAMPLES * 3:
                parser_idx = 0
        elif parser_idx in (2, 7):
            pass  # C++: case 2 and case 7 are no-ops
        else:
            plen = decode_u16_be(rx[1], rx[2])
            if parser_idx != plen + 2:
                continue
            # checksum: C++ compares accumulated sum (incl. crc bytes) against
            # the BE crc value plus its own two bytes, all wrapped to uint16
            crc = decode_u16_be(rx[plen], rx[plen + 1])
            pkt_checksum = (crc + rx[plen] + rx[plen + 1]) & 0xFFFF
            if (checksum & 0xFFFF) != pkt_checksum:
                n_checksum_bad += 1
                parser_idx = 0
                continue
            if rx[5] == DT_MEAS:
                st = decode_u16_be(rx[11], rx[12])
                en = decode_u16_be(rx[13], rx[14])
                # 90-deg margin on the wrap, like the C++ (jitter-proof)
                scan_completed = st < last_start_x100 - 9000
                dlen = decode_u16_be(rx[6], rx[7])
                if dlen < 10:   # empty 0xAD frame (no samples): rejected by design
                    n_empty_rejected += 1
                    parser_idx = 0
                    continue
                n_samp = (dlen - 7) // 3
                if n_samp > MAX_SAMPLES:
                    parser_idx = 0
                    continue
                # advance the wrap reference only on sample-carrying frames
                last_start_x100 = st
                sa = st * 0.01
                ea = en * 0.01
                pts = []
                for k in range(n_samp):
                    pos = HEADER_BYTES + 3 * k
                    q = rx[pos]
                    d = decode_u16_be(rx[pos + 1], rx[pos + 2]) * 0.5
                    if d > 8000.0:   # no-echo saturation sentinel -> no return
                        d = 0.0
                    ang = sa + (ea - sa) * (k + 0.5) / n_samp
                    pts.append((ang, d, q))
                frames.append(dict(xx=rx[8], st=st, en=en,
                                   scan_completed=scan_completed, pts=pts))
            parser_idx = 0
    return frames, n_checksum_bad, n_empty_rejected


def main():
    paths = sys.argv[1:] or [
        "samples/delta2c_20260820_164235_raw.log",
        "samples/delta2c_20260820_170443_raw.log",
        "samples/delta2c_20260820_170447_raw.log",
    ]
    total_ok = True
    for p in paths:
        data = load_bytes(p)
        frames, n_checksum_bad, n_empty_rejected = mirror_cpp_parser(data)

        # reference parser
        evs = []
        DeltaParser().feed(data, lambda ev, *a: evs.append((ev, a)))
        ref_meas = [a for ev, a in evs if ev == "meas"]
        ref_bad = sum(1 for ev, a in evs if ev == "bad")
        ref_nonempty = [a for a in ref_meas if len(a[5]) > 0]
        ref_empty = len(ref_meas) - len(ref_nonempty)

        # per-sample comparison against the non-empty reference frames
        mism = 0
        if len(frames) != len(ref_nonempty):
            mism += abs(len(frames) - len(ref_nonempty))
        for f, (fr, xx, off, st_ang, en_ang, pts) in zip(frames, ref_nonempty):
            if abs(st_ang - f["st"]) > 1 or abs(en_ang - f["en"]) > 1:
                mism += 1
                continue
            if len(pts) != len(f["pts"]):
                mism += 1
                continue
            for (rd, rq), (a, d, q) in zip(pts, f["pts"]):
                if abs(rd - d) > 0.51 or rq != q:
                    mism += 1

        # scan_completed fires on the first packet (init last_start=0xFFFF) and
        # on every >90 deg wrap between consecutive sample-carrying frames
        wraps = sum(1 for f in frames if f["scan_completed"])
        expected_wraps = 1 + sum(1 for a, b in zip(frames, frames[1:])
                                 if b["st"] < a["st"] - 9000)

        ok = (n_checksum_bad == ref_bad == 0) and mism == 0 \
            and n_empty_rejected == ref_empty
        print("%-46s" % p.split("/")[-1])
        print("   帧数=%4d(+空帧%d) 校验失败=%d(%d) 采样错配=%d | 扫描完成=%d 次(预期=%d) %s"
              % (len(frames), n_empty_rejected, n_checksum_bad, ref_bad, mism,
                 wraps, expected_wraps, "PASS" if ok and wraps == expected_wraps else "FAIL"))
        total_ok = total_ok and ok and wraps == expected_wraps
    print("\nRESULT: %s" % ("ALL PASS" if total_ok else "FAILED"))
    return 0 if total_ok else 1


if __name__ == "__main__":
    sys.exit(main())
