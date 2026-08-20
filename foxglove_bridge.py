#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Foxglove bridge for the Camsense DELTA-2C PRO lidar
====================================================
Parses the DELTA-2C serial stream (or replays a saved raw log) and publishes
standard `foxglove.LaserScan` messages over the Foxglove WebSocket protocol
(server subprotocol `foxglove.sdk.v1`, negotiated automatically by Foxglove
Studio). Visualize with Foxglove Studio — free, GPU-rendered (WebGL),
professional RViz-style viewer. No custom map UI needed.

Usage:
    # live from the lidar (115200):
    python foxglove_bridge.py --port COM5

    # replay a saved raw log (no hardware; loops until Ctrl+C):
    python foxglove_bridge.py --log logs/delta2c_20260820_164235_raw.log

Then in Foxglove Studio:
    Open Connection -> WebSocket -> ws://127.0.0.1:8765
    Add a "2D" panel -> LaserScan (topic /scan), or 3D panel -> LaserScan.

Each published message is one full revolution as a 360-ray LaserScan:
angle 0 = robot front (+x in Foxglove), ranges in meters, intensities = quality.
Invalid samples (q == 0) are set beyond range_max (no return).
"""
import argparse
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from delta2c_debug_gui import DeltaParser  # noqa: E402

N_RAYS = 360            # LaserScan resolution per revolution
RANGE_MAX_M = 12.0      # beyond this = no return (invalid sample)
CW = True               # flip to False if the map is mirrored in Foxglove


class ScanAssembler:
    """Accumulate one revolution of points, emit as a 360-ray LaserScan."""

    def __init__(self):
        self.pts = []
        self.last_st = None

    def add_frame(self, st_ang, en_ang, pts):
        st = st_ang * 0.01
        en = en_ang * 0.01
        n = len(pts)
        if n == 0:
            return None
        wrapped = self.last_st is not None and st < self.last_st - 90
        self.last_st = st
        for k, (d, q) in enumerate(pts):
            if q == 0 or d == 0:   # d==0 且 q!=0 也是无回波
                continue
            a = (st + (en - st) * (k + 0.5) / n) % 360.0
            self.pts.append((a, d, q))
        if wrapped or len(self.pts) > 500:
            scan, self.pts = self.pts, []
            return scan
        return None

    @staticmethod
    def to_laserscan(scan, sec, nsec):
        from foxglove.messages import LaserScan, Timestamp
        ranges = [RANGE_MAX_M + 1.0] * N_RAYS   # > range_max => no return
        intensities = [0.0] * N_RAYS
        step = 360.0 / N_RAYS
        for a_deg, d_mm, q in scan:
            a = a_deg if CW else (360.0 - a_deg) % 360.0
            idx = int(round(a / step)) % N_RAYS
            ranges[idx] = min(d_mm / 1000.0, RANGE_MAX_M)
            intensities[idx] = float(q)
        return LaserScan(
            timestamp=Timestamp(sec=sec, nsec=nsec),
            frame_id="lidar",
            start_angle=0.0,
            end_angle=2.0 * math.pi,
            ranges=ranges,
            intensities=intensities,
        )


def read_raw_log(path):
    """Yield the bytes of a saved *_raw.log (RECV HEX/N <<< format)."""
    hexlines = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            if re.fullmatch(r"([0-9A-F]{2} ?)+", s):
                hexlines.append(s)
    return bytes(int(h, 16) for h in " ".join(hexlines).split())


def latest_log():
    """Newest logs/delta2c_*_raw.log, or None."""
    import glob
    logs = sorted(glob.glob(os.path.join(LOG_DIR, "delta2c_*_raw.log")))
    return logs[-1] if logs else None


LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def main():
    ap = argparse.ArgumentParser(description="Stream DELTA-2C lidar as foxglove.LaserScan")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port, e.g. COM5 (live)")
    src.add_argument("--log", help="replay a saved *_raw.log (loops); use 'latest' for the newest")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port-ws", type=int, default=8765, help="Foxglove WebSocket port")
    ap.add_argument("--once", action="store_true", help="replay log only once")
    ns = ap.parse_args()

    # auto-select source if neither given
    if not ns.port and not ns.log:
        try:
            import serial.tools.list_ports as lp
            ports = [p.device for p in lp.comports()]
        except Exception:
            ports = []
        if len(ports) == 1:
            ns.port = ports[0]
            print("auto-selected serial port: %s" % ns.port)
        elif len(ports) > 1:
            print("multiple serial ports found: %s — pass --port explicitly" % ports)
            sys.exit(1)
        else:
            print("no serial port found — replaying the newest saved log instead")
            ns.log = "latest"
    if ns.log == "latest":
        ns.log = latest_log()
        if not ns.log:
            print("no saved logs found in %s — connect the lidar and use --port COMx" % LOG_DIR)
            sys.exit(1)
        print("replaying newest log: %s" % ns.log)

    try:
        import foxglove
        from foxglove.channels import LaserScanChannel
    except ImportError:
        print("foxglove-sdk missing: pip install foxglove-sdk")
        sys.exit(1)

    foxglove.set_log_level("WARN")
    server = foxglove.start_server(host=ns.host, port=ns.port_ws)
    channel = LaserScanChannel("/scan")
    assembler = ScanAssembler()
    print("Foxglove server: ws://%s:%d   topic /scan (foxglove.LaserScan)"
          % (ns.host, ns.port_ws))
    print("In Foxglove Studio: Open Connection -> WebSocket -> ws://%s:%d"
          % (ns.host, ns.port_ws))
    print("Then add panel: 2D -> LaserScan  (or 3D -> LaserScan)")

    def on_meas(st_ang, en_ang, pts):
        nonlocal last_print
        scan = assembler.add_frame(st_ang, en_ang, pts)
        if scan is not None:
            now = time.time()
            sec = int(now)
            nsec = int((now - sec) * 1e9)
            channel.log(assembler.to_laserscan(scan, sec, nsec))
            if now - last_print > 2.0:   # throttle console output
                n_valid = len([p for p in scan if p[2] != 0])
                print("scan: %d pts (%d valid)" % (len(scan), n_valid), flush=True)
                last_print = now

    last_print = 0.0
    parser = DeltaParser()
    handler = lambda ev, *a: on_meas(a[3], a[4], a[5]) if ev == "meas" else None

    if ns.log:
        data = read_raw_log(ns.log)
        print("replaying %d bytes from %s (Ctrl+C to stop)" % (len(data), ns.log))
        try:
            while True:
                parser.feed(data, handler)
                if ns.once:
                    break
                time.sleep(0.5)  # let clients pull the buffered scans, then loop
        except KeyboardInterrupt:
            pass
        server.stop()
        return 0

    # live serial mode
    try:
        import serial
    except ImportError:
        print("pyserial missing: pip install pyserial")
        sys.exit(1)
    try:
        ser = serial.Serial(ns.port, ns.baud, timeout=0.05)
    except Exception as e:
        print("cannot open %s: %s" % (ns.port, e))
        server.stop()
        sys.exit(1)
    print("reading %s @ %d ..." % (ns.port, ns.baud))
    try:
        while True:
            n = ser.in_waiting
            if n == 0:
                time.sleep(0.01)
                continue
            data = ser.read(n)
            parser.feed(data, handler)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
