#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camsense DELTA-2C PRO Lidar Debug Tool  (v3)
=============================================
Xiaomi Robot Vacuum 2 teardown lidar (Camsense/3irobotix DELTA-2C PRO ODM, UART 115200).

Features:
  - Serial port / baud selection, live DELTA-2C protocol parsing
  - Big live rotor-speed readout (Hz), divisor 20/40 switchable
  - Auto-detects 0xAE (speed-only) vs 0xAD (measurement+angle) frames
  - Real-time 2D top-down map (RViz-style): robot centered, up = front,
    1 m grid, room outline polyline, previous-revolution gray overlay
  - "Data acquired" notification when 0xAD measurement frames appear
  - Raw hex log + parsed log (bilingual per UI language)
  - UI language switch: 中文 / English

DELTA-2C frame formats (verified byte-exact against real captures, 618/618 frames):
  0xAE speed-only (11 bytes):
    AA | len=9 | ver=0x13 | type=0x61 | AE | dlen=1 | xx | chk(u16 BE)
    Hz = xx / 20
  0xAD measurement (len = 8 + 7+3N + 2):
    AA | len | ver=0x13 | 61 | AD | dlen=7+3N | xx | off(i16) | start_angle(u16)
    | end_angle(u16) | N x [q d_hi d_lo] | chk(u16 BE)
    q = quality (0 = invalid); dist_mm = ((d_hi<<8)|d_lo) * 0.5  (2C 实测 0.5mm/格)
    d == 0 -> 无回波;  d > 8000mm (1号机无回波饱和 ~0x3FE0 = 8.17m) -> 归零当无回波
    sample k angle = start + (end-start)*(k+0.5)/N ; 16 packets/revolution (measured)
  Checksum = 16-bit sum of every byte before the 2-byte checksum (big-endian).
  Differences vs kaiaai 2A/2B/2D/2G: ver=0x13, extra end_angle field, 采样结构相同
  ([quality, dist_u16 BE]), 但 2C 比例是 0.5mm/格 而非 kaiaai 的 0.25 —— 2026-08-20
  双雷达实测 + 手/胸标注日志校准确认; kaiaai 代码对 2C 直接套 0.25 会差一倍。

Author note: reverse-engineered from user serial captures, 2026-08-20.
License: MIT
"""
import os
import sys
import time
import math
import threading
import queue
import datetime
import collections

import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None

try:
    import winsound
except Exception:
    winsound = None

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAS_MPL = True
except Exception:
    HAS_MPL = False

if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

# ---------------- protocol constants ----------------
START = 0xAA
PTYPE = 0x61
DT_MEAS = 0xAD
DT_SPEED = 0xAE
VERS_OK = (0x01, 0x13)          # 2C PRO uses 0x13
TARGET_BAND = (4.0, 8.5)        # Hz window where measurement frames appear (measured)
DIVISORS = (20, 40)             # Hz = xx / divisor; 20 = kaiaai convention (verified)
FIXED_RANGE_M = 12.0            # scan view fixed range when auto-range is off

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")

# ---------------- i18n ----------------
STRINGS = {
    "zh": {
        "title": "Camsense DELTA-2C PRO 调试助手 v3",
        "port": "串口:", "refresh": "刷新", "baud": "波特率:", "divisor": "转速系数:",
        "divisor_tip": "Hz=xx/系数。默认20(与kaiaai一致,实测2.9V≈7.9Hz);若转子10秒只转~40圈则改40。仅影响Hz显示,不影响扫描图。",
        "connect": "连接", "disconnect": "断开",
        "log": "日志:", "open_logs": "打开日志文件夹", "clear": "清空显示",
        "banner_idle": "未连接 — 请选择串口后点击“连接”",
        "banner_connected": "已连接 {port} @ {baud} — 等待数据…",
        "banner_speed": "正在接收纯转速帧(0xAE) — 当前 {hz:.2f} Hz — 继续调节电压, 目标 {lo}~{hi} Hz",
        "banner_in_band": "转速 {hz:.2f} Hz 在目标区间({lo}~{hi}Hz) — 保持/微调, 等待测距帧(0xAD)",
        "banner_noise": "收到大量无法解析的字节 — 检查波特率(115200)/TX-RX 接线/共地",
        "banner_ok": "✓ 已成功获得数据!0xAD 测距帧正在输出, 可以正常使用了",
        "panel_values": "实时数值", "panel_trend": "转速趋势 (最近 60 秒, 绿色=测距帧, 目标 {lo}~{hi} Hz)",
        "panel_scan": "2D 激光点云 (RViz 风格: 黑底, 颜色=距离, 上=前)",
        "scan_accum": "累积模式", "dir_label": "扫描方向:",
        "dir_cw": "顺时针", "dir_ccw": "逆时针",
        "scan_legend_cur": "本圈(颜色=距离)", "scan_legend_recent": "最新段", "scan_legend_prev": "上一圈",
        "auto_range": "自动缩放(关=固定12m)",
        "dir_front": "前", "dir_back": "后", "dir_left": "左", "dir_right": "右",
        "hz": "当前转速", "dtype": "数据类型", "last_meas": "最后测距帧",
        "scan_points": "扫描点数", "nearest": "最近点",
        "xx_fmt": "xx={xx}  (Hz=xx/{div}; 若系数{other} → {hz2:.2f} Hz)",
        "cnt_fmt": "总 {total} | 转速 {speed} | 测距 {meas} | 校验错 {bad} | 杂散 {noise}",
        "meas_fmt": "{n}点 起{st:.1f}°→终{en:.1f}° / d={d:.0f}mm q={q}",
        "nearest_far_fmt": "最近 {nd:.2f}m @{na:.0f}° · 最远 {fd:.2f}m",
        "dtype_speed": "0xAE 纯转速", "dtype_meas": "0xAD 测距帧 ✓", "dtype_none": "--",
        "mpl_missing": "未安装 matplotlib, 无法显示扫描图。\npip install matplotlib",
        "tip": "操作: ① 选串口→连接 ② 调可调电源 M+/M- 电压(实测 2.9V≈7.9Hz 即出测距帧) ③ 看 2D 地图: 中心=雷达, 上=前(箭头), 蓝线=本圈房间轮廓, 灰线=上一圈, 1格=1米 ④ 移动雷达后灰蓝两圈会错开。若地图左右镜像, 把“扫描方向”切到另一项。",
        "panel_log": "运行日志(解析)",
        "msg_ok_title": "已成功获得数据",
        "msg_ok_body": "0xAD 测距帧已出现!\n转速 {hz:.2f} Hz\n每帧 {n} 个采样点({valid} 个有效)\n\n可以正常使用了。",
        "msg_err_dep": "缺少依赖", "msg_err_dep_body": "未安装 pyserial, 请运行: pip install pyserial",
        "msg_warn_port": "提示", "msg_warn_port_body": "请先选择串口",
        "msg_err_conn": "连接失败", "msg_err_conn_body": "{err}\n\n检查串口号/占用/接线",
        "msg_info_logdir": "日志目录",
        "L_speed": "0xAE speed | len={l} ver=0x{v:02X} xx={xx} | Hz={hz:.2f}",
        "L_meas": "0xAD meas | len={l} ver=0x{v:02X} xx={xx} Hz={hz:.2f} | N={n} start={s:.2f}° end={e:.2f}° | first_valid d={d:.1f}mm q={q}",
        "L_noise": "stray bytes {n} (boot burst / wrong baud?)",
        "L_bad": "checksum FAIL chk=0x{c:04X} sum=0x{s:04X} len={l} | {hx}",
        "L_unknown": "unknown frame (not DELTA ver/type) len={l} | {hx}",
        "L_conn": "=== Connected {port} @ {baud} ===",
        "L_disc": "=== Disconnected ===",
        "L_err": "serial error: {e}",
    },
    "en": {
        "title": "Camsense DELTA-2C PRO Lidar Debug Tool v3",
        "port": "Port:", "refresh": "Refresh", "baud": "Baud:", "divisor": "Speed div:",
        "divisor_tip": "Hz = xx / divisor. Default 20 (kaiaai convention; 2.9V measured ~7.9Hz). If the rotor makes only ~40 turns in 10 s, use 40. Affects the Hz readout only, not the scan.",
        "connect": "Connect", "disconnect": "Disconnect",
        "log": "Log:", "open_logs": "Open log folder", "clear": "Clear",
        "banner_idle": "Not connected — pick a port and press Connect",
        "banner_connected": "Connected {port} @ {baud} — waiting for data…",
        "banner_speed": "Receiving speed frames (0xAE) — {hz:.2f} Hz — adjust voltage, target {lo}~{hi} Hz",
        "banner_in_band": "{hz:.2f} Hz is in the target band ({lo}~{hi}Hz) — hold/tweak, waiting for 0xAD frames",
        "banner_noise": "Lots of unparseable bytes — check baud (115200)/TX-RX wiring/ground",
        "banner_ok": "✓ Data acquired! 0xAD measurement frames are flowing — ready to use",
        "panel_values": "Live values", "panel_trend": "Speed trend (last 60 s, green = meas frame, target {lo}~{hi} Hz)",
        "panel_scan": "2D laser point cloud (RViz style: dark bg, color = distance, up = front)",
        "scan_accum": "Accumulate", "dir_label": "Sweep dir:",
        "dir_cw": "CW", "dir_ccw": "CCW",
        "scan_legend_cur": "current rev. (color = distance)", "scan_legend_recent": "latest seg.", "scan_legend_prev": "previous rev.",
        "auto_range": "Auto range (off = fixed 12 m)",
        "dir_front": "F", "dir_back": "B", "dir_left": "L", "dir_right": "R",
        "hz": "Rot. speed", "dtype": "Data type", "last_meas": "Last meas frame",
        "scan_points": "Scan points", "nearest": "Nearest",
        "xx_fmt": "xx={xx}  (Hz=xx/{div}; if /{other} → {hz2:.2f} Hz)",
        "cnt_fmt": "total {total} | speed {speed} | meas {meas} | bad {bad} | stray {noise}",
        "meas_fmt": "{n}pts {st:.1f}°→{en:.1f}° / d={d:.0f}mm q={q}",
        "nearest_far_fmt": "nearest {nd:.2f}m @{na:.0f}° · farthest {fd:.2f}m",
        "dtype_speed": "0xAE speed-only", "dtype_meas": "0xAD measurement ✓", "dtype_none": "--",
        "mpl_missing": "matplotlib not installed, scan view unavailable.\npip install matplotlib",
        "tip": "Usage: ① pick port → Connect ② adjust M+/M- voltage (measured: 2.9V ≈ 7.9Hz unlocks measurement) ③ watch the 2D map: center = lidar, up = front (arrow), blue = current room outline, gray = previous revolution, 1 cell = 1 m ④ moving the lidar separates the gray/blue rings. If the map is mirrored left-right, flip 'Sweep dir'.",
        "panel_log": "Runtime log (parsed)",
        "msg_ok_title": "Data acquired",
        "msg_ok_body": "0xAD measurement frames appeared!\nSpeed {hz:.2f} Hz\n{n} samples/frame ({valid} valid)\n\nReady to use.",
        "msg_err_dep": "Missing dependency", "msg_err_dep_body": "pyserial not installed, run: pip install pyserial",
        "msg_warn_port": "Notice", "msg_warn_port_body": "Please select a serial port first",
        "msg_err_conn": "Connection failed", "msg_err_conn_body": "{err}\n\nCheck port / in use / wiring",
        "msg_info_logdir": "Log folder",
        "L_speed": "0xAE speed | len={l} ver=0x{v:02X} xx={xx} | Hz={hz:.2f}",
        "L_meas": "0xAD meas | len={l} ver=0x{v:02X} xx={xx} Hz={hz:.2f} | N={n} start={s:.2f}° end={e:.2f}° | first_valid d={d:.1f}mm q={q}",
        "L_noise": "stray bytes {n} (boot burst / wrong baud?)",
        "L_bad": "checksum FAIL chk=0x{c:04X} sum=0x{s:04X} len={l} | {hx}",
        "L_unknown": "unknown frame (not DELTA ver/type) len={l} | {hx}",
        "L_conn": "=== Connected {port} @ {baud} ===",
        "L_disc": "=== Disconnected ===",
        "L_err": "serial error: {e}",
    },
}
LANGS = ("zh", "en")


def signed16(hi, lo):
    v = (hi << 8) | lo
    return v - 0x10000 if v >= 0x8000 else v


class DeltaParser:
    """DELTA-2C stream parser. feed(data, emit) -> emit(event, *args)"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data, emit):
        self.buf += data
        while True:
            i = self.buf.find(bytes([START]))
            if i < 0:
                if self.buf:
                    emit("noise", bytes(self.buf))
                self.buf.clear()
                return
            if i > 0:
                emit("noise", bytes(self.buf[:i]))
                del self.buf[:i]
            if len(self.buf) < 3:
                return
            plen = (self.buf[1] << 8) | self.buf[2]
            if not (6 <= plen <= 1024):
                del self.buf[:1]
                continue
            total = plen + 2
            if len(self.buf) < total:
                return
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            chk = (frame[plen] << 8) | frame[plen + 1]
            s = sum(frame[:plen]) & 0xFFFF
            if chk != s:
                emit("bad", frame, s, chk)
                continue
            if len(frame) < 6 or frame[4] != PTYPE or frame[5] not in (DT_MEAS, DT_SPEED) \
                    or frame[3] not in VERS_OK:
                emit("unknown", frame)
                continue
            xx = frame[8] if plen > 8 else 0
            if frame[5] == DT_SPEED:
                emit("speed", frame, xx)
            else:
                off = signed16(frame[9], frame[10]) if plen > 10 else 0
                st = ((frame[11] << 8) | frame[12]) if plen > 12 else 0
                en = ((frame[13] << 8) | frame[14]) if plen > 14 else 0
                dlen = (frame[6] << 8) | frame[7]
                n = (dlen - 7) // 3 if dlen >= 7 else 0
                pts = []  # (distance_mm, quality)
                pos = 15
                for _ in range(min(n, 96)):
                    if pos + 3 > plen:
                        break
                    q = frame[pos]
                    d = ((frame[pos + 1] << 8) | frame[pos + 2]) * 0.5
                    if d > 8000.0:   # 无回波饱和(1号机 ~0x3FE0=8.17m) -> 按无回波处理
                        d = 0.0
                    pts.append((d, q))
                    pos += 3
                emit("meas", frame, xx, off, st, en, pts)


class Reader(threading.Thread):
    """Serial reader: parse + dual logging + push events to the queue."""

    def __init__(self, ser, parser, q, raw_path, parsed_path, stop, tr):
        super().__init__(daemon=True)
        self.ser = ser
        self.parser = parser
        self.q = q
        self.stop = stop
        self.tr = tr  # callable(key, **kw) resolving against the current UI language
        self.raw_f = open(raw_path, "a", encoding="utf-8", buffering=1)
        self.parsed_f = open(parsed_path, "a", encoding="utf-8", buffering=1)
        self.stats = dict(total=0, speed=0, meas=0, bad=0, unknown=0, noise=0)
        self.last_xx = -999
        self.last_speed_log = 0.0
        self.noise_sum = 0
        self.last_noise_log = 0.0
        self.trend = collections.deque()      # (ts, xx), sampled every 100 ms
        self.meas_mark = collections.deque()  # timestamps of 0xAD arrivals
        self._last_trend_t = 0.0
        self.lock = threading.Lock()

    # ---- logging ----
    def _w_raw(self, data):
        ts = datetime.datetime.now()
        line = "[%s]# RECV HEX/%d <<<\n" % (ts.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3], len(data))
        self.raw_f.write(line)
        hx = data.hex(" ").upper()
        for i in range(0, len(hx), 240):
            self.raw_f.write(hx[i:i + 240] + "\n")
        self.raw_f.flush()

    def _w_parsed(self, msg):
        ts = datetime.datetime.now()
        self.parsed_f.write("[%s] %s\n" % (ts.strftime("%H:%M:%S.%f")[:-3], msg))
        self.parsed_f.flush()

    # ---- events ----
    def handle(self, ev, *args):
        with self.lock:
            st = self.stats
            if ev == "noise":
                st["noise"] += len(args[0])
                self.noise_sum += len(args[0])
                now = time.time()
                if now - self.last_noise_log > 3:
                    self._w_parsed(self.tr("L_noise", n=self.noise_sum))
                    self.last_noise_log = now
                return
            if ev == "bad":
                st["bad"] += 1
                fr, s, c = args
                if st["bad"] <= 20:
                    self._w_parsed(self.tr("L_bad", c=c, s=s, l=len(fr), hx=fr.hex(" ").upper()))
                return
            if ev == "unknown":
                st["unknown"] += 1
                fr = args[0]
                if st["unknown"] <= 20:
                    self._w_parsed(self.tr("L_unknown", l=len(fr), hx=fr.hex(" ").upper()))
                return
            st["total"] += 1
            if ev == "speed":
                st["speed"] += 1
                xx = args[1]
                self._trend(xx, False)
                now = time.time()
                if abs(xx - self.last_xx) >= 2 or now - self.last_speed_log > 2:
                    self._w_parsed(self.tr("L_speed", l=len(args[0]), v=args[0][3],
                                           xx=xx, hz=xx / 20.0))
                    self.last_xx = xx
                    self.last_speed_log = now
            elif ev == "meas":
                st["meas"] += 1
                fr, xx, off, st_ang, en_ang, pts = args
                self._trend(xx, True)
                valid = [p for p in pts if p[1] != 0]
                d0 = valid[0][0] if valid else 0.0
                q0 = valid[0][1] if valid else 0
                self._w_parsed(self.tr("L_meas", l=len(fr), v=fr[3], xx=xx, hz=xx / 20.0,
                                       n=len(pts), s=st_ang * 0.01, e=en_ang * 0.01,
                                       d=d0, q=q0))
            self.q.put((ev, args))

    def _trend(self, xx, is_meas):
        now = time.time()
        if now - self._last_trend_t >= 0.1:
            self.trend.append((now, xx))
            if len(self.trend) > 4000:
                self.trend.popleft()
            self._last_trend_t = now
        if is_meas:
            self.meas_mark.append(now)
            if len(self.meas_mark) > 4000:
                self.meas_mark.popleft()

    # ---- main loop ----
    def run(self):
        self._w_parsed(self.tr("L_conn", port=self.ser.port, baud=self.ser.baudrate))
        while not self.stop.is_set():
            try:
                n = self.ser.in_waiting
                if n == 0:
                    time.sleep(0.01)
                    continue
                data = self.ser.read(n)
            except Exception as e:
                self.q.put(("error", str(e)))
                break
            if not data:
                continue
            self._w_raw(data)
            self.parser.feed(data, lambda ev, *a: self.handle(ev, *a))
        try:
            self.ser.close()
        except Exception:
            pass
        self._w_parsed(self.tr("L_disc"))
        self.raw_f.close()
        self.parsed_f.close()

    def snapshot(self):
        with self.lock:
            return dict(self.stats)

    def trend_snapshot(self):
        """Thread-safe copy of the trend data (GUI must not iterate the live deque)."""
        with self.lock:
            return list(self.trend), list(self.meas_mark)


class App:
    def __init__(self, root):
        self.root = root
        self.lang = "zh"
        self.S = STRINGS[self.lang]

        self.ser = None
        self.reader = None
        self.stop = threading.Event()
        self.q = queue.Queue()
        self.parser = DeltaParser()
        self.saw_speed = False
        self.saw_meas = False
        self.meas_notified = False
        self.divisor = 20
        self.accumulate = False
        self.raw_path = ""
        self.parsed_path = ""

        self.scan_buf = []      # (rev, angle_deg, dist_mm, q); rolling last 3 revolutions
        self.prev_buf = []      # last completed revolution (gray overlay)
        self.last_st = None
        self.rev = 0
        self.auto_range = False
        self.scan_dirty = False     # scan view needs a redraw
        self._closing = False       # window close requested
        self._after_id = None       # current poll timer id
        self.cw = True          # sweep direction: True = CW on map (flip if mirrored)
        self._last_scan_draw = 0.0
        self._dyn_labels = []   # (label_widget, i18n_key) re-applied on language switch
        self.ax = None
        self.sc_prev = None
        self.sc_base = None
        self.sc_recent = None
        self.canvas = None

        os.makedirs(LOG_DIR, exist_ok=True)
        self._build()
        self.refresh_ports()
        self.root.after(80, self._poll)

    def tr(self, key, **kw):
        s = self.S.get(key, key)
        return s.format(**kw) if kw else s

    # ---------------- UI ----------------
    def _build(self):
        f = ttk.Frame(self.root, padding=8)
        f.pack(fill=tk.BOTH, expand=True)

        r0 = ttk.Frame(f)
        r0.pack(fill=tk.X)
        ttk.Label(r0, text=self.tr("port")).pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(r0, width=12, state="readonly")
        self.cb_port.pack(side=tk.LEFT, padx=(2, 0))
        self.btn_refresh = ttk.Button(r0, text=self.tr("refresh"), width=8, command=self.refresh_ports)
        self.btn_refresh.pack(side=tk.LEFT, padx=4)
        self.lb_baud_t = ttk.Label(r0, text=self.tr("baud"))
        self.lb_baud_t.pack(side=tk.LEFT, padx=(10, 0))
        self.cb_baud = ttk.Combobox(r0, width=8, state="readonly",
                                    values=["9600", "57600", "115200", "230400", "460800"])
        self.cb_baud.set("115200")
        self.cb_baud.pack(side=tk.LEFT, padx=(2, 0))
        self.lb_div_t = ttk.Label(r0, text=self.tr("divisor"))
        self.lb_div_t.pack(side=tk.LEFT, padx=(10, 0))
        self.cb_div = ttk.Combobox(r0, width=5, state="readonly",
                                   values=[str(d) for d in DIVISORS])
        self.cb_div.set(str(self.divisor))
        self.cb_div.bind("<<ComboboxSelected>>", lambda e: self._set_divisor())
        self.cb_div.pack(side=tk.LEFT, padx=(2, 0))
        self.btn_conn = ttk.Button(r0, text=self.tr("connect"), width=8, command=self.toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=(14, 0))
        self.lb_lang_t = ttk.Label(r0, text="语言/Lang:")
        self.lb_lang_t.pack(side=tk.LEFT, padx=(16, 0))
        self.cb_lang = ttk.Combobox(r0, width=7, state="readonly",
                                    values=["中文", "English"])
        self.cb_lang.set("中文")
        self.cb_lang.bind("<<ComboboxSelected>>", lambda e: self._set_lang())
        self.cb_lang.pack(side=tk.LEFT, padx=(2, 0))

        r1 = ttk.Frame(f)
        r1.pack(fill=tk.X, pady=(6, 0))
        self.lb_log_t = ttk.Label(r1, text=self.tr("log"))
        self.lb_log_t.pack(side=tk.LEFT)
        self.lb_log = tk.Label(r1, text="(not connected)", fg="#666", anchor="w")
        self.lb_log.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
        self.btn_logs = ttk.Button(r1, text=self.tr("open_logs"), width=16, command=self.open_logs)
        self.btn_logs.pack(side=tk.RIGHT)
        self.btn_clear = ttk.Button(r1, text=self.tr("clear"), width=8,
                                    command=lambda: self.txt.delete("1.0", tk.END))
        self.btn_clear.pack(side=tk.RIGHT, padx=4)

        self.banner = tk.Label(f, text=self.tr("banner_idle"),
                               font=("Microsoft YaHei UI", 13, "bold"), fg="white", bg="#9E9E9E",
                               anchor="center", height=2, width=46)
        self.banner.pack(fill=tk.X, pady=(8, 0))

        mid = ttk.Frame(f)
        mid.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        mid.columnconfigure(0, weight=3)
        mid.columnconfigure(1, weight=5)

        left = ttk.Frame(mid)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.pv = ttk.LabelFrame(left, text=self.tr("panel_values"), padding=8)
        self.pv.pack(fill=tk.X)
        self.pv.columnconfigure((0, 1), weight=1)
        self.lb_hz = self._big(self.pv, 0, 0, "hz")
        self.lb_type = self._big(self.pv, 1, 0, "dtype")
        self.lb_meas = self._big(self.pv, 0, 1, "last_meas")
        self.lb_scan = self._big(self.pv, 1, 1, "scan_points")
        self.lb_nearest = tk.Label(self.pv, font=("Consolas", 10), fg="#1565C0",
                                   text=self.tr("nearest") + ": --", width=36, anchor="w")
        self.lb_nearest.grid(row=2, column=0, sticky="w", padx=4)
        self.lb_xx = tk.Label(self.pv, font=("Consolas", 10), fg="#555", text="xx=--",
                              width=42, anchor="w")
        self.lb_xx.grid(row=2, column=1, sticky="w", padx=4)
        self.lb_cnt = tk.Label(self.pv, font=("Consolas", 10), fg="#555", width=46, anchor="w",
                               text=self.tr("cnt_fmt", total=0, speed=0, meas=0, bad=0, noise=0))
        self.lb_cnt.grid(row=3, column=0, columnspan=2, sticky="w", padx=4)

        self.pt = ttk.LabelFrame(left, text=self.tr("panel_trend", lo=TARGET_BAND[0], hi=TARGET_BAND[1]),
                                 padding=4)
        self.pt.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.cv = tk.Canvas(self.pt, height=130, bg="white", highlightthickness=1,
                            highlightbackground="#ccc")
        self.cv.pack(fill=tk.BOTH, expand=True)

        self.ps = ttk.LabelFrame(mid, text=self.tr("panel_scan"), padding=4)
        self.ps.grid(row=0, column=1, sticky="nsew")
        if HAS_MPL:
            import matplotlib.ticker as mticker
            from matplotlib.patches import Circle
            self.fig = Figure(figsize=(4.6, 4.6), dpi=100, facecolor="#1c1c1c")
            self.ax = self.fig.add_subplot(111)
            self.ax.set_facecolor("#1c1c1c")
            self.ax.set_aspect("equal")
            self.ax.set_xlim(-FIXED_RANGE_M, FIXED_RANGE_M)
            self.ax.set_ylim(-FIXED_RANGE_M, FIXED_RANGE_M)
            self.ax.xaxis.set_major_locator(mticker.MultipleLocator(2))
            self.ax.xaxis.set_minor_locator(mticker.MultipleLocator(1))
            self.ax.yaxis.set_major_locator(mticker.MultipleLocator(2))
            self.ax.yaxis.set_minor_locator(mticker.MultipleLocator(1))
            self.ax.grid(True, which="major", ls="-", alpha=0.5, color="#3d3d3d")
            self.ax.grid(True, which="minor", ls=":", alpha=0.25, color="#2a2a2a")
            self.ax.set_xlabel("X (m)")
            self.ax.set_ylabel("Y (m)")
            for t in self.ax.get_xticklabels() + self.ax.get_yticklabels():
                t.set_color("#9e9e9e")
            self.ax.xaxis.label.set_color("#9e9e9e")
            self.ax.yaxis.label.set_color("#9e9e9e")
            for sp in self.ax.spines.values():
                sp.set_color("#555555")
            # RViz LaserScan style: individual points colored by distance (Jet)
            self.sc_base = self.ax.scatter([], [], c=[], cmap="jet", vmin=0,
                                           vmax=FIXED_RANGE_M, s=8, zorder=3)
            self.sc_prev = self.ax.scatter([], [], s=7, c="#6d6d6d", alpha=0.55, zorder=1)
            self.sc_recent = self.ax.scatter([], [], s=34, c="#E91E63", marker="o", zorder=4)
            self.cbar = self.fig.colorbar(self.sc_base, ax=self.ax, pad=0.02)
            self.cbar.set_label("m")
            self.cbar.outline.set_edgecolor("#555555")
            for t in self.cbar.ax.get_yticklabels():
                t.set_color("#9e9e9e")
            # robot pose (RViz-style red arrow) + direction labels
            self.robot_patch = Circle((0, 0), 0.15, fc="#ECEFF1", ec="#B0BEC5", lw=1.5, zorder=5)
            self.ax.add_patch(self.robot_patch)
            self.robot_arrow = self.ax.annotate(
                "", xy=(0, 0.62), xytext=(0, 0.05),
                arrowprops=dict(arrowstyle="-|>", color="#F44336", lw=2.5), zorder=5)
            self.txt_front = self.ax.text(0, 0, "", ha="center", va="center",
                                          fontsize=11, fontweight="bold", color="#CFD8DC", zorder=5)
            self.txt_back = self.ax.text(0, 0, "", ha="center", va="center",
                                         fontsize=10, color="#6d6d6d", zorder=5)
            self.txt_left = self.ax.text(0, 0, "", ha="center", va="center",
                                         fontsize=10, color="#6d6d6d", zorder=5)
            self.txt_right = self.ax.text(0, 0, "", ha="center", va="center",
                                          fontsize=10, color="#6d6d6d", zorder=5)
            self.canvas = FigureCanvasTkAgg(self.fig, master=self.ps)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            leg = ttk.Frame(self.ps)
            leg.pack(anchor="w", pady=(2, 0))
            for color, key in (("#6d6d6d", "scan_legend_prev"),
                               ("#E91E63", "scan_legend_recent")):
                ttk.Label(leg, text="●", foreground=color).pack(side=tk.LEFT, padx=(0, 2))
                lab = ttk.Label(leg, text=self.tr(key))
                lab.pack(side=tk.LEFT, padx=(0, 10))
                self._dyn_labels.append((lab, key))
            lab = ttk.Label(leg, text=self.tr("scan_legend_cur"))
            lab.pack(side=tk.LEFT, padx=(0, 10))
            self._dyn_labels.append((lab, "scan_legend_cur"))
        else:
            tk.Label(self.ps, text=self.tr("mpl_missing"), fg="#c00").pack()
        ctl = ttk.Frame(self.ps)
        ctl.pack(anchor="w", pady=(2, 0))
        self.chk_accum = ttk.Checkbutton(ctl, text=self.tr("scan_accum"), command=self._tog_accum)
        self.chk_accum.pack(side=tk.LEFT)
        self.chk_auto = ttk.Checkbutton(ctl, text=self.tr("auto_range"), command=self._tog_auto)
        self.chk_auto.pack(side=tk.LEFT, padx=(12, 0))
        self.lb_dir_t = ttk.Label(ctl, text=self.tr("dir_label"))
        self.lb_dir_t.pack(side=tk.LEFT, padx=(12, 0))
        self.cb_dir = ttk.Combobox(ctl, width=6, state="readonly",
                                   values=[self.tr("dir_cw"), self.tr("dir_ccw")])
        self.cb_dir.set(self.tr("dir_cw"))
        self.cb_dir.bind("<<ComboboxSelected>>", lambda e: self._set_dir())
        self.cb_dir.pack(side=tk.LEFT, padx=(2, 0))

        self.tip = tk.Label(f, justify="left", anchor="w", fg="#333", wraplength=940,
                            text=self.tr("tip"))
        self.tip.pack(fill=tk.X, pady=(6, 0))
        self.tip2 = tk.Label(f, justify="left", anchor="w", fg="#888", wraplength=940,
                             text=self.tr("divisor_tip"))
        self.tip2.pack(fill=tk.X, pady=(2, 0))

        self.pl = ttk.LabelFrame(f, text=self.tr("panel_log"), padding=4)
        self.pl.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.txt = tk.Text(self.pl, height=8, font=("Consolas", 9), state="disabled",
                           bg="#1E1E1E", fg="#D4D4D4")
        sb = ttk.Scrollbar(self.pl, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _big(self, parent, col, row, key):
        box = ttk.Frame(parent)
        box.grid(row=row, column=col, sticky="ew", padx=4, pady=2)
        ttl = ttk.Label(box, text=self.tr(key), font=("Microsoft YaHei UI", 9), foreground="#888")
        ttl.pack(anchor="w")
        self._dyn_labels.append((ttl, key))
        lab = tk.Label(box, text="--", font=("Consolas", 17, "bold"), fg="#222", anchor="w", width=13)
        lab.pack(fill=tk.X)
        return lab

    # ---------------- language ----------------
    def _set_lang(self):
        idx = self.cb_lang.current()
        self.lang = LANGS[1] if idx == 1 else LANGS[0]
        self.S = STRINGS[self.lang]
        self.root.title(self.tr("title"))
        for lbl, key in self._dyn_labels:
            lbl.config(text=self.tr(key))
        self.lb_baud_t.config(text=self.tr("baud"))
        self.lb_div_t.config(text=self.tr("divisor"))
        self.lb_log_t.config(text=self.tr("log"))
        self.btn_refresh.config(text=self.tr("refresh"))
        self.btn_logs.config(text=self.tr("open_logs"))
        self.btn_clear.config(text=self.tr("clear"))
        self.chk_accum.config(text=self.tr("scan_accum"))
        self.chk_auto.config(text=self.tr("auto_range"))
        self.lb_dir_t.config(text=self.tr("dir_label"))
        cw_sel = self.cb_dir.current()
        self.cb_dir["values"] = [self.tr("dir_cw"), self.tr("dir_ccw")]
        self.cb_dir.current(cw_sel)
        connected = bool(self.ser and self.ser.is_open)
        self.btn_conn.config(text=self.tr("disconnect") if connected else self.tr("connect"))
        self.pv.config(text=self.tr("panel_values"))
        self.pt.config(text=self.tr("panel_trend", lo=TARGET_BAND[0], hi=TARGET_BAND[1]))
        self.ps.config(text=self.tr("panel_scan"))
        self.pl.config(text=self.tr("panel_log"))
        self.tip.config(text=self.tr("tip"))
        self.tip2.config(text=self.tr("divisor_tip"))
        self._render_banner()
        self._refresh_display()

    # ---------------- logic ----------------
    def refresh_ports(self):
        if serial is None:
            return
        ports = [p.device for p in list_ports.comports()]
        self.cb_port["values"] = ports
        if ports and not self.cb_port.get():
            self.cb_port.set(ports[0])

    def _set_divisor(self):
        try:
            self.divisor = int(self.cb_div.get())
        except Exception:
            self.divisor = 20
        self._refresh_display()

    def _tog_accum(self):
        self.accumulate = bool(self.chk_accum.instate(["selected"]))
        self.scan_buf = []
        self.prev_buf = []
        self.last_st = None
        self.rev = 0

    def _tog_auto(self):
        self.auto_range = bool(self.chk_auto.instate(["selected"]))

    def _set_dir(self):
        self.cw = self.cb_dir.current() != 1

    def toggle_conn(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if serial is None:
            messagebox.showerror(self.tr("msg_err_dep"), self.tr("msg_err_dep_body"))
            return
        port = self.cb_port.get()
        baud = int(self.cb_baud.get() or 115200)
        if not port:
            messagebox.showwarning(self.tr("msg_warn_port"), self.tr("msg_warn_port_body"))
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            messagebox.showerror(self.tr("msg_err_conn"),
                                 self.tr("msg_err_conn_body", err=e))
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(LOG_DIR, "delta2c_%s" % stamp)
        self.raw_path = base + "_raw.log"
        self.parsed_path = base + "_parsed.log"
        self.stop = threading.Event()
        self.parser = DeltaParser()
        self.q = queue.Queue()
        self.saw_speed = self.saw_meas = self.meas_notified = False
        self.scan_buf = []
        self.prev_buf = []
        self.last_st = None
        self.rev = 0
        self.reader = Reader(self.ser, self.parser, self.q,
                             self.raw_path, self.parsed_path, self.stop, self.tr)
        self.reader.start()
        self.btn_conn.config(text=self.tr("disconnect"))
        self.lb_log.config(text=os.path.basename(self.parsed_path), fg="#0a0")
        self._set_banner(self.tr("banner_connected", port=port, baud=baud), "#1565C0")
        self._append_log(self.tr("banner_connected", port=port, baud=baud))

    def _disconnect(self):
        self.stop.set()
        if self.reader:
            try:
                self.reader.join(timeout=2)
            except Exception:
                pass
        self.reader = None
        self.btn_conn.config(text=self.tr("connect"))
        self._set_banner(self.tr("banner_idle"), "#9E9E9E")

    def _set_banner(self, text, color):
        self.banner.config(text=text, bg=color)

    def _render_banner(self):
        """Re-render the banner after a language switch."""
        if not self.reader:
            self._set_banner(self.tr("banner_idle"), "#9E9E9E")
        elif self.saw_meas:
            self._set_banner(self.tr("banner_ok"), "#1B5E20")
        elif self.saw_speed:
            hz = 0
            tr, _ = self.reader.trend_snapshot()
            if tr:
                hz = tr[-1][1] / float(self.divisor)
            if TARGET_BAND[0] <= hz <= TARGET_BAND[1]:
                self._set_banner(self.tr("banner_in_band", hz=hz, lo=TARGET_BAND[0], hi=TARGET_BAND[1]), "#E65100")
            else:
                self._set_banner(self.tr("banner_speed", hz=hz, lo=TARGET_BAND[0], hi=TARGET_BAND[1]), "#1565C0")
        else:
            self._set_banner(self.tr("banner_idle"), "#9E9E9E")

    def _append_log(self, line):
        self.txt.config(state="normal")
        self.txt.insert(tk.END, line + "\n")
        self.txt.see(tk.END)
        self.txt.config(state="disabled")
        if int(self.txt.index("end-1c").split(".")[0]) > 1500:
            self.txt.delete("1.0", "400.0")

    def _poll(self):
        """Main GUI tick. Never dies: any error is logged and the loop continues."""
        if self._closing:
            return
        try:
            try:
                while True:
                    ev, args = self.q.get_nowait()
                    if ev == "speed":
                        fr, xx = args
                        self.saw_speed = True
                        self._refresh_display()
                    elif ev == "meas":
                        fr, xx, off, st_ang, en_ang, pts = args
                        self.saw_meas = True
                        self._add_scan_points(st_ang, en_ang, pts)
                        self._refresh_display(meas=(st_ang, en_ang, pts))
                        if not self.meas_notified:
                            self.meas_notified = True
                            self._set_banner(self.tr("banner_ok"), "#1B5E20")
                            self._beep()
                            n = len(pts)
                            valid = len([p for p in pts if p[1] != 0])
                            self._toast(self.tr("msg_ok_title"),
                                        self.tr("msg_ok_body", hz=xx / float(self.divisor),
                                                n=n, valid=valid))
                    elif ev == "bad":
                        self._refresh_display()
                    elif ev == "error":
                        self._append_log(self.tr("L_err", e=args[0]))
                        self._disconnect()
            except queue.Empty:
                pass

            st = self.reader.snapshot() if self.reader else None
            if self.reader and st:
                hz = 0
                tr, _ = self.reader.trend_snapshot()
                if tr:
                    hz = tr[-1][1] / float(self.divisor)
                if not self.saw_meas:
                    if self.saw_speed:
                        if TARGET_BAND[0] <= hz <= TARGET_BAND[1]:
                            self._set_banner(self.tr("banner_in_band", hz=hz,
                                                     lo=TARGET_BAND[0], hi=TARGET_BAND[1]), "#E65100")
                        else:
                            self._set_banner(self.tr("banner_speed", hz=hz,
                                                     lo=TARGET_BAND[0], hi=TARGET_BAND[1]), "#1565C0")
                    elif st["noise"] > 200 and st["total"] == 0:
                        self._set_banner(self.tr("banner_noise"), "#C62828")
            self._draw_trend()
            self._refresh_display()
            now = time.time()
            if self.scan_dirty and now - self._last_scan_draw > 0.12:
                self._draw_scan()
                self._last_scan_draw = now
                self.scan_dirty = False
        except Exception as e:
            self._append_log("INTERNAL: %r" % e)
        if not self._closing:
            self._after_id = self.root.after(80, self._poll)

    def _toast(self, title, body):
        """Non-modal auto-closing notification (never blocks the window)."""
        try:
            top = tk.Toplevel(self.root)
            top.title(title)
            top.attributes("-topmost", True)
            top.resizable(False, False)
            tk.Label(top, text=body, justify="left", padx=16, pady=12).pack()
            top.after(4000, top.destroy)
        except Exception:
            pass

    def _add_scan_points(self, st_ang, en_ang, pts):
        st = st_ang * 0.01
        en = en_ang * 0.01
        n = len(pts)
        if n == 0:
            return
        if self.last_st is not None and st < self.last_st - 90:
            # new revolution: rolling window of the last 3 (blue=current, gray=older)
            self.rev += 1
            if not self.accumulate:
                self.scan_buf = [p for p in self.scan_buf if p[0] > self.rev - 3]
        self.last_st = st
        for k, (d, q) in enumerate(pts):
            if q == 0:
                continue
            a = (st + (en - st) * (k + 0.5) / n) % 360.0
            self.scan_buf.append((self.rev, a, d, q))
        cap = 4000 if self.accumulate else 1200
        if len(self.scan_buf) > cap:
            del self.scan_buf[:len(self.scan_buf) - cap]
        self.scan_dirty = True

    def _xy(self, a_deg, d_mm):
        """Lidar polar -> map cartesian. a=0 points up (+Y); CW sweep -> +X is right."""
        rad = math.radians(a_deg)
        d = d_mm / 1000.0
        if self.cw:
            return d * math.sin(rad), d * math.cos(rad)
        return -d * math.sin(rad), d * math.cos(rad)

    def _draw_scan(self):
        if self.ax is None or self.canvas is None:
            return
        pts = self.scan_buf
        cur = [p for p in pts if p[0] == self.rev]
        prev = [p for p in pts if p[0] != self.rev]
        if len(cur) > 800:      # downsample for fast drawing
            cur = cur[::len(cur) // 800 + 1]
        if len(prev) > 1200:
            prev = prev[::len(prev) // 1200 + 1]
        rmax = FIXED_RANGE_M
        if cur:
            cur_xy = [self._xy(a, d) for r, a, d, q in cur]
            ds = [min(d / 1000.0, 50.0) for r, a, d, q in cur]
            self.sc_base.set_offsets(cur_xy)
            self.sc_base.set_array(ds)
            self.sc_recent.set_offsets([self._xy(a, d) for r, a, d, q in cur[-24:]])
            if self.auto_range:
                rmax = max(5.0, min(max(max(x for x, y in cur_xy), max(y for x, y in cur_xy),
                                        -min(x for x, y in cur_xy), -min(y for x, y in cur_xy)) * 1.2, 50.0))
            self.sc_base.set_clim(0, rmax)
            nearest = min(cur, key=lambda p: p[2])
            farthest = max(cur, key=lambda p: p[2])
            self.lb_nearest.config(text=self.tr(
                "nearest_far_fmt", nd=nearest[2] / 1000.0, na=nearest[1],
                fd=farthest[2] / 1000.0))
        else:
            self.sc_base.set_offsets([])
            self.sc_base.set_array([])
            self.sc_recent.set_offsets([])
            self.lb_nearest.config(text=self.tr("nearest") + ": --")
        if prev:
            self.sc_prev.set_offsets([self._xy(a, d) for r, a, d, q in prev])
        else:
            self.sc_prev.set_offsets([])
        self.ax.set_xlim(-rmax, rmax)
        self.ax.set_ylim(-rmax, rmax)
        self.txt_front.set_position((0, rmax * 0.92))
        self.txt_front.set_text(self.tr("dir_front"))
        self.txt_back.set_position((0, -rmax * 0.92))
        self.txt_back.set_text(self.tr("dir_back"))
        self.txt_left.set_position((-rmax * 0.92, 0))
        self.txt_left.set_text(self.tr("dir_left"))
        self.txt_right.set_position((rmax * 0.92, 0))
        self.txt_right.set_text(self.tr("dir_right"))
        try:
            self.canvas.draw_idle()
        except Exception:
            pass

    def _refresh_display(self, meas=None):
        if not self.reader:
            return
        st = self.reader.snapshot()
        tr, _ = self.reader.trend_snapshot()
        xx = tr[-1][1] if tr else None
        if xx is not None:
            hz = xx / float(self.divisor)
            self.lb_hz.config(text="%.2f Hz" % hz)
            other = DIVISORS[1] if self.divisor == DIVISORS[0] else DIVISORS[0]
            self.lb_xx.config(text=self.tr("xx_fmt", xx=xx, div=self.divisor,
                                           other=other, hz2=xx / float(other)))
        else:
            self.lb_hz.config(text="--")
        if self.saw_meas:
            self.lb_type.config(text=self.tr("dtype_meas"), fg="#1B5E20")
        elif self.saw_speed:
            self.lb_type.config(text=self.tr("dtype_speed"), fg="#1565C0")
        else:
            self.lb_type.config(text=self.tr("dtype_none"), fg="#222")
        if meas:
            st_ang, en_ang, pts = meas
            valid = [p for p in pts if p[1] != 0]
            d0 = valid[0][0] if valid else 0.0
            q0 = valid[0][1] if valid else 0
            self.lb_meas.config(text=self.tr("meas_fmt", n=len(pts), st=st_ang * 0.01,
                                             en=en_ang * 0.01, d=d0, q=q0))
        self.lb_scan.config(text="%d" % len(self.scan_buf))
        self.lb_cnt.config(text=self.tr("cnt_fmt", total=st["total"], speed=st["speed"],
                                        meas=st["meas"], bad=st["bad"], noise=st["noise"]))

    def _draw_trend(self):
        cv = self.cv
        cv.delete("all")
        W = cv.winfo_width() or 760
        H = cv.winfo_height() or 130
        if W < 50 or not self.reader:
            return
        tr, marks = self.reader.trend_snapshot()
        y_band_top = H - (TARGET_BAND[1] / 12.0) * H
        y_band_bot = H - (TARGET_BAND[0] / 12.0) * H
        cv.create_rectangle(0, y_band_top, W, y_band_bot, fill="#E8F5E9", outline="")
        for hz in (0, 3, 6, 9, 12):
            y = H - (hz / 12.0) * H
            cv.create_line(0, y, W, y, fill="#E0E0E0")
            cv.create_text(4, y - 4, anchor="w", text="%d" % hz, fill="#999", font=("Consolas", 8))
        if len(tr) < 2:
            return
        t0 = tr[0][0]
        t1 = tr[-1][0]
        span = max(t1 - t0, 1.0)
        div = float(self.divisor)
        pts = []
        for (t, xx) in tr:
            x = W * (1 - (t1 - t) / span)
            y = H - (xx / div / 12.0) * H
            pts.append((x, y))
        cv.create_line(pts, fill="#1565C0", width=2)
        for t in marks:
            if t >= t0:
                x = W * (1 - (t1 - t) / span)
                cv.create_line(x, 0, x, H, fill="#1B5E20", dash=(2, 2))
        x, y = pts[-1]
        cv.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#D32F2F", outline="")

    def _beep(self):
        if winsound:
            winsound.Beep(1046, 180)
            winsound.Beep(1568, 240)

    def open_logs(self):
        try:
            os.startfile(LOG_DIR)
        except Exception:
            messagebox.showinfo(self.tr("msg_info_logdir"), LOG_DIR)

    def on_close(self):
        self._closing = True
        if self._after_id:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self.stop.set()
        if self.reader:
            try:
                self.reader.join(timeout=2)
            except Exception:
                pass
        try:
            self.root.destroy()
        except Exception:
            pass


def main():
    if serial is None:
        r = tk.Tk()
        messagebox.showerror("Missing dependency", "pyserial not installed, run: pip install pyserial")
        r.destroy()
        return
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
