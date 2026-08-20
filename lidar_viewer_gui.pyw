#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DELTA-2C 激光雷达 3D 查看器 — GUI 启动器
==========================================
选择串口(实机)或日志(重放)后一键启动网页 3D/2D 点云查看器,
浏览器自动打开 http://127.0.0.1:8080 。运行状态实时显示在本窗口。

用法: 双击运行(需已 pip install pyserial)
"""
import glob
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial.tools.list_ports as list_ports
except Exception:
    list_ports = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")
VIEWER = os.path.join(SCRIPT_DIR, "lidar_webview.py")
URL = "http://127.0.0.1:8080"


class App:
    def __init__(self, root):
        self.root = root
        root.title("DELTA-2C 激光雷达 3D 查看器")
        root.geometry("780x540")
        root.minsize(660, 430)
        self.proc = None
        self.out_q = queue.Queue()
        self._build()
        self.refresh_ports()
        self.refresh_logs()
        self._mode_changed()
        self.root.after(100, self._poll)

    def _build(self):
        f = ttk.Frame(self.root, padding=10)
        f.pack(fill=tk.BOTH, expand=True)

        mf = ttk.LabelFrame(f, text="数据源", padding=8)
        mf.pack(fill=tk.X)
        self.mode = tk.StringVar(value="serial")
        ttk.Radiobutton(mf, text="实机串口(雷达已接)", variable=self.mode,
                        value="serial", command=self._mode_changed).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(mf, text="重放日志(无需硬件)", variable=self.mode,
                        value="log", command=self._mode_changed).grid(row=1, column=0, sticky="w")

        self.lb_port = ttk.Label(mf, text="串口:")
        self.lb_port.grid(row=0, column=1, sticky="e", padx=(24, 2))
        self.cb_port = ttk.Combobox(mf, width=16, state="normal")  # 可手输, 如 COM4
        self.cb_port.grid(row=0, column=2, sticky="w")
        self.btn_refresh = ttk.Button(mf, text="刷新", width=6, command=self.refresh_ports)
        self.btn_refresh.grid(row=0, column=3, padx=4)
        ttk.Label(mf, text="(优先显示 USB 串口; COM1/COM2 一般是主板自带,不是雷达)").grid(
            row=0, column=4, sticky="w", padx=8)

        self.lb_log = ttk.Label(mf, text="日志:")
        self.lb_log.grid(row=1, column=1, sticky="e", padx=(24, 2))
        self.cb_log = ttk.Combobox(mf, width=52, state="readonly")
        self.cb_log.grid(row=1, column=2, columnspan=3, sticky="w")

        bf = ttk.Frame(f)
        bf.pack(fill=tk.X, pady=(12, 0))
        self.btn_start = ttk.Button(bf, text="▶ 启动 3D 查看器", command=self.start)
        self.btn_start.pack(side=tk.LEFT)
        self.btn_stop = ttk.Button(bf, text="■ 停止", command=self.stop, state="disabled")
        self.btn_stop.pack(side=tk.LEFT, padx=8)
        ttk.Label(bf, text=URL + "  (启动后自动打开浏览器)", foreground="#0a0").pack(
            side=tk.LEFT, padx=16)

        lf = ttk.LabelFrame(f, text="运行状态", padding=4)
        lf.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.txt = tk.Text(lf, height=13, font=("Consolas", 9), state="disabled")
        sb = ttk.Scrollbar(lf, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # ---------------- 数据源 ----------------
    def refresh_ports(self):
        if list_ports is None:
            self.cb_port["values"] = []
            self._append("未安装 pyserial, 无法自动枚举串口 — 请运行: pip install pyserial (或直接手动输入端口号)")
            return
        entries = []
        for p in list_ports.comports():
            desc = (p.description or "").lower()
            prio = 1 if ("usb" in desc or "ch340" in desc or "cp210" in desc
                         or "serial" in desc or "uart" in desc) else 0
            entries.append((prio, p.device, p.description))
        entries.sort(key=lambda x: (-x[0], x[1]))
        self.cb_port["values"] = [e[1] for e in entries]
        if entries:
            self.cb_port.set(entries[0][1])

    def refresh_logs(self):
        logs = sorted(glob.glob(os.path.join(LOG_DIR, "delta2c_*_raw.log")), reverse=True)
        items = ["自动(数据最丰富的日志)"] + [os.path.basename(p) for p in logs]
        self.cb_log["values"] = items
        if items:
            self.cb_log.set(items[0])

    def _mode_changed(self):
        serial_mode = self.mode.get() == "serial"
        self.cb_port.config(state="normal" if serial_mode else "disabled")
        self.btn_refresh.config(state="normal" if serial_mode else "disabled")
        self.cb_log.config(state="readonly" if not serial_mode else "disabled")

    # ---------------- 启动/停止 ----------------
    def start(self):
        if self.proc and self.proc.poll() is None:
            return
        if self.mode.get() == "serial":
            port = self.cb_port.get().strip()
            if not port:
                messagebox.showwarning("提示", "请选择或直接输入串口(雷达的 USB 转串口, 通常不是 COM1/COM2)")
                return
            cmd = [sys.executable, VIEWER, "--port", port]
        else:
            choice = self.cb_log.get()
            if not choice:
                messagebox.showwarning("提示", "请选择日志")
                return
            arg = "latest" if choice.startswith("自动") else os.path.join(LOG_DIR, choice)
            cmd = [sys.executable, VIEWER, "--log", arg]
        self._append(">>> 启动: " + " ".join(cmd))
        try:
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except Exception as e:
            self._append("启动失败: %r" % e)
            return
        threading.Thread(target=self._reader, daemon=True).start()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")

    def _reader(self):
        for line in iter(self.proc.stdout.readline, ""):
            self.out_q.put(line.rstrip())
        self.out_q.put("__EOF__")

    def _poll(self):
        try:
            while True:
                line = self.out_q.get_nowait()
                if line == "__EOF__":
                    self._append("进程已退出")
                    self.btn_start.config(state="normal")
                    self.btn_stop.config(state="disabled")
                    self.proc = None
                else:
                    self._append(line)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate()
            except Exception:
                pass
            self._append("已停止")

    def _append(self, s):
        self.txt.config(state="normal")
        self.txt.insert(tk.END, s + "\n")
        self.txt.see(tk.END)
        self.txt.config(state="disabled")
        if int(self.txt.index("end-1c").split(".")[0]) > 2000:
            self.txt.delete("1.0", "800.0")

    def on_close(self):
        self.stop()
        try:
            self.root.destroy()
        except Exception:
            pass


def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
