#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DELTA-2C 3D/2D web viewer
==========================
Self-contained lidar viewer for the Camsense DELTA-2C PRO (Xiaomi Robot
Vacuum 2 teardown). Starts a local web page: 3D point cloud (WebGL/GPU via
Three.js) with a 2D top-down inset, mouse orbit/zoom. No installation,
no third-party viewer — open http://127.0.0.1:8080 in any browser.

Usage:
    python lidar_webview.py --port COM5        # live from the lidar
    python lidar_webview.py --log latest       # replay newest saved log
    python lidar_webview.py                    # auto: port if 1 COM, else latest log
"""
import argparse
import asyncio
import base64
import json
import numpy as np
import math
import os
import queue
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from delta2c_debug_gui import DeltaParser  # noqa: E402

HTTP_PORT = 8080
WS_PORT = 8766

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def latest_log():
    """Newest saved log that actually contains real scan data (>=3000 valid points)."""
    import glob
    logs = sorted(glob.glob(os.path.join(LOG_DIR, "delta2c_*_raw.log")), reverse=True)
    if not logs:
        return None
    parser = DeltaParser()
    for path in logs:
        if os.path.getsize(path) < 200_000:
            continue
        print("checking %s ..." % os.path.basename(path), flush=True)
        try:
            data = read_raw_log(path)
        except Exception:
            continue
        valid = [0]

        def count(ev, *a):
            if ev == "meas":
                valid[0] += sum(1 for p in a[5] if p[1] != 0)

        parser.feed(data, count)
        if valid[0] >= 3000:
            return path
    return logs[0] if logs else None


HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>DELTA-2C 激光雷达 3D 查看器</title>
<link rel="icon" href="data:,">
<style>
  html,body{margin:0;height:100%;overflow:hidden;background:#0d1117;color:#c9d1d9;
    font:13px/1.5 "Segoe UI","Microsoft YaHei",sans-serif}
  #ui{position:fixed;top:10px;left:10px;z-index:10;background:rgba(13,17,23,.88);
    border:1px solid #30363d;border-radius:8px;padding:10px 12px;max-width:360px}
  #ui b{font-size:15px}
  .btn{display:inline-block;margin:4px 4px 0 0;padding:5px 12px;background:#21262d;
    color:#c9d1d9;border:1px solid #30363d;border-radius:6px;cursor:pointer;user-select:none}
  .btn:hover{background:#30363d}
  #stats{color:#8b949e;font-size:12px;margin-top:6px}
  #hint{color:#6e7681;font-size:11px;margin-top:4px}
  #cvmap{position:fixed;inset:0;z-index:1;width:100%;height:100%;background:#1c1c1c}
  #cv2d{position:fixed;right:12px;bottom:12px;z-index:10;width:230px;height:230px;
    border:1px solid #30363d;border-radius:8px;background:rgba(13,17,23,.88)}
  #status{color:#58a6ff}
</style>
</head>
<body>
<div id="ui">
  <b>DELTA-2C 激光雷达</b> <span id="status">连接中…</span>
  <div id="stats">等待数据…</div>
  <div id="stats2" style="color:#58a6ff;font-size:12px">地图: 构建中…</div>
  <div>
    <span class="btn" onclick="setViewMode('map')">建图地图</span>
    <span class="btn" onclick="setViewMode('3d'); setView('3d')">3D 视图</span>
    <span class="btn" onclick="saveMap()">保存地图</span>
    <span class="btn" onclick="toggleScale()">距离×2切换</span>
    <span class="btn" onclick="setView('top')">俯视 2D</span>
    <span class="btn" onclick="toggleRotate()">自动旋转</span>
    <span class="btn" onclick="clearPts()">清屏</span>
    <span class="btn" onclick="toggleAccum()">累积模式</span>
    <span class="btn" onclick="setRange(6)">量程6m</span>
    <span class="btn" onclick="setRange(12)">量程12m</span>
    <span class="btn" onclick="setRange(24)">量程24m</span>
  </div>
  <div id="hint">左键拖拽旋转 · 滚轮缩放 · 右下角为俯视 2D 图(上=前) · 暗灰=无回波/近距杂波 · 建图地图=占用栅格(黑墙白地灰未知, 1秒更新) · 可保存PNG</div>
</div>
<canvas id="cvmap"></canvas>
<canvas id="cv2d" width="460" height="460"></canvas>
<script type="importmap">
{
  "imports": {
    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
var scene, camera, renderer, controls, ptsGeo, sweepSphere, robotArrow;
var data = [], hzText = "--", autoRot = false;
var rangeM = 6, noEchoM = 11.0, grid = null;   // 无回波(>11m)与近距(<0.15m)压暗
var mapCanvas = document.getElementById('cvmap'), mapCtx = mapCanvas.getContext('2d');
var mapImg = null, mapStatsEl = document.getElementById('stats2'), viewMode = 'map', mapRes = 0.05, scaleMult = 1.0;
var accumMode = false, accum = [];
var statsEl = document.getElementById('stats');
var statusEl = document.getElementById('status');

function jet(v){ // 0..1 -> [r,g,b] jet colormap
  v = Math.max(0, Math.min(1, v));
  return [
    Math.max(0, Math.min(1, 1.5 - Math.abs(4*v - 3))),
    Math.max(0, Math.min(1, 1.5 - Math.abs(4*v - 2))),
    Math.max(0, Math.min(1, 1.5 - Math.abs(4*v - 1)))
  ];
}

function init(){
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d1117);
  camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.05, 200);
  camera.position.set(9, 7, 11);
  camera.lookAt(0,0,0);
  renderer = new THREE.WebGLRenderer({antialias:true});
  renderer.setSize(innerWidth, innerHeight);
  document.body.appendChild(renderer.domElement);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0,0,0);
  scene.add(new THREE.AxesHelper(1.5));
  setRange(6);
  // robot: red arrow, front = +Y
  robotArrow = new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(0,0,0),
                                     1.1, 0xff5555, 0.4, 0.25);
  scene.add(robotArrow);
  // scan points
  ptsGeo = new THREE.BufferGeometry();
  var mat = new THREE.PointsMaterial({size:0.12, vertexColors:true, sizeAttenuation:true});
  scene.add(new THREE.Points(ptsGeo, mat));
  // sweep head marker
  sweepSphere = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 12),
                               new THREE.MeshBasicMaterial({color:0xffffff}));
  sweepSphere.visible = false;
  scene.add(sweepSphere);
  addEventListener('resize', function(){
    camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });
  animate();
}

function toggleAccum(){
  accumMode = !accumMode;
  if (!accumMode){ accum = []; data = []; ptsGeo.setDrawRange(0,0); sweepSphere.visible = false; }
}

function setRange(m){
  rangeM = m;
  if (grid){ scene.remove(grid); }
  grid = new THREE.GridHelper(rangeM*2, rangeM*2, 0x30363d, 0x21262d);
  grid.rotation.x = Math.PI/2;
  scene.add(grid);
  draw2d();
}

function updatePts(){
  var n = data.length;
  var pos = new Float32Array(n*3), col = new Float32Array(n*3);
  var near = 1e9, far = -1, last = null, real = 0;
  for (var i=0;i<n;i++){
    var a = data[i][0]*Math.PI/180, d = data[i][1]/1000, q = data[i][2];
    var x = d*Math.sin(a), y = d*Math.cos(a);   // a=0 -> +Y (front)
    pos[i*3]=x; pos[i*3+1]=y; pos[i*3+2]=0;
    if (d >= noEchoM*scaleMult || d < 0.15*scaleMult){   // 无回波或低于最小量程 -> 暗灰, 不参与统计
      col[i*3]=0.28; col[i*3+1]=0.28; col[i*3+2]=0.28;
    } else {
      var c = jet(d/rangeM);
      col[i*3]=c[0]; col[i*3+1]=c[1]; col[i*3+2]=c[2];
      real++;
      if (d<near) near=d;
      if (d>far) far=d;
      last = {x:x, y:y};
    }
  }
  ptsGeo.setAttribute('position', new THREE.BufferAttribute(pos,3));
  ptsGeo.setAttribute('color', new THREE.BufferAttribute(col,3));
  ptsGeo.setDrawRange(0, n);
  if (last){ sweepSphere.position.set(last.x, last.y, 0); sweepSphere.visible = true; }
  if (real === 0){ near = 0; far = 0; }
  statsEl.textContent = '真实点 ' + real + ' · 最近 ' + near.toFixed(2) +
    ' m · 最远 ' + far.toFixed(2) + ' m · ' + hzText + ' · 量程 ' + rangeM + 'm';
  draw2d();
}

function draw2d(){
  var cv = document.getElementById('cv2d'), ctx = cv.getContext('2d');
  var W = cv.width, H = cv.height, cx = W/2, cy = H/2, scale = (W/2 - 14)/rangeM;
  ctx.fillStyle = '#0d1117'; ctx.fillRect(0,0,W,H);
  ctx.strokeStyle = '#1f242c';
  for (var m=1; m<=rangeM; m++){
    ctx.beginPath(); ctx.arc(cx, cy, m*scale, 0, Math.PI*2); ctx.stroke();
  }
  ctx.strokeStyle = '#30363d';
  ctx.beginPath(); ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.moveTo(cx,0); ctx.lineTo(cx,H); ctx.stroke();
  for (var i=0;i<data.length;i++){
    var a = data[i][0]*Math.PI/180, dm = data[i][1]/1000;
    if (data[i][2] === 0 || dm >= noEchoM*scaleMult || dm < 0.15*scaleMult) continue;
    var d = Math.min(dm, rangeM);
    var x = cx + d*Math.sin(a)*scale, y = cy - d*Math.cos(a)*scale;
    var c = jet(dm/rangeM);
    ctx.fillStyle = 'rgb(' + (c[0]*255|0) + ',' + (c[1]*255|0) + ',' + (c[2]*255|0) + ')';
    ctx.fillRect(x-1.5, y-1.5, 3, 3);
  }
  ctx.fillStyle = '#ff5555';
  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI*2); ctx.fill();
  ctx.fillStyle = '#c9d1d9'; ctx.font = '11px sans-serif';
  ctx.fillText('前', cx-7, cy - 12);
}

function setViewMode(mode){
  viewMode = mode;
  var tc = renderer.domElement;
  tc.style.display = (mode === '3d') ? 'block' : 'none';
  mapCanvas.style.display = (mode === 'map') ? 'block' : 'none';
  if (mode === 'map') drawMap();
  resizeMap();
}

function resizeMap(){
  mapCanvas.width = innerWidth;
  mapCanvas.height = innerHeight;
  drawMap();
}

function drawMap(){
  if (!mapImg) return;
  var W = mapCanvas.width, H = mapCanvas.height;
  mapCtx.fillStyle = '#1c1c1c'; mapCtx.fillRect(0,0,W,H);
  var tmp = document.createElement('canvas');
  tmp.width = mapImg.width; tmp.height = mapImg.height;
  tmp.getContext('2d').putImageData(mapImg, 0, 0);
  var s = Math.min(W, H) / mapImg.width;
  var ox = (W - mapImg.width*s)/2, oy = (H - mapImg.height*s)/2;
  mapCtx.imageSmoothingEnabled = false;
  mapCtx.drawImage(tmp, ox, oy, mapImg.width*s, mapImg.height*s);
  // 1 米方格线
  mapCtx.strokeStyle = 'rgba(130,130,130,0.4)';
  mapCtx.lineWidth = 1;
  var cpm = Math.round(1 / mapRes), ms = mapImg.width * s;
  for (var g=1; g<mapImg.width/cpm; g++){
    var p = ox + g*cpm*s;
    mapCtx.beginPath(); mapCtx.moveTo(p, oy); mapCtx.lineTo(p, oy+ms); mapCtx.stroke();
    mapCtx.beginPath(); mapCtx.moveTo(ox, p); mapCtx.lineTo(ox+ms, p); mapCtx.stroke();
  }
  // 中心标记(雷达位置)
  mapCtx.fillStyle = '#ff5555';
  mapCtx.beginPath(); mapCtx.arc(ox + mapImg.width*s/2, oy + mapImg.height*s/2, 4, 0, Math.PI*2); mapCtx.fill();
}

function renderMap(m){
  var n = m.n;
  var bin = atob(m.cells), bytes = new Uint8Array(n*n);
  for (var i=0;i<n*n;i++) bytes[i] = bin.charCodeAt(i);
  if (!mapImg || mapImg.width !== n) mapImg = mapCtx.createImageData(n, n);
  var d = mapImg.data;
  for (var i=0;i<n*n;i++){
    var v = bytes[i], off = i*4;
    if (v === 0){ d[off]=60; d[off+1]=60; d[off+2]=60; }
    else if (v === 128){ d[off]=245; d[off+1]=245; d[off+2]=245; }
    else { d[off]=12; d[off+1]=12; d[off+2]=12; }
    d[off+3]=255;
  }
  mapRes = m.res;
  mapStatsEl.textContent = '地图: 占用 ' + (100*m.occ/m.tot).toFixed(1) +
    '% · 已扫描 ' + m.hits + ' 点 · ' + (m.n*m.res).toFixed(1) + 'm 见方 · 1格=1m';
  drawMap();
}

function toggleScale(){
  if (ws && ws.readyState === 1){ ws.send(JSON.stringify({op:'toggle_scale'})); }
}

function saveMap(){
  if (ws && ws.readyState === 1){ ws.send(JSON.stringify({op:'save_map'})); }
  mapStatsEl.textContent = '保存中…';
}

function setView(mode){
  if (mode === 'top'){
    camera.position.set(0.01, 18, 0.01); camera.up.set(0, 0, -1);
  } else {
    camera.position.set(9, 7, 11); camera.up.set(0, 1, 0);
  }
  camera.lookAt(0,0,0); controls.target.set(0,0,0); controls.update();
}
function toggleRotate(){ autoRot = !autoRot; controls.autoRotate = autoRot; controls.autoRotateSpeed = 3; }
function clearPts(){ data = []; ptsGeo.setDrawRange(0,0); sweepSphere.visible = false; draw2d(); }

function connect(){
  var ws = new WebSocket('ws://' + location.hostname + ':8766');
  ws.onopen = function(){ statusEl.textContent = '已连接'; statusEl.style.color = '#3fb950'; };
  ws.onmessage = function(e){
    var m = JSON.parse(e.data);
    if (m.type === 'map'){ renderMap(m); return; }
    if (m.type === 'saved'){ mapStatsEl.textContent = '已保存: ' + m.path; return; }
    if (m.type === 'scale'){ scaleMult = m.mult; mapStatsEl.textContent = '距离倍率: ×' + m.mult + ' (地图已重建)'; return; }
    hzText = m.hz.toFixed(2) + ' Hz';
    if (accumMode){
      accum = accum.concat(m.pts);
      if (accum.length > 6000) accum = accum.slice(-6000);
      data = accum;
    } else {
      data = m.pts;
    }
    updatePts();
  };
  ws.onclose = function(){
    statusEl.textContent = '已断开, 重连中…'; statusEl.style.color = '#f85149';
    setTimeout(connect, 1500);
  };
  ws.onerror = function(){ ws.close(); };
}

function animate(){
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

// 暴露给内联 onclick(module 作用域默认不是全局)
window.toggleAccum = toggleAccum;
window.setRange = setRange;
window.setView = setView;
window.toggleRotate = toggleRotate;
window.clearPts = clearPts;
window.setViewMode = setViewMode;
window.saveMap = saveMap;
window.toggleScale = toggleScale;

init();
connect();
addEventListener('resize', resizeMap);
setViewMode('map');
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, *a):
        pass


scan_q = queue.Queue()
clients = set()


async def ws_broadcaster():
    while True:
        try:
            payload = scan_q.get_nowait()
        except queue.Empty:
            await asyncio.sleep(0.01)
            continue
        if clients:
            await asyncio.gather(*(c.send(payload) for c in list(clients)),
                                 return_exceptions=True)


async def ws_handler(ws):
    global DIST_MULT, grid_holder
    clients.add(ws)
    try:
        async for raw in ws:
            try:
                cmd = json.loads(raw)
                if cmd.get("op") == "save_map":
                    path = os.path.join(LOG_DIR,
                                        "map_%d.png" % int(time.time()))
                    grid_holder.save_png(path)
                    await ws.send(json.dumps({"type": "saved", "path": path}))
                elif cmd.get("op") == "toggle_scale":
                    DIST_MULT = 2.0 if DIST_MULT == 1.0 else 1.0
                    grid_holder = OccupancyGrid()   # 重建地图
                    await ws.send(json.dumps({"type": "scale", "mult": DIST_MULT}))
            except Exception:
                pass
    except Exception:
        pass
    finally:
        clients.discard(ws)


grid_holder = None   # ws_handler 用; main() 里创建真实实例
DIST_MULT = 1.0      # 解析器已按 0.5mm/格 输出正确 mm; 1.0=原样, 2.0=放大2倍(调试用)


def run_ws():
    asyncio.run(websockets_main())


async def websockets_main():
    import websockets
    async with websockets.serve(ws_handler, "127.0.0.1", WS_PORT):
        await ws_broadcaster()


def bresenham(x0, y0, x1, y1):
    """Line cells from (x0,y0) to (x1,y1), inclusive."""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if (x, y) == (x1, y1):
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


class OccupancyGrid:
    """2D occupancy grid (log-odds). Black = wall, white = free, gray = unknown.
    Ray-trace every valid scan point: cells origin->point become free,
    the endpoint cell becomes occupied — the same idea SLAM mappers use."""

    RES = 0.05          # meters per cell
    SIZE_M = 24.0       # total map size (centered on the lidar)

    def __init__(self):
        self.n = int(self.SIZE_M / self.RES)
        self.grid = np.zeros((self.n, self.n), dtype=np.float32)
        self.hits = 0
        self.ox, self.oy = self._cell(0.0, 0.0)

    def _cell(self, x_m, y_m):
        return (int((x_m + self.SIZE_M / 2) / self.RES),
                int((y_m + self.SIZE_M / 2) / self.RES))

    def add_scan(self, pts, cw=True):
        for a_deg, d_mm, q in pts:
            if q == 0 or d_mm < 150:
                continue
            rad = math.radians(a_deg)
            d = d_mm / 1000.0
            if cw:
                x, y = d * math.sin(rad), d * math.cos(rad)
            else:
                x, y = -d * math.sin(rad), d * math.cos(rad)
            ex, ey = self._cell(x, y)
            if not (0 <= ex < self.n and 0 <= ey < self.n):
                continue
            for cx, cy in bresenham(self.ox, self.oy, ex, ey):
                if (cx, cy) == (ex, ey):
                    self.grid[cy, cx] = min(self.grid[cy, cx] + 0.9, 5.0)
                    self.hits += 1
                else:
                    self.grid[cy, cx] = max(self.grid[cy, cx] - 0.18, -5.0)

    def render(self):
        """0 = unknown, 128 = free, 255 = occupied."""
        img = np.full((self.n, self.n), 0, dtype=np.uint8)
        img[self.grid < -0.4] = 128
        img[self.grid > 0.6] = 255
        return img

    def map_msg(self):
        img = self.render()
        occ = int((img == 255).sum())
        free = int((img == 128).sum())
        tot = self.n * self.n
        return {
            "type": "map",
            "n": self.n,
            "res": self.RES,
            "size": self.SIZE_M,
            "cells": base64.b64encode(img.tobytes()).decode(),
            "occ": occ, "free": free, "tot": tot,
            "hits": self.hits,
        }

    def save_png(self, path):
        from PIL import Image, ImageDraw
        img = self.render()
        rgb = np.zeros((self.n, self.n, 3), dtype=np.uint8)
        rgb[img == 0] = (60, 60, 60)      # unknown gray
        rgb[img == 128] = (245, 245, 245)  # free white
        rgb[img == 255] = (10, 10, 10)     # occupied black
        im = Image.fromarray(rgb)
        d = ImageDraw.Draw(im)
        step = int(1.0 / self.RES)          # 1 米 = 20 格
        for g in range(step, self.n, step):  # 1m 方格线
            d.line([(g, 0), (g, self.n)], fill=(95, 95, 95), width=1)
            d.line([(0, g), (self.n, g)], fill=(95, 95, 95), width=1)
        im.save(path)
        return path


class ScanAssembler:
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
            out, self.pts = self.pts, []
            return out
        return None


def read_raw_log(path):
    hexlines = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            if re.fullmatch(r"([0-9A-F]{2} ?)+", s):
                hexlines.append(s)
    return bytes(int(h, 16) for h in " ".join(hexlines).split())


def main():
    ap = argparse.ArgumentParser(description="DELTA-2C 3D/2D web viewer")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port, e.g. COM5 (live)")
    src.add_argument("--log", help="replay a saved *_raw.log (loop); 'latest' for newest")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--once", action="store_true", help="replay log once")
    ns = ap.parse_args()

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
            print("multiple ports: %s — use --port explicitly" % ports)
            sys.exit(1)
        else:
            print("no serial port — replaying newest log")
            ns.log = "latest"
    if ns.log == "latest":
        ns.log = latest_log()
        if not ns.log:
            print("no saved logs found — connect the lidar and use --port COMx")
            sys.exit(1)
        print("replaying: %s" % ns.log)

    # HTTP server (web page)
    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    # WebSocket server
    threading.Thread(target=run_ws, daemon=True).start()

    url = "http://127.0.0.1:%d" % HTTP_PORT
    print("=" * 52)
    print(" 打开浏览器: %s  (3D 点云 + 俯视 2D)" % url)
    print(" 数据源: %s" % ("串口 " + ns.port if ns.port else "日志重放 " + ns.log))
    print(" Ctrl+C 退出")
    print("=" * 52)
    try:
        import webbrowser
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    except Exception:
        pass

    assembler = ScanAssembler()
    global grid_holder
    grid_holder = OccupancyGrid()
    last_print = 0.0
    last_map = 0.0
    last_png = 0.0
    hz = 0.0

    def on_meas(st_ang, en_ang, pts, xx):
        nonlocal last_print, last_map, last_png, hz
        hz = xx / 20.0
        scan = assembler.add_frame(st_ang, en_ang, pts)
        if scan:
            if DIST_MULT != 1.0:
                scan = [(a, d * DIST_MULT, q) for a, d, q in scan]
            scan_q.put(json.dumps({"type": "scan", "pts": scan, "hz": hz}))
            grid_holder.add_scan(scan)
            now = time.time()
            if now - last_print > 2.0:
                print("scan: %d pts, %.2f Hz" % (len(scan), hz), flush=True)
                last_print = now
        now = time.time()
        if now - last_map > 1.0:          # push the occupancy map ~1 Hz
            scan_q.put(json.dumps(grid_holder.map_msg()))
            last_map = now
        if now - last_png > 10.0:         # autosave map every 10 s
            try:
                grid_holder.save_png(os.path.join(LOG_DIR, "map_latest.png"))
                last_png = now
            except Exception:
                pass

    parser = DeltaParser()
    handler = lambda ev, *a: (on_meas(a[3], a[4], a[5], a[1]) if ev == "meas" else None)

    if ns.log:
        data = read_raw_log(ns.log)
        try:
            while True:
                parser.feed(data, handler)
                if ns.once:
                    break
                time.sleep(0.4)
        except KeyboardInterrupt:
            pass
        return 0

    import serial
    try:
        ser = serial.Serial(ns.port, ns.baud, timeout=0.05)
    except Exception as e:
        print("cannot open %s: %s" % (ns.port, e))
        sys.exit(1)
    try:
        while True:
            n = ser.in_waiting
            if n == 0:
                time.sleep(0.01)
                continue
            parser.feed(ser.read(n), handler)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
