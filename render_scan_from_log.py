#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Render a 2D laser point cloud (RViz LaserScan style) from a DELTA-2C raw log.

Usage:
    python render_scan_from_log.py logs/delta2c_xxx_raw.log [-o out.png] [-r 12] [--accum]

RViz style: dark background, individual points ONLY (never connected by lines),
colored by distance (Jet: blue = near, red = far), colorbar in meters.
Robot centered, up = front (red arrow), 1 grid cell = 1 m.

Default renders the last two revolutions (gray = previous). Use --accum to
render every revolution stacked (denser point cloud of the room).
"""
import re
import sys
import math
import argparse
import os

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker as mticker
    from matplotlib.patches import Circle
except ImportError:
    print("matplotlib required: pip install matplotlib")
    sys.exit(1)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from delta2c_debug_gui import DeltaParser, FIXED_RANGE_M  # noqa: E402

CW = True  # sweep direction on map; flip to False if the map is mirrored


def xy(a_deg, d_mm):
    rad = math.radians(a_deg)
    d = d_mm / 1000.0
    if CW:
        return d * math.sin(rad), d * math.cos(rad)
    return -d * math.sin(rad), d * math.cos(rad)


def setup_axes(ax, R):
    ax.set_facecolor("#1c1c1c")
    ax.set_aspect("equal")
    ax.set_xlim(-R, R)
    ax.set_ylim(-R, R)
    ax.xaxis.set_major_locator(mticker.MultipleLocator(2))
    ax.xaxis.set_minor_locator(mticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(mticker.MultipleLocator(2))
    ax.yaxis.set_minor_locator(mticker.MultipleLocator(1))
    ax.grid(True, which="major", ls="-", alpha=0.5, color="#3d3d3d")
    ax.grid(True, which="minor", ls=":", alpha=0.25, color="#2a2a2a")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    for t in ax.get_xticklabels() + ax.get_yticklabels():
        t.set_color("#9e9e9e")
    ax.xaxis.label.set_color("#9e9e9e")
    ax.yaxis.label.set_color("#9e9e9e")
    for sp in ax.spines.values():
        sp.set_color("#555555")


def draw_robot(ax, R):
    ax.add_patch(Circle((0, 0), 0.15, fc="#ECEFF1", ec="#B0BEC5", lw=1.5, zorder=5))
    ax.annotate("", xy=(0, 0.62), xytext=(0, 0.05),
                arrowprops=dict(arrowstyle="-|>", color="#F44336", lw=2.5), zorder=5)
    ax.text(0, R * 0.93, "FRONT", ha="center", fontsize=11, fontweight="bold", color="#CFD8DC")
    ax.text(0, -R * 0.93, "BACK", ha="center", fontsize=10, color="#6d6d6d")
    ax.text(-R * 0.93, 0, "LEFT", ha="center", va="center", fontsize=10, color="#6d6d6d")
    ax.text(R * 0.93, 0, "RIGHT", ha="center", va="center", fontsize=10, color="#6d6d6d")


def main():
    ap = argparse.ArgumentParser(description="Render DELTA-2C log to an RViz-style point cloud PNG")
    ap.add_argument("log", help="path to *_raw.log")
    ap.add_argument("-o", "--out", default="scan_map.png", help="output PNG")
    ap.add_argument("-r", "--range", type=float, default=FIXED_RANGE_M,
                    help="map half-range in meters (default %g)" % FIXED_RANGE_M)
    ap.add_argument("--accum", action="store_true",
                    help="stack all revolutions (denser cloud); default = last two")
    ap.add_argument("--ccw", action="store_true", help="sweep direction counter-clockwise")
    ns = ap.parse_args()
    global CW
    CW = not ns.ccw

    hexlines = []
    with open(ns.log, encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            if re.fullmatch(r"([0-9A-F]{2} ?)+", s):
                hexlines.append(s)
    data = bytes(int(h, 16) for h in " ".join(hexlines).split())

    evs = []
    DeltaParser().feed(data, lambda ev, *a: evs.append((ev, a)))
    meas = [e for e in evs if e[0] == "meas"]
    if not meas:
        print("no 0xAD measurement frames found in the log")
        return 1

    # group into revolutions
    rings = []
    cur = []
    last_st = None
    for ev, args in meas:
        fr, xx, off, st_ang, en_ang, pts = args
        st = st_ang * 0.01
        en = en_ang * 0.01
        n = len(pts)
        if n == 0:
            continue
        if last_st is not None and st < last_st - 90:
            if cur:
                rings.append(cur)
            cur = []
        last_st = st
        for k, (d, q) in enumerate(pts):
            if q == 0 or d == 0:   # d==0 且 q!=0 也是无回波
                continue
            a = (st + (en - st) * (k + 0.5) / n) % 360.0
            cur.append((a, d))
    if cur:
        rings.append(cur)
    print("measurement frames: %d, revolutions: %d" % (len(meas), len(rings)))

    R = ns.range
    fig, ax = plt.subplots(figsize=(8.4, 8.4), dpi=110, facecolor="#1c1c1c")
    setup_axes(ax, R)

    if ns.accum:
        draw_rings = rings
        sc_prev = None
    else:
        draw_rings = [rings[-1]]
        sc_prev = rings[-2] if len(rings) > 1 else None

    if sc_prev:
        ax.scatter([xy(a, d)[0] for a, d in sc_prev], [xy(a, d)[1] for a, d in sc_prev],
                   s=7, c="#6d6d6d", alpha=0.55, zorder=1)

    all_pts = [(a, d) for ring in draw_rings for a, d in ring]
    ax.scatter([xy(a, d)[0] for a, d in all_pts], [xy(a, d)[1] for a, d in all_pts],
               s=8, c=[d / 1000.0 for a, d in all_pts], cmap="jet", vmin=0, vmax=R, zorder=3)
    if len(all_pts) > 24:
        recent = all_pts[-24:]
        ax.scatter([xy(a, d)[0] for a, d in recent], [xy(a, d)[1] for a, d in recent],
                   s=34, c="#E91E63", zorder=4)
    sc = ax.scatter([], [], c=[], cmap="jet", vmin=0, vmax=R)
    fig.colorbar(sc, ax=ax, pad=0.02).set_label("m")

    draw_robot(ax, R)
    if all_pts:
        nd = min(all_pts, key=lambda p: p[1])
        fd = max(all_pts, key=lambda p: p[1])
        ax.set_title("nearest %.2f m @ %.0f deg  |  farthest %.2f m  (%d pts)"
                     % (nd[1] / 1000.0, nd[0], fd[1] / 1000.0, len(all_pts)),
                     color="#cfd8dc")

    fig.tight_layout()
    fig.savefig(ns.out, dpi=110, facecolor="#1c1c1c")
    print("saved:", ns.out, "(%d points)" % len(all_pts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
