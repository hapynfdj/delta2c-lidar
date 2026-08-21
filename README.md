# DELTA-2C PRO — Support for the kaiaai/LDS ecosystem

This repository provides **DELTA-2C PRO** (Camsense / 3irobotix) lidar support for
the [kaiaai/LDS](https://github.com/kaiaai/LDS) Arduino library, plus a set of
**Python debug/diagnostic tools** useful for reverse-engineering and testing this
LDS variant.

The DELTA-2C PRO is the teardown lidar from the **Xiaomi Robot Vacuum 2** (米家
扫拖机器人2). Its label reads `DELTA2C Pro-D-V001`; the manufacturer is
**Shenzhen LD Robot Co., Ltd.** (乐动机器人).

**This is a work-in-progress contribution to the kaiaai/LDS ecosystem.** The
Arduino library variant (`LDS_DELTA_2C_115200`) is being prepared for a pull
request to the upstream repository. The Python tools here are supplementary
utilities for testing and calibration.

---

## Supported LDS models (kaiaai/LDS + this contribution)

The 2C PRO joins the growing list of supported LDS variants:

- **DELTA-2C PRO** (this) — **experimental**, protocol version 0x13, UART 115200
- LDROBOT LD06, LD19 (same protocol as LD19)
- Neato XV11, XV11H
- Slamtec RPLIDAR A1/A2/A3, S1, S2, S3
- YDLIDAR X4, X4-PRO, X3, X3-PRO, X2/X2L, SCL, G4, G6, TG series
- Camsense LDS02RR, LDS08RR, LDS30RR, LDS03RR, LDS10RR
- 3irobotix Delta-2A, -2B, -2D, -2G
- 3irobotix Delta-2C (115200 baud, this contribution)
- Seeed Studio LD08, LD10
- Xiaomi LDS01RR
- Various other models (see [kaiaai/LDS](https://github.com/kaiaai/LDS) for full list)

---

## DELTA-2C PRO differences from the Delta family (2A/2B/2D/2G)

| Feature | 2A/2B/2D/2G | 2C PRO |
|---------|-------------|--------|
| Protocol version | 0x01 | 0x13 |
| end_angle field | absent | present (2 bytes after start_angle) |
| Sample layout | `[quality][dist_u16 BE]` | same |
| Distance scale | 0.25 mm/unit | 0.5 mm/unit (2C empirical) |
| Packets per rev | 16 | 16 |
| Checksum | 16-bit sum of preceding bytes | same |

The sample byte order is **identical** to the existing Delta family (quality byte
first, then distance as a big-endian u16). The main differences are the protocol
version, the extra 2-byte end_angle in the packet header, and the distance
resolution (0.5 mm/unit for the 2C vs. 0.25 mm/unit for the other variants).

---

## Hardware wiring (5-pin)

| Pin | Connect to |
|-----|-----------|
| M+ / M− | Adjustable DC supply for the rotor motor (2.5–3.3 V range) |
| VCC / GND | Logic supply **5 V** (must be 5 V) |
| TX | USB-TTL **RX** (cross), share GND |

No RX, no PWM, no EN pin — the module is fully autonomous. The **only tuning
parameter is the motor voltage**. If the lidar only emits 0xAE speed frames,
the rotor speed is above the measurement window — lower the voltage.

Motor voltage observations (individual units may vary):
- ~3.3 V → ~9.3 Hz, only 0xAE frames
- ~2.9 V → ~7.9 Hz, 0xAD measurement frames appear
- **~2.6 V → ~7 Hz, most stable** (experimental finding)
- Below ~2.5 V → ~5 Hz, may stop transmitting 0xAD

---

## Tools

### C++ Arduino library variant (for kaiaai/LDS)

| File | Description |
|------|-------------|
| `LDS_DELTA_2C_115200.h` | Header for the 2C variant (inherits from `LDS` base class) |
| `LDS_DELTA_2C_115200.cpp` | Implementation: 0xAD frame parsing, PID motor control |

These files follow the same convention as the other Delta variants in the
kaiaai/LDS repository. They are the primary contribution for the upstream PR.

### Python diagnostic tools (standalone)

| Tool | Purpose |
|------|---------|
| `delta2c_debug_gui.pyw` | Debug GUI: live speed, 0xAE/0xAD detection, dual logging, bilingual UI |
| `lidar_webview.py` | 3D point cloud (Three.js) + 2D overhead + occupancy grid mapping |
| `lidar_viewer_gui.pyw` | Launcher GUI for the web viewer |
| `foxglove_bridge.py` | Streams `foxglove.LaserScan` over WebSocket to Foxglove Studio |
| `render_scan_from_log.py` | Offline RViz-style point cloud PNG renderer |
| `analyze_delta2c_log.py` | Log forensic analysis (frame stats, wall histogram, event detection) |

Dependencies for Python tools: `pip install pyserial websockets numpy pillow matplotlib`

---

## Quick start

```bash
# Debug GUI (voltage tuning, live Hz)
python delta2c_debug_gui.pyw

# Web 3D viewer + occupancy grid (auto-opens http://127.0.0.1:8080)
python lidar_webview.py --port COM5
# or replay a saved log:
python lidar_webview.py --log latest
```

Windows users can double-click the `启动*.bat` scripts.

---

## Protocol summary (detailed in PROTOCOL.md)

- UART **115200 8N1**, frame sync byte `0xAA`, protocol version **0x13**
- **0xAE** speed-only frame: `Hz = xx / 20`
- **0xAD** measurement frame: `AA|len|13|61|AD|dlen=7+3N|xx|off|start_angle|end_angle|N×[q d_hi d_lo]|chk`
  - `dist_mm = ((d_hi<<8)|d_lo) × 0.5` (2C uses 0.5 mm/unit)
  - `q` = quality, 0 = invalid; d == 0 or d > 8000 mm → no return
  - Sample angle = `start + (end−start)×(k+0.5)/N`; 16 packets/rev, ~230 points/rev
  - Checksum = 16-bit sum of all preceding bytes

---

## Known pitfalls (detailed in docs/踩坑记录.md)

- **Sample byte order**: the 2C sample layout is `[quality][dist_u16 BE]`—same as
  kaiaai's Delta family. A parser that reads `[dist][quality]` instead will
  produce a circular map artifact (the quality byte, ~0x90–0x97, becomes the
  distance high byte, creating a ring at ~4.8 m/9.6 m).
- **Distance scale**: 0.5 mm/unit vs. 0.25 mm/unit for the other Delta variants.
  Using the wrong scale shrinks/expands the map by 2×.
- **No-echo saturation**: some units emit ~0x3FE0 (8.17 m) for no-return samples.
  In a room < 6 m this is a sentinel, not a wall.
- **kaiaai/LDS version check**: the library checks `protocol_version == 0x01` and
  will reject 2C frames (version 0x13). The proposed 2C variant patches this.
- **VCC must be 5 V**: 3.3 V was observed to degrade one unit's sensitivity
  (3.2 m near-field blind zone appeared).
- **Motor voltage gating**: measurement frames only appear in a ~7–8 Hz window.
  Higher speeds produce only 0xAE frames.

---

## Sample data (samples/)

- `delta2c_20260820_164235_raw.log` — 226 KB raw capture (618 measurement frames)
- `map_fixed_1726.png` / `map_fixed_1733.png` — occupancy grid renders of larger
  room scans (fixed-parser)
- `sample_164235.png` — RViz-style scatter plot of the sample log

---

## License

The Python tools and documentation in this repository are **MIT** (see LICENSE).
The C++ Arduino library variant (`LDS_DELTA_2C_115200.h/.cpp`) is intended for
contribution to the kaiaai/LDS repository, which is **Apache 2.0**.

---

## Contributing

This is a contribution to the [kaiaai/LDS](https://github.com/kaiaai/LDS)
ecosystem. Pull requests and issues are welcome. For the Arduino library variant,
please follow the conventions of the upstream repository.

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.