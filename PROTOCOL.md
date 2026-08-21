# DELTA-2C PRO Protocol

> **UART 115200 8N1** · Verified byte-exact against real captures
> (618/618 measurement frames, 0 checksum failures).

All multi-byte fields are **big-endian** (network byte order). The 2-byte checksum
is excluded from the `len` field.

---

## 0xAE — Speed-only frame (11 bytes)

```
AA | len=9 | ver=0x13 | 0x61 | AE | dlen=1 | xx | chk(u16 BE)
```

- `len` = 0x0009 (9 bytes before checksum)
- `xx` = scan frequency × 20  →  **Hz = xx / 20**
- `chk` = 16-bit sum of the 9 preceding bytes

No angle or range data in this frame type. The lidar emits 0xAE frames when the
rotor speed is outside the measurement window (typically above ~8 Hz).

---

## 0xAD — Measurement frame (8 + 7 + 3N + 2 bytes)

```
AA | len | ver=0x13 | 0x61 | AD | dlen=7+3N | xx | off(i16) | start_angle(u16)
    | end_angle(u16) | N × [q d_hi d_lo] | chk(u16 BE)
```

### Header fields

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | `0xAA` | Frame sync byte |
| 1–2 | 2 | `len` | Packet length **excluding** the 2-byte checksum; `len = 8 + dlen` |
| 3 | 1 | `ver` | Protocol version: **0x13** (2C PRO); other Delta-family members use 0x01 |
| 4 | 1 | `type` | Always 0x61 |
| 5 | 1 | `dtype` | **0xAD** = measurement, **0xAE** = speed-only |
| 6–7 | 2 | `dlen` | Data length (bytes after the dlen field, before checksum); `dlen = 7 + 3N` |
| 8 | 1 | `xx` | Scan frequency × 20 → Hz = xx / 20 |
| 9–10 | 2 | `off` | Signed offset angle × 100 (degrees) — not used in angle computation |
| 11–12 | 2 | `start_angle` | Start angle × 100 (unsigned, 0–36000) |
| 13–14 | 2 | `end_angle` | End angle × 100 (unsigned, 0–36000) |

### Sample layout (N samples)

Each sample is **3 bytes**:

```
[q (1 byte)] [d_hi (1 byte)] [d_lo (1 byte)]
```

- `q` = quality / echo intensity (0 = invalid point; discard)
- `distance_mm ≈ ((d_hi << 8) | d_lo) × 0.5`  (approximately 0.5 mm per unit —
  empirically determined; subject to calibration verification)
- `d == 0` with q ≠ 0 → no echo (no return received)
- `d > 8000 mm` → treat as no echo (saturation artifact; some units emit ~0x3FE0 = 8.17 m for no-return samples)

### Sample angle

```
angle_k = start_angle × 0.01 + (end_angle − start_angle) × 0.01 × (k + 0.5) / N
```

where `k = 0, 1, ..., N-1`.

### Checksum

```
chk = sum(frame[0] … frame[len-1]) & 0xFFFF
```

Stored big-endian in the last 2 bytes of the frame. The `len` field is the number
of bytes before the checksum.

### Revolution structure

- **16 packets per revolution** (measured). The 16 start angles form a grid
  starting at ~90°: 90, 112.5, 135, …, 337.5, 0, 22.5, 45, 67.5, then back to 90.
- **N** varies per packet (14–16 samples), giving **~230 points per revolution**.
- Frame rate ≈ 7 Hz → ~1600 sampling points/second at 7.9 Hz.

---

## No-echo & saturation handling

Two types of no-return samples have been observed:

1. **d = 0, q ≠ 0** — clean no-echo. The sensor reports quality but zero distance.
2. **d ≈ 0x3FE0 (16352 units ≈ 8.17 m)** — possible saturation sentinel on some
   units. The sensor may report a max-count value when no echo arrives within the
   measurement window.

The tools in this repository treat both as no-return (d > 8000 mm → set to 0).

---

## Difference from standard kaiaai Delta family (2A/2B/2D/2G)

| Feature | 2A/2B/2D/2G | 2C PRO (this) |
|---------|-------------|---------------|
| Protocol version | 0x01 | **0x13** |
| end_angle field | absent | **present** (2 bytes after start_angle) |
| Sample byte order | [quality][dist_u16 BE] | **same**: [quality][dist_u16 BE] |
| Distance scale | 0.25 mm/unit | **0.5 mm/unit** (2C empirical) |
| Packets per rev | 16 | 16 (same) |
| Checksum | 16-bit sum of preceding bytes | same |

**Note**: The kaiaai C++ library (LDS) checks `protocol_version == 0x01` and currently
rejects 2C frames. The proposed 2C variant in this repository patches the version
check and adds the end_angle field.

---

## Motor voltage & speed observations

| Voltage | Speed | Frames | Notes |
|---------|-------|--------|-------|
| ~3.3 V | ~9.3 Hz | 0xAE only | Above measurement window |
| 2.9 V | ~7.9 Hz | 0xAD + 0xAE | First 0xAD appears |
| **2.6 V** | **~7 Hz** | 0xAD | **Most stable** (user test) |
| <2.5 V | ~5 Hz | Marginal | May stop transmitting 0xAD |

- **VCC must be 5 V** (logic supply; 3.3 V damages the ToF circuit)
- **M+ / M−** is motor-only (2.5–3.3 V range)
- Different units may have slightly different voltage↔Hz curves
- The rotor has no PWM/EN pin — the module is fully autonomous