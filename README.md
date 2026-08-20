# DELTA-2C PRO Lidar Toolkit

逆向工程工具包：**Camsense（欢创）/ 3irobotix DELTA-2C PRO** 激光雷达 —— 小米扫地机器人 2（米家扫拖机器人 2）拆机件。标签：`DELTA2C Pro-D-V001`。

包含：完整协议说明（`PROTOCOL.md`）、UART 抓包解析器、调试 GUI（电压调校/实时转速/双日志）、网页 3D 点云查看器、占用栅格建图（SLAM 风格）、Foxglove 桥接、日志取证分析工具、以及从零开始的踩坑记录（`docs/踩坑记录.md`）。

> 协议已字节级验证（618/618 测距帧校验通过）。**2C 与其他 Delta 家族成员（2A/2B/2D/2G）不兼容**：协议版本 0x13、有 end_angle 字段、距离分辨率 0.5 mm/格。

---

## 硬件接线（5 针）

| 引脚 | 接法 |
|---|---|
| **M+ / M−** | 可调直流电源，只给转子电机。**2.6 V ≈ 7 Hz 最稳**（实测；个体有差异）。~7.9 Hz 开始出测距帧，~9.3 Hz 只出转速帧 |
| **VCC / GND** | 逻辑供电 **5 V**（必须 5 V，3.3 V 会损伤 ToF 测距电路，实测发生过失灵） |
| **TX** | USB-TTL 的 **RX**（交叉），共地 |

模块全自主运行，无 RX / 无 PWM / 无 EN 引脚——**唯一可调的就是电机电压**。若串口只有 0xAE 转速帧，说明转子速度不在测距窗口内，调低电压。

## 工具一览

| 工具 | 用途 |
|---|---|
| `delta2c_debug_gui.pyw` | 调试 GUI：实时转速、电压调校、0xAD/0xAE 自动识别、原始+解析双日志、中英文 |
| `lidar_webview.py` | 网页 3D 点云 + 2D 俯视 + **占用栅格建图**（黑墙白地灰未知，SLAM 风格，1 秒更新，可保存 PNG） |
| `lidar_viewer_gui.pyw` | 启动器 GUI：选串口/日志一键启动查看器 |
| `foxglove_bridge.py` | 推流标准 `foxglove.LaserScan` 到 Foxglove Studio（RViz 风格，GPU 渲染） |
| `render_scan_from_log.py` | 离线把 raw 日志渲染成 RViz 风格点云 PNG |
| `analyze_delta2c_log.py` | 日志取证：帧/校验统计、墙距直方图、时间轴、帧内平直段事件检测（手/胸/墙调试利器） |

依赖：`pip install pyserial websockets numpy pillow matplotlib`

## 快速开始

```bash
# 1) 调试 GUI（验证接线/调电机电压到 7Hz 左右）
python delta2c_debug_gui.pyw

# 2) 网页 3D 查看器 + 占用栅格建图（浏览器自动打开 http://127.0.0.1:8080）
python lidar_webview.py --port COM5
# 或离线重放日志:
python lidar_webview.py --log latest
```

Windows 用户可直接双击仓库里的 `启动3D网页查看器.bat`、`启动Foxglove实机.bat`、`启动Foxglove重放.bat`。

## 协议速览（完整版见 PROTOCOL.md）

- 串口 **115200 8N1**，帧头 `0xAA`，协议版本 **0x13**，大小端一律大端
- **0xAE** 转速帧：`Hz = xx / 20`
- **0xAD** 测距帧：`AA|len|13|61|AD|dlen=7+3N|xx|off|start_angle|end_angle|N×[q d_hi d_lo]|chk`
  - `dist_mm = ((d_hi<<8)|d_lo) × 0.5`（**0.5 mm/格，2C 实测**）
  - `q` 为质量/强度，`0` 抛弃；`d==0` 为无回波；`d>8000mm` 为饱和伪值（等同无回波）
  - 采样角度 = `start + (end−start)×(k+0.5)/N`；每圈 **16 包**，约 **230 点/圈**
  - 校验 = 校验字节前所有字节的 16 位大端累加和

## 已知坑（详见 docs/踩坑记录.md）

1. **采样字节序是 `[q, d_hi, d_lo]`**（质量在前）。解析器若按 `[d_hi, d_lo, q]` 解析，质量字节（0x90~0x97）会变成距离高字节，地图会永远画出一个 ~4.8 m/9.6 m 的**假圆环**——症状是"地图永远是圆形"。**这是本项目踩过最大的坑。**
2. **距离比例 0.5 mm/格**，不要套用 kaiaai 家族的 0.25 mm/格（会整体缩小一倍）。
3. 房间 < 6 m 时，任何 ≥ 8.17 m 的读数都是无回波饱和值，不是墙（"11m 墙"是假象）。
4. kaiaai / LDROBOT 官方驱动**不认 2C**（版本校验 0x01 ≠ 0x13），需要打补丁。
5. **VCC 必须 5 V**；3.3 V 供电曾导致一台模组出现 3.2 m 近距盲区（硬件损伤）。
6. 电机电压决定数据：~9.3 Hz 无测距帧，~7.9 Hz 出现，**2.6 V ≈ 7 Hz 最稳**（个体有差异）。

## 样例数据（samples/）

- `delta2c_20260820_164235_raw.log` — 226 KB 实测抓包（含 618 个测距帧）
- `map_fixed_1726.png` / `map_fixed_1733.png` — 修复解析后渲染的房间占用栅格
- `sample_164235.png` — 上述日志的 RViz 风格点云图

完整房间扫描的 raw 日志（10 MB+）因体积未入仓库，可用工具自行抓取。

## ROS2 支持

2C 无法直接用官方驱动（版本不兼容）。当前仓库提供的是自研工具链；若需 ROS2 接入，可基于 `delta2c_debug_gui.DeltaParser` 写一个 rclpy 节点发布 `sensor_msgs/LaserScan`（约 150 行），配合 slam_toolbox 可做小车动态建图（详见讨论记录）。

## 关于两台样机

本项目共实测两台同型号 DELTA-2C：一台健康（近距 0.15 m 起、墙 3-5 m 清晰）、一台退化为 3.2 m 近距盲区。**协议/工具均以健康机校准**；若你的模组近距测不到，先怀疑硬件（供电损伤/光学脏污），再排查软件。

## License

MIT — 见 [LICENSE](LICENSE)。协议事实来源于公开抓包与 3irobotix Delta 家族文档（kaiaai/LDS, Apache-2.0）。

## 贡献

欢迎 PR，但**所有合并必须经仓库所有者（hapynfdj）明确同意**——见 [CONTRIBUTING.md](CONTRIBUTING.md)。