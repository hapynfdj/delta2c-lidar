# 米家扫地机器人2 激光雷达 (LDS01RR) 数据查看器 (旧版, XV11)

> 归档说明: 这是早期基于 XV11(0xFA 22 字节帧)的查看器文档。后来实测确认拆机件
> 是 **Camsense DELTA-2C PRO**(见 `delta2c_debug_gui.pyw` 与主 README),此文档仅
> 保留给 `lidar_viewer.py` / `lidar_gui.pyw` 使用。

在 Windows 上双击 `启动查看器.bat` 即可打开图形界面,读取雷达数据。

## 文件说明

| 文件 | 说明 |
|---|---|
| `lidar_gui.pyw` | **图形界面程序** — 双击运行(需 Python 3.8+、pyserial、matplotlib) |
| `启动查看器.bat` | 双击启动 GUI 的批处理 |
| `lidar_viewer.py` | 命令行版本,功能相同 |
| `delta2c_debug_gui.pyw` | **新版 DELTA-2C 调试/扫描工具(推荐)** |

## 命令行用法

```bash
python lidar_viewer.py                     # 自动串口+自动协议
python lidar_viewer.py --port COM5 --text  # 文本模式
python lidar_viewer.py --demo --text       # 演示
python lidar_viewer.py --list-ports        # 列表串口
```

## 环境要求

- Python 3.8+(本机已装 3.14)
- `pyserial`、`matplotlib`、`numpy`(本机已装;如缺:`pip install pyserial matplotlib numpy`)
