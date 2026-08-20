@echo off
rem Foxglove bridge - live from the lidar (auto-detects the serial port)
cd /d %~dp0
echo Starting Foxglove bridge (live mode)...
echo Keep this window open. Then in Foxglove Studio:
echo   Open Connection -^> WebSocket -^> ws://127.0.0.1:8765
echo.
python foxglove_bridge.py
pause
