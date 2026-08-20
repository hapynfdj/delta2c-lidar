@echo off
rem Foxglove bridge - replay newest saved log (no hardware needed)
cd /d %~dp0
echo Starting Foxglove bridge (replay mode)...
echo Keep this window open. Then in Foxglove Studio:
echo   Open Connection -^> WebSocket -^> ws://127.0.0.1:8765
echo.
python foxglove_bridge.py --log latest
pause
