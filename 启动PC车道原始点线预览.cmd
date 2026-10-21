@echo off
setlocal
cd /d "%~dp0"
set "NVIDIA_LANE_PYTHON=D:\python\python.exe"
if defined ROADFUSION_PYTHON set "NVIDIA_LANE_PYTHON=%ROADFUSION_PYTHON%"
"%NVIDIA_LANE_PYTHON%" board_v8\tools\pc_ufld_video.py --playlist board_v8\output\pc_lane_raw_points_20261017\test_manifest.json
if errorlevel 1 pause
