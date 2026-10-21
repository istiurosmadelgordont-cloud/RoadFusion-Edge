@echo off
setlocal
cd /d "%~dp0"
set "LANE_PYTHON=D:\python\python.exe"
if defined ROADFUSION_PYTHON set "LANE_PYTHON=%ROADFUSION_PYTHON%"
if exist board_v8\output\pc_nvidia_middle_lane_library_20261017\test_manifest.json (
  "%LANE_PYTHON%" board_v8\tools\pc_ufld_video.py --playlist board_v8\output\pc_nvidia_middle_lane_library_20261017\test_manifest.json
  if errorlevel 1 pause
  exit /b
)
if exist board_v8\output\pc_nvidia_lane_test_20261016\test_manifest.json (
  "%LANE_PYTHON%" board_v8\tools\pc_ufld_video.py --playlist board_v8\output\pc_nvidia_lane_test_20261016\test_manifest.json
  if errorlevel 1 pause
  exit /b
)
"%LANE_PYTHON%" board_v8\tools\pc_ufld_video.py --play board_v8\output\pc_ufld_test\11_side_vehicle_bsd\ufld_pc.mp4 board_v8\output\pc_ufld_test\03_daylight_following\ufld_pc.mp4 board_v8\output\pc_ufld_test\04_clear_open_road\ufld_pc.mp4
if errorlevel 1 pause
