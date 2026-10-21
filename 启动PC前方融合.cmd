@echo off
setlocal
cd /d "%~dp0"
set "PANORAMA_PYTHON=D:\python\python.exe"
if defined ROADFUSION_PYTHON set "PANORAMA_PYTHON=%ROADFUSION_PYTHON%"
"%PANORAMA_PYTHON%" board_v8\tools\pc_front_panorama.py --play board_v8\output\pc_front_panorama_v2\01_daylight_intersection\front_panorama.mp4 board_v8\output\pc_front_panorama_v2\03_daylight_following\front_panorama.mp4
if errorlevel 1 pause
