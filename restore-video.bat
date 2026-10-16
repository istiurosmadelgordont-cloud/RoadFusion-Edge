@echo off
cd /d "%~dp0"
where python >nul 2>nul
if errorlevel 1 (
  echo Python 3 is required. Run restore_video.py with Python 3.
  pause
  exit /b 1
)
python restore_video.py
pause
