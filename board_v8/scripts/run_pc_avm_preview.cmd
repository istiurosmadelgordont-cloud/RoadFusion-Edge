@echo off
cd /d "%~dp0\..\.."
python board_v8\tools\pc_avm_preview.py --start-seconds 8 %*
if errorlevel 1 pause
