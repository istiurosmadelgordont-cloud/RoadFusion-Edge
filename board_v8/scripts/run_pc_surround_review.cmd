@echo off
setlocal
cd /d "%~dp0\..\.."
set "AVM_PYTHON=%ROADFUSION_PYTHON%"
if not defined AVM_PYTHON if exist "D:\python\python.exe" set "AVM_PYTHON=D:\python\python.exe"
if not defined AVM_PYTHON set "AVM_PYTHON=python"
"%AVM_PYTHON%" -c "import cv2, numpy" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python with OpenCV and NumPy was not found.
    echo Set ROADFUSION_PYTHON to your python.exe path and retry.
    pause
    exit /b 1
)
echo Opening the PC AVM player. Click scene buttons or press 1-8 to switch.
"%AVM_PYTHON%" board_v8\tools\pc_surround_review.py --play-all %*
if errorlevel 1 (
    echo [ERROR] See the message above. Read board_v8\PC_AVM_START.md for setup.
    pause
    exit /b 1
)
exit /b 0
