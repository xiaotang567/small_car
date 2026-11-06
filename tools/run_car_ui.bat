@echo off
chcp 65001 >nul
cd /d %~dp0
title small_car console
echo ==============================================
echo   small_car control console   (TCP server :8086)
echo.
echo   step 1: run this window first
echo   step 2: then power on the board
echo ==============================================
set PY=C:\Program Files\Python312\python.exe
if exist "%PY%" goto run
set PY=C:\Users\SHUIGUI\.workbuddy\binaries\python\versions\3.13.12\python.exe
if exist "%PY%" goto run
set PY=python
:run
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
"%PY%" car_ui.py 8086
echo.
echo [console exited] press any key to close
pause >nul
