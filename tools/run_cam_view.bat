@echo off
cd /d %~dp0
echo ==============================================
echo   small_car camera viewer  (TCP server :8086)
echo   run this FIRST, then power on the board
echo ==============================================
set PY=C:\Program Files\Python312\python.exe
if exist "%PY%" goto run
set PY=C:\Users\SHUIGUI\.workbuddy\binaries\python\versions\3.13.12\python.exe
:run
"%PY%" cam_view.py 8086
echo.
echo [viewer exited] press any key to close
pause >nul
