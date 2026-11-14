@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

rem  20 组舵机传动比，各自自动寻优：run_opt.bat [每组秒数] [组数]
set SECS=%1
if "%SECS%"=="" set SECS=45
set N=%2
if "%N%"=="" set N=20

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo 跑 %N% 组传动比，每组 %SECS% 秒，自动找最好的 w_cur / kff ...
echo （20 组大约 10~20 分钟，关窗口可以中途停）
echo ------------------------------------------------------------
python -u "%~dp0tools\sim_opt.py" %SECS% %N%
echo ------------------------------------------------------------
pause
endlocal
exit /b 0
