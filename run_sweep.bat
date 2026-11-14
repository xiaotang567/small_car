@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

rem  参数扫描：run_sweep.bat [每个配置的秒数]
set SECS=%1
if "%SECS%"=="" set SECS=90

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo 三轮扫描（每个配置 %SECS% 秒，会比较久，跑完自动给出排行）：
echo   1) 舵机传动比 x 控制器   2) 舵机满行程/死区   3) 陀螺参数
echo 结果同时写到屏幕；想中途停就关窗口。
echo ------------------------------------------------------------
python -u "%~dp0tools\sim_sweep.py" %SECS%
echo ------------------------------------------------------------
pause
endlocal
exit /b 0
