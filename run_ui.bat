@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0tools"

rem  实车上位机（连真车）。可选端口，默认 8086
set PORT=%1
if "%PORT%"=="" set PORT=8086

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo 上位机启动，监听 %PORT% ...
echo   · 真车：板子会主动连过来
echo   · 仿真：先开 run_ui_sim.bat
python car_ui.py %PORT%
if errorlevel 1 ( echo. & echo [上位机异常退出] & pause )
endlocal
exit /b 0
