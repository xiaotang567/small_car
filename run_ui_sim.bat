@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

rem  仿真 + 上位机 一起开：上位机在 8086 监听，仿真车当客户端连过去（和实车一样）
set PORT=%1
if "%PORT%"=="" set PORT=8086

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo [1/2] 启动上位机（监听 %PORT%）...
start "small_car 上位机" cmd /c "cd /d "%~dp0tools" && python car_ui.py %PORT%"
timeout /t 3 /nobreak >nul

echo [2/2] 启动仿真（会自动连上位机）...
start "small_car 仿真" cmd /c "cd /d "%~dp0sim" && python main.py %PORT% --auto"

echo.
echo 两个窗口都起来了：
echo   · 上位机窗口里点【应用 sp / 应用 sc】就是改仿真车的 PID
echo   · 仿真窗口里点 启动 / 加速 / 旋钮
endlocal
exit /b 0
