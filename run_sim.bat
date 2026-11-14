@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

rem  仿真窗口：run_sim.bat [倍速]
rem    例：run_sim.bat        -> 1 倍速
rem        run_sim.bat 4      -> 4 倍速
rem        run_sim.bat 1 bev  -> 用逆透视巡线

set SPEED=%1
if "%SPEED%"=="" set SPEED=1
set CTRL=%2
if "%CTRL%"=="" set CTRL=gyro

where python >nul 2>nul
if errorlevel 1 (
  echo [错误] 没找到 python，请先装 Python 3 并勾选 Add to PATH
  pause & exit /b 1
)

echo 启动仿真 速度 x%SPEED%  控制器 %CTRL%
echo 提示：窗口里可以点 启动 / 加速 / 旋钮改参数；关窗口即退出。
cd /d "%~dp0sim"
python main.py --auto --speed %SPEED% --ctrl %CTRL%
if errorlevel 1 (
  echo.
  echo [仿真异常退出] 上面是报错信息
  pause
)
endlocal
exit /b 0
