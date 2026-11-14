@echo off
chcp 936 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"

:menu
cls
echo ============================================================
echo   small_car 工具箱
echo ------------------------------------------------------------
echo   [1] 仿真窗口（3D追车视角 + 车载相机 + 逆透视 + 参数旋钮）
echo   [2] 仿真窗口（4倍速）
echo   [3] 地图绘制器（自己画直线/45度圆弧/十字/圆环）
echo   [4] 仿真 + 实车上位机（car_ui.py 连仿真车调PID）
echo   [5] 实车上位机（连真车，TCP 8086）
echo   [6] 无界面跑圈测试（看圈数/出界/偏离）
echo   [7] 参数扫描（舵机传动比 x 控制器）
echo   [8] 20 组传动比自动寻优
echo   [9] 编译固件（Keil）
echo   [F] 烧录固件（ST-LINK）
echo   [0] 退出
echo ============================================================
set /p ch= 请输入序号后回车:

if /i "%ch%"=="1" goto sim
if /i "%ch%"=="2" goto sim4
if /i "%ch%"=="3" goto editor
if /i "%ch%"=="4" goto uisim
if /i "%ch%"=="5" goto ui
if /i "%ch%"=="6" goto test
if /i "%ch%"=="7" goto sweep
if /i "%ch%"=="8" goto opt
if /i "%ch%"=="9" goto build
if /i "%ch%"=="F" goto flash
if "%ch%"=="" goto end
if /i "%ch%"=="0" goto end
echo 没这个选项
pause
goto menu

:sim
call run_sim.bat
goto menu

:sim4
call run_sim.bat 4
goto menu

:editor
call run_editor.bat
goto menu

:uisim
call run_ui_sim.bat
goto menu

:ui
call run_ui.bat
goto menu

:test
call run_simtest.bat
goto menu

:sweep
call run_sweep.bat
goto menu

:opt
call run_opt.bat
goto menu

:build
call build.bat
goto menu

:flash
call flash.bat
goto menu

:end
endlocal
exit /b 0
