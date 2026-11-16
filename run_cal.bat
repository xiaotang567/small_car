@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0tools"

rem  逆透视标定工具：连板子 -> 抓相机原图 -> 鼠标点正方形4个角 -> 生成查表 -> 编译 -> 烧录
rem  注意：8086 端口只能有一个监听，跑这个之前**先关掉 small_car 控制台**

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo ============================================================
echo   逆透视标定工具
echo   1) 先关掉 small_car 控制台窗口
echo   2) 点"连板子并抓图"
echo   3) 在图上点正方形 4 个角，填边长，点"生成+编译+烧录"
echo ============================================================
echo.
python ipm_cal.py
if errorlevel 1 ( echo. & echo [标定工具异常退出] & pause )
endlocal
exit /b 0