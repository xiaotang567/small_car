@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo 打开地图绘制器...
echo   · 直线 / 45°圆弧 / 90°圆弧 / 自定义圆弧 / 十字 / 圆环
echo   · 画完点【保存地图】-> sim\track_design.json
echo   · 然后进仿真就是你自己画的图
cd /d "%~dp0sim"
python map_editor.py
if errorlevel 1 ( echo. & echo [编辑器异常退出] & pause )
endlocal
exit /b 0
