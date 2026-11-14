@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

rem  无界面跑圈测试：run_simtest.bat [秒数]
set SECS=%1
if "%SECS%"=="" set SECS=90

where python >nul 2>nul
if errorlevel 1 ( echo [错误] 没找到 python & pause & exit /b 1 )

echo 无界面跑 %SECS% 秒（不渲染，快 7 倍以上）...
echo 看三件事：沿赛道走了多少米 / 最大偏离 / 出赛道累计时间
echo ------------------------------------------------------------
python "%~dp0tools\sim_test.py" %SECS%
echo ------------------------------------------------------------
pause
endlocal
exit /b 0
