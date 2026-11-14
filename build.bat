@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0MDK-ARM"

set KEIL=C:\Keil_v5\UV4\UV4.exe
if not exist "%KEIL%" (
  echo [错误] 没找到 Keil：%KEIL%
  echo        如果你装在别处，改这个 bat 里的 KEIL 变量
  pause & exit /b 1
)

echo 编译固件（Keil UV4）...
"%KEIL%" -b small_car.uvprojx -j0 -o build_out.txt

rem UV4 返回后日志还没写完，等一下再读
timeout /t 6 /nobreak >nul

echo ------------------------------------------------------------
findstr /C:"error" /C:"Error(s)" /C:"warning" /C:"Program Size" build_out.txt
echo ------------------------------------------------------------
if exist "small_car\small_car.hex" (
  for %%F in ("small_car\small_car.hex") do echo hex: %%~fF   %%~tF   %%~zF 字节
)
pause
endlocal
exit /b 0
