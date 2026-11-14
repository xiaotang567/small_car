@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0"

rem  烧录固件（ST-LINK）；烧完自动复位运行
set CLI="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
set HEX=%~dp0MDK-ARM\small_car\small_car.hex

if not exist %CLI% (
  echo [错误] 没找到 ST-LINK_CLI.exe
  echo        路径：%CLI%
  pause & exit /b 1
)
if not exist "%HEX%" (
  echo [错误] 没找到固件：%HEX%
  echo        先跑 build.bat 编译
  pause & exit /b 1
)

echo 固件：%HEX%
for %%F in ("%HEX%") do echo 时间：%%~tF   大小：%%~zF 字节
echo.
echo 烧录中（ST-LINK 必须插好）...
%CLI% -c SWD UR FREQ=4000 -P "%HEX%" -V -Rst
set RC=%errorlevel%
echo ------------------------------------------------------------
if not "%RC%"=="0" (
  echo [失败] ST-LINK 没插好 / 没被识别。检查 USB，然后重来。
) else (
  echo [完成] 已经烧进去并复位运行了。串口看 FW 标签确认版本。
)
pause
endlocal
exit /b 0
