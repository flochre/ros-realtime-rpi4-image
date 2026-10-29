@echo off
setlocal
REM ============================================================================
REM Launch PlotJuggler 4.0.0 on Windows + Native Windows ROS 2 Jazzy Bridges
REM ============================================================================
REM 1. Launches PlotJuggler 4.0.0 (C:\Users\albin\PlotJuggler4\bin\PlotJuggler4.exe)
REM 2. Starts native Windows PlotJuggler Bridge on ws://localhost:9090
REM 3. Starts native Windows Foxglove Bridge on ws://localhost:8765
REM ============================================================================
cd /d "%USERPROFILE%\ros2_jazzy_win"

if "%ROS_DOMAIN_ID%"=="" set "ROS_DOMAIN_ID=0"
set "PJ4_EXE=%USERPROFILE%\PlotJuggler4\bin\PlotJuggler4.exe"
set "PIXI_EXE=%USERPROFILE%\.pixi\bin\pixi.exe"
set "PIXI_MANIFEST=%USERPROFILE%\ros2_jazzy_win\pixi.toml"
set "FASTRTPS_DEFAULT_PROFILES_FILE=%USERPROFILE%\ros2_jazzy_win\fastdds_lan.xml"

echo ========================================================
echo  Starting PlotJuggler 4.0.0 + Native ROS 2 Jazzy Bridge
echo  ROS_DOMAIN_ID:       %ROS_DOMAIN_ID%
echo  PlotJuggler Bridge:  ws://localhost:9090
echo  Foxglove Bridge:     ws://localhost:8765
echo ========================================================

REM Stop any old PlotJuggler 3 process if running
taskkill /F /IM plotjuggler.exe >nul 2>&1

REM Launch PlotJuggler 4.0.0 if not already open
if exist "%PJ4_EXE%" (
    tasklist /FI "IMAGENAME eq PlotJuggler4.exe" 2>NUL | find /I /N "PlotJuggler4.exe">NUL
    if "%ERRORLEVEL%" NEQ "0" (
        start "" "%PJ4_EXE%"
    )
) else (
    echo [ERROR] PlotJuggler 4 executable not found at "%PJ4_EXE%"
)

REM Start Foxglove Bridge in background on port 8765 as well
start "Foxglove Bridge (8765)" /MIN "%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" foxglove_bridge

echo Starting Native Windows PlotJuggler Bridge on ws://localhost:9090 ...
echo In PlotJuggler 4:
echo   - Streaming -^> PlotJuggler Bridge -^> ws://localhost:9090
echo   - (or Streaming -^> Foxglove Bridge -^> ws://localhost:8765)
echo.

"%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" python "%USERPROFILE%\ros2_jazzy_win\pj_bridge_ros2_win.py"
