@echo off
REM ============================================================================
REM Open Native Windows ROS 2 Jazzy Shell (Pixi + RoboStack)
REM ============================================================================
cd /d "%USERPROFILE%\ros2_jazzy_win"

set "PIXI_EXE=%USERPROFILE%\.pixi\bin\pixi.exe"
set "PIXI_MANIFEST=%USERPROFILE%\ros2_jazzy_win\pixi.toml"
set "FASTRTPS_DEFAULT_PROFILES_FILE=%USERPROFILE%\ros2_jazzy_win\fastdds_lan.xml"

if "%ROS_DOMAIN_ID%"=="" set "ROS_DOMAIN_ID=0"

echo ========================================================
echo  Native Windows ROS 2 Jazzy Shell
echo  ROS_DOMAIN_ID: %ROS_DOMAIN_ID%
echo  FastDDS LAN Profile: %FASTRTPS_DEFAULT_PROFILES_FILE%
echo  Restarting ROS 2 daemon on LAN interface...
echo ========================================================

"%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" ros2 daemon stop >nul 2>&1
"%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" ros2 daemon start >nul 2>&1

echo.
echo Active ROS 2 topics on LAN:
"%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" ros2 topic list
echo.

"%PIXI_EXE%" shell --manifest-path "%PIXI_MANIFEST%"
