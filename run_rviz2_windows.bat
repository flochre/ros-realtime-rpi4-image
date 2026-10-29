@echo off
setlocal
REM ============================================================================
REM Launch Native Windows ROS 2 Jazzy RViz2 (Pixi + RoboStack)
REM ============================================================================
cd /d "%USERPROFILE%\ros2_jazzy_win"

set "PIXI_EXE=%USERPROFILE%\.pixi\bin\pixi.exe"
set "PIXI_MANIFEST=%USERPROFILE%\ros2_jazzy_win\pixi.toml"
set "DEFAULT_RVIZ_CONFIG=%USERPROFILE%\ros2_jazzy_win\qplatform_ament\share\robot_v2_recovered\qplatform_preview.rviz"
set "FASTRTPS_DEFAULT_PROFILES_FILE=%USERPROFILE%\ros2_jazzy_win\fastdds_lan.xml"

if "%ROS_DOMAIN_ID%"=="" set "ROS_DOMAIN_ID=0"

echo ========================================================
echo  Launching Native Windows RViz2 (ROS 2 Jazzy)
echo  ROS_DOMAIN_ID: %ROS_DOMAIN_ID%
echo  FastDDS Profile: %FASTRTPS_DEFAULT_PROFILES_FILE%
echo ========================================================

if "%~1"=="" (
    if exist "%DEFAULT_RVIZ_CONFIG%" (
        "%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" rviz2 -d "%DEFAULT_RVIZ_CONFIG%"
    ) else (
        "%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" rviz2
    )
) else (
    "%PIXI_EXE%" run --manifest-path "%PIXI_MANIFEST%" rviz2 %*
)
