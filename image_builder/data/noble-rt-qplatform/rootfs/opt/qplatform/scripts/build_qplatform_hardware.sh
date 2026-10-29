#!/bin/bash
set -e -o pipefail

echo "================================================="
echo " Building qplatform_hardware for ROS 2 Jazzy     "
echo "================================================="

source /opt/ros/jazzy/setup.bash

cd /opt/qplatform/ros2_ws
colcon build --packages-select qplatform_hardware --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✓ qplatform_hardware built successfully in /opt/qplatform/ros2_ws/install"
