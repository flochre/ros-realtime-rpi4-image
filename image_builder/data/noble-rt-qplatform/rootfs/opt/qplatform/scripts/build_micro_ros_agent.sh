#!/bin/bash
set -e -o pipefail

echo "=========================================="
echo " Building micro-ROS Agent for ROS 2 Jazzy "
echo "=========================================="

source /opt/ros/jazzy/setup.bash

# Ensure build requirements
apt-get update
apt-get install -y --no-install-recommends \
  git \
  cmake \
  build-essential \
  python3-colcon-common-extensions \
  ros-jazzy-micro-ros-msgs

mkdir -p /opt/micro_ros_ws/src
cd /opt/micro_ros_ws/src

if [ ! -d "micro-ROS-Agent" ]; then
  git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
else
  cd micro-ROS-Agent && git pull && cd ..
fi

cd /opt/micro_ros_ws
colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✓ micro-ROS Agent built successfully in /opt/micro_ros_ws"

# Reload and enable systemd service if available
if [ -f /etc/systemd/system/micro-ros-agent.service ]; then
  systemctl daemon-reload || true
  systemctl enable micro-ros-agent.service || true
  systemctl start micro-ros-agent.service || true
  echo "✓ micro-ros-agent.service enabled and started"
fi
