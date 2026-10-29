#!/usr/bin/env bash
# ==============================================================================
# Run ROS 2 Jazzy RViz2 in Docker on WSL2 (with WSLg GPU Acceleration)
# ==============================================================================
# Seamlessly forwards RViz2 window to Windows desktop via WSLg.
# Uses host networking to discover ROS 2 topics from your Raspberry Pi CM4.
#
# Usage:
#   ./run_rviz2.sh [RVIZ2_OPTIONS]
# Examples:
#   ./run_rviz2.sh
#   ./run_rviz2.sh -f odom
#   ./run_rviz2.sh -d /opt/qplatform/config/nav2.rviz
# ==============================================================================

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QPLATFORM_DIR="${REPO_DIR}/image_builder/data/noble-rt-qplatform/rootfs/opt/qplatform"

# WSLg Display settings
DISPLAY_VAL="${DISPLAY:-:0}"
WAYLAND_VAL="${WAYLAND_DISPLAY:-wayland-0}"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# GPU Acceleration flags for WSL2
GPU_FLAGS=""
if [ -e "/dev/dxg" ]; then
    GPU_FLAGS="--device /dev/dxg"
fi
if [ -d "/usr/lib/wsl/lib" ]; then
    GPU_FLAGS="${GPU_FLAGS} -v /usr/lib/wsl/lib:/usr/lib/wsl/lib:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib"
fi

# Ensure X11 permissions if xhost exists
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
fi

echo "=================================================="
echo " Launching RViz2 (ROS 2 Jazzy) via WSLg"
echo " Display:       $DISPLAY_VAL"
echo " ROS Domain ID: $DOMAIN_ID"
echo " Network:       host mode (discovers Pi automatically)"
echo "=================================================="

docker run -it --rm \
    --net=host \
    --ipc=host \
    -e DISPLAY="$DISPLAY_VAL" \
    -e WAYLAND_DISPLAY="$WAYLAND_VAL" \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e PULSE_SERVER=/mnt/wslg/PulseServer \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /mnt/wslg:/mnt/wslg:ro \
    -v "${QPLATFORM_DIR}:/opt/qplatform:ro" \
    $GPU_FLAGS \
    qplatform-rviz2:jazzy \
    rviz2 "$@"
