#!/usr/bin/env bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source optional workspace if mounted
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Fallback to software OpenGL rendering if hardware driver is not mounted
if [ ! -d "/usr/lib/wsl/lib" ] && [ -z "$LIBGL_ALWAYS_SOFTWARE" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
fi

exec "$@"
