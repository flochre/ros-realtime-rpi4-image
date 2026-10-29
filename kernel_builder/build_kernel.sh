#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="${REPO_ROOT}/cache/custom_kernel_debs"

echo "======================================================================"
echo " PREEMPT-RT Kernel Builder for Raspberry Pi 4 / Pi 5 (Ubuntu 24.04)"
echo " In-Tree Drivers Enabled: LSM6DSX (I2C/SPI), RT Preempt, etc."
echo " Output Destination: ${OUTPUT_DIR}"
echo "======================================================================"

mkdir -p "${OUTPUT_DIR}"

if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed or not in PATH."
    echo "Please install Docker or Docker Desktop, and ensure your user has permission to run docker."
    exit 1
fi

LOG_FILE="${REPO_ROOT}/kernel_build.log"
CONTAINER_NAME="kernel_builder_run"

echo "[1/3] Building docker build environment..."
docker build \
  --build-arg UBUNTU_VERSION=noble \
  --build-arg KERNEL_VERSION=6.8.0 \
  -t rtwg-noble-builder \
  "${SCRIPT_DIR}"

echo "[2/3] Compiling kernel .deb packages inside container..."
echo "Build log is being recorded to: ${LOG_FILE}"
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

set +e
docker run --name "${CONTAINER_NAME}" \
  -v "${OUTPUT_DIR}:/linux_build/output" \
  rtwg-noble-builder \
  bash -c "cd /linux_build/linux-raspi && \
           make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LOCALVERSION=-raspi -j\$(nproc) bindeb-pkg && \
           cp /linux_build/*.deb /linux_build/output/ && \
           echo 'Kernel deb packages generated successfully!'" 2>&1 | tee "${LOG_FILE}"
BUILD_STATUS=${PIPESTATUS[0]}
set -e

if [ ${BUILD_STATUS} -ne 0 ]; then
    echo ""
    echo "======================================================================"
    echo "❌ Kernel compilation failed! Extracting errors from ${LOG_FILE}:"
    echo "======================================================================"
    grep -E -C 3 "error:|Error [0-9]" "${LOG_FILE}" | tail -n 50 || true
    echo "======================================================================"
    echo "Container '${CONTAINER_NAME}' preserved. You can resume inside it with:"
    echo "  docker exec -it ${CONTAINER_NAME} bash"
    echo "Full log saved to: ${LOG_FILE}"
    echo "======================================================================"
    exit 1
fi

# Clean up successful container
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

echo "[3/3] Build complete! Generated debian packages:"
ls -lh "${OUTPUT_DIR}"/*.deb

echo ""
echo "======================================================================"
echo "You can now build the complete image by running:"
echo "  cd ${REPO_ROOT}"
echo "  sudo make noble-rt-ros2-qplatform"
echo "  (or: sudo make noble-rt-qplatform without ROS 2)"
echo "======================================================================"
