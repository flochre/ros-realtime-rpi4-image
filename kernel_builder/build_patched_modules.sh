#!/usr/bin/env bash
# ==============================================================================
# Fast Incremental Builder for Patched In-Tree Kernel Modules (Yocto-style)
# ==============================================================================
# 1. Applies all patches in kernel_builder/patches/*.patch via Docker layer
# 2. Cross-compiles the specified kernel module directory against the cached
#    PREEMPT-RT linux-headers .deb in ~10 seconds (no full kernel rebuild needed)
# 3. Stages the resulting .ko files into the image rootfs /lib/modules/.../updates/
#
# Usage:
#   ./kernel_builder/build_patched_modules.sh [MODULE_SUBDIR]
# Example:
#   ./kernel_builder/build_patched_modules.sh drivers/iio/imu/st_lsm6dsx
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODULE_SUBDIR="${1:-drivers/iio/imu/st_lsm6dsx}"
DEBS_DIR="${REPO_ROOT}/cache/custom_kernel_debs"

echo "======================================================================"
echo " Applying kernel_builder/patches/*.patch and building ${MODULE_SUBDIR}"
echo "======================================================================"

# 1. Rebuild top Docker layer to apply latest kernel_builder/patches/*.patch
docker build \
  --build-arg UBUNTU_VERSION=noble \
  --build-arg KERNEL_VERSION=6.8.0 \
  -t rtwg-noble-builder \
  "${SCRIPT_DIR}"

# 2. Detect kernel version from cached linux-headers deb
HEADERS_DEB=$(ls "${DEBS_DIR}"/linux-headers-*-raspi_*.deb 2>/dev/null | head -n 1)
if [ -z "$HEADERS_DEB" ]; then
  echo "ERROR: No linux-headers-*.deb found in ${DEBS_DIR}."
  echo "Run ./kernel_builder/build_kernel.sh first to generate initial kernel debs."
  exit 1
fi

KVER=$(basename "$HEADERS_DEB" | sed -E 's/^linux-headers-([^_]+)_.*$/\1/')
OUT_MOD_DIR="${REPO_ROOT}/image_builder/data/noble-rt-qplatform/rootfs/lib/modules/${KVER}/updates/${MODULE_SUBDIR}"
mkdir -p "${OUT_MOD_DIR}"

echo "==> Cross-compiling ${MODULE_SUBDIR} for kernel ${KVER}..."
docker run --rm -u root \
  -v "${DEBS_DIR}:/debs:ro" \
  -v "${OUT_MOD_DIR}:/out_mods" \
  rtwg-noble-builder:latest \
  bash -c "dpkg -i /debs/linux-headers-${KVER}_*.deb >/dev/null && \
           make -C /lib/modules/${KVER}/build M=/linux_build/linux-raspi/${MODULE_SUBDIR} \
                ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j\$(nproc) modules && \
           aarch64-linux-gnu-strip --strip-debug /linux_build/linux-raspi/${MODULE_SUBDIR}/*.ko && \
           cp -v /linux_build/linux-raspi/${MODULE_SUBDIR}/*.ko /out_mods/ && \
           chown -R $(id -u):$(id -g) /out_mods"

echo "======================================================================"
echo "✓ Patched modules staged in:"
echo "  ${OUT_MOD_DIR}"
ls -lh "${OUT_MOD_DIR}"/*.ko
echo "======================================================================"
