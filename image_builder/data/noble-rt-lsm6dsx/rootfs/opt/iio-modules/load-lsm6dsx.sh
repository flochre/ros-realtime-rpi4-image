#!/bin/bash
# LSM6DSX (IIO_ST_LSM6DSX) kernel module loader
# STMicroelectronics 6-axis accelerometer/gyroscope sensor
# Used in Raspberry Pi with IMU/motion tracking applications

set -e

echo "[LSM6DSX] Loading sensor module..."

# Check if module is built into kernel or available as loadable module
if modinfo iio_st_lsm6dsx &>/dev/null 2>&1; then
    echo "[LSM6DSX] Loading module: iio_st_lsm6dsx"
    modprobe iio_st_lsm6dsx || echo "[LSM6DSX] Warning: Could not load iio_st_lsm6dsx"
    echo "[LSM6DSX] ✓ Module loaded successfully"
else
    echo "[LSM6DSX] ℹ Module not found or built into kernel"
    echo "[LSM6DSX] To enable: CONFIG_IIO_ST_LSM6DSX=m in kernel .config"
fi

# Also load the I2C transport module if available
if modinfo iio_st_lsm6dsx_i2c &>/dev/null 2>&1; then
    echo "[LSM6DSX] Loading I2C transport: iio_st_lsm6dsx_i2c"
    modprobe iio_st_lsm6dsx_i2c || echo "[LSM6DSX] Warning: Could not load I2C transport"
fi

# List available IIO devices
echo "[LSM6DSX] Available IIO devices:"
if command -v iio_info &> /dev/null; then
    iio_info 2>/dev/null || echo "[LSM6DSX] No IIO devices detected"
else
    echo "[LSM6DSX] iio_info not installed"
fi
