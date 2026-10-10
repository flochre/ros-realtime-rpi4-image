# LSM6DSX Sensor Support - noble-rt Overlay

## Overview

This overlay profile extends the standard `noble-rt` (Ubuntu 24.04 Server + PREEMPT_RT) with:
- **LSM6DSX sensor driver** (CONFIG_IIO_ST_LSM6DSX) for 6-axis accelerometer/gyroscope
- **IIO (Industrial I/O) utilities** for sensor access and diagnostics
- **Device Tree overlay** describing the sensor on the I2C bus, so the kernel
  auto-probes/loads the driver at boot via the standard modalias mechanism -
  no custom modprobe script or systemd unit required
- **Ready for kernel customization** with sensor module compilation

## Quick Start

```bash
# Build Ubuntu 24.04 Server + PREEMPT_RT + LSM6DSX support
sudo ./ros-rt-img build noble-rt noble-rt-lsm6dsx
```

## Hardware: LSM6DSX Sensor

**What it is:**
- STMicroelectronics 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
- Industry standard in robotics, drones, and motion tracking
- Low latency, excellent for real-time applications
- Complementary to ROS 2 sensor fusion packages

**Supported Interfaces:**
- I2C (default on Raspberry Pi)
- SPI (faster alternative)

**Kernel Modules:**
```
CONFIG_IIO_ST_LSM6DSX=m          # Main driver
CONFIG_IIO_ST_LSM6DSX_I2C=m      # I2C transport
CONFIG_IIO_ST_LSM6DSX_SPI=m      # SPI transport
```

## Wiring Example (Raspberry Pi 4/5 + LSM6DSX)

```
LSM6DSX Breakout Board → Raspberry Pi GPIO Header

Pin Name  →  RPi Pin   →  GPIO
─────────────────────────────────
VCC       →  Pin 1     →  3.3V
GND       →  Pin 6     →  GND
SCL       →  Pin 5     →  GPIO 3 (I2C1_SCL)
SDA       →  Pin 3     →  GPIO 2 (I2C1_SDA)
CS/SA0    →  GND       →  (sets I2C address 0x6A)
```

**I2C Addresses:**
- CS/SA0 connected to GND: `0x6A`
- CS/SA0 connected to VCC: `0x6B`

Set `LSM6DSX_I2C_ADDR` (without the `0x` prefix) and `LSM6DSX_COMPATIBLE` (the
exact chip variant, e.g. `st,lsm6dsl`, `st,lsm6dso`, `st,lsm6dsox`, ...) in
`image_builder/data/noble-rt-lsm6dsx/config.ini` to match your wiring/part
before building. These are baked into the Device Tree overlay at build time.

**Verify Connection:**
```bash
sudo apt install i2c-tools
i2cdetect -y 1
# Should show device at 0x6a or 0x6b
```

## Testing the Image

### 1. Flash and Boot
```bash
# Decompress
zstd -d ubuntu-24.04.2-rt-lsm6dsx-arm64+raspi.img.zst

# Flash to SD card
sudo dd if=ubuntu-24.04.2-rt-lsm6dsx-arm64+raspi.img of=/dev/sdX bs=4M
sudo sync
```

### 2. Verify Kernel and Sensor Support
```bash
# SSH to device
ssh ubuntu@<ip-address>

# Verify PREEMPT_RT kernel
uname -a
# Should show PREEMPT_RT version

# Check the Device Tree overlay was installed and is loaded
cat /boot/firmware/config.txt | grep lsm6dsx
ls -la /boot/firmware/overlays/lsm6dsx.dtbo
ls /proc/device-tree/soc/i2c@*/lsm6dsx@*

# Check kernel logs for driver probe
dmesg | grep -i lsm6dsx
```

### 3. Sensor Access (when hardware present)

```bash
# List IIO devices
iio_info

# Read accelerometer data
cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
cat /sys/bus/iio/devices/iio:device0/in_accel_y_raw
cat /sys/bus/iio/devices/iio:device0/in_accel_z_raw

# Read gyroscope data
cat /sys/bus/iio/devices/iio:device0/in_anglvel_x_raw
cat /sys/bus/iio/devices/iio:device0/in_anglvel_y_raw
cat /sys/bus/iio/devices/iio:device0/in_anglvel_z_raw

# Read scale factors
cat /sys/bus/iio/devices/iio:device0/in_accel_scale
cat /sys/bus/iio/devices/iio:device0/in_anglvel_scale
```

### 4. ROS 2 Integration

```bash
# Install sensor packages
sudo apt install ros-jazzy-imu-tools

# Example: Sensor publisher node (you'll need to create this)
# Reads LSM6DSX data and publishes sensor_msgs/Imu
ros2 run <your_sensor_package> lsm6dsx_imu_node

# Monitor sensor output
ros2 topic echo /imu/data
```

## Real-Time Characteristics

**PREEMPT_RT Kernel Features:**
- Deterministic scheduling latency (~50-80μs worst-case on RPi 4)
- Disabled RT throttling (full real-time priority support)
- Disabled memory compaction (reduces GC pauses)
- Pinned CPU frequency (consistent performance)

**Sensor I/O Latency:**
- I2C communication: ~1-5ms per read (non-blocking)
- IIO buffer mode: <1ms with hardware timestamping
- Recommended: Use kernel IIO buffer interface for best latency

## Future Work

### Phase 1: Kernel Module Compilation
1. Rebuild PREEMPT_RT kernel with LSM6DSX enabled
2. Cross-compile on host system
3. Package into kernel snap

### Phase 2: ROS 2 Sensor Driver
1. Create ROS 2 package for LSM6DSX
2. Publish `sensor_msgs/Imu` messages
3. Integrate with `robot_localization` package

### Phase 3: Multi-Sensor Support
1. Add magnetometer support
2. Add temperature/pressure sensors
3. Sensor fusion with extended Kalman filter

### Phase 4: Production Hardening
1. Real-time latency benchmarking with sensor I/O
2. Load testing with multiple sensors
3. Power management optimization

## Troubleshooting

**Sensor not detected:**
```bash
# Check if driver is loaded
lsmod | grep lsm6dsx

# Check kernel logs
dmesg | grep -i lsm6dsx

# Verify I2C connection
i2cdetect -y 1

# Check device tree (if using custom device tree)
cat /proc/device-tree/i2c@7e205000/status
```

**I2C address conflicts:**
```bash
# Scan all I2C buses
for i in 0 1; do echo "=== I2C Bus $i ==="  && i2cdetect -y $i; done
```

**Permission denied reading sensor:**
```bash
# Add user to iio group
sudo usermod -a -G iio ubuntu
# Log out and back in
```

## Documentation References

- [Linux IIO Kernel Documentation](https://www.kernel.org/doc/html/latest/iio/)
- [STMicroelectronics LSM6DSX Datasheet](https://www.st.com/en/mems-and-sensors/lsm6dsx.html)
- [Raspberry Pi I2C Setup](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#i2c)
- [ROS 2 IMU Messages](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)
- [PREEMPT_RT Documentation](https://wiki.linuxfoundation.org/realtime/start)

## Contributing

To improve this overlay:

1. Test on Raspberry Pi 4 and 5 with LSM6DSX hardware
2. Implement ROS 2 sensor driver
3. Add SPI transport support
4. Performance benchmarking with real-time constraints
5. Multi-sensor integration
