# QPlatform LSM6DSx Sensor Support - noble-rt Overlay

## Overview

This overlay profile extends the standard `noble-rt` (Ubuntu 24.04 Server + PREEMPT_RT) with:
- **STMicroelectronics LSM6DSx IMU driver** (`CONFIG_IIO_ST_LSM6DSX`, `CONFIG_IIO_ST_LSM6DSX_SPI`, `CONFIG_IIO_ST_LSM6DSX_I2C`)
- **Device Tree Overlay (`lsm6dsx-spi.dtbo`)** compiled and installed in `/boot/firmware/overlays/`
- **SPI0 Hardware Configuration** automatically enabled in `/boot/firmware/config.txt`
- **IIO (Industrial I/O) utilities** for low-latency sensor telemetry
- **Systemd service** for automatic module loading

## Quick Start

```bash
# Build Ubuntu 24.04.5 Server + PREEMPT_RT + ROS 2 Jazzy + QPlatform LSM6DSx support
make noble-rt-ros2-qplatform

# Or build without ROS 2
make noble-rt-qplatform
```

## Hardware: LSM6DSx Sensor on SPI0 (GPIO 7–11)

### Raspberry Pi 4/5 40-pin Header Wiring

| LSM6DSx Pin | RPi Physical Pin | RPi Function / BCM GPIO | Notes |
|---|---|---|---|
| **VCC / 3V3** | Pin 1 or 17 | 3.3V Power | Clean 3.3V DC |
| **GND** | Pin 6, 9, 14, 20, 25, 30, 34, or 39 | Ground | System GND |
| **SCL / SCK** | Pin 23 | GPIO 11 (`SPI0_SCLK`) | Serial Clock |
| **SDA / MOSI**| Pin 19 | GPIO 10 (`SPI0_MOSI`) | Master Out Slave In |
| **SDO / MISO**| Pin 21 | GPIO 9 (`SPI0_MISO`)  | Master In Slave Out |
| **CS** | Pin 24 | GPIO 8 (`SPI0_CE0`)  | Chip Select 0 (Default) |
| *(alt CS)* | Pin 26 | GPIO 7 (`SPI0_CE1`)  | Chip Select 1 (Optional) |
| *(alt INT1)* | Pin 22 | GPIO 25 | Optional Data Ready IRQ |

## Bootloader Configuration (`/boot/firmware/config.txt`)

The image builder automatically adds the following configuration to `/boot/firmware/config.txt`:

```ini
dtparam=spi=on
dtoverlay=lsm6dsx-spi
```

### Overlay Options and Overrides

You can customize the overlay directly in `/boot/firmware/config.txt`:

```ini
# Default: SPI0 CE0 (GPIO 8), 10 MHz, LSM6DS3 compatible
dtoverlay=lsm6dsx-spi

# LSM6DSO chip on CE0:
dtoverlay=lsm6dsx-spi,lsm6dso

# LSM6DSOX on CE0:
dtoverlay=lsm6dsx-spi,lsm6dsox

# ISM330DHCX on CE0:
dtoverlay=lsm6dsx-spi,ism330dhcx

# Using CE1 (GPIO 7) instead of CE0:
dtoverlay=lsm6dsx-spi,cs1

# Custom SPI bus speed (e.g. 8 MHz):
dtoverlay=lsm6dsx-spi,speed=8000000

# Using hardware interrupt on GPIO 25:
dtoverlay=lsm6dsx-spi,interrupt=25
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

# Check sensor module infrastructure
ls -la /opt/iio-modules/
ls -la /lib/modules/*/extra/iio/

# Check systemd service
sudo systemctl status lsm6dsx-module-load

# View module loading logs
sudo journalctl -u lsm6dsx-module-load -n 20
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

### 4. ROS 2 Navigation & Omni Control Architecture

The platform provides a complete turnkey setup in `/opt/qplatform`:
- **`ros2_control` Omni Controller**: `/opt/qplatform/config/omni_controllers.yaml`
- **Nav2 Holonomic Navigation**: `/opt/qplatform/config/nav2_omni_params.yaml`
- **EKF Sensor Fusion (Odometry + IMU)**: `/opt/qplatform/config/ekf_localization.yaml`
- **Bringup Launch**: `/opt/qplatform/launch/omni_bringup.launch.py`

#### Running ros2_control & Nav2

```bash
# Launch omni controller + Nav2 stack
source /opt/ros/jazzy/setup.bash
ros2 launch /opt/qplatform/launch/omni_bringup.launch.py
```

### 5. micro-ROS Agent (Network Microcontrollers)

The micro-ROS agent runs as a systemd service listening on UDP port 8888 for microcontroller clients (ESP32, Teensy, STM32, Pico W):

```bash
# Check service status
sudo systemctl status micro-ros-agent

# View real-time logs from connected microcontrollers
sudo journalctl -u micro-ros-agent -f

# Manual execution / testing
source /opt/ros/jazzy/setup.bash
source /opt/micro_ros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
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
