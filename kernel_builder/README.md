# Custom PREEMPT-RT Linux Kernel Builder for Raspberry Pi 4 / Pi 5

This folder contains the complete toolchain to cross-compile a custom Linux PREEMPT-RT kernel for **Ubuntu 24.04 (Noble Numbat)** with custom in-tree drivers enabled (such as the **STMicroelectronics LSM6DSx 6-axis IMU** driver).

---

## 1. Enabling In-Tree Drivers

The Linux kernel has many drivers built directly into the source tree that simply need to be enabled via kernel configuration options (`CONFIG_*`).

### LSM6DSx IMU Driver
The LSM6DSx driver is located in-tree at `drivers/iio/imu/st_lsm6dsx/` and is already enabled in [`.config-fragment`](.config-fragment):
```kconfig
# Industrial I/O Core & Buffers
CONFIG_IIO=m
CONFIG_IIO_BUFFER=y
CONFIG_IIO_KFIFO_BUF=m
CONFIG_IIO_TRIGGERED_BUFFER=m

# ST LSM6DSX Sensor Driver
CONFIG_IIO_ST_LSM6DSX=m
CONFIG_IIO_ST_LSM6DSX_I2C=m
CONFIG_IIO_ST_LSM6DSX_SPI=m
```

### Adding Other In-Tree Drivers
To enable any other in-tree driver:
1. Open [`.config-fragment`](.config-fragment).
2. Append your required kernel config flags (e.g. `CONFIG_CAN_MCP251X=m`, `CONFIG_SPI_SPIDEV=m`, etc.).
3. If you want to explore all available kernel config options interactively with `menuconfig`, enter the build container:
   ```bash
   docker run -it rtwg-noble-builder bash
   cd /linux_build/linux-raspi
   make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- menuconfig
   ```

---

## 2. Building the Kernel Debian Packages

Run the build script:
```bash
./kernel_builder/build_kernel.sh
```

This will:
1. Build the Docker container with ARM64 cross-compiler and patched Ubuntu 24.04 `raspi` kernel source.
2. Merge `.config-fragment` into the kernel configuration.
3. Cross-compile the kernel with `make bindeb-pkg`.
4. Output the `.deb` files into `cache/custom_kernel_debs/`:
   - `linux-image-*.deb`
   - `linux-headers-*.deb`
   - `linux-libc-dev_*.deb`

---

## 3. Building the Final Image

Once the `.deb` files are generated in `cache/custom_kernel_debs/`, return to the repo root and run:
```bash
sudo make noble-rt-ros2-qplatform
# (or without ROS 2: sudo make noble-rt-qplatform)
```

The image builder will:
1. Detect and install your custom kernel packages.
2. Configure Ubuntu 24.04 Server for real-time operation.
3. Install ROS 2 Jazzy.
4. Set up the LSM6DSX driver, IIO utilities, I2C/SPI overlays, and the automatic module loading service.
5. Produce the final bootable SD card image in `out/`.
