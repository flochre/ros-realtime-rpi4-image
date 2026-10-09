# Ubuntu Core + Android Kernel Modules - Proof of Concept

## Overview

This proof-of-concept adds support for building ROS 2 images with:
- **Ubuntu Core 24.04** base (minimal, read-write rootfs for this POC)
- **PREEMPT_RT kernel** for real-time determinism
- **Android kernel module infrastructure** for future Android compatibility

## Motivation

Ubuntu Core provides a minimal, security-focused base that can be useful for:
- Embedded real-time systems with minimal attack surface
- Reduced storage and memory footprint
- Future Android app/module integration on RPi

## Current Implementation

### Profile Structure
```
image_builder/data/noble-rt-core/
├── config.ini              # Ubuntu Core 24.04 image source
├── scripts/
│   ├── extract-image       # XZ decompression (Ubuntu Core format)
│   ├── loop-device-setup   # Single-partition layout handling
│   └── phase1-target       # RT kernel + Android module setup
└── rootfs/
    ├── etc/systemd/system/ # RT and Android-related services
    └── opt/
        ├── ros2-rt-rpi4/   # RT performance scripts
        └── android-modules/# Android module loading infrastructure
```

### What's Included

1. **RT Kernel Integration**
   - Installs PREEMPT_RT 6.8.4-rt11 kernel
   - Disables RT throttling for deterministic scheduling
   - Disables memory compaction (latency reduction)
   - Proper kernel symlink setup

2. **Android Module Infrastructure**
   - Kernel headers for module compilation
   - Build tools (`build-essential`, `kmod`, `dkms`)
   - Directory structure for Android modules
   - Service file for automatic module loading
   - Placeholder for common Android modules (binder, ashmem)

3. **ROS 2 Jazzy**
   - Full ROS 2 Jazzy installation
   - Real-time tools (cyclictest, stress-ng)
   - CPU frequency utilities

### Image Specifications

| Property | Value |
|----------|-------|
| Base Image | Ubuntu Core 24.04 |
| Size | 3GB (expandable) |
| Kernel | PREEMPT_RT 6.8.4-rt11 |
| ROS | Jazzy |
| Default User | ubuntu / ubuntu |
| Architecture | ARM64 (Raspberry Pi 4/5) |

## Building the Image

```bash
# Option 1: Build just Ubuntu Core RT image
sudo ./ros-rt-img build noble-rt-core

# Option 2: Build Ubuntu Core + Android overlay
sudo ./ros-rt-img build noble-rt-core noble-rt-core-android
```

## Testing the Image

1. **Decompress the image**
   ```bash
   zstd -d ubuntu-24.04-core-rt-android-arm64+raspi.img.zst
   ```

2. **Flash to SD card**
   ```bash
   sudo dd if=ubuntu-24.04-core-rt-android-arm64+raspi.img of=/dev/sdX bs=4M
   sudo sync
   ```

3. **Boot and verify**
   ```bash
   # SSH to the Raspberry Pi
   ssh ubuntu@<ip-address>
   
   # Verify RT kernel
   uname -a  # should show PREEMPT_RT version
   
   # Check Android module infrastructure
   ls -la /opt/android-modules/
   ls -la /lib/modules/*/extra/android/
   
   # Test RT scheduling
   sudo cyclictest -p95 -m -n -l 1000
   
   # Verify ROS 2
   ros2 --version
   ```

## Future Work

### Phase 2: Custom Kernel Build
1. Enable Android-specific kernel config options:
   - `CONFIG_ANDROID_BINDER=y`
   - `CONFIG_ANDROID_LOW_MEMORY_KILLER=y`
   - `CONFIG_ASHMEM=y`
   - SELinux configuration for Android compatibility

2. Build kernel with Android modules included
3. Test module loading and integration

### Phase 3: Android Runtime Integration
1. Evaluate ART (Android Runtime) compatibility with PREEMPT_RT
2. Test Android app execution on real-time kernel
3. Measure latency impact of Android modules

### Phase 4: Production Hardening
1. Ubuntu Core read-only rootfs with overlays
2. Secure boot integration
3. Full latency testing suite

## Known Limitations

1. **No full Android ecosystem**: This is ROS 2 + RT + Android module support, not full Android OS
2. **Single partition**: Ubuntu Core single-partition layout; no separate boot partition
3. **Module loading placeholder**: Android module loading is infrastructure-only in POC
4. **Minimal testing**: POC has not been tested on hardware yet

## Differences from noble-rt

| Aspect | noble-rt | noble-rt-core |
|--------|----------|---------------|
| Base Image | Ubuntu Server 24.04 | Ubuntu Core 24.04 |
| Partitions | 2 (boot, rootfs) | 1 (rootfs) |
| Size | 4GB | 3GB |
| Android Support | No | Infrastructure only |
| Snapd | Removed | Removed |
| Default Services | Standard Ubuntu | Minimal |

## Contributing

To extend this POC:

1. Create additional profiles: `noble-rt-core-android-full`
2. Add kernel config customization scripts
3. Integrate Android module compilation
4. Add latency testing benchmarks
5. Document real-world use cases

## References

- [Ubuntu Core Documentation](https://ubuntu.com/core/docs)
- [PREEMPT_RT Kernel](https://wiki.linuxfoundation.org/realtime/start)
- [Android Kernel Common](https://android.googlesource.com/kernel/common/)
- [ROS 2 Real-Time Programming](https://docs.ros.org/en/humble/Tutorials/Demos/Real-Time-Programming.html)
