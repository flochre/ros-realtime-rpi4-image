#!/usr/bin/env bash
# ==============================================================================
# Setup IIO High-Resolution Timer (hrtimer) Software Trigger for LSM6DSx IMU
# ==============================================================================
# Resolves slow reading speeds by:
# 1. Loading the industrialio-sw-trigger and iio-trig-hrtimer kernel modules
# 2. Creating a virtual hrtimer trigger in configfs at the requested rate (e.g. 100Hz/200Hz)
# 3. Setting the sensor's internal Output Data Rate (ODR) so it doesn't sleep
# 4. Enabling IIO continuous triggered buffer streaming via /dev/iio:device*
# ==============================================================================

set -e

RATE="${1:-100}"
TRIGGER_NAME="lsm6dsx_trig"

echo "=================================================="
echo " Configuring IIO hrtimer Trigger & Hardware ODR"
echo " Target Frequency: ${RATE} Hz"
echo "=================================================="

# 1. Load kernel modules for virtual software triggers
modprobe industrialio 2>/dev/null || true
modprobe industrialio-sw-trigger 2>/dev/null || true
modprobe iio-trig-hrtimer 2>/dev/null || true
modprobe iio-trig-sysfs 2>/dev/null || true

# 2. Mount configfs if not already mounted
if ! mountpoint -q /sys/kernel/config; then
    mkdir -p /sys/kernel/config
    mount -t configfs none /sys/kernel/config 2>/dev/null || true
fi

# 3. Create virtual hrtimer trigger via configfs
CONFIGFS_TRIG="/sys/kernel/config/iio/triggers/hrtimer/${TRIGGER_NAME}"
if [ ! -d "$CONFIGFS_TRIG" ]; then
    echo "Creating hrtimer trigger in configfs: ${TRIGGER_NAME}"
    mkdir -p "$CONFIGFS_TRIG"
fi

# 4. Locate the created trigger device in sysfs
TRIG_SYSFS=""
for t in /sys/bus/iio/devices/trigger*; do
    if [ -f "$t/name" ] && grep -q "^${TRIGGER_NAME}$" "$t/name"; then
        TRIG_SYSFS="$t"
        break
    fi
done

if [ -n "$TRIG_SYSFS" ]; then
    echo "Trigger located at ${TRIG_SYSFS}. Setting sampling frequency to ${RATE} Hz..."
    echo "$RATE" > "${TRIG_SYSFS}/sampling_frequency" || true
else
    echo "Warning: Trigger device not found under /sys/bus/iio/devices/"
fi

# 5. Determine matching hardware ODR (Output Data Rate) >= target rate
# LSM6DS3TR-C supports ODR: 12.5, 26, 52, 104, 208, 416, 833 Hz
ODR="104"
if [ "$RATE" -gt 104 ]; then
    ODR="208"
fi
if [ "$RATE" -gt 208 ]; then
    ODR="416"
fi

# 6. Configure each LSM6DSx sensor device (accel and gyro)
for dev in /sys/bus/iio/devices/iio:device*; do
    if [ ! -f "$dev/name" ]; then continue; fi
    NAME=$(cat "$dev/name" | tr -d '\r\n')

    if [[ "$NAME" == *"lsm6ds"* ]] || [[ "$NAME" == *"ism330"* ]]; then
        echo "--> Configuring sensor: $dev ($NAME)"

        # Disable buffer temporarily before configuring
        if [ -f "$dev/buffer/enable" ]; then
            echo 0 > "$dev/buffer/enable" 2>/dev/null || true
        fi

        # Write hardware sampling frequency (CRITICAL: prevents driver sleep delay)
        if [ -f "$dev/sampling_frequency" ]; then
            echo "    Setting hardware ODR to ${ODR} Hz"
            echo "$ODR" > "$dev/sampling_frequency" 2>/dev/null || true
        fi

        # Enable all available channels in scan_elements
        if [ -d "$dev/scan_elements" ]; then
            for en in "$dev/scan_elements"/*_en; do
                if [ -f "$en" ]; then
                    echo 1 > "$en" 2>/dev/null || true
                fi
            done
        fi

        # Assign the virtual hrtimer trigger
        if [ -f "$dev/trigger/current_trigger" ]; then
            echo "    Attaching trigger ${TRIGGER_NAME}"
            echo "$TRIGGER_NAME" > "$dev/trigger/current_trigger" 2>/dev/null || true
        fi

        # Set buffer length (FIFO depth in kernel memory)
        if [ -f "$dev/buffer/length" ]; then
            echo 128 > "$dev/buffer/length" 2>/dev/null || true
        fi

        # Enable buffer for continuous streaming
        if [ -f "$dev/buffer/enable" ]; then
            echo "    Enabling IIO continuous buffer"
            echo 1 > "$dev/buffer/enable" 2>/dev/null || true
        fi
    fi
done

echo "✓ LSM6DSx IIO hrtimer trigger and continuous buffer active!"
