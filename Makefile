.PHONY: focal-rt-ros2 jammy-rt-ros2 noble-rt-ros2 noble-rt-qplatform noble-rt-ros2-qplatform clean

jammy-rt-ros2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble

# TODO: eventually the build.py should be a command line script that takes
#       arguments
focal-rt-ros2:
	sudo ./ros-rt-img build focal-rt focal-rt-galactic

noble-rt-ros2:
	sudo ./ros-rt-img build noble-rt noble-rt-jazzy

DTC ?= $(shell which dtc 2>/dev/null || echo ./bin/dtc)
DTBO_DIR = image_builder/data/noble-rt-qplatform/rootfs/boot/firmware/overlays
DTS_DIR = image_builder/data/noble-rt-qplatform/dts

dtbo: $(DTBO_DIR)/lsm6dsx-spi.dtbo

$(DTBO_DIR)/lsm6dsx-spi.dtbo: $(DTS_DIR)/lsm6dsx-spi.dts
	mkdir -p $(DTBO_DIR)
	$(DTC) -@ -Wno-unit_address_vs_reg -I dts -O dtb -o $@ $<

noble-rt-qplatform: dtbo
	sudo ./ros-rt-img build noble-rt noble-rt-qplatform

noble-rt-ros2-qplatform: dtbo
	sudo ./ros-rt-img build noble-rt noble-rt-jazzy noble-rt-qplatform

# Backward-compatibility aliases
noble-rt-lsm6dsx: noble-rt-qplatform
noble-rt-ros2-lsm6dsx: noble-rt-ros2-qplatform

clean:
	sudo ./ros-rt-img teardown
	sudo rm -rf out cache
