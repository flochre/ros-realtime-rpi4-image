.PHONY: focal-rt-ros2 jammy-rt-ros2 jammy-rt-stan2 jammy-rt-ruediger2 clean

# TODO: eventually the build.py should be a command line script that takes
#       arguments
focal-rt-ros2:
	sudo ./ros-rt-img build

jammy-rt:
	sudo ./ros-rt-img build jammy-rt

jammy-rt-ros2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble

jammy-rt-stan2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble-stanley
	sudo chown $(USER):$(USER) out/ out/*

jammy-rt-ruediger2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble jammy-rt-humble-ruediger

jammy-rt-ruediger2-online:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble jammy-rt-humble-ruediger-online

clean:
	sudo ./ros-rt-img teardown
	sudo rm -rf out cache
