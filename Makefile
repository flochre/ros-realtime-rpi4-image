.PHONY: focal-rt-ros2 jammy-rt-ros2 clean

# TODO: eventually the build.py should be a command line script that takes
#       arguments
focal-rt-ros2:
	sudo ./ros-rt-img build

jammy-rt-ros2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble

jammy-rt-stan2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble-stanley

clean:
	sudo ./ros-rt-img teardown
	sudo rm -rf out cache
