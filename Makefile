.PHONY: focal-rt-ros2 jammy-rt-ros2 clean

# TODO: eventually the build.py should be a command line script that takes
#       arguments

jammy-rt-ros2:
	sudo ./ros-rt-img build jammy-rt jammy-rt-humble
	sudo chown -R $$(id -u):$$(id -g) out cache

jammy-rt-ruediger2:
	sudo ./ros-rt-img build jammy-rt-humble-ruediger
	sudo chown -R $$(id -u):$$(id -g) out cache

jammy-rt-stanley2:
	sudo ./ros-rt-img build jammy-rt-humble-stanley
	sudo chown -R $$(id -u):$$(id -g) out cache

clean:
	sudo ./ros-rt-img teardown
	sudo rm -rf out cache
