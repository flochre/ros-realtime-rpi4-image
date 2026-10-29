#!/usr/bin/env python3
"""
Direct 3-Wheel Omni Teleop from Thrustmaster Gamepad (/dev/input/event0)
Computes inverse kinematics and streams step setpoints to the 3 STM32 motor nodes.
"""

import os
import sys
import math
import time
import struct
import select
import subprocess

# Kinematics and robot geometry parameters
R = 0.1686          # Robot center to wheel radius [m]
r = 0.030           # Omni wheel radius [m]
STEPS_PER_REV = 51200.0
GEAR_RATIO = 3.2777778
STEPS_PER_RAD = (STEPS_PER_REV * GEAR_RATIO) / (2.0 * math.pi)

# Wheel mounting angles (120 deg separation, symmetric omni base)
THETA_1 = 0.0
THETA_2 = 2.0 * math.pi / 3.0
THETA_3 = 4.0 * math.pi / 3.0

def omni_ik(vx, vy, omega):
    """
    Compute individual wheel angular velocities (rad/s) from base twist.
    """
    v1 = -math.sin(THETA_1) * vx + math.cos(THETA_1) * vy + R * omega
    v2 = -math.sin(THETA_2) * vx + math.cos(THETA_2) * vy + R * omega
    v3 = -math.sin(THETA_3) * vx + math.cos(THETA_3) * vy + R * omega
    return (v1 / r), (v2 / r), (v3 / r)

def main():
    event_path = "/dev/input/event0"
    if not os.path.exists(event_path):
        print(f"Error: {event_path} not found. Ensure controller is attached via usbipd.")
        sys.exit(1)

    try:
        fd = os.open(event_path, os.O_RDONLY | os.O_NONBLOCK)
    except PermissionError:
        print(f"Permission denied on {event_path}. Run: sudo chmod 666 {event_path}")
        sys.exit(1)

    print("========================================================")
    print(" QPlatform 3-Wheel Omni Teleop (Thrustmaster Dual Trigger)")
    print("========================================================")
    print(" Left Stick : Translation (Vx = Forward/Back, Vy = Strafe)")
    print(" Right Stick: Rotation (Omega)")
    print(" Press Ctrl+C in terminal to stop.")
    print("--------------------------------------------------------")

    raw_axes = {0: 128, 1: 128, 2: 128, 3: 128, 4: 128, 5: 128}
    buttons = {}

    max_linear_speed = 0.20   # 0.20 m/s max safe speed
    max_angular_speed = 1.0   # 1.0 rad/s max rotation

    loop_rate_hz = 50.0
    dt = 1.0 / loop_rate_hz

    # Start an interactive python ROS bridge process on the Pi via SSH
    ssh_cmd = [
        "ssh", "-o", "ConnectTimeout=5", "ubuntu@192.168.1.19",
        "bash -c 'source /opt/ros/jazzy/setup.bash && python3 -u -c \""
        "import sys, rclpy\\n"
        "from rclpy.node import Node\\n"
        "from std_msgs.msg import Int32\\n"
        "from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue\\n"
        "from rcl_interfaces.srv import SetParameters\\n"
        "rclpy.init()\\n"
        "node = Node(\\\"teleop_bridge\\\")\\n"
        "req = SetParameters.Request()\\n"
        "req.parameters = [\\n"
        "    Parameter(name=\\\"motor_enable\\\", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),\\n"
        "    Parameter(name=\\\"deadband\\\", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=4)),\\n"
        "    Parameter(name=\\\"command_timeout_ms\\\", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=500)),\\n"
        "    Parameter(name=\\\"maixsense_enable\\\", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),\\n"
        "]\\n"
        "for ns in (\\\"/motor_42626A\\\", \\\"/motor_58626B\\\", \\\"/motor_6F6260\\\"):\\n"
        "    cli = node.create_client(SetParameters, ns + \\\"/set_parameters\\\")\\n"
        "    if cli.wait_for_service(timeout_sec=1.0):\\n"
        "        fut = cli.call_async(req)\\n"
        "        rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)\\n"
        "p1 = node.create_publisher(Int32, \\\"/motor_42626A/cmd_pos\\\", 10)\\n"
        "p2 = node.create_publisher(Int32, \\\"/motor_58626B/cmd_pos\\\", 10)\\n"
        "p3 = node.create_publisher(Int32, \\\"/motor_6F6260/cmd_pos\\\", 10)\\n"
        "m = Int32()\\n"
        "for line in sys.stdin:\\n"
        "    parts = line.strip().split()\\n"
        "    if len(parts) == 3:\\n"
        "        m.data = int(parts[0]); p1.publish(m)\\n"
        "        m.data = int(parts[1]); p2.publish(m)\\n"
        "        m.data = int(parts[2]); p3.publish(m)\\n"
        "\"'"
    ]

    print("Connecting to robot micro-ROS bridge...")
    proc = subprocess.Popen(ssh_cmd, stdin=subprocess.PIPE, text=True, bufsize=1)
    time.sleep(1.0)
    print("Bridge connected! Ready for driving.")

    accum_steps_1 = 0.0
    accum_steps_2 = 0.0
    accum_steps_3 = 0.0

    try:
        while True:
            while True:
                r, _, _ = select.select([fd], [], [], 0.0)
                if not r:
                    break
                data = os.read(fd, 24)
                if len(data) == 24:
                    sec, usec, ev_type, code, val = struct.unpack('qqHHi', data)
                    if ev_type == 3:  # EV_ABS
                        raw_axes[code] = val
                    elif ev_type == 1: # EV_KEY
                        buttons[code] = val

            norm_x = (raw_axes.get(0, 128) - 128) / 128.0
            norm_y = -(raw_axes.get(1, 128) - 128) / 128.0   # Up positive
            norm_rot = (raw_axes.get(2, 128) - 128) / 128.0

            # Deadzone 0.10
            if abs(norm_x) < 0.10: norm_x = 0.0
            if abs(norm_y) < 0.10: norm_y = 0.0
            if abs(norm_rot) < 0.10: norm_rot = 0.0

            vx = norm_y * max_linear_speed
            vy = norm_x * max_linear_speed
            omega = norm_rot * max_angular_speed

            w1, w2, w3 = omni_ik(vx, vy, omega)

            delta_1 = w1 * STEPS_PER_RAD * dt
            delta_2 = w2 * STEPS_PER_RAD * dt
            delta_3 = w3 * STEPS_PER_RAD * dt

            accum_steps_1 += delta_1
            accum_steps_2 += delta_2
            accum_steps_3 += delta_3

            if abs(delta_1) > 0.01 or abs(delta_2) > 0.01 or abs(delta_3) > 0.01:
                cmd_line = f"{int(round(accum_steps_1))} {int(round(accum_steps_2))} {int(round(accum_steps_3))}\n"
                proc.stdin.write(cmd_line)
                proc.stdin.flush()

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopping robot...")
    finally:
        os.close(fd)
        if proc.poll() is None:
            proc.terminate()
            proc.wait()
        print("Teleop stopped safely.")

if __name__ == "__main__":
    main()
