#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Int32

def configure_motor_service(node, motor_name, deadband=4, timeout_ms=500, enable_maixsense=True):
    srv_name = f"/{motor_name}/set_parameters"
    client = node.create_client(SetParameters, srv_name)
    if not client.wait_for_service(timeout_sec=2.0):
        print(f" [WARN] Service {srv_name} not available; skipping service config.")
        node.destroy_client(client)
        return False

    req = SetParameters.Request()
    req.parameters = [
        Parameter(name="motor_enable", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
        Parameter(name="deadband", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(deadband))),
        Parameter(name="command_timeout_ms", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(timeout_ms))),
        Parameter(name="maixsense_enable", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enable_maixsense))),
    ]
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    ok = False
    if future.done() and future.result() is not None:
        res = future.result()
        ok = bool(res.results) and all(r.successful for r in res.results)
        print(f" -> Configured {srv_name} (deadband={deadband}, timeout_ms={timeout_ms}, maixsense_enable={enable_maixsense}): {'OK' if ok else 'PARTIAL'}")
    else:
        print(f" [WARN] Service call to {srv_name} timed out.")
    node.destroy_client(client)
    return ok

def main():
    rclpy.init()
    node = Node("motor_test_runner")
    best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

    motors = [
        ("Motor 1 (pivot_1)", "motor_42626A"),
        ("Motor 2 (pivot_2)", "motor_58626B"),
        ("Motor 3 (pivot_3)", "motor_6F6260"),
    ]

    for label, m in motors:
        print(f"\n==========================================")
        print(f" Testing {label} -> /{m}")
        print(f"==========================================")
        configure_motor_service(node, m, deadband=4, timeout_ms=500, enable_maixsense=True)
        current_pos = None

        def pos_cb(msg):
            nonlocal current_pos
            current_pos = msg.data

        sub = node.create_subscription(Int32, f"/{m}/position", pos_cb, best_effort_qos)
        pub = node.create_publisher(Int32, f"/{m}/cmd_pos", 10)

        # Wait up to 3s to get current position
        start = time.time()
        while current_pos is None and time.time() - start < 3.0:
            rclpy.spin_once(node, timeout_sec=0.05)

        if current_pos is None:
            print(f" [ERROR] Could not read feedback position from /{m}/position!")
            node.destroy_subscription(sub)
            node.destroy_publisher(pub)
            continue

        initial_pos = current_pos
        print(f" -> Initial Position: {initial_pos}")

        # Move forward slowly: 10240 microsteps (~0.2 motor rev) over 1.5 seconds at 50 Hz
        steps_delta = 10240
        duration = 1.5
        rate_hz = 50
        steps_per_tick = steps_delta / (duration * rate_hz)

        print(f" -> Moving forward +{steps_delta} steps slowly...")
        pos_target = float(initial_pos)
        for _ in range(int(duration * rate_hz)):
            pos_target += steps_per_tick
            msg = Int32()
            msg.data = int(round(pos_target))
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.001)
            time.sleep(1.0 / rate_hz)

        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
        print(f" -> Position after forward move: {current_pos} (Target was {initial_pos + steps_delta})")

        # Move back slowly to initial position
        print(f" -> Returning slowly back to {initial_pos}...")
        for _ in range(int(duration * rate_hz)):
            pos_target -= steps_per_tick
            msg = Int32()
            msg.data = int(round(pos_target))
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.001)
            time.sleep(1.0 / rate_hz)

        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
        print(f" -> Final Position after return: {current_pos} (Initial was {initial_pos})")
        print(f" -> Net position delta: {current_pos - initial_pos} steps")

        node.destroy_subscription(sub)
        node.destroy_publisher(pub)
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
