#!/usr/bin/env python3
"""
Safe Web Teleoperation Server for QPlatform Robot.
Key safety guarantees:
- Never publishes redundant cmd_pos messages when stationary or on timeout.
- TMC2160 enters true zero-current or silent IHOLD mode when motionless.
- 350ms watchdog cleanly zeros velocity on connection loss without PWM chatter.
- Integrated diagnostics endpoint (/diag) for live link monitoring.
"""

import os
import sys
import math
import time
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

# Kinematics parameters
R = 0.1686          # Robot center to wheel radius [m]
r = 0.030           # Omni wheel radius [m]
STEPS_PER_REV = 51200.0
GEAR_RATIO = 3.2777778
STEPS_PER_RAD = (STEPS_PER_REV * GEAR_RATIO) / (2.0 * math.pi)

THETA_1 = 0.0
THETA_2 = 2.0 * math.pi / 3.0
THETA_3 = 4.0 * math.pi / 3.0

def omni_ik(vx, vy, omega):
    v1 = -math.sin(THETA_1) * vx + math.cos(THETA_1) * vy + R * omega
    v2 = -math.sin(THETA_2) * vx + math.cos(THETA_2) * vy + R * omega
    v3 = -math.sin(THETA_3) * vx + math.cos(THETA_3) * vy + R * omega
    return (v1 / r), (v2 / r), (v3 / r)

lock = threading.Lock()
target_vx = 0.0
target_vy = 0.0
target_omega = 0.0

smooth_vx = 0.0
smooth_vy = 0.0
smooth_omega = 0.0

last_command_time = 0.0
TIMEOUT_SEC = 0.35

stat_packet_count = 0
stat_watchdog_trips = 0
stat_last_jitter_ms = 0.0
stat_is_timed_out = True
stat_last_received_time = 0.0

MOTOR_NAMESPACES = ("/motor_42626A", "/motor_58626B", "/motor_6F6260")
cfg_deadband_steps = 4
cfg_command_timeout_ms = 500
cfg_maixsense_enable = True
motor_config_status = {ns: False for ns in MOTOR_NAMESPACES}

ros_node = None
ros_publishers = None
ros_param_clients = {}
ros_config_pending = True

def configure_motors_service(deadband_steps=None, command_timeout_ms=None, maixsense_enable=None, motor_enable=True):
    """Send rcl_interfaces/srv/SetParameters requests to all 3 STM32 motor nodes."""
    global cfg_deadband_steps, cfg_command_timeout_ms, cfg_maixsense_enable, ros_config_pending
    if deadband_steps is not None:
        cfg_deadband_steps = max(0, int(deadband_steps))
    if command_timeout_ms is not None:
        cfg_command_timeout_ms = max(0, int(command_timeout_ms))
    if maixsense_enable is not None:
        cfg_maixsense_enable = bool(maixsense_enable)

    if not ros_param_clients:
        ros_config_pending = True
        return False

    try:
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
    except Exception:
        return False

    def make_int_param(name, val):
        p = Parameter()
        p.name = name
        p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(val))
        return p

    def make_bool_param(name, val):
        p = Parameter()
        p.name = name
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(val))
        return p

    req = SetParameters.Request()
    req.parameters = [
        make_bool_param("motor_enable", motor_enable),
        make_int_param("deadband", cfg_deadband_steps),
        make_int_param("command_timeout_ms", cfg_command_timeout_ms),
        make_bool_param("maixsense_enable", cfg_maixsense_enable),
    ]

    all_ready = True
    for ns, client in ros_param_clients.items():
        if client.service_is_ready():
            future = client.call_async(req)
            def _done_cb(fut, motor_ns=ns):
                try:
                    res = fut.result()
                    ok = bool(res.results) and all(r.successful for r in res.results)
                    motor_config_status[motor_ns] = ok
                except Exception:
                    motor_config_status[motor_ns] = False
            future.add_done_callback(_done_cb)
        else:
            motor_config_status[ns] = False
            all_ready = False

    ros_config_pending = not all_ready
    return all_ready

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>QPlatform Teleop</title>
  <style>
    :root {
      --bg: #090d16;
      --card: #131b2e;
      --accent: #00d2ff;
      --text: #f0f4f8;
      --muted: #627d98;
      --ok: #00e676;
      --warn: #ffab00;
      --danger: #ff1744;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 24px 16px;
    }
    .header {
      width: 100%;
      max-width: 680px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 20px;
      padding-bottom: 12px;
      border-bottom: 1px solid #1f2d47;
    }
    .title { font-size: 1.25rem; font-weight: 700; color: var(--accent); }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 6px 14px;
      border-radius: 999px;
      font-size: 0.82rem;
      font-weight: 600;
      background: #1a233a;
    }
    .dot {
      width: 10px; height: 10px; border-radius: 50%;
      background: var(--danger);
      box-shadow: 0 0 10px var(--danger);
    }
    .dot.connected {
      background: var(--ok);
      box-shadow: 0 0 10px var(--ok);
    }
    .card {
      width: 100%;
      max-width: 680px;
      background: var(--card);
      border: 1px solid #1f2d47;
      border-radius: 16px;
      padding: 20px;
      margin-bottom: 16px;
      box-shadow: 0 8px 30px rgba(0, 0, 0, 0.4);
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
      margin-top: 14px;
    }
    .metric {
      background: #0d1424;
      border: 1px solid #1a253d;
      border-radius: 10px;
      padding: 12px;
      text-align: center;
    }
    .metric-label { font-size: 0.72rem; text-transform: uppercase; color: var(--muted); }
    .metric-value { font-size: 1.35rem; font-weight: 700; color: var(--text); margin-top: 4px; font-variant-numeric: tabular-nums; }
    .status-alert {
      padding: 10px 14px;
      border-radius: 8px;
      font-size: 0.85rem;
      font-weight: 600;
      text-align: center;
      margin-top: 14px;
    }
    .alert-warn {
      background: rgba(255, 171, 0, 0.15);
      border: 1px solid var(--warn);
      color: var(--warn);
    }
    .alert-ok {
      background: rgba(0, 230, 118, 0.15);
      border: 1px solid var(--ok);
      color: var(--ok);
    }
  </style>
</head>
<body>
  <div class="header">
    <div class="title">QPlatform Robot Teleop</div>
    <div class="badge">
      <span class="dot" id="status-dot"></span>
      <span id="gamepad-status">Gamepad Disconnected</span>
    </div>
  </div>

  <div class="card">
    <div style="display:flex; justify-content:space-between; align-items:center;">
      <h3 style="font-size: 0.9rem; color: var(--muted); text-transform: uppercase;">Command Setpoints</h3>
      <span id="gamepad-name" style="font-size: 0.82rem; color: var(--accent); font-weight: 600;">Press any button...</span>
    </div>

    <div class="grid">
      <div class="metric">
        <div class="metric-label">Forward / Back (Vx)</div>
        <div class="metric-value" id="val-vx">0.00 m/s</div>
      </div>
      <div class="metric">
        <div class="metric-label">Strafe (Vy)</div>
        <div class="metric-value" id="val-vy">0.00 m/s</div>
      </div>
      <div class="metric">
        <div class="metric-label">Rotation (Omega)</div>
        <div class="metric-value" id="val-omega">0.00 rad/s</div>
      </div>
      <div class="metric">
        <div class="metric-label">Client Send Rate</div>
        <div class="metric-value" id="val-rate">0 Hz</div>
      </div>
    </div>
  </div>

  <div class="card">
    <h3 style="font-size: 0.9rem; color: var(--muted); text-transform: uppercase;">Link Health & Diagnostics</h3>
    <div class="grid">
      <div class="metric">
        <div class="metric-label">Watchdog Dropouts</div>
        <div class="metric-value" id="val-timeouts" style="color: var(--ok);">0</div>
      </div>
      <div class="metric">
        <div class="metric-label">Packet Interval</div>
        <div class="metric-value" id="val-jitter">0 ms</div>
      </div>
    </div>
    <div class="status-alert alert-ok" id="alert-banner" style="margin-top:14px;">Standby: Stationary (Drivers in silent hold)</div>
  </div>

  <div class="card">
    <h3 style="font-size: 0.9rem; color: var(--muted); text-transform: uppercase;">STM32 Firmware & LiDAR Service Configuration</h3>
    <div class="grid" style="grid-template-columns: repeat(3, 1fr);">
      <div class="metric">
        <div class="metric-label">Deadband (steps)</div>
        <input id="inp-deadband" type="number" min="0" max="256" value="4" style="margin-top:6px; width:100%; background:#131b2e; color:var(--text); border:1px solid #2b3c5e; border-radius:6px; padding:6px; text-align:center; font-weight:700;">
      </div>
      <div class="metric">
        <div class="metric-label">Timeout (ms)</div>
        <input id="inp-timeout" type="number" min="0" max="10000" value="500" style="margin-top:6px; width:100%; background:#131b2e; color:var(--text); border:1px solid #2b3c5e; border-radius:6px; padding:6px; text-align:center; font-weight:700;">
      </div>
      <div class="metric">
        <div class="metric-label">MaixSense LiDAR</div>
        <select id="inp-maixsense" style="margin-top:6px; width:100%; background:#131b2e; color:var(--text); border:1px solid #2b3c5e; border-radius:6px; padding:6px; text-align:center; font-weight:700;">
          <option value="true" selected>Enabled</option>
          <option value="false">Disabled</option>
        </select>
      </div>
    </div>
    <button onclick="applyConfig()" style="margin-top:12px; width:100%; padding:10px; border:none; border-radius:8px; background:var(--accent); color:#090d16; font-weight:700; cursor:pointer;">Apply via /set_parameters Service</button>
    <div id="cfg-status" style="margin-top:8px; font-size:0.78rem; color:var(--muted); text-align:center;">Configuring motors...</div>
  </div>

  <script>
    let activeGamepadIndex = null;
    const maxLinearSpeed = 0.25;
    const maxAngularSpeed = 1.2;
    let lastSent = 0;
    let sentCount = 0;
    let lastRateCalc = performance.now();

    window.addEventListener("gamepadconnected", (e) => {
      activeGamepadIndex = e.gamepad.index;
      document.getElementById("status-dot").classList.add("connected");
      document.getElementById("gamepad-status").innerText = "Gamepad Active";
      document.getElementById("gamepad-name").innerText = e.gamepad.id.substring(0, 24);
    });

    window.addEventListener("gamepaddisconnected", (e) => {
      if (activeGamepadIndex === e.gamepad.index) {
        activeGamepadIndex = null;
        document.getElementById("status-dot").classList.remove("connected");
        document.getElementById("gamepad-status").innerText = "Gamepad Disconnected";
        document.getElementById("gamepad-name").innerText = "Press any button...";
      }
    });

    function loop() {
      if (activeGamepadIndex !== null) {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[activeGamepadIndex];
        if (gp) {
          let rawX = gp.axes[0] || 0;
          let rawY = -(gp.axes[1] || 0);
          let rawRot = (gp.axes[2] !== undefined && Math.abs(gp.axes[2]) > 0.05) ? gp.axes[2] : (gp.axes[3] || 0);

          const deadzone = 0.10;
          let normX = Math.abs(rawX) > deadzone ? rawX : 0;
          let normY = Math.abs(rawY) > deadzone ? rawY : 0;
          let normRot = Math.abs(rawRot) > deadzone ? rawRot : 0;

          let vx = normY * maxLinearSpeed;
          let vy = normX * maxLinearSpeed;
          let omega = normRot * maxAngularSpeed;

          document.getElementById("val-vx").innerText = vx.toFixed(2) + " m/s";
          document.getElementById("val-vy").innerText = vy.toFixed(2) + " m/s";
          document.getElementById("val-omega").innerText = omega.toFixed(2) + " rad/s";

          const now = performance.now();
          if (now - lastSent >= 20) {
            lastSent = now;
            sentCount++;
            fetch("/cmd", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ vx: vx, vy: vy, omega: omega })
            }).catch(() => {});
          }

          if (now - lastRateCalc >= 1000) {
            document.getElementById("val-rate").innerText = sentCount + " Hz";
            sentCount = 0;
            lastRateCalc = now;
          }
        }
      }
      requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);

    function applyConfig() {
      const deadband = parseInt(document.getElementById("inp-deadband").value || "4", 10);
      const timeout_ms = parseInt(document.getElementById("inp-timeout").value || "500", 10);
      const maixsense = document.getElementById("inp-maixsense").value === "true";
      fetch("/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ deadband_steps: deadband, command_timeout_ms: timeout_ms, maixsense_enable: maixsense })
      }).then(r => r.json()).then(d => {
        document.getElementById("cfg-status").innerText = `Sent set_parameters (deadband=${d.deadband_steps}, timeout=${d.command_timeout_ms}ms, maixsense=${d.maixsense_enable})`;
      }).catch(() => {});
    }

    setInterval(() => {
      fetch("/diag").then(r => r.json()).then(data => {
        document.getElementById("val-timeouts").innerText = data.watchdog_trips;
        document.getElementById("val-jitter").innerText = data.last_interval_ms + " ms";
        if (data.motor_config_status) {
          const entries = Object.entries(data.motor_config_status).map(([k, v]) => `${k}: ${v ? "OK" : "pending"}`);
          document.getElementById("cfg-status").innerText = entries.join(" | ");
        }
        const banner = document.getElementById("alert-banner");
        if (data.watchdog_trips > 0) {
          document.getElementById("val-timeouts").style.color = "var(--warn)";
          banner.className = "status-alert alert-warn";
          banner.innerText = `Watchdog Warning: ${data.watchdog_trips} Wi-Fi dropouts. Last interval: ${data.last_interval_ms}ms (Threshold: ${data.timeout_threshold_ms}ms)`;
        } else {
          document.getElementById("val-timeouts").style.color = "var(--ok)";
          banner.className = "status-alert alert-ok";
          banner.innerText = "Stream Healthy: Constant commands, zero drops.";
        }
      }).catch(() => {});
    }, 500);
  </script>
</body>
</html>
"""

class TeleopHTTPHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
        elif self.path == "/diag":
            with lock:
                diag = {
                    "watchdog_trips": stat_watchdog_trips,
                    "last_interval_ms": int(round(stat_last_jitter_ms)),
                    "is_timed_out": stat_is_timed_out,
                    "timeout_threshold_ms": int(TIMEOUT_SEC * 1000),
                    "total_packets": stat_packet_count,
                    "deadband_steps": cfg_deadband_steps,
                    "command_timeout_ms": cfg_command_timeout_ms,
                    "maixsense_enable": cfg_maixsense_enable,
                    "motor_config_status": dict(motor_config_status),
                }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(diag).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global target_vx, target_vy, target_omega, last_command_time
        global stat_packet_count, stat_last_jitter_ms, stat_last_received_time
        if self.path == "/cmd":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
                now = time.time()
                with lock:
                    if stat_last_received_time > 0:
                        stat_last_jitter_ms = (now - stat_last_received_time) * 1000.0
                    stat_last_received_time = now
                    stat_packet_count += 1
                    target_vx = float(data.get("vx", 0.0))
                    target_vy = float(data.get("vy", 0.0))
                    target_omega = float(data.get("omega", 0.0))
                    last_command_time = now
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception:
                self.send_response(400)
                self.end_headers()
        elif self.path == "/config":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
                configure_motors_service(
                    deadband_steps=data.get("deadband_steps", data.get("deadband")),
                    command_timeout_ms=data.get("command_timeout_ms", data.get("timeout_ms")),
                    maixsense_enable=data.get("maixsense_enable", data.get("enable_maixsense")),
                    motor_enable=data.get("motor_enable", True),
                )
                resp = {
                    "status": "ok",
                    "deadband_steps": cfg_deadband_steps,
                    "command_timeout_ms": cfg_command_timeout_ms,
                    "maixsense_enable": cfg_maixsense_enable,
                    "motor_config_status": dict(motor_config_status),
                }
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(resp).encode("utf-8"))
            except Exception:
                self.send_response(400)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        return

def watchdog_and_publisher_loop():
    global target_vx, target_vy, target_omega, smooth_vx, smooth_vy, smooth_omega
    global stat_watchdog_trips, stat_is_timed_out

    rate_hz = 50.0
    dt = 1.0 / rate_hz

    accum_steps_1 = 0.0
    accum_steps_2 = 0.0
    accum_steps_3 = 0.0

    last_pub_steps_1 = None
    last_pub_steps_2 = None
    last_pub_steps_3 = None

    ALPHA = 0.35

    print("Control & Watchdog loop active (50 Hz).")

    while True:
        with lock:
            now = time.time()
            if now - last_command_time > TIMEOUT_SEC:
                if not stat_is_timed_out and (abs(smooth_vx) > 0.01 or abs(smooth_vy) > 0.01 or abs(smooth_omega) > 0.01):
                    stat_watchdog_trips += 1
                stat_is_timed_out = True
                desired_vx = 0.0
                desired_vy = 0.0
                desired_omega = 0.0
            else:
                stat_is_timed_out = False
                desired_vx = target_vx
                desired_vy = target_vy
                desired_omega = target_omega

        smooth_vx += ALPHA * (desired_vx - smooth_vx)
        smooth_vy += ALPHA * (desired_vy - smooth_vy)
        smooth_omega += ALPHA * (desired_omega - smooth_omega)

        if abs(smooth_vx) < 0.002: smooth_vx = 0.0
        if abs(smooth_vy) < 0.002: smooth_vy = 0.0
        if abs(smooth_omega) < 0.002: smooth_omega = 0.0

        is_moving = (abs(smooth_vx) > 0.0 or abs(smooth_vy) > 0.0 or abs(smooth_omega) > 0.0)

        if is_moving:
            w1, w2, w3 = omni_ik(smooth_vx, smooth_vy, smooth_omega)

            accum_steps_1 += w1 * STEPS_PER_RAD * dt
            accum_steps_2 += w2 * STEPS_PER_RAD * dt
            accum_steps_3 += w3 * STEPS_PER_RAD * dt

            target_1 = int(round(accum_steps_1))
            target_2 = int(round(accum_steps_2))
            target_3 = int(round(accum_steps_3))

            if ros_publishers is not None:
                p1, p2, p3, msg = ros_publishers
                if target_1 != last_pub_steps_1:
                    msg.data = target_1; p1.publish(msg); last_pub_steps_1 = target_1
                if target_2 != last_pub_steps_2:
                    msg.data = target_2; p2.publish(msg); last_pub_steps_2 = target_2
                if target_3 != last_pub_steps_3:
                    msg.data = target_3; p3.publish(msg); last_pub_steps_3 = target_3

        time.sleep(dt)

def main():
    global ros_node, ros_publishers, ros_param_clients, ros_config_pending

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Int32
        from rcl_interfaces.srv import SetParameters
        rclpy.init()
        node = Node("web_teleop_server")
        ros_node = node
        p1 = node.create_publisher(Int32, "/motor_42626A/cmd_pos", 10)
        p2 = node.create_publisher(Int32, "/motor_58626B/cmd_pos", 10)
        p3 = node.create_publisher(Int32, "/motor_6F6260/cmd_pos", 10)
        for ns in MOTOR_NAMESPACES:
            ros_param_clients[ns] = node.create_client(SetParameters, f"{ns}/set_parameters")
        msg = Int32()
        ros_publishers = (p1, p2, p3, msg)
        configure_motors_service()

        def _ros_spin_and_config_loop():
            last_retry = time.time()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time()
                if (ros_config_pending or not all(motor_config_status.values())) and (now - last_retry >= 2.0):
                    last_retry = now
                    configure_motors_service()

        spin_thread = threading.Thread(target=_ros_spin_and_config_loop, daemon=True)
        spin_thread.start()
        print("Connected to ROS 2 topics and /set_parameters services.")
    except Exception as e:
        print(f"Running without ROS 2: {e}")

    worker = threading.Thread(target=watchdog_and_publisher_loop, daemon=True)
    worker.start()

    port = 8080
    server = HTTPServer(("0.0.0.0", port), TeleopHTTPHandler)
    print(f"Web Teleop Server listening on http://0.0.0.0:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

if __name__ == "__main__":
    main()
