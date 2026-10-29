#include "qplatform_hardware/qplatform_stepper_system.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace qplatform_hardware
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kDegToRad = M_PI / 180.0;

bool parse_bool(const std::string & val, bool default_val = false)
{
  if (val.empty()) {
    return default_val;
  }
  return val == "true" || val == "True" || val == "TRUE" || val == "1";
}

double parse_double(const std::string & val, double default_val)
{
  if (val.empty()) {
    return default_val;
  }
  try {
    return std::stod(val);
  } catch (...) {
    return default_val;
  }
}

int64_t parse_int64(const std::string & val, int64_t default_val)
{
  if (val.empty()) {
    return default_val;
  }
  try {
    return std::stoll(val);
  } catch (...) {
    return default_val;
  }
}

rcl_interfaces::msg::Parameter make_int_param(const std::string & name, int64_t value)
{
  rcl_interfaces::msg::Parameter p;
  p.name = name;
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p.value.integer_value = value;
  return p;
}

rcl_interfaces::msg::Parameter make_bool_param(const std::string & name, bool value)
{
  rcl_interfaces::msg::Parameter p;
  p.name = name;
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  p.value.bool_value = value;
  return p;
}
}  // namespace

CallbackReturn QPlatformStepperSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // System-level optional defaults
  double default_steps_per_rev = 51200.0;
  double default_gear_ratio = 1.0;
  bool default_use_encoder_angle = false;
  bool default_open_loop_fallback = true;
  int64_t default_deadband_steps = 4;
  int64_t default_command_timeout_ms = 500;
  bool default_enable_maixsense = true;

  auto sys_it = info_.hardware_parameters.find("steps_per_rev");
  if (sys_it != info_.hardware_parameters.end()) {
    default_steps_per_rev = parse_double(sys_it->second, default_steps_per_rev);
  }
  sys_it = info_.hardware_parameters.find("gear_ratio");
  if (sys_it != info_.hardware_parameters.end()) {
    default_gear_ratio = parse_double(sys_it->second, default_gear_ratio);
  }
  sys_it = info_.hardware_parameters.find("use_encoder_angle");
  if (sys_it != info_.hardware_parameters.end()) {
    default_use_encoder_angle = parse_bool(sys_it->second, default_use_encoder_angle);
  }
  sys_it = info_.hardware_parameters.find("open_loop_fallback");
  if (sys_it != info_.hardware_parameters.end()) {
    default_open_loop_fallback = parse_bool(sys_it->second, default_open_loop_fallback);
  }
  sys_it = info_.hardware_parameters.find("always_publish_cmd");
  if (sys_it != info_.hardware_parameters.end()) {
    always_publish_cmd_ = parse_bool(sys_it->second, false);
  }
  sys_it = info_.hardware_parameters.find("deadband_steps");
  if (sys_it == info_.hardware_parameters.end()) {
    sys_it = info_.hardware_parameters.find("deadband");
  }
  if (sys_it != info_.hardware_parameters.end()) {
    default_deadband_steps = parse_int64(sys_it->second, default_deadband_steps);
  }
  sys_it = info_.hardware_parameters.find("command_timeout_ms");
  if (sys_it == info_.hardware_parameters.end()) {
    sys_it = info_.hardware_parameters.find("timeout_ms");
  }
  if (sys_it != info_.hardware_parameters.end()) {
    default_command_timeout_ms = parse_int64(sys_it->second, default_command_timeout_ms);
  }
  sys_it = info_.hardware_parameters.find("enable_maixsense");
  if (sys_it == info_.hardware_parameters.end()) {
    sys_it = info_.hardware_parameters.find("maixsense_enable");
  }
  if (sys_it != info_.hardware_parameters.end()) {
    default_enable_maixsense = parse_bool(sys_it->second, default_enable_maixsense);
  }

  channels_.clear();
  channels_.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    auto ch = std::make_unique<StepperJointChannel>();
    ch->joint_name = joint.name;

    // Default topic prefix: /motor_1, /motor_2, /motor_3 (or overridden in URDF)
    ch->topic_prefix = "/motor_" + std::to_string(i + 1);
    auto it = joint.parameters.find("topic_prefix");
    if (it != joint.parameters.end() && !it->second.empty()) {
      ch->topic_prefix = it->second;
      if (ch->topic_prefix.back() == '/') {
        ch->topic_prefix.pop_back();
      }
    }

    ch->steps_per_rev = default_steps_per_rev;
    it = joint.parameters.find("steps_per_rev");
    if (it != joint.parameters.end()) {
      ch->steps_per_rev = parse_double(it->second, default_steps_per_rev);
    }

    ch->gear_ratio = default_gear_ratio;
    it = joint.parameters.find("gear_ratio");
    if (it != joint.parameters.end()) {
      ch->gear_ratio = parse_double(it->second, default_gear_ratio);
    }

    bool invert = false;
    it = joint.parameters.find("invert_direction");
    if (it != joint.parameters.end()) {
      invert = parse_bool(it->second, false);
    }
    ch->direction_sign = invert ? -1.0 : 1.0;

    ch->use_encoder_angle = default_use_encoder_angle;
    it = joint.parameters.find("use_encoder_angle");
    if (it != joint.parameters.end()) {
      ch->use_encoder_angle = parse_bool(it->second, default_use_encoder_angle);
    }

    ch->open_loop_fallback = default_open_loop_fallback;
    it = joint.parameters.find("open_loop_fallback");
    if (it != joint.parameters.end()) {
      ch->open_loop_fallback = parse_bool(it->second, default_open_loop_fallback);
    }

    ch->deadband_steps = default_deadband_steps;
    it = joint.parameters.find("deadband_steps");
    if (it == joint.parameters.end()) {
      it = joint.parameters.find("deadband");
    }
    if (it != joint.parameters.end()) {
      ch->deadband_steps = parse_int64(it->second, default_deadband_steps);
    }

    ch->command_timeout_ms = default_command_timeout_ms;
    it = joint.parameters.find("command_timeout_ms");
    if (it == joint.parameters.end()) {
      it = joint.parameters.find("timeout_ms");
    }
    if (it != joint.parameters.end()) {
      ch->command_timeout_ms = parse_int64(it->second, default_command_timeout_ms);
    }

    ch->enable_maixsense = default_enable_maixsense;
    it = joint.parameters.find("enable_maixsense");
    if (it == joint.parameters.end()) {
      it = joint.parameters.find("maixsense_enable");
    }
    if (it != joint.parameters.end()) {
      ch->enable_maixsense = parse_bool(it->second, default_enable_maixsense);
    }

    ch->steps_per_wheel_rad = (ch->steps_per_rev * ch->gear_ratio) / kTwoPi;
    if (std::abs(ch->steps_per_wheel_rad) < 1e-6) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid steps_per_wheel_rad for joint '%s'", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    ch->rad_per_step = 1.0 / ch->steps_per_wheel_rad;

    RCLCPP_INFO(
      get_logger(),
      "Configured joint '%s' -> prefix='%s' (cmd_pos='%s/cmd_pos', pos='%s/position', angle='%s/angle', "
      "steps_per_rev=%.1f, gear_ratio=%.4f, steps_per_wheel_rad=%.2f, invert=%s)",
      ch->joint_name.c_str(),
      ch->topic_prefix.c_str(),
      ch->topic_prefix.c_str(),
      ch->topic_prefix.c_str(),
      ch->topic_prefix.c_str(),
      ch->steps_per_rev,
      ch->gear_ratio,
      ch->steps_per_wheel_rad,
      invert ? "true" : "false");

    channels_.push_back(std::move(ch));
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn QPlatformStepperSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(get_logger(), "HardwareComponentInterface default node is null!");
    return CallbackReturn::ERROR;
  }

  // Use SensorDataQoS / KeepLast(1) BestEffort or Reliable compatible with micro-ROS default (Reliable, KeepLast 10)
  const auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  for (auto & ch : channels_) {
    StepperJointChannel * raw_ch = ch.get();

    const std::string cmd_topic = ch->topic_prefix + "/cmd_pos";
    const std::string pos_topic = ch->topic_prefix + "/position";
    const std::string angle_topic = ch->topic_prefix + "/angle";
    const std::string set_params_srv = ch->topic_prefix + "/set_parameters";

    ch->cmd_pos_pub = node->create_publisher<std_msgs::msg::Int32>(cmd_topic, pub_qos);
    ch->set_params_client = node->create_client<rcl_interfaces::srv::SetParameters>(set_params_srv);

    ch->pos_sub = node->create_subscription<std_msgs::msg::Int32>(
      pos_topic,
      sub_qos,
      [raw_ch](const std_msgs::msg::Int32::SharedPtr msg) {
        raw_ch->latest_steps.store(msg->data, std::memory_order_relaxed);
        raw_ch->has_steps_msg.store(true, std::memory_order_release);
      });

    ch->angle_sub = node->create_subscription<std_msgs::msg::Float32>(
      angle_topic,
      sub_qos,
      [raw_ch](const std_msgs::msg::Float32::SharedPtr msg) {
        raw_ch->latest_angle_deg.store(msg->data, std::memory_order_relaxed);
        raw_ch->has_angle_msg.store(true, std::memory_order_release);
      });

    // Initialize states and commands to 0.0
    set_state(ch->joint_name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
    set_state(ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    set_command(ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }

  RCLCPP_INFO(
    get_logger(),
    "QPlatformStepperSystem configured with %zu micro-ROS stepper channels.",
    channels_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn QPlatformStepperSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & ch : channels_) {
    set_command(ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);

    if (ch->has_steps_msg.load(std::memory_order_acquire)) {
      const int32_t current_steps = ch->latest_steps.load(std::memory_order_relaxed);
      ch->target_steps_accum = static_cast<double>(current_steps);
      ch->last_published_steps = current_steps;
      ch->target_initialized = true;
      ch->current_wheel_pos_rad =
        ch->direction_sign * static_cast<double>(current_steps) * ch->rad_per_step;
      ch->prev_wheel_pos_rad = ch->current_wheel_pos_rad;
      set_state(
        ch->joint_name + "/" + hardware_interface::HW_IF_POSITION,
        ch->current_wheel_pos_rad);
    } else {
      ch->target_initialized = false;
      ch->target_steps_accum = 0.0;
      ch->last_published_steps = 0;
    }
    set_state(ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);

    ch->motor_configured.store(false, std::memory_order_relaxed);
    ch->config_in_flight.store(false, std::memory_order_relaxed);
    ch->time_since_config_attempt = 10.0;
    send_motor_configuration(ch.get());
  }

  RCLCPP_INFO(get_logger(), "QPlatformStepperSystem activated.");
  return CallbackReturn::SUCCESS;
}

void QPlatformStepperSystem::send_motor_configuration(StepperJointChannel * ch)
{
  if (!ch || !ch->set_params_client) {
    return;
  }
  if (ch->config_in_flight.load(std::memory_order_acquire)) {
    return;
  }
  if (!ch->set_params_client->service_is_ready()) {
    return;
  }

  ch->config_in_flight.store(true, std::memory_order_release);
  ch->time_since_config_attempt = 0.0;

  auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  req->parameters.push_back(make_bool_param("motor_enable", true));
  req->parameters.push_back(make_int_param("deadband", ch->deadband_steps));
  req->parameters.push_back(make_int_param("command_timeout_ms", ch->command_timeout_ms));
  req->parameters.push_back(make_bool_param("maixsense_enable", ch->enable_maixsense));

  auto logger = get_logger();
  ch->set_params_client->async_send_request(
    req,
    [ch, logger](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
      ch->config_in_flight.store(false, std::memory_order_release);
      try {
        const auto res = future.get();
        bool all_ok = !res->results.empty();
        for (const auto & r : res->results) {
          if (!r.successful) {
            all_ok = false;
          }
        }
        if (all_ok) {
          ch->motor_configured.store(true, std::memory_order_release);
          RCLCPP_INFO(
            logger,
            "Configured %s/set_parameters: deadband=%ld, command_timeout_ms=%ld, maixsense_enable=%s",
            ch->topic_prefix.c_str(),
            static_cast<long>(ch->deadband_steps),
            static_cast<long>(ch->command_timeout_ms),
            ch->enable_maixsense ? "true" : "false");
        } else {
          RCLCPP_WARN(
            logger,
            "Partial failure configuring %s/set_parameters; will retry.",
            ch->topic_prefix.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          logger,
          "Exception calling %s/set_parameters: %s",
          ch->topic_prefix.c_str(), e.what());
      }
    });
}

CallbackReturn QPlatformStepperSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & ch : channels_) {
    set_command(ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }
  RCLCPP_INFO(get_logger(), "QPlatformStepperSystem deactivated.");
  return CallbackReturn::SUCCESS;
}

return_type QPlatformStepperSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (auto & ch : channels_) {
    if (!ch->motor_configured.load(std::memory_order_acquire)) {
      ch->time_since_config_attempt += dt;
      if (ch->time_since_config_attempt >= 2.0) {
        ch->time_since_config_attempt = 0.0;
        send_motor_configuration(ch.get());
      }
    }

    const bool has_steps = ch->has_steps_msg.load(std::memory_order_acquire);
    const bool has_angle = ch->has_angle_msg.load(std::memory_order_acquire);

    // Sync initial target accumulator on first received /position message so motor never jumps
    if (has_steps && !ch->target_initialized) {
      const int32_t init_steps = ch->latest_steps.load(std::memory_order_relaxed);
      ch->target_steps_accum = static_cast<double>(init_steps);
      ch->last_published_steps = init_steps;
      ch->target_initialized = true;
      ch->prev_wheel_pos_rad =
        ch->direction_sign * static_cast<double>(init_steps) * ch->rad_per_step;
    }

    if (ch->use_encoder_angle && has_angle) {
      const double angle_deg =
        static_cast<double>(ch->latest_angle_deg.load(std::memory_order_relaxed));
      if (!ch->angle_initialized) {
        ch->prev_angle_deg = angle_deg;
        ch->unwrapped_motor_rad = angle_deg * kDegToRad;
        ch->angle_initialized = true;
      } else {
        double delta_deg = angle_deg - ch->prev_angle_deg;
        if (delta_deg > 180.0) {
          delta_deg -= 360.0;
        } else if (delta_deg < -180.0) {
          delta_deg += 360.0;
        }
        ch->prev_angle_deg = angle_deg;
        ch->unwrapped_motor_rad += delta_deg * kDegToRad;
      }
      ch->current_wheel_pos_rad =
        ch->direction_sign * (ch->unwrapped_motor_rad / ch->gear_ratio);
    } else if (has_steps) {
      const int32_t steps = ch->latest_steps.load(std::memory_order_relaxed);
      ch->current_wheel_pos_rad =
        ch->direction_sign * static_cast<double>(steps) * ch->rad_per_step;
    } else if (ch->open_loop_fallback) {
      ch->current_wheel_pos_rad =
        ch->direction_sign * ch->target_steps_accum * ch->rad_per_step;
    }

    if (dt > 1e-6) {
      ch->current_wheel_vel_rad_s =
        (ch->current_wheel_pos_rad - ch->prev_wheel_pos_rad) / dt;
    }
    ch->prev_wheel_pos_rad = ch->current_wheel_pos_rad;

    set_state(
      ch->joint_name + "/" + hardware_interface::HW_IF_POSITION,
      ch->current_wheel_pos_rad);
    set_state(
      ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
      ch->current_wheel_vel_rad_s);
  }

  return return_type::OK;
}

return_type QPlatformStepperSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0 || dt > 1.0) {
    return return_type::OK;
  }

  for (auto & ch : channels_) {
    double vel_cmd_rad_s =
      get_command(ch->joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    if (!std::isfinite(vel_cmd_rad_s)) {
      vel_cmd_rad_s = 0.0;
    }

    const double delta_steps =
      ch->direction_sign * vel_cmd_rad_s * ch->steps_per_wheel_rad * dt;
    ch->target_steps_accum += delta_steps;

    const int32_t target_steps_i32 =
      static_cast<int32_t>(std::llround(ch->target_steps_accum));

    if (
      ch->cmd_pos_pub &&
      (always_publish_cmd_ || target_steps_i32 != ch->last_published_steps ||
      std::abs(vel_cmd_rad_s) > 1e-5))
    {
      std_msgs::msg::Int32 msg;
      msg.data = target_steps_i32;
      ch->cmd_pos_pub->publish(msg);
      ch->last_published_steps = target_steps_i32;
    }
  }

  return return_type::OK;
}

}  // namespace qplatform_hardware

PLUGINLIB_EXPORT_CLASS(
  qplatform_hardware::QPlatformStepperSystem,
  hardware_interface::SystemInterface)
