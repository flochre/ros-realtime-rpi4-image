#ifndef QPLATFORM_HARDWARE__QPLATFORM_STEPPER_SYSTEM_HPP_
#define QPLATFORM_HARDWARE__QPLATFORM_STEPPER_SYSTEM_HPP_

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

namespace qplatform_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;

struct StepperJointChannel
{
  std::string joint_name;
  std::string topic_prefix;

  // Kinematics & scaling
  double steps_per_rev{51200.0};   // 200 full steps * 256 microsteps (TMC2160 mres=0)
  double gear_ratio{1.0};          // Motor revolutions per 1 wheel revolution
  double direction_sign{1.0};      // +1.0 or -1.0 (invert_direction)
  double steps_per_wheel_rad{8148.7330863};
  double rad_per_step{1.0 / 8148.7330863};

  // Feedback configuration
  bool use_encoder_angle{false};   // false: use /position (Int32 steps), true: use unwrapped /angle (Float32 deg)
  bool open_loop_fallback{true};   // If no micro-ROS telemetry received yet, integrate command into state

  // STM32 micro-ROS service configuration parameters (<topic_prefix>/set_parameters)
  int64_t deadband_steps{4};
  int64_t command_timeout_ms{500};
  bool enable_maixsense{true};
  std::atomic<bool> motor_configured{false};
  std::atomic<bool> config_in_flight{false};
  double time_since_config_attempt{10.0};

  // Atomic telemetry updated by ROS 2 subscription callbacks
  std::atomic<int32_t> latest_steps{0};
  std::atomic<bool> has_steps_msg{false};

  std::atomic<float> latest_angle_deg{0.0f};
  std::atomic<bool> has_angle_msg{false};

  // Unwrapping state for 0..360 deg MT6816 magnetic encoder
  bool angle_initialized{false};
  double prev_angle_deg{0.0};
  double unwrapped_motor_rad{0.0};

  // Integrated step command & state tracking
  bool target_initialized{false};
  double target_steps_accum{0.0};
  int32_t last_published_steps{0};
  double prev_wheel_pos_rad{0.0};
  double current_wheel_pos_rad{0.0};
  double current_wheel_vel_rad_s{0.0};

  // ROS 2 publishers, subscriptions & service clients
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pos_pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pos_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client;
};

class QPlatformStepperSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(QPlatformStepperSystem)

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void send_motor_configuration(StepperJointChannel * ch);

  std::vector<std::unique_ptr<StepperJointChannel>> channels_;
  bool always_publish_cmd_{false};
};

}  // namespace qplatform_hardware

#endif  // QPLATFORM_HARDWARE__QPLATFORM_STEPPER_SYSTEM_HPP_
