cmake_minimum_required(VERSION 3.16)
project(qplatform_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(qplatform_hardware SHARED
  src/qplatform_stepper_system.cpp
)
target_compile_features(qplatform_hardware PUBLIC cxx_std_17)
target_include_directories(qplatform_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/qplatform_hardware>
)
ament_target_dependencies(qplatform_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface qplatform_hardware.xml)

install(
  TARGETS qplatform_hardware
  EXPORT export_qplatform_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/qplatform_hardware
)

ament_export_targets(export_qplatform_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  std_msgs
)

ament_package()
