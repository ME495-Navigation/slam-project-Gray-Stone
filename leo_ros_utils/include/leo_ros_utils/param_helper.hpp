#ifndef LEO_ROS_UTILS_PARAM_HELPER_INCLUDE_GUARD_HPP
#define LEO_ROS_UTILS_PARAM_HELPER_INCLUDE_GUARD_HPP

#include <optional>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace leo_ros_helper {

template <typename T>
T GetParam(rclcpp::Node &ros_node, std::string name, std::string description,
           std::optional<T> default_value = std::nullopt) {
  auto desc = rcl_interfaces::msg::ParameterDescriptor();
  desc.name = name;
  desc.description = description;
  if (default_value.has_value()) {
    ros_node.declare_parameter<T>(name, default_value.value(), desc);
    return ros_node.get_parameter(name).get_value<T>();
  } else {
    ros_node.declare_parameter<T>(name, rclcpp::PARAMETER_NOT_SET, desc);
    T val = ros_node.get_parameter(name).get_value<T>();
    if (val == rclcpp::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(ros_node.get_logger(), "Missing parameter " << name);
      throw std::invalid_argument("Bad parameters");
    }
    return val;
  }
}

// The Rclcpp parameter is initially DESIGNED TO NOT SUPPORT NON_OPTIONAL
// ARGUMENT!

// The non templated declare_parameter will be "week typed" and throw when
// calling get_value if param not set.
// The templated declare_parameter will never throw, It will return the dummy
// object.

// https://github.com/ros2/rclcpp/issues/1691#issuecomment-859624222

// So we need a special version that catches on get_value.

// Because we can't have a declare_parameter<std::string> (
// rclcpp::PARAMETER_STRING) need to make a special version for string.

std::string
GetParamStr(rclcpp::Node &ros_node, std::string name, std::string description,
            std::optional<std::string> default_value = std::nullopt);
} // namespace leo_ros_helper

#endif