#ifndef LEO_ROS_UTILS_PARAM_HELPER_INCLUDE_GUARD_HPP
#define LEO_ROS_UTILS_PARAM_HELPER_INCLUDE_GUARD_HPP

#include <optional>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <turtlelib/to_string.hpp>
namespace leo_ros_utils {
    // These utilities make a lot of sense
// The Rclcpp parameter is initially DESIGNED TO NOT SUPPORT NON_OPTIONAL
// ARGUMENT!

// The non templated declare_parameter will be "week typed" and throw when
// calling get_value if param not set.
// The templated declare_parameter will never throw, It will return the dummy
// object.

// https://github.com/ros2/rclcpp/issues/1691#issuecomment-859624222

// So we need a special version that catches on get_value.

// Because we can't have a declare_parameter<std::string> (
// rclcpp::PARAMETER_STRING) and also for array types. So the whole thing is
// quite complicated.

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
    rclcpp::ParameterType dummy_default_value;
    // I can't use decltype(rclcpp::PARAMETER_DOUBLE_ARRAY) here.
    if constexpr (std::is_same_v<T, std::vector<double>>) {
      dummy_default_value = rclcpp::PARAMETER_DOUBLE_ARRAY;
    } else if constexpr (std::is_same_v<T, std::string>) {
      dummy_default_value = rclcpp::PARAMETER_STRING;
    } else {
      // The reason for keeping both stype of checking is because I can;t do
      // declare_parameter(name, rclcpp::PARAMETER_NOT_SET, desc).
      // This will compile fine, but directly throw at run time.
      // But the array, string stuff can't be handled using the templated
      // declar_parameter. Thus they need to go the try catch route.
      ros_node.declare_parameter<T>(name, rclcpp::PARAMETER_NOT_SET, desc);
      T val = ros_node.get_parameter(name).get_value<T>();
      if (val == rclcpp::PARAMETER_NOT_SET) {
        RCLCPP_ERROR_STREAM(ros_node.get_logger(),
                            "Missing parameter " << name);
        throw std::invalid_argument(turtlelib::ToString()
                                    << "Bad parameters" << name);
      }
      return val;
    }

    // This is the non templated but throw version.
    ros_node.declare_parameter(name, dummy_default_value, desc);
    try {
      T val = ros_node.get_parameter(name).get_value<T>();
      return val;
    } catch (rclcpp::exceptions::ParameterUninitializedException &) {
      RCLCPP_ERROR_STREAM(ros_node.get_logger(), "Missing parameter " << name);
      throw std::invalid_argument(turtlelib::ToString()
                                  << "Bad parameters" << name);
    }
  }
}
} // namespace leo_ros_utils

#endif
