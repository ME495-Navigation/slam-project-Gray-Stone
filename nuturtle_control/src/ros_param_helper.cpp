#include "nuturtle_control/ros_param_helper.hpp"

namespace leo_ros_helper {

std::string
GetParamStr(rclcpp::Node &ros_node, std::string name, std::string description,
            std::optional<std::string> default_value) {
  auto desc = rcl_interfaces::msg::ParameterDescriptor();
  desc.name = name;
  desc.description = description;
  if (default_value.has_value()) {
    ros_node.declare_parameter<std::string>(name, default_value.value(), desc);
    return ros_node.get_parameter(name).get_value<std::string>();
  } else {
    ros_node.declare_parameter(name, rclcpp::PARAMETER_NOT_SET, desc);
    try {

      std::string val = ros_node.get_parameter(name).get_value<std::string>();
      return val;
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
      RCLCPP_ERROR_STREAM(ros_node.get_logger(), "Missing parameter " << name);
      throw std::invalid_argument("Bad parameters");
    }
  }
}
} // namespace leo_ros_helper