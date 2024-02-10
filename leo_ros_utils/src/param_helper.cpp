#include "leo_ros_utils/param_helper.hpp"
#include <rclcpp/parameter_value.hpp>
#include <turtlelib/to_string.hpp>

namespace leo_ros_utils {

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
    // Note: Because we are using non templated one, It's now all live dynamic
    // casting inside this. So have to use PARAMETER_STRING. Or the next line
    // will throw.
    ros_node.declare_parameter(name, rclcpp::PARAMETER_STRING, desc);
    try {

      std::string val = ros_node.get_parameter(name).get_value<std::string>();
      return val;
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
      RCLCPP_ERROR_STREAM(ros_node.get_logger(), "Missing parameter " << name);

      throw std::invalid_argument(turtlelib::ToString() << "Bad parameters" << name);
    }
  }
}



} // namespace leo_ros_helper