#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_value.hpp>
#include <string>
#include <stdexcept>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace {

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
} // namespace

class TurtleControl : public rclcpp::Node {

public:
  TurtleControl()
      : Node("turtle_control"), wheel_radius(GetParam<double>(
                                    *this, "wheel_radius", "Radius of wheel")),
        track_width(GetParam<double>(*this, "track_width",
                                     "track width between wheel")),
        motor_cmd_max(
            GetParam<int>(*this, "motor_cmd_max", "max motor cmd value ")),
        motor_cmd_per_rad_sec(GetParam<double>(
            *this, "motor_cmd_per_rad_sec", "motor command to rad/sec ratio")),
        encoder_ticks_per_rad(
            GetParam<double>(*this, "encoder_ticks_per_rad",
                             "encoder ticks of wheel per radius")),
        collision_radius(GetParam<double>(*this, "collision_radius",
                                          "collision radius of robot")) {}


private:
  // rclcpp::Node::SharedPtr ros_node;
  double wheel_radius;
  double track_width;
  double motor_cmd_max;
  double motor_cmd_per_rad_sec;
  int encoder_ticks_per_rad;
  double collision_radius;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  //   rclcpp::Node::SharedPtr node_ptr =
  //   std::make_shared<rclcpp::Node>("turtle_control") ; TurtleControl
  //   t_ctrl{node_ptr};
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}