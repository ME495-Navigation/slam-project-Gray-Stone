
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <chrono>

#include "catch_ros2/catch_ros2.hpp"
#include <catch2/catch_test_macros.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <leo_ros_utils/param_helper.hpp>
#include <memory>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using leo_ros_utils::GetParam;

bool SpinSomeUntil(rclcpp::Node::SharedPtr node,
                   std::chrono::nanoseconds duration,
                   std::function<bool(void)> done_func) {
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (rclcpp::ok() &&
         ((rclcpp::Clock().now() - start_time) < rclcpp::Duration(duration))) {
    // Repeatedly check for the dummy service until its found
    if (done_func()) {
      return true;
    }

    rclcpp::spin_some(node);
  }
  return false;
}

TEST_CASE("wheel cmd test") {
  // I would like to make it a not shared ptr. But no choise as some
  // ros functions demand a shared pointer style usage.
  rclcpp::Node::SharedPtr probe_node =
      std::make_shared<rclcpp::Node>("wheel_cmd_test_probe");
  auto recv_timeout = std::chrono::milliseconds{
      int(GetParam<double>(*probe_node, "recv_timeout",
                           "timeout for receiving any messages", 10.0) *
          1000)};

  double wheel_radius =
      GetParam<double>(*probe_node, "wheel_radius", "Radius of wheel");
  double track_width =
      GetParam<double>(*probe_node, "track_width", "track width between wheel");
  double motor_cmd_max =
      GetParam<int>(*probe_node, "motor_cmd_max", "max motor cmd value ");
  double motor_cmd_per_rad_sec = GetParam<double>(
      *probe_node, "motor_cmd_per_rad_sec", "motor command to rad/sec ratio");
  double encoder_ticks_per_rad =
      GetParam<double>(*probe_node, "encoder_ticks_per_rad",
                       "encoder ticks of wheel per radius");

  CAPTURE(recv_timeout);
  geometry_msgs::msg::Twist cmd;
  auto cmd_pub =
      probe_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  SECTION("pure translation") {
    // One meter per second cmd
    cmd.linear.x = 0.1;

    // r * rad  = distance.
    auto wheel_rad = (cmd.linear.x / wheel_radius);
    double expected_cmd = wheel_rad / motor_cmd_per_rad_sec;

    // rclcpp::wait_for_message(wheel_cmd,wheel_sub,);
    std::optional<nuturtlebot_msgs::msg::WheelCommands> wheel_cmd_opt;
    auto wheel_sub =
        probe_node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
            "wheel_cmd", 10,
            [&wheel_cmd_opt](nuturtlebot_msgs::msg::WheelCommands msg) {
              wheel_cmd_opt = msg;
            });

    cmd_pub->publish(cmd);
    INFO("Waiting for the wheel command message");
    REQUIRE(SpinSomeUntil(probe_node, recv_timeout,
                          [&]() { return wheel_cmd_opt.has_value(); }));

    INFO("Waiting for the wheel command message");

    // The output is int, our expected value is double. To account for round
    // up or round down, giving it a tolerance of 1.
    REQUIRE_THAT(wheel_cmd_opt->left_velocity, WithinAbs(expected_cmd, 1.0));
    REQUIRE_THAT(wheel_cmd_opt->right_velocity, WithinAbs(expected_cmd, 1.0));
  }

  SECTION("pure rotation") {
    cmd.angular.z = 0.1;

    double track_length = cmd.angular.z * track_width;
    double wheel_rad = track_length / wheel_radius;
    double expected_cmd = wheel_rad / motor_cmd_per_rad_sec;

    std::optional<nuturtlebot_msgs::msg::WheelCommands> wheel_cmd_opt;
    auto wheel_sub =
        probe_node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
            "wheel_cmd", 10,
            [&wheel_cmd_opt](nuturtlebot_msgs::msg::WheelCommands msg) {
              wheel_cmd_opt = msg;
            });

    cmd_pub->publish(cmd);

    INFO("Waiting for the wheel command message");
    REQUIRE(SpinSomeUntil(probe_node, recv_timeout,
                          [&]() { return wheel_cmd_opt.has_value(); }));

    REQUIRE_THAT(wheel_cmd_opt->left_velocity, WithinAbs(-expected_cmd, 1.0));
    REQUIRE_THAT(wheel_cmd_opt->right_velocity, WithinAbs(+expected_cmd, 1.0));
  }
}

// // #include <catch2/matchers/catch_matchers.hpp>
// // #include <catch2/matchers/catch_matchers_floating_point.hpp>

// #include "catch_ros2/catch_ros2.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include <catch2/catch_test_macros.hpp>

// TEST_CASE("example_integration_test", "[integration]") {
//   // Create a simple client node to check if the auxiliary node
//   // has a service available
//   auto node = rclcpp::Node::make_shared("integration_test_node");
//   auto client = node->create_client<std_srvs::srv::Empty>("test_service");

//   bool found = client->wait_for_service();
//   CHECK(found);

// }