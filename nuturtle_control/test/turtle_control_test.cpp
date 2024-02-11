
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <chrono>

#include "catch_ros2/catch_ros2.hpp"
#include <catch2/catch_test_macros.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <leo_ros_utils/param_helper.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <memory>
using leo_ros_utils::GetParam;
using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("wheel cmd test") {
    // I would like to make it a not shared ptr. But no choise as some 
    // ros functions demand a shared pointer style usage.
    rclcpp::Node::SharedPtr probe_node =
        std::make_shared<rclcpp::Node>("wheel_cmd_test_probe");

    auto recv_timeout = std::chrono::milliseconds{ int (
            GetParam<double>(*probe_node, "recv_timeout",
                             "timeout for receiving any messages", 3.0)* 1000)};

    double wheel_radius =
        GetParam<double>(*probe_node, "wheel_radius", "Radius of wheel");
    double track_width = GetParam<double>(*probe_node, "track_width",
                                          "track width between wheel");
    double motor_cmd_max =
        GetParam<int>(*probe_node, "motor_cmd_max", "max motor cmd value ");
    double motor_cmd_per_rad_sec = GetParam<double>(
        *probe_node, "motor_cmd_per_rad_sec", "motor command to rad/sec ratio");
    double encoder_ticks_per_rad =
        GetParam<double>(*probe_node, "encoder_ticks_per_rad",
                         "encoder ticks of wheel per radius");

    CAPTURE(recv_timeout);



    SECTION("pure translation") {


      geometry_msgs::msg::Twist cmd;
      // One meter per second cmd
      cmd.linear.x = 0.1;

      // r * rad  = distance.
      auto wheel_rad = (cmd.linear.x / wheel_radius);
      double expected_cmd = wheel_rad / motor_cmd_per_rad_sec;

      auto cmd_pub =
          probe_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      cmd_pub->publish(cmd);

      // rclcpp::wait_for_message(wheel_cmd,wheel_sub,);
      nuturtlebot_msgs::msg::WheelCommands wheel_cmd ;
      
      bool received = rclcpp::wait_for_message(
          wheel_cmd, probe_node, "wheel_cmd", std::chrono::seconds{2});
    REQUIRE(received);

    // The output is int, our expected value is double. To account for round up
    // or round down, giving it a tolerance of 1.
    REQUIRE_THAT(wheel_cmd.left_velocity, WithinAbs(expected_cmd, 1.0));
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