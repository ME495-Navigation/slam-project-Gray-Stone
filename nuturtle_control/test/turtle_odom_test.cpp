
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__traits.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <optional>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <chrono>
#include <rclcpp/logging.hpp>

#include "catch_ros2/catch_ros2.hpp"
#include "nuturtle_control/srv/detail/init_pose__struct.hpp"
#include <catch2/catch_test_macros.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <leo_ros_utils/param_helper.hpp>
#include <memory>
#include <nuturtle_control/srv/init_pose.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>

using Catch::Matchers::VectorContains;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using leo_ros_utils::GetParam;

TEST_CASE("turtle_odom") {

  rclcpp::Node::SharedPtr probe_node =
      std::make_shared<rclcpp::Node>("wheel_cmd_test_probe");

  auto odom_id = GetParam<std::string>(*probe_node, "odom_id",
                                       "name of the odometry frame", "odom");
  auto body_id =
      GetParam<std::string>(*probe_node, "body_id", "name of the body frame");
  auto init_pose_client =
      probe_node->create_client<nuturtle_control::srv::InitPose>("initial_pose",
                                                                 10);

  RCLCPP_ERROR(probe_node->get_logger(),"Waiting for client");
  while (!init_pose_client->wait_for_service(std::chrono::seconds{1})) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(probe_node->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(probe_node->get_logger(), "service not available, waiting again...");
  }

  RCLCPP_ERROR(probe_node->get_logger(),"Done Waiting for client");

  // The test always run ahead to look up when js is not yet published.
  // This hack doesn't work
  rclcpp::sleep_for(std::chrono::seconds{20});

  // Test initial_pose service

  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(probe_node->get_clock());
  auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto init_req = std::make_shared<nuturtle_control::srv::InitPose_Request>();
  init_req->x0 = 0.7;
  init_req->y0 = 0.2;
  init_req->theta0 = 2.2;

  auto future = init_pose_client->async_send_request(init_req);
  REQUIRE(rclcpp::spin_until_future_complete(probe_node, future) ==
          rclcpp::FutureReturnCode::SUCCESS);
  // no resp message.
  // try {
  geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(odom_id, body_id, tf2::TimePointZero);
  // } catch (const tf2::TransformException & ex) {
  //   RCLCPP_INFO(
  //     this->get_logger(), "Could not transform %s to %s: %s",
  //     toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
  //   return;
  // }

  std::cout<<geometry_msgs::msg::to_yaml(t);

  REQUIRE_THAT(t.transform.translation.x, WithinRel(0.7));

}

// Write one test case that uses a tf2_listener to verify that a transform from
// odom to a base_footprint is being published

//     The transform should be the identity transformation, the joint states
//     will not change throughout the test

// Write a test launch file called test/turtle_odom_test.launch.xml that runs a
// joint_state_publisher as well as the turtle_odom_test_node

// Because the odometry node was implemented in terms of DiffDrive we will not
// test the the actual odometry calculations here
