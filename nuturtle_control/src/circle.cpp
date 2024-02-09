

#include <leo_ros_utils/param_helper.hpp>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nuturtle_control/srv/control.hpp>
#include <nuturtle_control/srv/init_pose.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/se2d.hpp>
// using namespace std::chrono_literals;

using leo_ros_utils::GetParam;

class Circle : public rclcpp::Node {

public:
  Circle() : Node("nusim") {

    int rate = GetParam<int>(*this, "frequency",
                             "The frequency for publishing cmd_vel", 100);

    // Setup pub/sub and srv/client
    cmd_vel_publisher =
        create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    control_service_ = create_service<nuturtle_control::srv::Control>(
        "control", std::bind(&Circle::control_srv, this, std::placeholders::_1,
                             std::placeholders::_2));

    reverse_service_ = create_service<std_srvs::srv::Empty>(
        "reverse", std::bind(&Circle::reverse_srv, this, std::placeholders::_1,
                             std::placeholders::_2));

    stop_service_ = create_service<std_srvs::srv::Empty>(
        "stop", std::bind(&Circle::stop_srv, this, std::placeholders::_1,
                          std::placeholders::_2));

    // Setup timer and set things in motion.
    main_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000 / rate),
                                std::bind(&Circle::main_timer_callback, this));
  }
  enum ControlState { kRunning,kStopping ,kStopped };

private:
  // Private functions

  //! @brief Main timer callback function.
  void main_timer_callback() {
    geometry_msgs::msg::Twist cmd;
    switch (state) {
      case ControlState::kRunning:
      cmd.linear.x = cmd_radius_ * cmd_angular_velocity_;
      cmd.angular.z = cmd_angular_velocity_;
      cmd_vel_publisher->publish(cmd);
      break;
      case ControlState::kStopping:
      state = kStopped;
      cmd_vel_publisher->publish(cmd);
      break;
      case ControlState::kStopped:
      break;
    }


  }

  //! @brief service callback for control. Will start the motion
  //! @param / service request providing radius and velocity
  //! @param / service respond (not used)
  void control_srv(const nuturtle_control::srv::Control::Request::SharedPtr req,
                   nuturtle_control::srv::Control::Response::SharedPtr) {
    cmd_radius_ = req->radius;
    cmd_angular_velocity_ = req->velocity;
    state = ControlState::kRunning;
  }

  //! @brief service callback for reverse
  //! @param / service request (not used)
  //! @param / service respond (not used)
  void reverse_srv(const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr) {
    cmd_angular_velocity_ = -cmd_angular_velocity_;
    return;
  }

  //! @brief service callback for stop
  //! @param req teleport request data
  //! @param / service response, not used
  void stop_srv(const std_srvs::srv::Empty::Request::SharedPtr,
                std_srvs::srv::Empty::Response::SharedPtr) {
    state = ControlState::kStopping;
    return;
  }

private:
  // This two get sets by service
  ControlState state = kStopped;
  double cmd_radius_;
  double cmd_angular_velocity_;

  // Private Members
  //! atomic prevent timer and service call modify this together
  // Private ros members
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
};

//! @brief Main entry point for the nusim node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
