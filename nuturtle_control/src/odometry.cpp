#include "nuturtle_control/ros_math_helper.hpp"
#include "nuturtle_control/ros_param_helper.hpp"
#include "nuturtle_control/srv/detail/init_pose__struct.hpp"
#include <cstddef>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <optional>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/se2d.hpp>
#include <nuturtle_control/srv/init_pose.hpp>

using leo_ros_helper::GetParam;
using leo_ros_helper::GetParamStr;

class Odometry : public rclcpp::Node {
public:
  Odometry()
      : Node("odometry"),
        body_id(GetParamStr(*this, "body_id", "name of the body frame")),
        odom_id(GetParamStr(*this, "odom_id", "name of the odometry frame",
                            "odom")),
        wheel_left(
            GetParamStr(*this, "wheel_left", "name of left wheel joint")),
        wheel_right(
            GetParamStr(*this, "wheel_right", "name of right wheel joint")),
        wheel_radius(
            GetParam<double>(*this, "wheel_radius", "Radius of wheel")),
        track_width(GetParam<double>(*this, "track_width",
                                     "track width between wheel")),
        diff_bot(track_width, wheel_radius), tf_broadcaster(*this) {

    js_linstener = create_subscription<sensor_msgs::msg::JointState>(
        "sensor_data", 10,
        std::bind(&Odometry::JointStateCb, this, std::placeholders::_1));

    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    init_pose_srv = create_service<nuturtle_control::srv::InitPose>(
        "initial_pose", std::bind(&Odometry::init_pose_cb, this,
                             std::placeholders::_1, std::placeholders::_2));
  }

  void JointStateCb(const sensor_msgs::msg::JointState &msg) {

    turtlelib::WheelConfig new_config;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name.at(i) == "wheel_left_joint") {
        new_config.left = msg.position.at(i);
      } else if (msg.name.at(i) == "wheel_right_joint") {
        new_config.right = msg.position.at(i);
      }
    }
    diff_bot.UpdateRobotConfig(new_config);

    turtlelib::Transform2D bot_tf = diff_bot.GetBodyConfig();
    double current_time =
        rclcpp::Time{msg.header.stamp.sec, msg.header.stamp.nanosec}.seconds();
    double delta_time = current_time - last_stamped_tf2d.first;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg.header.stamp;
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;

    odom_msg.pose.pose = leo_ros_helper::Convert(bot_tf);

    odom_msg.twist.twist.linear.x =
        (bot_tf.translation().x - last_stamped_tf2d.second.translation().x) /
        delta_time;
    odom_msg.twist.twist.linear.y =
        (bot_tf.translation().y - last_stamped_tf2d.second.translation().y) /
        delta_time;
    odom_msg.twist.twist.angular.z =
        (bot_tf.rotation() - last_stamped_tf2d.second.rotation()) / delta_time;

    odom_publisher->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_stamped;

    tf_stamped.header.frame_id = body_id;
    tf_stamped.child_frame_id = odom_id;
    tf_stamped.header.stamp = msg.header.stamp;
    tf_stamped.transform = leo_ros_helper::Convert(odom_msg.pose.pose);

    tf_broadcaster.sendTransform(tf_stamped);
    last_stamped_tf2d = std::pair{current_time, bot_tf};
  }

  void
  init_pose_cb(const nuturtle_control::srv::InitPose::Request::SharedPtr req,
               nuturtle_control::srv::InitPose::Response::SharedPtr) {
    diff_bot.SetBodyConfig( turtlelib::Transform2D{{req->x0, req->y0}, req->theta0});
    return;
  }

private:
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;

  // Diff drive needed param
  double wheel_radius;
  double track_width;
  turtlelib::DiffDrive diff_bot;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  std::pair<double, turtlelib::Transform2D> last_stamped_tf2d;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_linstener;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Service<nuturtle_control::srv::InitPose>::SharedPtr init_pose_srv;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  //   rclcpp::Node::SharedPtr node_ptr =
  //   std::make_shared<rclcpp::Node>("turtle_control") ; TurtleControl
  //   t_ctrl{node_ptr};
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}