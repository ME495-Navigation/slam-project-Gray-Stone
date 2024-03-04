//! @file odometry calculation node
//! @brief Generate odometry base on wheel encoder change
// Parameters:

// Publishers:
//  tf : world to green odom to blue robot.

// Subscriber:
//  odom - nav_msgs::msg::Odometry : calculated odometry value

// Service Server:
//  initial_pose - nuturtle_control::srv::InitPose : Set the initial pose of the
//  robot when called.


#include <cstddef>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <optional>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/joint_state__traits.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/se2d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <leo_ros_utils/math_helper.hpp>
#include <leo_ros_utils/param_helper.hpp>

using leo_ros_utils::GetParam;

namespace  {

const std::string kWorldFrame = "nusim/world";
}

class Slam : public rclcpp::Node {
public:
  Slam()
      : Node("odometry"),
        body_id(GetParam<std::string>(*this, "body_id", "name of the body frame")),
        odom_id(GetParam<std::string>(*this, "odom_id", "name of the body frame")),
        tf_broadcaster(*this)
        {
    // Uncomment this to turn on debug level and enable debug statements
    // rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    sensor_estimate_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("estimate_sensor", 10);

    // Listen to odom

    // odom topic is always odom
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Slam::OdomCb, this, std::placeholders::_1));

    // Listen to fake sensor

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor", 10, std::bind(&Slam::SensorCb, this, std::placeholders::_1));
  }

  void JointStateCb(const sensor_msgs::msg::JointState &msg) {
    std::stringstream debug_ss;
    debug_ss << "\n===>\n";

    // // Publish the tf stamped.
    // geometry_msgs::msg::TransformStamped tf_stamped;
    // tf_stamped.header.frame_id = odom_id;
    // tf_stamped.child_frame_id = body_id;
    // tf_stamped.header.stamp = msg.header.stamp;
    // tf_stamped.transform = leo_ros_utils::Convert(odom_msg.pose.pose);

    // tf_broadcaster.sendTransform(tf_stamped);
    // last_stamped_tf2d = std::pair{current_time, bot_tf};

    // // Publish the track

    // geometry_msgs::msg::PoseStamped new_pose;
    // new_pose.pose = odom_msg.pose.pose;
    // new_pose.header.frame_id = odom_id;
    // new_pose.header.stamp = get_clock()->now();

    // odom_path_history.push_back(new_pose);
    // if (odom_path_history.size() >= kRobotPathHistorySize) {
    //   odom_path_history.pop_front();
    // }
    // nav_msgs::msg::Path path_msg;
    // path_msg.header = new_pose.header;
    // path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>{odom_path_history.begin(),
    //                                                               odom_path_history.end()};
    // path_publisher_->publish(path_msg);
  }

  void OdomCb(const nav_msgs::msg::Odometry &msg) { new_odom = msg; }

  void SensorCb(const visualization_msgs::msg::MarkerArray& msg ){

    // Static last_used_odom,
    static turtlelib::Transform2D T_world_old_odom;
    // update slam's robot pose guess with the delta in odom (odom is accurate with delta)
    turtlelib::Transform2D  T_world_new_odom = leo_ros_utils::ConvertBack(new_odom.pose.pose);

    // Want T_old_new = T_old_world * T_world_new = T_world_old.inv() * T_world_new
    turtlelib::Transform2D T_old_new_odom = T_world_old_odom.inv() * T_world_new_odom;

    // Apply this delta onto the slam guess
    guessed_robot_world *= T_old_new_odom;



    T_world_old_odom = T_world_new_odom;
    PublishWorldOdomRobot(guessed_robot_world, T_world_new_odom, new_odom.header.stamp);
  }

  void PublishWorldOdomRobot(turtlelib::Transform2D T_world_robot,
                             turtlelib::Transform2D T_odom_robot,
                             builtin_interfaces::msg::Time stamp) {
    auto T_world_odom = T_world_robot * (T_odom_robot.inv());

    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.frame_id = odom_id;
    tf_stamped.child_frame_id = body_id;
    tf_stamped.header.stamp = stamp;
    tf_stamped.transform = leo_ros_utils::Convert(leo_ros_utils::Convert(T_odom_robot));
    tf_broadcaster.sendTransform(tf_stamped);

    tf_stamped.header.frame_id = kWorldFrame;
    tf_stamped.child_frame_id = odom_id;
    tf_stamped.transform = leo_ros_utils::Convert(leo_ros_utils::Convert(T_world_odom));
    tf_broadcaster.sendTransform(tf_stamped);

    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header.frame_id = body_id;
    new_pose.header.stamp = stamp;
    

    nav_msgs::msg::Path path_msg;
    path_msg.header = new_pose.header;

    new_pose.pose.position.x += 0.005;

    path_publisher_->publish(path_msg);


  }

private:
  std::string body_id;
  std::string odom_id;

  nav_msgs::msg::Odometry new_odom;

  turtlelib::Transform2D guessed_robot_world;

  // ROS IDL stuff
  tf2_ros::TransformBroadcaster tf_broadcaster;
  std::pair<double, turtlelib::Transform2D> last_stamped_tf2d;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_estimate_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;

  

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  //   rclcpp::Node::SharedPtr node_ptr =
  //   std::make_shared<rclcpp::Node>("turtle_control") ; TurtleControl
  //   t_ctrl{node_ptr};
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}