//! @file nu turtlebot simulator
//! @brief Simulator node for turtlebot.
// Parameters:
//    rate: int - frequency of simulation timer updates (hz)
//    x0: double - Initial x position
//    y0: double - Initial y position
//    theta0: double - Initial theta position
//    arena_x_length: double - x length of arena
//    arena_y_length: double - y length of arena
//    obstacles/x: vector<double> - List of obstical's x coordinates
//    obstacles/y: vector<double> - List of obstical's y coordinates
//    obstacles/r: double - obstacle's radius, all obstacle share this radius
//
// Publishers:
//   /nusim/obstacles: visualization_msgs/msg/MarkerArray
//   /nusim/timestep: std_msgs/msg/UInt64
//   /nusim/walls: visualization_msgs/msg/MarkerArray
//   /parameter_events: rcl_interfaces/msg/ParameterEvent
//   /tf: tf2_msgs/msg/TFMessage
//
// Service Servers:
//   /nusim/reset: std_srvs/srv/Empty
//   /nusim/teleport: nusim/srv/Teleport


#include <rclcpp/parameter_value.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <optional>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/se2d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nusim/srv/teleport.hpp"

#include "leo_ros_utils/param_helper.hpp"
#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace {
std::string kWorldFrame = "nusim/world";
}

using leo_ros_utils::GetParam;

//! @brief This class is the nusim node itself. Holds everything the node needs.
class NuSim : public rclcpp::Node {
public:
  NuSim()
      : Node("nusim"),
        update_period(std::chrono::milliseconds(
            1000 / GetParam<int>(*this, "rate", "The rate of simulator", 200))),
        x0(GetParam<double>(*this, "x0", "inital robot x location")),
        y0(GetParam<double>(*this, "y0", "inital robot y location")),
        theta0(GetParam<double>(*this, "theta0", "initial robot theta")),
        motor_cmd_max(GetParam<int>(*this, "motor_cmd_max", "radius of wheel")),
        motor_cmd_per_rad_sec(
            GetParam<double>(*this, "motor_cmd_per_rad_sec",
                             "motor cmd per rad/s (actually the inverse)")),
        encoder_ticks_per_rad(GetParam<double>(*this, "encoder_ticks_per_rad",
                                               "encoder_ticks_per_rad")),
        red_bot(turtlelib::DiffDrive{
            GetParam<double>(*this, "track_width",
                             "robot center to wheel-track distance"),
            GetParam<double>(*this, "wheel_radius", "wheel radius"),
            turtlelib::Transform2D{{x0, y0}, theta0}})
  {

    // Arena wall stuff
    const double arena_x_length =
        GetParam<double>(*this, "arena_x_length", "x length of arena", 5.0);
    const double arena_y_length =
        GetParam<double>(*this, "arena_y_length", "x length of arena", 3.0);

    PublishArenaWalls(arena_x_length, arena_y_length);

    // obstacle stuff
    std::cout<<"Doing array stuff" << std::endl;

    const std::vector<double> obstacles_x = GetParam<std::vector<double>>(
        *this, "obstacles/x", "list of obstacle's x coord");
    const std::vector<double> obstacles_y = GetParam<std::vector<double>>(
        *this, "obstacles/y", "list of obstacle's y coord");
    const double obstacles_r =
        GetParam<double>(*this, "obstacles/r", "obstacle radius");
    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(get_logger(), "Mismatch obstacle x y numbers");
      exit(1);
    }
    PublishObstacles(obstacles_x, obstacles_y, obstacles_r);

    // Setup pub/sub and srv/client
    time_step_publisher_ =
        create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    red_sensor_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "red/sensor_data", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reset_service_ = create_service<std_srvs::srv::Empty>(
        "~/reset", std::bind(&NuSim::reset_srv, this, std::placeholders::_1,
                             std::placeholders::_2));

    teleport_service_ = create_service<nusim::srv::Teleport>(
        "~/teleport", std::bind(&NuSim::teleport_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

    // Setup timer and set things in motion.
    main_timer_ = this->create_wall_timer(
        update_period, std::bind(&NuSim::main_timer_callback, this));
    wheel_cmd_listener_ =
        create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
            "red/wheel_cmd", 10,
            std::bind(&NuSim::WheelCmdCb, this, std::placeholders::_1));
  }

private:
  // Private functions

  //! @brief Main timer callback function.
  void main_timer_callback() {
    std_msgs::msg::UInt64 msg;
    msg.data = ++time_step_;

    // Let's move the red bot

    // Why I get warning saying int to double is narrowing conversion?
    turtlelib::WheelVelocity vel = {
        static_cast<double>(latest_wheel_cmd.left_velocity),
        static_cast<double>(latest_wheel_cmd.right_velocity)};

    // Directly using duration constructor does work! (default std::ratio is
    // 1:1) Proof here : https://godbolt.org/z/TxMMqvrKh
    double period_sec = std::chrono::duration<double>(update_period).count();
    red_bot.UpdateBodyConfigWithVel(vel * motor_cmd_per_rad_sec * period_sec);

    auto new_wheel_config = red_bot.GetWheelConfig();
    if (std::abs(new_wheel_config.left - old_wheel_config.left) >
            turtlelib::PI ||
        std::abs(new_wheel_config.right - old_wheel_config.right) >
            turtlelib::PI) {
      turtlelib::WheelConfig bad_delta{
          new_wheel_config.left - old_wheel_config.left,
          new_wheel_config.right - old_wheel_config.right};
      RCLCPP_WARN_STREAM(get_logger(),
                         "This steps's wheel increment is more then PI! "
                             << bad_delta);
    }

    nuturtlebot_msgs::msg::SensorData red_sensor_msg;
    red_sensor_msg.left_encoder = encoder_ticks_per_rad * new_wheel_config.left;
    red_sensor_msg.right_encoder =
        encoder_ticks_per_rad * new_wheel_config.right;
    red_sensor_msg.stamp = get_clock()->now();
    red_sensor_publisher_->publish(red_sensor_msg);

    old_wheel_config = new_wheel_config;
    time_step_publisher_->publish(msg);

    // Publish TF for red robot
    auto tf = Gen2DTransform(red_bot.GetBodyConfig(), kWorldFrame,
                             "red/base_footprint");
    tf_broadcaster_->sendTransform(tf);
  }

  //! @brief service callback for reset
  //! @param / service request (not used)
  //! @param / service respond (not used)
  void reset_srv(const std_srvs::srv::Empty::Request::SharedPtr,
                 std_srvs::srv::Empty::Response::SharedPtr) {
    time_step_ = 0;
    red_bot.SetBodyConfig({{x0, y0}, theta0});
    return;
  }

  //! @brief service callback for teleport
  //! @param req teleport request data
  //! @param / service response, not used
  void teleport_callback(const nusim::srv::Teleport::Request::SharedPtr req,
                         nusim::srv::Teleport::Response::SharedPtr) {
    red_bot.SetBodyConfig({{req->x, req->y}, req->theta});
    return;
  }

  void WheelCmdCb(const nuturtlebot_msgs::msg::WheelCommands &msg) {
    latest_wheel_cmd = msg;
  }

  // rclcpp time
  // From https://en.cppreference.com/w/cpp/language/default_arguments
  // A default argument is evaluated each time the function is called with no
  // argument for the corresponding parameter.

  //! @brief Generate a TransformStamped object for 2D transform
  //! @param x x position of transform
  //! @param y y position of transform
  //! @param theta rotation in z axis
  //! @param parent_frame_id frame name for parent
  //! @param child_frame_id frame name for child
  //! @param time_stamp_opt optional time stamp (default to current time)
  //! @return a populated TransformStamped object from given infos.
  geometry_msgs::msg::TransformStamped
  Gen2DTransform(turtlelib::Transform2D trans2d, std::string parent_frame_id,
                 std::string child_frame_id,
                 std::optional<rclcpp::Time> time_stamp_opt = std::nullopt) {

    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = time_stamp_opt.value_or(get_clock()->now());
    tf_stamped.header.frame_id = parent_frame_id;
    tf_stamped.child_frame_id = child_frame_id;
    tf_stamped.transform.translation.x = trans2d.translation().x;
    tf_stamped.transform.translation.y = trans2d.translation().y;
    tf_stamped.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, trans2d.rotation());
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();
    return tf_stamped;
  }

  //! @brief Publish visualization markers for arena walls
  //! @param x_length Wall length in x
  //! @param y_length Wall length in y
  void PublishArenaWalls(double x_length, double y_length) {
    auto profile = rmw_qos_profile_default;
    profile.durability =
        rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // rclcpp::QoSInitialization(rclcpp::KeepAll(),profile)
    rclcpp::QoS qos(rclcpp::KeepLast(2), profile);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    area_wall_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);

    visualization_msgs::msg::MarkerArray msg;

    visualization_msgs::msg::Marker wall_marker;
    wall_marker.type = wall_marker.CUBE;
    wall_marker.header.frame_id = kWorldFrame;
    wall_marker.header.stamp = get_clock()->now();
    wall_marker.scale.z = 0.25;
    wall_marker.scale.y = y_length;
    wall_marker.scale.x = x_length;
    wall_marker.color.r = 1.0;
    wall_marker.color.a = 1.0;
    wall_marker.pose.position.z = 0.25 / 2;
    visualization_msgs::msg::Marker x_plus_wall = wall_marker;
    x_plus_wall.scale.x = 0.01;
    x_plus_wall.pose.position.x = x_length / 2;
    x_plus_wall.id = 1;

    visualization_msgs::msg::Marker x_minus_wall = wall_marker;
    x_minus_wall.scale.x = 0.01;
    x_minus_wall.pose.position.x = -x_length / 2;
    x_minus_wall.id = 2;

    visualization_msgs::msg::Marker y_plus_wall = wall_marker;
    y_plus_wall.scale.y = 0.01;
    y_plus_wall.pose.position.y = y_length / 2;
    y_plus_wall.id = 3;

    visualization_msgs::msg::Marker y_minus_wall = wall_marker;
    y_minus_wall.scale.y = 0.01;
    y_minus_wall.pose.position.y = -y_length / 2;
    y_minus_wall.id = 4;

    msg.markers.push_back(x_plus_wall);
    msg.markers.push_back(x_minus_wall);
    msg.markers.push_back(y_plus_wall);
    msg.markers.push_back(y_minus_wall);
    area_wall_publisher_->publish(msg);

    std::cout << "Published markers" << std::endl;
  }

  //! @brief Publish vitilization markers for obstacles
  //! @param x_s list of x for each obstacle
  //! @param y_s list of y for each obstacle
  //! @param rad radius for all obstacle
  void PublishObstacles(std::vector<double> x_s, std::vector<double> y_s,
                        double rad) {
    auto profile = rmw_qos_profile_default;
    profile.durability =
        rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // rclcpp::QoSInitialization(rclcpp::KeepAll(),profile)
    rclcpp::QoS qos(rclcpp::KeepLast(2), profile);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    obstacle_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles",
                                                               qos);

    visualization_msgs::msg::MarkerArray msg;

    for (size_t i = 0; i < x_s.size(); ++i) {
      visualization_msgs::msg::Marker obstacle_marker;
      obstacle_marker.header.frame_id = kWorldFrame;
      obstacle_marker.header.stamp = get_clock()->now();
      obstacle_marker.type = obstacle_marker.CYLINDER;
      obstacle_marker.id = 10 + i;
      obstacle_marker.scale.x = rad * 2;
      obstacle_marker.scale.y = rad * 2;
      obstacle_marker.scale.z = 0.25;
      obstacle_marker.pose.position.x = x_s.at(i);
      obstacle_marker.pose.position.y = y_s.at(i);
      obstacle_marker.pose.position.z = 0.25 / 2;

      obstacle_marker.color.r = 1.0;
      obstacle_marker.color.a = 1.0;
      msg.markers.push_back(obstacle_marker);
    }
    obstacle_publisher_->publish(msg);
  }

  // Private Members

  // Ros Params
  std::chrono::nanoseconds update_period; // period for each cycle of update
  double x0 = 0;                          // Init x location
  double y0 = 0;                          // Init y location
  double theta0 = 0;                      // Init theta location

  int motor_cmd_max;
  double motor_cmd_per_rad_sec;
  double encoder_ticks_per_rad;
  turtlelib::DiffDrive red_bot;

  // These are member variable
  std::atomic<uint64_t> time_step_ = 0;
  turtlelib::WheelConfig old_wheel_config;
  nuturtlebot_msgs::msg::WheelCommands latest_wheel_cmd;

  // Ros objects
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr
      red_sensor_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr
      wheel_cmd_listener_;

  // The publisher need to be kept so the transient local message can still be
  // available later
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      area_wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      obstacle_publisher_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

//! @brief Main entry point for the nusim node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
