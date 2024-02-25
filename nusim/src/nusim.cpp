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

#include <nuturtlebot_msgs/msg/detail/sensor_data__traits.hpp>
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
#include <random>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/se2d.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nusim/srv/teleport.hpp"

#include "leo_ros_utils/param_helper.hpp"
#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace {

std::random_device rd{};
std::mt19937 rand_eng{rd()};
auto get_transient_local_qos() {
  auto profile = rmw_qos_profile_default;
  profile.durability =
      rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  // rclcpp::QoSInitialization(rclcpp::KeepAll(),profile)
  rclcpp::QoS transient_local_qos(rclcpp::KeepLast(2), profile);
  transient_local_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  return transient_local_qos;
}
auto transient_local_qos = get_transient_local_qos();

const std::string kWorldFrame = "nusim/world";
const std::string kSimRobotBaseFrameID = "red/base_footprint";

} // namespace

using leo_ros_utils::GetParam;

//! @brief This class is the nusim node itself. Holds everything the node needs.
class NuSim : public rclcpp::Node {
public:
  NuSim()
      : Node("nusim"),

        // Ros related params
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
            turtlelib::Transform2D{{x0, y0}, theta0}}),
        obstacles_x(GetParam<std::vector<double>>(
            *this, "obstacles/x", "list of obstacle's x coord")),
        obstacles_y(GetParam<std::vector<double>>(
            *this, "obstacles/y", "list of obstacle's y coord")),
        obstacles_r(GetParam<double>(*this, "obstacles/r", "obstacle radius")),

        // Simulation only params
        input_noise(GetParam<double>(
            *this, "input_noise",
            "input noise variance when applying wheel velocity", 0)),
        slip_fraction(GetParam<double>(
            *this, "slip_fraction",
            "abs range of wheel slip amount on each cycle.", 0)),
        max_range(GetParam<double>(*this, "max_range",
                                   "maxinum range for basic sensor to see a "
                                   "obsticle. negative will disable max range",
                                   -1.0)),

        // Member variable, not param
        input_gauss_distribution(0.0, input_noise),
        wheel_uniform_distribution(-slip_fraction, slip_fraction),
        basic_sensor_gauss_distribution(
            0.0,
            GetParam<double>(*this, "basic_sensor_variance",
                             "variance of noise in basic sensor's reading.", 0))

  {
    // Uncomment this to turn on debug level and enable debug statements
    // rcutils_logging_set_logger_level(get_logger().get_name(),
    // RCUTILS_LOG_SEVERITY_DEBUG);

    // Arena wall stuff
    const double arena_x_length =
        GetParam<double>(*this, "arena_x_length", "x length of arena", 5.0);
    const double arena_y_length =
        GetParam<double>(*this, "arena_y_length", "x length of arena", 3.0);

    PublishArenaWalls(arena_x_length, arena_y_length);

    // obstacle stuff
    std::cout << "Doing array stuff" << std::endl;

    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(get_logger(), "Mismatch obstacle x y numbers");
      exit(1);
    }
    PublishStaticObstacles(obstacles_x, obstacles_y, obstacles_r);

    // Setup pub/sub and srv/client
    time_step_publisher_ =
        create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    red_sensor_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "red/sensor_data", 10);
    fake_sensor_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/fake_sensor",
                                                               10);

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

    // 5HZ update rate
    fake_sensor_timer_ =
        this->create_wall_timer(std::chrono::milliseconds{200},
                                std::bind(&NuSim::fake_sensor_callback, this));
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

    std::stringstream debug_ss;
    debug_ss << "\n===>\n";

    // Let's move the red bot
    // Directly using duration constructor does work! (default std::ratio is
    // 1:1) Proof here : https://godbolt.org/z/TxMMqvrKh
    double period_sec = std::chrono::duration<double>(update_period).count();

    // Why I get warning saying int to double is narrowing conversion?
    turtlelib::WheelVelocity raw_cmd_vel = {
        static_cast<double>(latest_wheel_cmd.left_velocity),
        static_cast<double>(latest_wheel_cmd.right_velocity)};
    turtlelib::WheelVelocity wheel_cmd_vel =
        raw_cmd_vel * motor_cmd_per_rad_sec * period_sec;

    // This is a special catch showing we have jumped more then pi/2 on
    // wheel in one iteration, which should not happen! (but let's just let
    // error play out)
    if (std::abs(wheel_cmd_vel.left) > turtlelib::PI ||
        std::abs(wheel_cmd_vel.right) > turtlelib::PI) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "This steps's wheel increment is more then PI! "
                             << wheel_cmd_vel);
    }

    debug_ss << "wheel_cmd " << raw_cmd_vel << "\n";
    debug_ss << "wheel_vel " << wheel_cmd_vel << "\n";
    // inject noise between wheel command, and how much motor actually turned.
    if (turtlelib::almost_equal(wheel_cmd_vel.left, 0)) {
      wheel_cmd_vel.left += input_gauss_distribution(rand_eng);
    }
    if (turtlelib::almost_equal(wheel_cmd_vel.right, 0)) {
      wheel_cmd_vel.left += input_gauss_distribution(rand_eng);
    }

    debug_ss << "wheel_vel with noise" << wheel_cmd_vel << "\n";

    red_bot.UpdateBodyConfigWithVel(wheel_cmd_vel);
    debug_ss << "new bot body " << red_bot.GetBodyConfig() << "\n";

    auto new_wheel_config = red_bot.GetWheelConfig();
    debug_ss << "new wheel_config " << new_wheel_config << "\n";

    // The internal tracking of our simulated robot doesn't have wheel slip.
    // Wheel slip only show up as a encoder reading goes off, not robot itself
    // goes off.

    new_wheel_config.left += wheel_uniform_distribution(rand_eng);
    new_wheel_config.right += wheel_uniform_distribution(rand_eng);

    // Since we let diff bot to track our wheel config, it needs to know about
    // this slip-age update as well. Or next cycle it will ignore this.
    red_bot.SetWheelConfig(new_wheel_config);

    debug_ss << "new wheel_config with noise" << new_wheel_config << "\n";

    nuturtlebot_msgs::msg::SensorData red_sensor_msg;
    red_sensor_msg.left_encoder = encoder_ticks_per_rad * new_wheel_config.left;
    red_sensor_msg.right_encoder =
        encoder_ticks_per_rad * new_wheel_config.right;
    red_sensor_msg.stamp = get_clock()->now();
    red_sensor_publisher_->publish(red_sensor_msg);

    time_step_publisher_->publish(msg);

    debug_ss << "encoder sensor value"
             << nuturtlebot_msgs::msg::to_yaml(red_sensor_msg, true);

    RCLCPP_DEBUG_STREAM(get_logger(), debug_ss.str());
    // Publish TF for red robot
    auto tf = Gen2DTransform(red_bot.GetBodyConfig(), kWorldFrame,
                             kSimRobotBaseFrameID);
    tf_broadcaster_->sendTransform(tf);
  }

  void fake_sensor_callback() {

    auto sensor_markers = GenerateObstacleMarkerArray(
        obstacles_x, obstacles_y, obstacles_r, kSimRobotBaseFrameID,
        kFakeSenorStartingID, max_range, red_bot.GetBodyConfig(),
        basic_sensor_gauss_distribution, 0.6);
    fake_sensor_publisher_->publish(sensor_markers);
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
    area_wall_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/walls", transient_local_qos);

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
  void PublishStaticObstacles(std::vector<double> x_s, std::vector<double> y_s,
                              double rad) {
    static_obstacle_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/obstacles", transient_local_qos);
    static_obstacle_publisher_->publish(GenerateObstacleMarkerArray(
        x_s, y_s, rad, kWorldFrame, kStaticObstacleStartingID));
  }

  visualization_msgs::msg::MarkerArray GenerateObstacleMarkerArray(
      std::vector<double> x_s, std::vector<double> y_s, double rad,
      std::string parent_frame, int32_t starting_id,
      std::optional<double> max_range_opt = std::nullopt,
      std::optional<turtlelib::Transform2D> center_transform = std::nullopt,
      std::optional<std::normal_distribution<double>> loc_variance_opt =
          std::nullopt,
      double height = 0.25) {

    visualization_msgs::msg::MarkerArray msg;

    for (size_t i = 0; i < x_s.size(); ++i) {

      // If the center offset is a thing, try to calculate a new xy.
      turtlelib::Point2D obs_loc{x_s.at(i), y_s.at(i)}; // P_world_obs
      if (center_transform) {
        // T_world_robot needs to be flipped
        obs_loc = (*center_transform).inv()(obs_loc); // P_center_obs
      }

      visualization_msgs::msg::Marker obstacle_marker;
      obstacle_marker.header.frame_id = parent_frame;
      obstacle_marker.header.stamp = get_clock()->now();
      obstacle_marker.type = obstacle_marker.CYLINDER;
      obstacle_marker.id = starting_id + i;

      // Only enforce this when its set.
      if (max_range_opt) {
        double range = turtlelib::Vector2D{obs_loc.x, obs_loc.y}.magnitude();
        if (range > *max_range_opt && *max_range_opt >= 0.0) {
          obstacle_marker.action = obstacle_marker.DELETE;
        }
      }

      // Sensor noise after detection
      if (loc_variance_opt) {
        obs_loc.x += (*loc_variance_opt)(rand_eng);
        obs_loc.y += (*loc_variance_opt)(rand_eng);
      }
      obstacle_marker.scale.x = rad * 2;
      obstacle_marker.scale.y = rad * 2;
      obstacle_marker.scale.z = height;
      obstacle_marker.pose.position.x = obs_loc.x;
      obstacle_marker.pose.position.y = obs_loc.y;
      obstacle_marker.pose.position.z = height / 2;

      obstacle_marker.color.r = 1.0;
      obstacle_marker.color.a = 0.8;
      msg.markers.push_back(obstacle_marker);
    }
    return msg;
  }

  // Private Members

  // Constants

  constexpr static int32_t kStaticObstacleStartingID = 10;
  constexpr static int32_t kFakeSenorStartingID = 150;

  // Ros Params
  const std::chrono::nanoseconds
      update_period;       // period for each cycle of update
  const double x0 = 0;     // Init x location
  const double y0 = 0;     // Init y location
  const double theta0 = 0; // Init theta location

  const int motor_cmd_max;
  const double motor_cmd_per_rad_sec;
  const double encoder_ticks_per_rad;
  turtlelib::DiffDrive red_bot;

  // The order of these array also defined their ID. so it matters they don't
  // change
  const std::vector<double> obstacles_x;
  const std::vector<double> obstacles_y;
  const double obstacles_r;

  // simulation only param
  const double input_noise;
  const double slip_fraction;
  const double max_range;
  // These are member variable
  std::normal_distribution<double> input_gauss_distribution;
  std::uniform_real_distribution<double> wheel_uniform_distribution;
  std::normal_distribution<double> basic_sensor_gauss_distribution;

  std::atomic<uint64_t> time_step_ = 0;
  nuturtlebot_msgs::msg::WheelCommands latest_wheel_cmd;

  // Ros objects
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr
      red_sensor_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      fake_sensor_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr
      wheel_cmd_listener_;

  // The publisher need to be kept so the transient local message can still be
  // available later
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      area_wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      static_obstacle_publisher_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

//! @brief Main entry point for the nusim node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
