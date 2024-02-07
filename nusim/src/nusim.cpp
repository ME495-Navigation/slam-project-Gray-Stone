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


#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <optional>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nusim/srv/teleport.hpp"

#include <geometry_msgs/msg/pose_with_covariance.hpp>
// using namespace std::chrono_literals;
namespace
{

std::string kWorldFrame = "nusim/world";
//! @brief Generate param descriptor object
//! @param name name of the parameter
//! @param description description of the parameter
//! @return constructed descriptor.
auto GenParamDescriptor(std::string name, std::string description)
{
  auto desc = rcl_interfaces::msg::ParameterDescriptor();
  desc.name = name;
  desc.description = description;
  return desc;
}
}  // namespace

//! @brief This class is the nusim node itself. Holds everything the node needs.
class NuSim : public rclcpp::Node
{
public:
  NuSim()
  : Node("nusim"),
    time_step_(0)
    // TODO(LEO) I hope to init these param properly, but ros param is only accessible after node is
    // made. x0(x_init), y0(y_init), theta0(theta_init), current_x(x_init), current_y(y_init),
    // current_theta(theta_init)
  {
    // Parameters
    declare_parameter<int>("rate", 200, GenParamDescriptor("rate", "The rate of main timer"));
    const int rate = get_parameter("rate").as_int();

    declare_parameter<double>("x0", 0.0, GenParamDescriptor("x0", "Initial x position"));
    x0 = get_parameter("x0").as_double();
    declare_parameter<double>("y0", 0.0, GenParamDescriptor("y0", "Initial y position"));
    y0 = get_parameter("y0").as_double();
    declare_parameter<double>("theta0", 0.0, GenParamDescriptor("theta0", "Initial z position"));
    theta0 = get_parameter("theta0").as_double();


    // Arena wall stuff
    declare_parameter<double>(
      "arena_x_length", 5.0,
      GenParamDescriptor("arena_x_length", "x length of arena"));
    const double arena_x_length = get_parameter("arena_x_length").as_double();
    declare_parameter<double>(
      "arena_y_length", 3.0,
      GenParamDescriptor("arena_y_length", "x length of arena"));
    const double arena_y_length = get_parameter("arena_y_length").as_double();

    PublishArenaWalls(arena_x_length, arena_y_length);

    // obstacle stuff
    declare_parameter<std::vector<double>>(
      "obstacles/x",
      GenParamDescriptor("obstacles/x", "list of obstacle's x coord"));
    const std::vector<double> obstacles_x = get_parameter("obstacles/x").as_double_array();
    declare_parameter<std::vector<double>>(
      "obstacles/y",
      GenParamDescriptor("obstacles/y", "list of obstacle's y coord"));
    const std::vector<double> obstacles_y = get_parameter("obstacles/y").as_double_array();
    declare_parameter<double>(
      "obstacles/r",
      GenParamDescriptor("obstacles/r", "obstacle radius"));
    const double obstacles_r = get_parameter("obstacles/r").as_double();

    if (obstacles_x.size() != obstacles_y.size() ) {
      RCLCPP_ERROR(get_logger(), "Mismatch obstacle x y numbers");
      exit(1);
    }
    PublishObstacles(obstacles_x, obstacles_y, obstacles_r);

    // Rest of init

    current_x = x0;
    current_y = y0;
    current_theta = theta0;
    // Setup pub/sub and srv/client
    time_step_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NuSim::reset_srv, this, std::placeholders::_1, std::placeholders::_2));

    teleport_service_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&NuSim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));


    // Setup timer and set things in motion.
    main_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&NuSim::main_timer_callback, this));
  }

private:
  // Private functions

  //! @brief Main timer callback function.
  void main_timer_callback()
  {
    std_msgs::msg::UInt64 msg;
    msg.data = ++time_step_;
    time_step_publisher_->publish(msg);

    // Publish TF for red robot
    auto tf =
      Gen2DTransform(current_x, current_y, current_theta, kWorldFrame, "red/base_footprint");
    tf_broadcaster_->sendTransform(tf);
  }

  //! @brief service callback for reset
  //! @param / service request (not used)
  //! @param / service respond (not used)
  void reset_srv(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    time_step_ = 0;
    current_x = x0;
    current_y = y0;
    current_theta = theta0;
    return;
  }

  //! @brief service callback for teleport
  //! @param req teleport request data
  //! @param / service response, not used
  void teleport_callback(
    const nusim::srv::Teleport::Request::SharedPtr req,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    current_x = req->x;
    current_y = req->y;
    current_theta = req->theta;
    return;
  }

  // rclcpp time
  // From https://en.cppreference.com/w/cpp/language/default_arguments
  // A default argument is evaluated each time the function is called with no argument for the
  // corresponding parameter.

  //! @brief Generate a TransformStamped object for 2D transform
  //! @param x x position of transform
  //! @param y y position of transform
  //! @param theta rotation in z axis
  //! @param parent_frame_id frame name for parent
  //! @param child_frame_id frame name for child
  //! @param time_stamp_opt optional time stamp (default to current time)
  //! @return a populated TransformStamped object from given infos.
  geometry_msgs::msg::TransformStamped Gen2DTransform(
    double x, double y, double theta, std::string parent_frame_id, std::string child_frame_id,
    std::optional<rclcpp::Time> time_stamp_opt = std::nullopt)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = time_stamp_opt.value_or(get_clock()->now());
    tf_stamped.header.frame_id = parent_frame_id;
    tf_stamped.child_frame_id = child_frame_id;
    tf_stamped.transform.translation.x = x;
    tf_stamped.transform.translation.y = y;
    tf_stamped.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();
    return tf_stamped;
  }

  //! @brief Publish visualization markers for arena walls
  //! @param x_length Wall length in x
  //! @param y_length Wall length in y
  void PublishArenaWalls(double x_length, double y_length)
  {
    auto profile = rmw_qos_profile_default;
    profile.durability = rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
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
  void PublishObstacles(std::vector<double> x_s, std::vector<double> y_s, double rad)
  {
    auto profile = rmw_qos_profile_default;
    profile.durability = rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // rclcpp::QoSInitialization(rclcpp::KeepAll(),profile)
    rclcpp::QoS qos(rclcpp::KeepLast(2), profile);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    obstacle_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos);

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
  //! atomic prevent timer and service call modify this together
  std::atomic<uint64_t> time_step_ = 0;
  double x0 = 0;             // Init x location
  double y0 = 0;             // Init y location
  double theta0 = 0;         // Init theta location
  double current_x = 0;      // Current x location
  double current_y = 0;      // Current y location
  double current_theta = 0;  // Current theta location

  // Private ros members
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  // The publisher need to be kept so the transient local message can still be available later
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr area_wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

//! @brief Main entry point for the nusim node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
