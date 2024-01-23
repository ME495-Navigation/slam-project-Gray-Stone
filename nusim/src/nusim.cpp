#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <optional>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


// using namespace std::chrono_literals;
namespace
{
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

class NuSim : public rclcpp::Node
{
public:
  NuSim()
  : Node("nusim"),
    time_step_(0)
    // TODO(LEO) I hope to init these param properly, but ros param is only accessible after node is made.
    // x0(x_init),
    // y0(y_init),
    // theta0(theta_init),
    // current_x(x_init),
    // current_y(y_init),
    // current_theta(theta_init)
  {

    // Parameters
    declare_parameter<int>("rate", 200, GenParamDescriptor("rate", "The rate of main timer"));
    int rate = get_parameter("rate").as_int();

    declare_parameter<double>(
      "x0", 0.0,
      GenParamDescriptor("x0", "Initial x position"));
    x0 = get_parameter("x0").as_double();
    declare_parameter<double>(
      "y0", 0.0,
      GenParamDescriptor("y0", "Initial y position"));
    y0 = get_parameter("y0").as_double();
    declare_parameter<double>(
      "theta0", 0.0,
      GenParamDescriptor("theta0", "Initial z position"));
    theta0 = get_parameter("theta0").as_double();

    current_x = x0;
    current_y = y0;
    current_theta = theta0;
    // Setup pub/sub and srv/client
    time_step_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(&NuSim::reset_srv, this, std::placeholders::_1, std::placeholders::_2));

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
    auto tf = Gen2DTransform(
      current_x, current_y, current_theta, "nusim/world",
      "red/base_footprint");
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

  // Private Members
  //! atomic prevent timer and service call modify this together
  std::atomic<uint64_t> time_step_ = 0;
  double x0 = 0;  // Init x location
  double y0 = 0;  // Init y location
  double theta0 = 0;  // Init theta location
  double current_x = 0;  // Current x location
  double current_y = 0;  // Current y location
  double current_theta = 0;  // Current theta location

  // Private ros members
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
