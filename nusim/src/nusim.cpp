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

// using namespace std::chrono_literals;
namespace {
//! @brief Generate param descriptor object
//! @param name name of the parameter
//! @param description description of the parameter
//! @return constructed descriptor.
auto GenParamDescriptor(std::string name , std::string description){
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
  : Node("nusim"), time_step_(0)
  {

    // Parameters
    declare_parameter<int>("rate", 200, GenParamDescriptor("rate","The rate of main timer"));
    int rate = get_parameter("rate").as_int();

    declare_parameter<double>("x0", 0.0,
                              GenParamDescriptor("x0", "Initial x position"));
    x0 = get_parameter("x0").as_double();
    declare_parameter<double>("y0", 0.0,
                              GenParamDescriptor("y0", "Initial y position"));
    y0 = get_parameter("y0").as_double();
    declare_parameter<double>("theta0", 0.0,
                              GenParamDescriptor("theta0", "Initial z position"));
    theta0 = get_parameter("theta0").as_double();

    // Also set the robot to init location.
    current_x = x0;
    current_y = y0;
    current_theta = theta0;

    // Setup pub/sub and srv/client
    time_step_publisher_ =
      create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(
        &NuSim::reset_srv, this, std::placeholders::_1,
        std::placeholders::_2));

    teleport_service_ = create_service<nusim::srv::Teleport>(
      "~/teleport", std::bind(
        &NuSim::teleport_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Setup timer and set things in motion.
    main_timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&NuSim::main_timer_callback, this));
  }

private:
  // Private function
  void main_timer_callback()
  {
    std_msgs::msg::UInt64 msg;
    msg.data = ++time_step_;
    time_step_publisher_->publish(msg);
  }

  void reset_srv(const std_srvs::srv::Empty::Request::SharedPtr,
                 std_srvs::srv::Empty::Response::SharedPtr) {
    time_step_ = 0;
    current_x = x0;
    current_y = y0;
    current_theta = theta0;
    return;
  }
  void teleport_callback(const nusim::srv::Teleport::Request::SharedPtr req,
                         nusim::srv::Teleport::Response::SharedPtr) {
    current_x = req->x;
    current_y = req->y;
    current_theta = req->theta;
    return;
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
