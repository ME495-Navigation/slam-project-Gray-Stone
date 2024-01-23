#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class NuSim : public rclcpp::Node
{
public:
  NuSim()
  : Node("nusim"), time_step_(0)
  {
    auto rate_desc = rcl_interfaces::msg::ParameterDescriptor();
    rate_desc.set__name("rate");
    rate_desc.set__description("The rate of main timer");
    declare_parameter<int>("rate", 200, rate_desc);
    rclcpp::Parameter rate_param = get_parameter("rate");
    int rate = rate_param.as_int();

    // time_step_publisher_ =
    // this->create_publisher<std_msgs::msg::String>("topic", 10);
    time_step_publisher_ =
      create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(
        &NuSim::reset_srv, this, std::placeholders::_1,
        std::placeholders::_2));

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

  void reset_srv(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    time_step_ = 0;

    return;
  }

  // Private Members
  std::atomic<uint64_t> time_step_ =
    0;    ///!< prevent timer and service call modify this together

  // Private ros members
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
