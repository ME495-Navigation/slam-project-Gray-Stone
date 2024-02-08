#include <algorithm>
#include <cstdint>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <optional>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>
#include <string>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/se2d.hpp>

#include "nuturtle_control/ros_param_helper.hpp"

using leo_ros_helper::GetParam ;

class TurtleControl : public rclcpp::Node {

public:
  TurtleControl()
      : Node("turtle_control"), wheel_radius(GetParam<double>(
                                    *this, "wheel_radius", "Radius of wheel")),
        track_width(GetParam<double>(*this, "track_width",
                                     "track width between wheel")),
        motor_cmd_max(
            GetParam<int>(*this, "motor_cmd_max", "max motor cmd value ")),
        motor_cmd_per_rad_sec(GetParam<double>(
            *this, "motor_cmd_per_rad_sec", "motor command to rad/sec ratio")),
        encoder_ticks_per_rad(
            GetParam<double>(*this, "encoder_ticks_per_rad",
                             "encoder ticks of wheel per radius")),
        collision_radius(GetParam<double>(*this, "collision_radius",
                                          "collision radius of robot")),
        diff_bot(track_width, wheel_radius)
  {
    cmd_vel_listener = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&TurtleControl::CmdVelCb, this, std::placeholders::_1));

    wheel_cmd_publisher =
        create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    sensor_data_listener =
        create_subscription<nuturtlebot_msgs::msg::SensorData>(
            "sensor_data", 10,
            std::bind(&TurtleControl::SensorDataCb, this,
                      std::placeholders::_1));

    joint_state_publisher =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    last_js.header.stamp = get_clock()->now();

    // Stuff the last_js up so on first cycle, it will survive
    last_js.name.push_back("wheel_left_joint");
    last_js.name.push_back("wheel_right_joint");
    last_js.position.push_back(0);
    last_js.position.push_back(0);
  }

private:
  // Callbacks
  void CmdVelCb(const geometry_msgs::msg::Twist &msg) const {
    // map the twist msg into Twist2D
    auto wheel_calc = diff_bot.CommandFromTwist(turtlelib::Twist2D{msg.angular.z , msg.linear.x,msg.linear.y});
    // wheel_cmd.left
    // msg.angular.z
    
    // Publish wheel_cmd to make turtlebot 3 walk

    auto wheel_cmd = nuturtlebot_msgs::msg::WheelCommands();


    // Need to the speed clipping in double.
     double motor_cmd_left=  wheel_calc.left/ motor_cmd_per_rad_sec;
     double motor_cmd_right= wheel_calc.right/ motor_cmd_per_rad_sec;
    int32_t higher_velocity = std::max(motor_cmd_left,motor_cmd_right);
    double scale_factor = 1;
    if (higher_velocity > motor_cmd_max) {
      scale_factor = motor_cmd_max / higher_velocity;
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Motor command velocity is maxing out. Clipping back down. "
          << scale_factor << " times over");
    }

    // Now finally put it back into integer.
    wheel_cmd.left_velocity  = static_cast<int32_t>(motor_cmd_left * scale_factor);
    wheel_cmd.right_velocity = static_cast<int32_t>(motor_cmd_right * scale_factor);

    wheel_cmd_publisher->publish(wheel_cmd);
  }
  void SensorDataCb(const nuturtlebot_msgs::msg::SensorData &msg) {

    auto left_p = EncoderToRad(msg.left_encoder);
    auto right_p = EncoderToRad(msg.right_encoder);

    double time_diff =
        rclcpp::Time{msg.stamp.sec, msg.stamp.nanosec}.seconds() -
        rclcpp::Time{last_js.header.stamp.sec, msg.stamp.nanosec}.seconds();

    // TODO check the time_diff. it might not be doing velocity right.
    sensor_msgs::msg::JointState js;
    js.header.stamp.sec = msg.stamp.sec;
    js.header.stamp.nanosec = msg.stamp.nanosec;

    js.name.push_back("wheel_left_joint");
    js.position.push_back(left_p);
    js.velocity.push_back((left_p - last_js.position.at(0)) / time_diff);

    js.name.push_back("wheel_right_joint");
    js.position.push_back(right_p);
    js.velocity.push_back((right_p - last_js.position.at(1)) / time_diff);

    // Publish to joint_states
    joint_state_publisher->publish(js);
    last_js = js;
  }

  double EncoderToRad(int32_t tick){
    return static_cast<double>(tick) /
                     static_cast<double>(encoder_ticks_per_rad);
  }

  std::pair<double,double> CalcOneEncoder(int32_t new_tick , int32_t old_tick){
    double new_rad = static_cast<double>(new_tick) /
                     static_cast<double>(encoder_ticks_per_rad);

    double old_rad = static_cast<double>(old_tick) /
                     static_cast<double>(encoder_ticks_per_rad);

    double rad_diff = new_rad - old_rad ;

    // If the encoder never wrap around, we don't need to normalize
    return {new_rad, turtlelib::normalize_angle(rad_diff)};
  }

  // parameters
  double wheel_radius;
  double track_width;
  double motor_cmd_max;
  double motor_cmd_per_rad_sec;
  int encoder_ticks_per_rad;
  double collision_radius;


  turtlelib::DiffDrive diff_bot;

  sensor_msgs::msg::JointState last_js;

  // Ros comm components
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr
      wheel_cmd_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_listener;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr
      sensor_data_listener;
  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  //   rclcpp::Node::SharedPtr node_ptr =
  //   std::make_shared<rclcpp::Node>("turtle_control") ; TurtleControl
  //   t_ctrl{node_ptr};
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}