#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace turtlelib {

WheelConfig &WheelConfig::operator+=(const WheelConfig &rhs) {
  *this = *this + rhs;
  return (*this);
}

WheelConfig operator+(WheelConfig lhs, WheelConfig rhs) {
  return WheelConfig{lhs.left += rhs.left, lhs.right += rhs.right};
}

WheelConfig operator*(WheelConfig lhs, double rhs) {
  return {lhs.left * rhs, lhs.right * rhs};
}

WheelConfig operator*(double lhs, WheelConfig rhs) { return rhs * lhs; }

std::ostream &operator<<(std::ostream &os, const WheelConfig &config) {
  os << "[L:" << config.left << " R:" << config.right << "]";
  return os;
}

DiffDrive::DiffDrive(double wheel_track_to_body, double wheel_radius,
                     Transform2D init_pose, WheelConfig init_wheel_config)
    : wheel_body_track_(wheel_track_to_body),
      wheel_radius_(wheel_radius), current_state_{init_pose},
      current_wheel_config_(init_wheel_config){};

Transform2D DiffDrive::UpdateRobotConfig(WheelConfig new_config) {
  return UpdateBodyConfigWithVel(
      {normalize_angle(new_config.left - current_wheel_config_.left),
       normalize_angle(new_config.right - current_wheel_config_.right)});
}

// This should be forward kinematic. From robot joint reading (wheel joint) to
// configuration in space (body config)
Transform2D DiffDrive::UpdateBodyConfigWithVel(WheelVelocity wheel_delta) {

  // Using the body twist way. This give use a Tbb', which needs to be Tsb*Tbb'
  Twist2D body_twist = TwistFromWheelDelta(wheel_delta);
  // With space twist (not implemented), the result transform x and y need to
  // be added to current config
  current_state_ *= integrate_twist(body_twist);
  current_wheel_config_ += wheel_delta;
  return current_state_;
}

// A design note:
// Some might ask why not use overload to make two function, one for
// WheelConfig, one for WheelVelocity. This is simply not possible. The overload
// resolution does not separate a new type made from using.
// Example: https://godbolt.org/z/rhehM5fb5

// If really want to make overloading work, could use inheritance
// struct WheelVelocity: public WheelConfig {};
// But this will just be a cold smell in itself

Twist2D DiffDrive::TwistFromWheelDelta(WheelConfig wheel_delta,
                                       double body_phi) {
  // See equation 5 in kinematics.md

  // Modern robotics Chapter 13.3, equation 13.15
  //    [ -1/(2d)  1/(2d) ]
  // r* [ 0.5 *cos(phi)  0.5 *cos(phi)] * [\delta theta L ]
  //    [ 0.5 *sin(phi)  0.5 *sin(phi)]   [\delta theta R]

  // Body angle phi is given by caller. By not providing one (phi=0),
  // Body twist is computed.
  double delta_omega =
      wheel_radius_ * (-1.0 / (2.0 * wheel_body_track_) * wheel_delta.left +
                       1.0 / (2.0 * wheel_body_track_) * wheel_delta.right);
  double delta_x =
      wheel_radius_ * (0.5 * std::cos(body_phi) * wheel_delta.left +
                       0.5 * std::cos(body_phi) * wheel_delta.right);
  double delta_y =
      wheel_radius_ * (0.5 * std::sin(body_phi) * wheel_delta.left +
                       0.5 * std::sin(body_phi) * wheel_delta.right);
  return Twist2D{delta_omega, delta_x, delta_y};
}

WheelVelocity DiffDrive::CommandFromTwist(Twist2D cmd) const {
  // See equation 8 and 9 in kinematics.md

  if (!almost_equal(cmd.y, 0)) {
    throw std::logic_error("Cannot handle cmd_vel with non zero y");
  }
  return WheelVelocity{
      (-wheel_body_track_ / wheel_radius_ * cmd.omega + cmd.x / wheel_radius_),
      (wheel_body_track_ / wheel_radius_ * cmd.omega + cmd.x / wheel_radius_)};
}

Transform2D DiffDrive::GetBodyConfig() { return current_state_; }

void DiffDrive::SetBodyConfig(const Transform2D &new_tf) {
  current_state_ = new_tf;
}

WheelConfig DiffDrive::GetWheelConfig() { return current_wheel_config_; }

} // namespace turtlelib