#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>

namespace turtlelib {

  DiffDrive::DiffDrive(double wheel_track_to_body, double wheel_radius,
            Transform2D init_pose,
            WheelConfig init_wheel_config)
    : wheel_body_track_(wheel_track_to_body),
      wheel_radius_(wheel_radius), current_state_{init_pose} , 
      current_wheel_config_(init_wheel_config) {};

// Both wheels started rotating simultaneously
// Both wheels stopped rotating simultaneously
// The wheels rotated at constant velocity
// The wheels achieved their velocity instantaneously
// The wheels did not slip
// The wheels rotated the shortest distance possible to get to their final
// configuration from the starting configuration

DiffDrive::WheelConfig &
DiffDrive::WheelConfig::operator+=(const WheelConfig &rhs) {
  left += rhs.left;
  right += rhs.right;
  return (*this);
}

// This should be forward kinematic. From robot joint reading (wheel joint) to
// configuration in space (body config)
Transform2D DiffDrive::UpdateBodyConfig(WheelConfig wheel_delta) {

  // Using the body twist way. This give use a Tbb', which needs to be Tsb*Tbb'
  Twist2D body_twist = TwistFromDeltaWheel(wheel_delta);
  // With space twist (not implemented), the result transform x and y need to
  // be added to current config
  current_state_ *= integrate_twist(body_twist);
  current_wheel_config_ += wheel_delta;
  return current_state_;
}

Twist2D DiffDrive::TwistFromDeltaWheel(WheelConfig wheel_delta,
                                       double body_phi) {
  // See equation 4 in kinematics.md

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

DiffDrive::WheelVelocity DiffDrive::CommandFromTwist(Twist2D cmd) {
  // See equation 8 in kinematics.md

  return WheelVelocity{
      -1.0 / (2.0 * wheel_body_track_) * cmd.omega + 0.5 * cmd.x,
      -1.0 / (2.0 * wheel_body_track_) * cmd.omega + 0.5 * cmd.x};
}

Transform2D DiffDrive::GetBodyConfig() { return current_state_; }

} // namespace turtlelib