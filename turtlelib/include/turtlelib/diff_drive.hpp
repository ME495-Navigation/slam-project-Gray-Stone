#ifndef TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#include "turtlelib/se2d.hpp"

namespace turtlelib {

//! @brief Data type for holding left and right wheel value together.
struct WheelConfig {
  double left;
  double right;
  //! @brief self increment operator for wheel config and left and right value
  //! to respective members
  //! @param rhs delta amount to be incremented to the count
  //! @return
  WheelConfig &operator+=(const WheelConfig &rhs);
};

//! @brief Addition operator for two wheelconfig. simply add left to left, right
//! to right
//! @param lhs Wheel config 1
//! @param rhs Wheel config 2
//! @return Sum of two wheen config
WheelConfig operator+(WheelConfig lhs, WheelConfig rhs);

//! @brief Subtraction operator for two wheelconfig. simply subtract left to left, right
//! to right
//! @param lhs Wheel config 1
//! @param rhs Wheel config 2
//! @return DIff of two wheen config
WheelConfig operator-(WheelConfig lhs, WheelConfig rhs) ;


//! @brief Scale wheel config by a floating point scaler. Will scale each wheel
//! value independently (more commonly used by WheelVelocity)
//! @param lhs Wheel config value to scale
//! @param rhs scaling scaler
//! @return scaled wheel config 
WheelConfig operator*(WheelConfig lhs, double rhs);

//! @brief See the other operator*.
WheelConfig operator*(double lhs, WheelConfig  rhs);

/// \brief print the WheelConfig in the format [L:left R:right]
/// \param os out the ostream to write to
/// \param config the wheel config to output
/// \returns the ostream os  with the wheel config data inserted
std::ostream &operator<<(std::ostream &os, const WheelConfig &config);

// Same underlying data, but different type for better naming.
using WheelVelocity = WheelConfig;

//           Wheel track
//           to body
//           |◄──────►|
//   ┌─────────────────┐
//   │                 │
//   │ Drive Direction │
//   │                 │
//   │                 │
//   │        x        │
//  ┌┤        ▲        ├┐  ───
//  ││        │        ││   ▲  Wheel
//  ││    y◄──┘        ││   │  Radius
//  ││                 ││   ▼
//  └┤                 ├┘  ───
//   └─────────────────┘
//! @brief
class DiffDrive {
public:
  //! @brief Construct the diff drive robot.
  //! @param wheel_track_to_body distance from wheel track to body center. Also
  //! half of wheel-wheel distance
  //! @param wheel_radius Wheel radius.
  //! @param init_pose Initial transform of the robot.
  //! @param init_wheel_config Initial wheel config (count) of the robot.
  DiffDrive(double wheel_track_to_body, double wheel_radius,
            Transform2D init_pose = Transform2D{{0.0, 0.0}, 0.0},
            WheelConfig init_wheel_config = WheelConfig{0.0, 0.0});

  //! @brief Update the config of robot with a new wheel config
  //! @param new_config new wheel config value. Must be within +-PI of previous
  //! config reading. Internally the config delta is normalized to remove
  //! encoder wrap-around (assume encoder warp around at 1 revolution)
  //! @return New updated body config with the new wheel config. The value is
  //! also recorded internally
  Transform2D UpdateRobotConfig(WheelConfig new_config);

  //! @brief Update the body configuration with wheel velocity.
  //! @param wheel_delta wheel velocity * amount of time. aka Delta of wheel
  //! config
  //! @return New updated body config. This value is also recorded internally.
  Transform2D UpdateBodyConfigWithVel(WheelVelocity wheel_delta);

  //! @brief The twist resulted from wheel velocity
  //! @param wheel_delta wheel velocity over one unit time
  //! @param body_phi current body angle. default=0. When using default 0,
  //! return is a body twist. When given a value, the result is a space twist
  //! @return Twist (body or space) result from the given wheel velocity (delta)
  Twist2D TwistFromWheelDelta(WheelConfig wheel_delta, double body_phi = 0.0);

  //! @brief Generate wheel velocity cmd from given body twist command.
  //! @param cmd Desire commanded twist
  //! @return The wheel velocity (rad) that matches this twist command.
  WheelVelocity CommandFromTwist(Twist2D cmd) const;

  //! @brief Get the current body configuration (in world frame)
  //! @return Transform2D of curent body config
  Transform2D GetBodyConfig();

  //! @brief Set the body configuration (teleport)
  //! @param new_tf - New transformation robot should teleport to
  void SetBodyConfig(const Transform2D& new_tf);

  //! @brief Get the current wheel config
  //! @return Current wheel config
  WheelConfig GetWheelConfig();

private:
  // These parameters are required in constructor
  double wheel_body_track_; // half distance between two wheels
  double wheel_radius_;
  Transform2D current_state_{{0.0, 0.0}, 0.0};
  WheelConfig current_wheel_config_{0.0, 0.0};
};

} // namespace turtlelib

#endif