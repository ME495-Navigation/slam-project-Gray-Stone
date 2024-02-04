#ifndef TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib {


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

class DiffDrive {
public:

  //! @brief 
  struct WheelConfig {
    double left;
    double right;
    //! @brief self increment operator for wheel config and left and right value
    //! to respective members
    //! @param rhs
    //! @return
    WheelConfig &operator+=(const WheelConfig &rhs);
  };
  // Same underlaying data, but different type for better naming.
  
  using WheelVelocity = WheelConfig;  

  //! @brief Construct the diff drive robot.
  //! @param wheel_track_to_body distance from wheel track to body center. Also half of wheel-wheel distance 
  //! @param wheel_radius Wheel radius. 
  //! @param init_pose Initial transform of the robot.
  //! @param init_wheel_config Initial wheel config (count) of the robot.
  DiffDrive(double wheel_track_to_body, double wheel_radius,
            Transform2D init_pose = Transform2D{{0.0, 0.0}, 0.0},
            WheelConfig init_wheel_config = WheelConfig{0.0, 0.0});

  //! @brief 
  //! @param new_config 
  //! @return 
  Transform2D UpdateBodyConfig(WheelConfig new_config);

  //! @brief 
  //! @param wheel_delta 
  //! @param body_phi 
  //! @return 
  Twist2D TwistFromDeltaWheel(WheelConfig wheel_delta , double body_phi= 0.0);
  
  //! @brief Generate wheel velocity cmd from given body twist command.
  //! @param cmd Desire commanded twist 
  //! @return The wheel velocity (rad) that matches this twist command.
  WheelVelocity CommandFromTwist(Twist2D cmd);
  
  //! @brief Get the current body configuration (in world frame)
  //! @return Transform2D of curent body config
  Transform2D GetBodyConfig();

private:
  // These parameters are required in constructor
  double wheel_body_track_; // half distance between two wheels
  double wheel_radius_;
  Transform2D current_state_{{0.0, 0.0}, 0.0};
  WheelConfig current_wheel_config_{0.0, 0.0};
};

} // namespace turtlelib

#endif