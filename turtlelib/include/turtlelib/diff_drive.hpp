#ifndef TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD

namespace turtlelib {

struct BodyConfig {
  double x;
  double y;
  double theta;
};

class DiffDrive {
public:
  struct WheelConfig {
    double left;
    double right;
  };

  DiffDrive(double wheel_track, double wheel_radius);

  void ForwardKindmatic(WheelConfig new_config);

  

private:
  // These parameters are required in constructor
  double wheel_track_;
  double wheel_radius_;
  BodyConfig current_state_{0.0, 0.0, 0.0};
  WheelConfig current_wheel_phi_{0.0, 0.0};
};

} // namespace turtlelib

#endif