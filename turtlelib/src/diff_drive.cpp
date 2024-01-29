#include "turtlelib/diff_drive.hpp"

namespace turtlelib {

DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
    : wheel_track_(wheel_track), wheel_radius_(wheel_radius){};

void DiffDrive::ForwardKindmatic(WheelConfig new_config) {
    
}
} // namespace turtlelib