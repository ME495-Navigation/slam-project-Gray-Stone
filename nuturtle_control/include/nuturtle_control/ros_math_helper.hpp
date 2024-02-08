#ifndef NUTURTLE_CONTROL_ROS_MATH_HELPER_INCLUDE_GUARD_HPP
#define NUTURTLE_CONTROL_ROS_MATH_HELPER_INCLUDE_GUARD_HPP

#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlelib/se2d.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform.hpp>

namespace leo_ros_helper {

// Wild guess Matt don't like this naming
geometry_msgs::msg::Pose Convert(turtlelib::Transform2D trans2d);

geometry_msgs::msg::Transform Convert( geometry_msgs::msg::Pose pose);

}

#endif